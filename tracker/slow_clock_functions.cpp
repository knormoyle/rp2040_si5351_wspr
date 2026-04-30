
#include <Arduino.h>
// #include <stdlib.h>
// #include <ctype.h>

// does this close putty if true?
extern bool ALLOW_USB_DISABLE_MODE;
extern bool ALLOW_KAZU_12MHZ_MODE;
extern bool ALLOW_TEMP_12MHZ_MODE;
extern bool ALLOW_LOWER_CORE_VOLTAGE_MODE;

#include "led_functions.h"
#include "print_functions.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"
#include "tusb.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

extern bool VERBY[10];
extern bool BALLOON_MODE;

extern uint32_t PLL_SYS_MHZ;
extern uint32_t GPS_WAIT_FOR_NMEA_BURST_MAX;

//#***************************************************************************
// The function now writes four handshake booleans that kazuClocksRestore will read:
static bool kazu_slow_didDeinitSysPll       = false;
static bool kazu_slow_didDeinitUsbPll        = false;
static bool kazu_slow_didRetargetPeripherals = false;
static bool kazu_slow_didLowerCoreVoltage    = false;

// Each step that actually ran flips its corresponding bool to true. 
// Steps gated off by config leave their bool at false. 
// At the top of the function, all three are reset to false so a stale value from a prior cycle can't lie to the restore side.

// The restore side reads these statics to decide what to do:
// kazu_slow_didDeinitSysPll && !ALLOW_KAZU_12MHZ_MODE ...Restore pll_sys / clk_sys
// kazu_slow_didDeinitUsbPll... Restore pll_usb and Serial
// kazu_slow_didRetargetPeripherals and !ALLOW_KAZU_12MHZ_MODE ...Restore clk_peri / clk_adc / clk_rtc

// The !ALLOW_KAZU_12MHZ_MODE checks on restore handle the "stay at 12 MHz" case, 
// where we deliberately don't undo the slow steps even though they happened.
//#***************************************************************************

// =============================================================================
// kazuClocksSlow
// -----------------------------------------------------------------------------
// Aggressive RP2040 power reduction. Three independent steps gated by config:
//
//   ALLOW_TEMP_12MHZ_MODE   -- drop clk_sys to 12 MHz (XOSC) and deinit pll_sys.
//                              We always go down to 12 MHz for lowest power;
//                              whether we *stay* there is governed by the
//                              ALLOW_KAZU_12MHZ_MODE flag below.
//
//   ALLOW_USB_DISABLE_MODE  -- deinit pll_usb (kills printing). Skipped in
//                              BALLOON_MODE because it causes a reboot
//                              (no usb).
//
//   ALLOW_KAZU_12MHZ_MODE   -- also reroute clk_peri / clk_rtc / clk_adc to
//                              XOSC so they survive pll_sys being off. Only
//                              do this if we're staying at 12 MHz, since we
//                              otherwise restore to a pll_sys value and the
//                              peripherals should go back exactly as they were.
//
//   ALLOW_LOWER_CORE_VOLTAGE_MODE -- drop core voltage to 0.95V. Safe at
//                                    12 MHz or 18 MHz. kazuClocksRestore
//                                    bumps voltage back up before
//                                    restoring clk_sys to a higher freq.
//
// Records what it actually did into module-level statics so kazuClocksRestore
// can do the right thing without the caller threading state through.
// Contract:
//   - Each kazuClocksSlow() is paired with exactly one kazuClocksRestore()
//     before the next kazuClocksSlow().
//   - Nothing else writes to the kazu_slow_did* statics.
// =============================================================================


// -----------------------------------------------------------------------------
// Drop clk_sys to the 12 MHz external crystal and power down pll_sys.
// -----------------------------------------------------------------------------
static void switchClkSysTo12MhzXosc(void) {
    // Change clk_sys to be 12MHz (the external crystal is 12mhz).
    clock_configure(clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
        12 * MHZ,
        12 * MHZ);
    // now can turn off pll sys to save power
    pll_deinit(pll_sys);
}

// -----------------------------------------------------------------------------
// Power down pll_usb. Caller is responsible for the BALLOON_MODE / disable
// guards -- skipping pll_usb deinit in BALLOON_MODE avoids a reboot, and
// once it's off we lose the ability to print.
// -----------------------------------------------------------------------------
static void disableUsbPll(void) {
    pll_deinit(pll_usb);
}

// -----------------------------------------------------------------------------
// Reroute clk_peri / clk_rtc / clk_adc onto the XOSC so they keep running
// after pll_sys is off and we stay at 12 MHz indefinitely.
// CLK peri is clocked from clk_sys, so we need to change clk_peri's freq.
// -----------------------------------------------------------------------------
static void retargetPeripheralClocksToXosc(void) {
    // clk_peri = XOSC 12 MHz
    clock_configure(clk_peri,
        0,  // No GLMUX
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
        12 * MHZ,
        12 * MHZ);
        // tried also:
        // 48 * MHZ,
        // 8 * MHZ);  // should this be 8 per the link above?

    // clk_rtc = XOSC 12 MHz / 256 = 46875 Hz
    // FIX! this should be usb clk / 1024 ?? around
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_runtime_init/runtime_init_clocks.c
    clock_configure(clk_rtc,
        0,  // No GLMUX
        CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
        12 * MHZ,
        46875);

    // clk_adc = XOSC 12 MHz / 1 = 12 MHz
    clock_configure(clk_adc,
        0,  // No GLMUX
        CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
        12 * MHZ,
        12 * MHZ);
}

// -----------------------------------------------------------------------------
// Drop the core voltage to 0.95V. Safe at 12 or 18 MHz.
// 0.85V crashed for him. 0.90V worked for him. 0.95V is conservative.
//
// Voltage enums:
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h
// -----------------------------------------------------------------------------
static void lowerCoreVoltage(void) {
    vreg_set_voltage(VREG_VOLTAGE_0_95);
    // Voltage regulator needs a moment to settle before further clock work.
    busy_wait_ms(5);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void kazuClocksSlow(void) {
    V1_println(F("kazuClocksSlow START" EOL));
    V1_flush();
    Watchdog.reset();
    // Reset handshake state. Whatever a previous slow/restore cycle did is
    // no longer relevant -- only what THIS call does matters to the matching
    // kazuClocksRestore().
    kazu_slow_didDeinitSysPll        = false;
    kazu_slow_didDeinitUsbPll        = false;
    kazu_slow_didRetargetPeripherals = false;
    kazu_slow_didLowerCoreVoltage    = false;

    // -------------------------------------------------------------------------
    // Step 1: drop clk_sys to XOSC 12 MHz and power down pll_sys.
    // (This must happen BEFORE lowering voltage -- we need to be safely at
    // a low frequency before reducing the voltage headroom.)
    // -------------------------------------------------------------------------
    if (ALLOW_TEMP_12MHZ_MODE) {
        switchClkSysTo12MhzXosc();
        kazu_slow_didDeinitSysPll = true;
    }
    // -------------------------------------------------------------------------
    // Step 2: lower core voltage to 0.95V.
    // Only safe to do once we're at 12 MHz (or 18 MHz). The previous code
    // gated this on PLL_SYS_MHZ == 18, but if step 1 above just dropped us
    // to 12 MHz, 0.95V is also safe at 12 MHz, so we can drop unconditionally
    // here as long as the flag is set.
    // -------------------------------------------------------------------------
    if (ALLOW_LOWER_CORE_VOLTAGE_MODE) {
        lowerCoreVoltage();
        kazu_slow_didLowerCoreVoltage = true;
    }
    // -------------------------------------------------------------------------
    // Step 3: power down pll_usb (kills printing).
    // Skipped in BALLOON_MODE: deiniting pll_usb causes a reboot (no usb).
    // -------------------------------------------------------------------------
    if (!BALLOON_MODE && ALLOW_USB_DISABLE_MODE) {
        disableUsbPll();
        kazu_slow_didDeinitUsbPll = true;
    }
    // Visual marker so we can confirm we got here even with serial dead.
    blockingLongBlinkLED(3);
    // -------------------------------------------------------------------------
    // Step 3: retarget clk_peri / clk_rtc / clk_adc to XOSC.
    // Only when we plan to STAY at 12 MHz. If we're going to restore to a
    // pll_sys value later, leave these alone -- they should go back just
    // the same as before.
    // -------------------------------------------------------------------------
    if (ALLOW_KAZU_12MHZ_MODE) {
        retargetPeripheralClocksToXosc();
        kazu_slow_didRetargetPeripherals = true;
    }
    // can't print without USB now
    // V1_println(F("kazuClocksSlow END" EOL));
}

// =============================================================================
// kazuClocksRestore
// -----------------------------------------------------------------------------
// Undo what kazuClocksSlow() did. Reads the kazu_slow_did* handshake statics
// (set by kazuClocksSlow) to decide which steps need undoing -- caller does
// NOT thread state through.
//
// Note: kazuClocksSlow declares the handshake statics:
//   static bool kazu_slow_didDeinitSysPll;
//   static bool kazu_slow_didDeinitUsbPll;
//   static bool kazu_slow_didRetargetPeripherals;
//   static bool kazu_slow_didLowerCoreVoltage;
// They must be visible to this function (same .c file is the simplest setup).
//
// Restore order matters:
//   1. Restore core voltage if slow dropped it AND target needs more   -- BEFORE pll_sys
//   2. Restore pll_sys / clk_sys (if slow killed pll_sys AND not staying at 12 MHz)
//   3. Restore pll_usb / Serial (if slow killed pll_usb)               -- before step 4
//   4. Restore clk_peri / clk_adc / clk_rtc (if slow retargeted them
//      AND not staying at 12 MHz)                                      -- needs pll_usb
//   5. Reinit Serial2 if clk_peri's effective frequency changed
//
// Step 1 must come before step 2: bringing pll_sys up at a higher frequency
// while core voltage is still 0.95V can produce instability or glitches.
// Step 3 must come before step 4 because clk_adc and clk_rtc are sourced
// from pll_usb after the restore.
//
// Note: VERBY is NOT cleared while USB was off, so it'd be tempting to
// V1_print at the top of this function -- but don't, USB hasn't been
// re-inited yet.
//
// SDK reference for clocks:
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__clocks.html#gae78816cc6112538a12adcc604be4b344
//
// Voltage enums:
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h
// =============================================================================

// -----------------------------------------------------------------------------
// Restore core voltage based on the target system clock frequency.
// Only called if kazuClocksSlow actually dropped voltage (gated on
// kazu_slow_didLowerCoreVoltage by the caller).
//
// Voltage / frequency map (community-validated, conservative):
//   <= 18 MHz   ->  0.95V is fine (no bump needed)
//   <= 50 MHz   ->  1.00V
//   >  50 MHz   ->  1.10V (default)
// -----------------------------------------------------------------------------
static void restoreCoreVoltageForTargetFreq(uint32_t target_mhz) {
    if (target_mhz <= 18) {
        // Already at 0.95V (set by kazuClocksSlow). Nothing to do --
        // 0.95V is safe at 18 MHz and below.
        return;
    } else if (target_mhz <= 50) {
        vreg_set_voltage(VREG_VOLTAGE_1_00);
    } else {
        // Default 1.10V handles everything up through stock 125-133 MHz.
        vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
    }

    // Voltage regulator needs a moment to settle before we crank the clock.
    busy_wait_ms(5);
}

// -----------------------------------------------------------------------------
// Restore pll_sys / clk_sys to the requested frequency. set_sys_clock_khz()
// re-inits pll_sys internally if it was deinited.
// -----------------------------------------------------------------------------
static void restoreSysPll(uint32_t freq_khz) {
    busy_wait_ms(500);
    set_sys_clock_khz(freq_khz, true);
    PLL_SYS_MHZ = freq_khz / 1000UL;
}

// -----------------------------------------------------------------------------
// Bring pll_usb back up to 48 MHz and re-initialize the USB stack and the
// debug Serial.
//
// pll_init signature:
//   void pll_init(PLL pll, uint ref_div, uint vco_freq,
//                 uint post_div1, uint post_div2);
//   pll        pll_sys or pll_usb
//   ref_div    Input clock divider.
//   vco_freq   Requested output from the VCO (voltage controlled oscillator)
//   post_div1  Post Divider 1 -- range 1-7. Must be >= post_div2
//   post_div2  Post Divider 2 -- range 1-7
//
// 1440 MHz VCO / 6 / 5 = 48 MHz USB reference clock.
// -----------------------------------------------------------------------------
static void restoreUsbAndSerial(void) {
    pll_init(pll_usb, 1, 1440000000, 6, 5);  // return USB pll to 48mhz
    busy_wait_ms(1500);

    // High-level Adafruit TinyUSB init code, does many things to get USB
    // back online.
    tusb_init();
    Serial.begin(115200);
    busy_wait_ms(500);
    V1_print(F("Restored USB pll to 48Mhz, and did Serial.begin()" EOL));
}

// -----------------------------------------------------------------------------
// Restore clk_peri / clk_adc / clk_rtc to their normal sources. Only needed
// if kazuClocksSlow's retargetPeripheralClocksToXosc actually ran.
//
// clk_peri  <- clk_sys      (UART baud rates scale with the system clock)
// clk_adc   <- pll_usb @ 48 MHz
// clk_rtc   <- pll_usb / 1024 = 46875 Hz
//
// Caller must have already restored pll_usb before calling this (clk_adc and
// clk_rtc both source from pll_usb).
// -----------------------------------------------------------------------------
static void restorePeripheralClockSourcesToNormal(uint32_t freq_khz) {
    uint32_t freq_hz = freq_khz * 1000UL;

    clock_configure(clk_peri,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        freq_hz,
        freq_hz);

    clock_configure(clk_adc,
        0,
        CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        48 * MHZ,
        48 * MHZ);

    clock_configure(clk_rtc,
        0,
        CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        48 * MHZ,
        46875);
}

// -----------------------------------------------------------------------------
// Reinit Serial2 to force its UART baud divider to be recomputed against
// the new clk_peri.
// -----------------------------------------------------------------------------
static void reinitSerial2ForNewClkPeri(int currentGpsBaud) {
    Serial2.end();
    busy_wait_ms(10);
    Serial2.begin(currentGpsBaud);
}


// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
//
// `PLL_SYS_MHZ_restore` -- target sys clock in MHz (e.g. 18, 50, 125)
// `currentGpsBaud`      -- baud the GPS chip is currently at; used to
//                          re-begin Serial2.
// -----------------------------------------------------------------------------
void kazuClocksRestore(uint32_t PLL_SYS_MHZ_restore, int currentGpsBaud) {
    // Don't print here -- USB might still be off (see header comment).
    // V1_println(F("kazuClocksRestore START" EOL));
    // V1_flush();

    uint32_t freq_khz = PLL_SYS_MHZ_restore * 1000UL;

    // Snapshot whether clk_peri actually changed during slow, so we can
    // decide whether to reinit Serial2 in step 5 after the handshake bits
    // have been cleared.
    bool clkPeriWillChange =
        ((kazu_slow_didDeinitSysPll || kazu_slow_didRetargetPeripherals) && !ALLOW_KAZU_12MHZ_MODE);

    // -------------------------------------------------------------------------
    // Step 1: bump core voltage BEFORE we crank the clock back up.
    // Only act if slow actually dropped voltage AND we're not staying at
    // kazu 12 MHz.
    // -------------------------------------------------------------------------
    if (kazu_slow_didLowerCoreVoltage && !ALLOW_KAZU_12MHZ_MODE) {
        restoreCoreVoltageForTargetFreq(PLL_SYS_MHZ_restore);
        kazu_slow_didLowerCoreVoltage = false;
    }
    // -------------------------------------------------------------------------
    // Step 2: restore pll_sys / clk_sys, unless caller wants to STAY at
    // kazu 12 MHz.
    // -------------------------------------------------------------------------
    if (ALLOW_KAZU_12MHZ_MODE) {
        // ? have to force this? but will we try to set_freq_hz in tracker.ino?
        // where do we calc the pwm dividers? Recalc?
        // can't use it for the normal set_hz
        PLL_SYS_MHZ = 12;
        // shouldn't be used
        freq_khz = 12000;
    }
    if (kazu_slow_didDeinitSysPll && !ALLOW_KAZU_12MHZ_MODE) {
        restoreSysPll(freq_khz);
        kazu_slow_didDeinitSysPll = false;
    }
    busy_wait_ms(500);
    // -------------------------------------------------------------------------
    // Step 3: restore pll_usb (and TinyUSB / Serial) if slow took it down.
    // Must come before step 4 since clk_adc and clk_rtc source from pll_usb.
    // -------------------------------------------------------------------------
    if (kazu_slow_didDeinitUsbPll) {
        restoreUsbAndSerial();
        kazu_slow_didDeinitUsbPll = false;
    }
    // -------------------------------------------------------------------------
    // Step 4: undo peripheral clock retargeting if slow did it AND we are
    // not staying at kazu 12 MHz.
    // (If staying at 12 MHz, leave clk_peri/adc/rtc on XOSC -- that's the
    //  whole point of "kazu 12 MHz mode".)
    // -------------------------------------------------------------------------
    if (kazu_slow_didRetargetPeripherals && !ALLOW_KAZU_12MHZ_MODE) {
        restorePeripheralClockSourcesToNormal(freq_khz);
        kazu_slow_didRetargetPeripherals = false;
    }
    // -------------------------------------------------------------------------
    // Step 5: reinit Serial2 if clk_peri's effective frequency changed.
    // -------------------------------------------------------------------------
    if (clkPeriWillChange) {
        reinitSerial2ForNewClkPeri(currentGpsBaud);
    }
    // -------------------------------------------------------------------------
    // Status print. By this point USB is up (if it's coming up at all).
    // -------------------------------------------------------------------------
    V1_print(F("After long sleep,"));
    if (ALLOW_KAZU_12MHZ_MODE) {
        V1_printf(" left it at kazu 12Mhz? PLL_SYS_MHZ %lu" EOL, PLL_SYS_MHZ);
    } else {
        V1_printf(" Restored sys_clock_khz() and PLL_SYS_MHZ to %lu" EOL, PLL_SYS_MHZ);
    }
    V1_println(F("kazuClocksRestore END" EOL));

    // Serial communication uses the same system clock as everything else.
    // Baud rate of the serial communication is derived from this main clock
    // frequency. (At 12 MHz Serial2 may misbehave if dividers weren't
    // recalculated -- that's what reinitSerial2ForNewClkPeri is for.)
}

//************************************************
// blurb on pll_usb -> clk_peri uart 48 Mhz (clk_peri) and i2c can be different
// https://github.com/raspberrypi/pico-sdk/issues/841
// rosc @ 1-12 MHz
//
// xosc @ 12 MHz
//     |
//     \-- clk_ref @ 12 MHz
//             |
//             \-- watchdog tick 1:12, @ 1 Mhz
//             |       |
//             |       \-- timer/alarm: get_absolute_time() in micro seconds
//             |
//             \-- pll_sys @ 125 MHz
//             |       |
//             |       \-- clk_sys @ 125 MHz
//             |
//             \-- pll_usb @ 48 MHz
//                   |
//                   \-- clk_peri @ 48 MHz, for UART but not I2C
//                   |       |
//                   |       \-- DMA pacing timers
//                   |
//                   \-- clk_usb
//                   |
//                   \-- clk_adc
//                   |
//                   \-- clk_rtc 1:1024 @ 46,875 Hz
//                             |
//                             \-- RTC 1:46875 @ 1Hz


