// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// OPEN (2025-07-10): SIM65M CB-labelled parts won't accept baud rate change via
// PAIR864/PAIR860. Confirmed on two units. Unlabelled SIM65M units do accept it.
// Workaround: stay at 9600 for CB-labelled parts. See SIM65M_USE_PAIR864 below.

// NMEA sentence output by constellation — from AT6558 datasheet v1.14, §NMEA output table.
// Verified empirically on ATGM336H rev3 2025-01. Re-verify behaviour for SIM65M firmware.
//
// GPS+GLONASS firmware:
//   GPS fix only:          GPRMC GPVTG GPGGA GPGSA GPGSV GPGLL
//   GLONASS fix only:      GNRMC GPVTG GPGGA GNGSA GPGSV GLGSV GNGLL
//   GPS+GLONASS fix:       GNRMC GPVTG GPGGA GNGSA GPGSV GLGSV GNGLL
//   No fix (cold/warm/hot start window): GPRMC GPVTG GPGGA GPGSA GPGSV GPGLL
//
// GPS+BeiDou firmware:
//   GPS fix only:          GPRMC GPVTG GPGGA GPGSA GPGSV GPGLL
//   BeiDou fix only:       BDRMC BDVTG BDGGA BDGSA BDGSV BDGLL
//   GPS+BeiDou fix:        GNRMC GNVTG GNGGA GPGSA BDGSA GPGSV BDGSV GNGLL
//   No fix (cold/warm/hot start window): GNRMC GNVTG GNGGA GNGLL

// REMEMBER: no references to Serial.* or usb in BALLOON_MODE!

// -----------------------------------------------------------------------------
// 1. Corresponding header first (catches missing self-containment)
// -----------------------------------------------------------------------------
#include "gps_functions.h"

// -----------------------------------------------------------------------------
// 2. Arduino / platform headers
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <stdlib.h>
#include <ctype.h>  // for isprint()

// RP2040 SDK hardware headers
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h
#include "hardware/vreg.h"
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__pll.html
#include "hardware/pll.h"
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
#include "hardware/gpio.h"

// for re-init of the tinyUSB (needed for tusb_init())
#include "tusb.h"

// -----------------------------------------------------------------------------
// 3. Third-party libraries
// -----------------------------------------------------------------------------
// wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
#include <TimeLib.h>          // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// -----------------------------------------------------------------------------
// 4. Project headers
// -----------------------------------------------------------------------------
#include "global_structs.h"
#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"
#include "time_functions.h"
#include "pps_functions.h"
#include "slow_clock_functions.h"

// -----------------------------------------------------------------------------
// Extern declarations — all defined in tracker.ino unless noted
// -----------------------------------------------------------------------------

// GPS chip variant selector
extern bool USE_SIM65M;            // false = ATGM336H
// Adjusted in setGpsBroadcast() based on 1-sec vs 5-sec burst interval
extern uint32_t GPS_WAIT_FOR_NMEA_BURST_MAX;
extern uint32_t setTime_millis;    // millis() of last setTime() call
extern bool BALLOON_MODE;

// GPS UART and control pins (defined in tracker.ino)
extern const int GpsPwr;
extern const int GPS_NRESET_PIN;
extern const int GPS_ON_PIN;
extern const int GPS_1PPS_PIN;     // input only; used for PPS calibration
extern const int GPS_UART1_RX_PIN;
extern const int GPS_UART1_TX_PIN;
extern const int SERIAL2_FIFO_SIZE;  // hardware UART FIFO depth = 32 bytes
extern const int ATGM336H_BAUD_RATE;
extern const int SIM65M_BAUD_RATE;

// GPS objects (defined in tracker.ino)
extern TinyGPSPlus gps;
extern absolute_time_t GpsStartTime;  // usecs; reset on each GpsON()

// Verbosity and mode flags
extern bool VERBY[10];            // decode of cc._verbose 0-9
extern uint32_t GpsInvalidAllCnt; // count-down to suppress stale TinyGPS++ state
extern bool GpsInvalidAll;
extern bool IGNORE_KEYBOARD_CHARS;  // set during GPS clock transition; core0 respects it
extern uint32_t PLL_SYS_MHZ;     // current system clock; may be lowered during cold reset

// PPS tracking (zeroed in GpsOFF; written by PPS IRQ)
extern int32_t PPS_rise_millis;
extern int32_t PPS_rise_micros;
extern int32_t PPS_rise_cnt;
extern bool PPS_rise_valid;

// Low-power mode enables (defined in tracker.ino)
extern bool ALLOW_USB_DISABLE_MODE;
extern bool ALLOW_KAZU_12MHZ_MODE;
extern bool ALLOW_TEMP_12MHZ_MODE;
extern bool ALLOW_LOWER_CORE_VOLTAGE_MODE;  // intermittent fails if true?

// Config and telemetry structs
extern ConfigStruct cc;

// -----------------------------------------------------------------------------
// File-scope definitions — module configuration and state
// -----------------------------------------------------------------------------

// true = ramp GPS power via bit-banged PWM (soft-start, limits inrush current).
// false = slam the mosfet on directly; faster but risks the LNA latch-up scar
//         described in initGpsPwrPin(). Leave true for all normal operation.
bool PWM_GPS_POWER_ON_MODE = true;

// true = write "quiet" GPS config to GPS flash after cold reset.
// false = skip (default for flight; avoids flash wear and the risk of getting
//         stuck if we write a non-9600 baud rate to flash).
// 7/10/25: disabled pending SIM65M investigation — re-enable for ground config only.
bool ALLOW_UPDATE_GPS_FLASH_MODE = false;

// Reapply constellation/broadcast/balloon-mode config on every hot reset.
// Risk of skipping: VBAT glitch could wipe GPS config silently.
// Cost of doing it: ~2 sec added to hot reset time.
static bool HOT_RESET_REDO_CONFIG = true;

// 5-second burst interval for SIM65M. False = 1-second (default).
// Set true only when manually debugging constellation counts over a longer window.
static bool SIM65M_BROADCAST_5SECS = false;

// 5-second burst interval for ATGM336H. True = 5-sec; false = 1-sec (default).
// Currently true to reduce serial bus load during balloon flight.
static bool ATGM336H_BROADCAST_5SECS = true;

// Set from cc._const_group in GpsINIT(). 1=GPS 3=GPS+BDS 7=GPS+BDS+GLONASS.
static int CONSTELLATIONS_GROUP = 0;

// -----------------------------------------------------------------------------
// ATGM336H uses AT6558 silicon
// AT6558 BDS/GNSS Full Constellation SOC Chip Data Sheet Version 1.14
// AT6558-5N-3X is GPS + BDS, QFN package 40 pin 5x5x0.8mm
// https://www.icofchina.com/d/file/xiazai/2016-12-05/b1be6f481cdf9d773b963ab30a2d11d8.pdf
// VDD_POR or nRST going low causes internal reset (nRESET).
// nRST can be asserted/deasserted after power-on; tcxo_xref must be running.

// -----------------------------------------------------------------------------
// Timing constraint: must call Serial2.read() within ~1.1 ms per char at 9600
// baud (effective rate ~900 chars/sec from GPS chip). RP2040 UART FIFO is 32
// bytes — that is ~33 ms headroom, but do not rely on it.
// Polling mode is used (not interrupt-driven) for deterministic loop timing.

// -----------------------------------------------------------------------------
// SIM65M contains AG3352 core chip from Airoha.
// https://www.airoha.com/products/p/zy4r082hgNywp1bg
// Adds B1C and L1C frequency bands of Beidou-3 and GPS.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
static bool GpsIsOn_state = false;
bool GpsIsOn(void) {
    return GpsIsOn_state;
}

// -----------------------------------------------------------------------------
// 8 NMEA sentences per burst (max), each up to 255 bytes per NMEA spec.
// Buffer holds one full burst without blocking the serial read loop.
#define NMEA_BUFFER_SIZE (8 * 255)
static char nmeaBuffer[NMEA_BUFFER_SIZE] = { 0 };

// -----------------------------------------------------------------------------
// Shared UART drain state for updateGpsDataAndTime() and nmeaBufferFastPoll().
// Static: internal to this file; not exported. Use getChar() to update both.
static char s_incomingChar   = '\0';
static int  s_charsAvailable = 0;

static void getChar() {
    s_charsAvailable = Serial2.available();
    if (s_charsAvailable) s_incomingChar = Serial2.read();
    else s_incomingChar = '\0';
}
// =============================================================================
// nmeaBufferFastPoll
// -----------------------------------------------------------------------------
// Tries to get all data from the GPS without losing any over a blocking
// window of `duration_millis`. Loops as fast as possible, draining all
// available characters into a RAM buffer each pass.
//
// Filters out spaces and null characters and anything non-printable (CR/LF
// included -- those are unprintable). We add appropriate EOLs when printing
// the buffer, so dropping CR/LF from the captured stream is intentional and
// saves buffer room.
//
// OPEN: if baud rate is wrong, garbage chars arrive. Non-printables are filtered
// out before buffering, so the log may be empty even when a baud mismatch exists.
// Consider: count non-printable chars and log a warning if ratio is high.
// =============================================================================

// -----------------------------------------------------------------------------
// True if `c` is a character we want to capture into the buffer.
// We drop spaces, NULs, and anything non-printable.
// -----------------------------------------------------------------------------
static bool shouldCaptureChar(char c) {
    if (c == '\0') return false;
    if (c == ' ')  return false;
    return isprint((unsigned char)c) != 0;
}

// -----------------------------------------------------------------------------
// Drain all characters currently available from the GPS UART into the buffer,
// filtering on the way. Uses the existing globals: incomingChar, charsAvailable.
// -----------------------------------------------------------------------------
static void drainAvailableCharsIntoBuffer(bool printIfFull) {
    getChar();
    while (s_charsAvailable) {
        if (shouldCaptureChar(s_incomingChar)) {
            nmeaBufferAndPrint(s_incomingChar, printIfFull);
        }
        getChar();
    }
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void nmeaBufferFastPoll(uint32_t duration_millis, bool printIfFull) {
    V1_println(F(EOL "nmeaBufferFastPoll START"));

    // nmeaBuffer should be empty the first time we use this; should be no
    // harm (delay) in checking here.
    nmeaBufferPrintAndClear();

    uint32_t start_millis = millis();
    while (millis() - start_millis < duration_millis) {
        drainAvailableCharsIntoBuffer(printIfFull);
        busy_wait_ms(1);  // just wait 1 milli?
    }

    nmeaBufferPrintAndClear();
    V1_println(F("nmeaBufferFastPoll END"));
}
// -----------------------------------------------------------------------------
// Outputs the content of the nmea buffer to stdio (UART and/or USB)
void nmeaBufferPrintAndClear(void) {
    if (!VERBY[1]) return;

    if (nmeaBuffer[0] != 0) {
        // don't add an EOL to the print since we can accumulate multiple to look good?
        // Might have been missing a EOL. Add one
        V1_println(nmeaBuffer);
        nmeaBuffer[0] = 0;  // Clear the buffer
    }
    // whenever something might have taken a long time like printing the big buffer
    updateStatusLED();
    Watchdog.reset();
}

// add one char at a time
void nmeaBufferAndPrint(const char charToAdd, bool printIfFull) {
    if (!VERBY[1]) return;

    // we might add a EOL before a '$' that begins a sentence. so check for +2
    // EOL might be /r /n or /r/n (two chars). so check for 3.
    // possible 2 in front. 0 null term at end
    if ((strlen(nmeaBuffer) + 3) >= NMEA_BUFFER_SIZE) {
        // make NMEA_BUFFER_SIZE bigger or
        // can just do more nmeaBufferPrint() if we run into a problem realtime
        // we shouldn't have to add EOL to the sentences. they come with CR LF ?
        V1_printf(
            "WARNING: with NMEA_BUFFER_SIZE %d strlen(nmeaBuffer) %d "
            "there is no room for char %c <newline>",
            NMEA_BUFFER_SIZE, strlen(nmeaBuffer), charToAdd);
        V1_println(F("..flushing by emptying first (no print)"));
        // we can't afford to print it before flushing..we'll drop NMEA incoming in the UART
        if (printIfFull) nmeaBufferPrintAndClear();
        else nmeaBuffer[0] = 0;
    }

    int n = strlen(nmeaBuffer);
    if (charToAdd == '$') {
        // put a EOL in first (at the start of every NMEA sentence)
        // https://subethasoftware.com/2024/08/26/in-c-you-can-sizeof-a-string-constant/
        // sizeof EOL will have the null term, so keeping nmeaBuffer always good
        // doing the extra + 1 will put the 0 null term in, also
        strncpy(nmeaBuffer + n, EOL, strlen(EOL) + 1);
        n += strlen(EOL);
    }
    nmeaBuffer[n] = charToAdd;
    nmeaBuffer[n + 1] = 0;
}

// =============================================================================
// gpsSleepForMillis
// -----------------------------------------------------------------------------
// Sleep approximately `n` milliseconds in 10ms increments. While sleeping:
//   - Kicks the watchdog and updates the status LED every 100ms
//     (every 10 ticks of 10ms each), regardless of total duration.
//   - Optionally exits early if Serial2 has data available (a GPS character
//     arrived).
//
// `n` must be in [0, 120000]. Out-of-range values are silently clamped/ignored
// because this function is called while USB is disabled, and BALLOON_MODE /
// VERBY don't protect us from accidental prints in that state -- so we just
// don't print at all here.
// =============================================================================

// 2 minutes: upper bound on any single sleep call; enforced silently (no print safe here).
static const int GPS_SLEEP_MAX_MILLIS        = 120000;
static const int GPS_SLEEP_TICK_MILLIS       = 10;    // ms per sleep tick
static const int GPS_SLEEP_TICKS_PER_SERVICE = 10;    // kick watchdog + LED every 100 ms

void gpsSleepForMillis(int n, bool enableEarlyOut) {
    Watchdog.reset();

    // Out-of-range n is silently ignored: this runs while USB may be disabled
    // so no printing is safe here. Caller is responsible for a valid value.
    if (n < 0) n = 0;
    if (n > GPS_SLEEP_MAX_MILLIS) n = GPS_SLEEP_MAX_MILLIS;

    // Number of 10ms ticks we need.
    int tickCount = n / GPS_SLEEP_TICK_MILLIS;

    for (int i = 0; i < tickCount; i++) {
        // Early-out on incoming GPS data, if the caller asked for it.
        if (enableEarlyOut && Serial2.available()) break;

        // every 100ms kick watchdog and update the LED.
        // No prints here -- USB may be disabled.
        // https://docs.arduino.cc/language-reference/en/functions/time/delay/
        if ((i % GPS_SLEEP_TICKS_PER_SERVICE) == 0) {
            updateStatusLED();
            Watchdog.reset();
        }

        // faster recovery with delay?
        busy_wait_ms(GPS_SLEEP_TICK_MILLIS);
    }
}
// -----------------------------------------------------------------------------
// Internal helper: validate and normalise a requested GPS baud rate.
// Falls back to 9600 for any unsupported value. Only called within this file.
static int checkGpsBaudRate(int desiredBaud) {
    int usedBaud = desiredBaud;
    switch (desiredBaud) {
        case 4800: break;
        case 9600: break;
        case 19200: break;
        case 38400: break;
        case 57600: break;
        case 115200: break;
        default: usedBaud = 9600;
    }
    return usedBaud;
}

// =============================================================================
// getInitialGpsOutput
// -----------------------------------------------------------------------------
// After a hot or cold GPS reset, read whatever the chip emits for up to 5
// seconds, looking for proof-of-life. Returns true if we saw at least 2
// NMEA sentence starts ('$').
//
// There can be a lot of bogus chars after hot/cold reset (over 200). If we
// can get effective 900 chars/sec, we probably want 5x that as our cap --
// hence the 5000-char early-out.
//
// Three early-exit conditions:
//   1. Saw 2+ NMEA sentence starts ('$') -> success
//   2. Read 5000+ characters             -> bail (probably noise)
//   3. 5 seconds elapsed                 -> bail (timeout)
// =============================================================================

static const uint32_t GPS_INITIAL_OUTPUT_TIMEOUT_MS = 5000;  // 5 sec proof-of-life window
// At ~900 chars/sec effective rate, 5000 chars is ~5× a normal burst; bail if exceeded.
static const uint32_t GPS_INITIAL_OUTPUT_MAX_CHARS  = 5000;
static const uint32_t GPS_INITIAL_OUTPUT_MIN_NMEA   = 2;     // 2 sentence-starts = alive

// -----------------------------------------------------------------------------
// Drain everything currently waiting on Serial2 into the NMEA buffer.
// Increments *charCount and *sentenceCount as we go. Skips non-printable
// characters so the captured log is clean (otherwise dos2unix on putty.log
// would choke on stray binary bytes).
//
// Returns true once we hit one of the early-exit thresholds, signalling
// the outer loop to break.
// -----------------------------------------------------------------------------
static bool drainSerial2AndCount(uint32_t *charCount, uint32_t *sentenceCount) {
    while (Serial2.available()) {
        char c = Serial2.read();  // local 'c', not the file-scope s_incomingChar alias
        // skip any non-printable, as we won't be able to dos2unix the
        // putty.log if those are in there
        if (!isprint((unsigned char)c)) continue;
        // buffer it up like we do normally below, so we can see sentences
        nmeaBufferAndPrint(c, true);  // print if full
        *charCount += 1;
        if (c == '$') *sentenceCount += 1;
    }
    if (*charCount >= GPS_INITIAL_OUTPUT_MAX_CHARS) return true;
    if (*sentenceCount >= GPS_INITIAL_OUTPUT_MIN_NMEA) return true;
    return false;
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
bool getInitialGpsOutput(void) {
    V1_println(F("getInitialGpsOutput START"));
    V1_println(F("Look for some Serial2 bytes for 5 secs or 5000 chars or 2 sentences"));
    uint32_t incomingCharCnt     = 0;
    uint32_t incomingSentenceCnt = 0;
    uint32_t start_millis = millis();
    while ((millis() - start_millis) < GPS_INITIAL_OUTPUT_TIMEOUT_MS) {
        Watchdog.reset();
        // Drain whatever's waiting. If we hit an early-exit threshold
        // (enough chars or enough sentences), break out.
        if (Serial2.available()) {
            if (drainSerial2AndCount(&incomingCharCnt, &incomingSentenceCnt)) break;
        }
        // Sleep up to 1 sec, returning early if Serial2 has data again.
        gpsSleepForMillis(1000, true);
    }
    nmeaBufferPrintAndClear();
    updateStatusLED();
    Watchdog.reset();
    V1_println(F("getInitialGpsOutput END"));
    return (incomingSentenceCnt >= GPS_INITIAL_OUTPUT_MIN_NMEA);
}

// -----------------------------------------------------------------------------
void setGpsBalloonMode(void) {
    V1_println(F("setGpsBalloonMode START"));
    //************************
    // Interesting! what kind of extra debuglog output?
    // SIM65M
    // 2.3.54 Packet Type:087 PAIR_COMMON_GET_DEBUGLOG_OUTPUT
    // Query setting of debug log output.
    // $PAIR087*CS<CR><LF>
    // 2. $PAIR087,<Status>*<checksum>
    // DataField:
    // 0: Disable
    // 1: Enable with full debuglog output
    // 2: Enable with lite debuglog output
    // Example:
    // $PAIR087,1*28*35 ==> Enable Debuglog output

    //************************
    // SIM65M
    // wow! just noticed the default mode is fitness mode
    // Navigation mode is set via the NAV_MODE constant below.
    // Packet Type:080 PAIR_COMMON_SET_NAVIGATION_MODE
    // Set navigation mode
    // $PAIR080,<CmdType>*<checksum>
    // '0' Normal mode: For general purpose
    // '1' [Default Value] Fitness mode: For running and walking activities
    //     so that the low-speed (< 5 m/s) movement will have more of an effect
    //     on the position calculation.
    // '2' Reserved Could this be Aviation mode?
    // '3' Reserved Could this be Balloon mode?
    // quectel_l70.pdf has 3 = balloon mode (2015)
    // '4' Stationary mode: For stationary applications where a zero dynamic assumed.
    // '5' Reserved
    // is this really drone mode
    // Drone mode: used for drone applications with equivalent dynamics range
    // and vertical application on different flight phase (Ex. Hovering, cruising etc)
    // '6' Reserved
    // '7' Swimming mode: For swimming purpose so that it smooths the trajectory and
    //     improves the accuracy of distance calculation.
    //************************

    if (USE_SIM65M) {
        // PAIR080 navigation mode selection.
        // Only one of these three is active at a time; change the constant to switch.
        // Mode 3 (balloon) and mode 5 (drone) are marked Reserved in SIM65M spec;
        // Quectel L70 (2015) confirms mode 3 = balloon. Verified working on SIM65M.
        // Disabled alternatives are kept for quick re-testing; they are parsed by the
        // compiler so they will not silently rot.
        static const uint8_t NAV_MODE = 3;  // 0=Normal 3=Balloon(Reserved) 5=Drone(Reserved)
        if (NAV_MODE == 3) {
            Serial2.print("$PAIR080,3*2D" CR LF);
            V1_print(F("Balloon mode: sent $PAIR080,3*2D" CR LF));
        } else if (NAV_MODE == 5) {
            Serial2.print("$PAIR080,5*2B" CR LF);
            V1_print(F("Drone mode: sent $PAIR080,5*2B" CR LF));
        } else {
            Serial2.print("$PAIR080,0*2E" CR LF);
            V1_print(F("Normal mode: sent $PAIR080,0*2E" CR LF));
        }
        Serial2.flush();
        sleep_ms(1000);
    }
    // Should not worry about setting balloon mode (3) for ATGM336?
    // doesn't seem like ATGM336 has a balloon mode in the CASIC specifiction pdf

    V1_println(F("setGpsBalloonMode END"));
}

// -----------------------------------------------------------------------------
void setGnssOn_SIM65M(void) {
    // Packet Type:002 PAIR_GNSS_SUBSYS_POWER_ON
    // Power on the GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // Please send this command before using any location service.
    // $PAIR002*38

    // Packet Type:003 PAIR_GNSS_SUBSYS_POWER_OFF
    // Power off GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // CM4 also can receive commands after sending this command
    // (Include the AT command / the race Command / the part of PAIR
    // command which is not dependent on DSP.)
    // $PAIR003*39

    // Packet Type:020 PAIR_GET_VERSION
    // Query the firmware release information

    // Packet Type:021 PAIR_GET_SETTING_INFO
    // Query the customer related setting,
    // such as the firmware release information, DCB values, HW interface,
    // ULP enable and NVRAM auto saving.

    V1_println(F("setGnssOn_SIM65M START"));
    // PAIR_GET_VERSION
    Serial2.print("$PAIR020*38" CR LF);
    // Serial2.flush();
    // can see that nmeaBufferFastPoll() does get us the responses
    // we don't validate or wait for responses (yet??)
    // we can see version numbers here, et
    // interesting: AG3352Q_V2.5.0.AG3352_20230420

    // I suppose there is some flow control, i.e. I shouldn't send
    // to many overlapping requests. I guess it depends on the service
    // that is absorbing and responding to requests..the ACK/NACK responses
    // aid sw flow control

    // $GNGGA,213449.096,,,,,0,0,,,M,,M
    // $PAIR001,000,4*3F
    // $PAIR001,020,0*39
    // $PAIR020,AG3352Q_V2.5.0.AG3352_20230420,S,N,9ec1cc8,2210141406,2ba,3,,,5bebcf5b,2210141404,72555ce,2210141406,,*17
    // 2/16/25 faster
    nmeaBufferFastPoll(1000, true);  // duration_millis, printIfFull

    //*****************
    if (false) {
        // PAIR_GET_SETTING
        Serial2.print("$PAIR021*39" CR LF);
        // 2/16/25 faster
        nmeaBufferFastPoll(1000, true);  // duration_millis, printIfFull
    }

    //*****************
    // this worked but does it already default to on after power on or ?? Seems to
    // OPEN: changing the power-on default to GNSS-off would reduce peak inrush
    // current on boot. Requires always sending PAIR002 before any location use.
    // Baud rate could also be configured in flash (see writeGpsConfigNoBroadcastToFlash).

    // PAIR_GNSS_SUBSYS_POWER_ON
    // in case we changed the default config to powered off
    Serial2.print("$PAIR002*38" CR LF);
    nmeaBufferFastPoll(2000, true);  // duration_millis, printIfFull
    // 2/16/25 faster
    V1_println(F(EOL "setGnssOn_SIM65M END"));
}

// -----------------------------------------------------------------------------
void setGnssOff_SIM65M(void) {
    V1_println(F("setGnssOff_SIM65M START"));
    // PAIR_GNSS_SUBSYS_POWER_OFF
    Serial2.print("$PAIR003*38" CR LF);
    Serial2.flush();
    sleep_ms(2000);
    V1_println(F("setGnssOff_SIM65M END"));
}

// -----------------------------------------------------------------------------
// always GGA GSA GSV RMC
// never ZDA TXT
// never VTG GLL?
// no GST (ATM336H
void setGpsBroadcast(void) {
    V1_print(F("setGpsBroadcast START" EOL));
    updateStatusLED();
    Watchdog.reset();
    // room for a 60 char sentence with CR and LF also
    char nmeaSentence[62] = { 0 };

    if (USE_SIM65M) {
        //****************
        // Packet Type:050 PAIR_COMMON_SET_FIX_RATE
        // Set Position Fix Interval.
        // If set less than 1000 ms, ASCII NMEA will automatically increase
        // the update interval in order to decrease IO throughput.
        // It will return false if the operating voltage setting is not correct.

        // For SIM65M module, <Fix_Interval> parameter only support 1000 ms.
        // $PAIR050,<Fix_Interval>*<checksum>
        // Fix_Interval--Position fix interval in milliseconds (ms).
        // [Range: 100 ~ 1000]
        // [Example]
        // $PAIR050,1000*12

        //****************
        // Packet Type:062 PAIR_COMMON_SET_NMEA_OUTPUT_RATE
        // Set the NMEA sentence output interval of corresponding NMEA type
        // $PAIR062,<Type>,<Output_Rate>*<checksum>
        // -1 Reset all sentence to default value
        // 0 NMEA_SEN_GGA  // GGA interval - GPS Fix Data
        // 1 NMEA_SEN_GLL, // GLL interval - Geographic Position - Latitude longitude
        // 2 NMEA_SEN_GSA, // GSA interval - GNSS DOPS and Active Satellites
        // 3 NMEA_SEN_GSV, // GSV interval - GNSS Satellites in View
        // 4 NMEA_SEN_RMC, // RMC interval - Minimum Specific GNSS Sentence
        // 5 NMEA_SEN_VTG, // VTG interval - Course Over Ground and Ground Speed
        // 6 NMEA_SEN_ZDA, // ZDA interval - Time & Date

        // Output interval: default 1?
        // 1 per 5 position fixes?
        // is the position fix rate 1 per sec to 10 per sec?
        // see elsewhere. they might have bugs if fix rate is reduced
        // 0 - Disabled or not supported sentence
        // 1 - Output once every one position fix
        // 2 - Output once every two position fixes
        // 3 - Output once every three position fixes
        // 4 - Output once every four position fixes
        // 5 - Output once every five position fixes

        // enable this, because we're disabling broadcast in default config now
        // for SIM65M. Assumes the default fix rate is 1000ms (1 per sec?)

        // why are we getting repeated GNGSA? disable it (just used for custom hdop/vdop/pdop)
        // $GNGSA,A,3,28,32,31,02,10,01,,,,,,,1.56,0.69,1.40,1*02
        // $GNGSA,A,3,67,82,66,76,83,81,,,,,,,1.56,0.69,1.40,2*02
        // $GNGSA,A,3,29,33,,,,,,,,,,,1.56,0.69,1.40,3*00
        // $GNGSA,A,3,19,20,35,29,,,,,,,,,1.56,0.69,1.40,4*0B

        // none of this works?
        // PAIR_COMMON_SET_FIX_RATE $PAIR050,time*<checksum>
        // SIM65M says: 100ms to 1000ms
        // want to straddle the 5 sec broadcast
        // GPS_WAIT_FOR_NMEA_BURST_MAX = 5200;
        // interesting: fix rate for Quectel L76, says this:
        // If the set frequency exceeds 1 Hz,
        // only RMC, GGA and GNS massages will be output at the set frequency,
        // whereas VTG, GLL, ZDA, GRS and GST messages will not be output,
        // and GSA and GSV messages will be output at 1Hz
        // this is the default
        // strncpy(nmeaSentence, "$PAIR050,1000*12" CR LF, 62);

        // 1000ms to 10000ms at 1 sec boundaries ? who said that?
        // ULP mode only support 1Hz.
        // doesn't work?
        // strncpy(nmeaSentence, "$PAIR050,5000*16" CR LF, 62);
        // Serial2.print(nmeaSentence);

        // is typical NMEA talker order this? GGA GLL GSA GSV RMC VTG ZDA)
        // wait.we're getting GSA?

        // all the same size?
        static const char *pair062_1sec[] = {
            "$PAIR062,0,1*3F" CR LF,  // GGA on
            "$PAIR062,1,0*3F" CR LF,  // GLL off (2/17/25)
            // "$PAIR062,2,0*3C" CR LF,  // GSA off (2/24/25)
            "$PAIR062,2,1*3D" CR LF,  // GSA on 4/30/26
            "$PAIR062,3,1*3C" CR LF,  // GSV on
            "$PAIR062,4,1*3B" CR LF,  // RMC on
            "$PAIR062,5,0*3B" CR LF,  // VTG off (2/17/25)
            "$PAIR062,6,0*38" CR LF,  // ZDA off (keep burst <= 1 sec)
        };

        // all the same size?
        // compiler adds a null terminator to each of these
        static const char *pair062_5sec[] = {
            "$PAIR062,0,5*3B" CR LF,  // GGA on
            "$PAIR062,1,0*3F" CR LF,  // GLL off (2/17/25)
            // "$PAIR062,2,0*3C" CR LF,  // GSA off (2/24/25)
            "$PAIR062,2,5*39" CR LF,  // GSA on 4/30/26
            "$PAIR062,3,5*38" CR LF,  // GSV on
            "$PAIR062,4,5*3F" CR LF,  // RMC on
            "$PAIR062,5,0*3B" CR LF,  // VTG off (2/17/25)
            "$PAIR062,6,0*38" CR LF,  // ZDA off (keep burst <= 1 sec)
        };

        // the fix rate can't be slower than 1 sec?
        // 1 Hz (1000 ms): $PCAS02,1000*2E<CRLF>
    
        // don't really need the strncpy
        // strncpy(nmeaSentence, pair062_1sec[i], 62);

        const char **cmds = SIM65M_BROADCAST_5SECS ? pair062_5sec : pair062_1sec;
        size_t n = SIM65M_BROADCAST_5SECS
            ? sizeof(pair062_5sec)/sizeof(*pair062_5sec)
            : sizeof(pair062_1sec)/sizeof(*pair062_1sec);

        for (size_t i = 0; i < n; i++) {
            // could get away without this
            strncpy(nmeaSentence, cmds[i], 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            V1_printf("setGpsBroadcast sent %s" EOL, nmeaSentence);
        }

        // 4/26/26 try enabling NAV-STATUS binary output
        // byte message[] = {0xBA,0xCE,0x04,0x00,0x06,0x01,0x01,0x01,0x01,0x00,0x05,0x01,0x07,0x01,0x0D,0x0A};
        // enables DOP messages?
        // BA CE 04 00 06 01 01 01 01 00 05 01 07 01 0D 0A
        // Serial2.write(message, sizeof(message));
        // busy_wait_us(500);

        Serial2.flush();
        busy_wait_us(2000);
        // Gate 2 must be < the call interval (~5000 ms) but > burst duration (~1000 ms).
        // 4500 ms satisfies both. Previous value of 5200 ms was larger than the call
        // interval, causing gate 2 to block every update after the first.
        GPS_WAIT_FOR_NMEA_BURST_MAX = SIM65M_BROADCAST_5SECS ? 4500 : 900;

    } else {
        //*************************************************
        // ATGM336H
        // ZDA. does this exist for ATGM336H? disabled?
        // this time info is in other sentences also?
        // $–ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx
        // hhmmss.ss = UTC
        // xx = Day, 01 to 31
        // xx = Month, 01 to 12
        // xxxx = Year // xx = Local zone description, 00 to +/- 13 hours
        // xx = Local zone minutes description (same sign as hours)

        //*************************************************
        // from the latest CASIC_ProtocolSpecification_english.pdf
        // Field 1 is the PCAS03

        // 2  nGGA output frequency,
        // statement output frequency is based on positioning update rate
        // n (0~9) means output once every n positioning times, 0 means no output
        // If this statement is left blank, the original configuration will be retained.

        // 3  nGLL GLL output frequency,  same as nGGA
        // 4  nGSA GSA output frequency,  same as nGGA
        // 5  nGSV SV output frequency,   same as nGGA
        // 6  nRMC RMC output frequency,  same as nGGA
        // 7  nVTG VTG output frequency,  same as nGGA
        // 8  nZDA ZDA output frequency,  same as nGGA
        // 9  nANT ANT output frequency,  same as nGGA (this is the antenna open TXT ?)
        // 10 nDHV DHV output frequency,  same as nGGA
        // 11 nLPS LPS output frequency,  same as nGGA
        // 12 res1 reserve
        // 13 res2 reserve
        // 14 nUTC UTC output frequency,  same as nGGA
        // 15 nGST GST output frequency,  same as nGST
        // 16 res3 reserve
        // 17 res4 reserve
        // 18 res5 reserve
        // 19 nTIM TIM (PCAS60) output frequency, same as nGGA

        // 20 CSvalue Hexadecimal value checksum,
        //    XOR result of all characters between $ and * (excluding $ and *)
        // 21 <CR><LF> characters Carriage return and line feed

        // spec has more/new detail. see below
        // strncpy(nmeaSentence, "$PCAS03,1,1,1,1,1,1,0,0,0,0,,,1,1,,,,1*33" CR LF, 62);

        // 2/17/25 no VTG or GLL
        // 2/17/25 disable UTC GST, TIM (wasn't ever getting UTC, TIM?)
        // 2/24/25 no GSA
        // strncpy(nmeaSentence, "$PCAS03,1,0,0,1,1,0,0,0,0,0,,,0,0,,,,0*33" CR LF, 62);
        // 4/25/26 reenable GSA to see 2d vs 3d fix
        const char *cmd = ATGM336H_BROADCAST_5SECS
            ? "$PCAS03,5,0,5,5,5,0,0,0,0,0,,,0,0,,,,0*32" CR LF
            : "$PCAS03,1,0,1,1,1,0,0,0,0,0,,,0,0,,,,0*32" CR LF;
        strncpy(nmeaSentence, cmd, 62);
        Serial2.print(nmeaSentence);
        Serial2.flush();
        busy_wait_us(2000);
        // Gate 2 must be < the call interval (~5000 ms) but > burst duration (~1000 ms).
        // 4500 ms satisfies both. Previous value of 5200 ms was larger than the call
        // interval, causing gate 2 to block every update after the first.
        GPS_WAIT_FOR_NMEA_BURST_MAX = ATGM336H_BROADCAST_5SECS ? 4500 : 900;
        V1_printf("setGpsBroadcast sent %s" EOL, nmeaSentence);
    }

    V1_print(F("setGpsBroadcast END" EOL));
}
// -----------------------------------------------------------------------------
void disableGpsBroadcast(void) {
    // OPEN: SIM65M per-sentence disable verified working 2026-04-30. ATGM path unchanged.
    V1_print(F("disableGpsBroadcast START" EOL));
    updateStatusLED();
    Watchdog.reset();
    char nmeaSentence[64] = { 0 };

    if (USE_SIM65M) {
        // SIM65M: disable each NMEA sentence type individually.
        static const char *pair062_all_off[] = {
            "$PAIR062,0,0*3E" CR LF,
            "$PAIR062,1,0*3F" CR LF,
            "$PAIR062,2,0*3C" CR LF,
            "$PAIR062,3,0*3D" CR LF,
            "$PAIR062,4,0*3A" CR LF,
            "$PAIR062,5,0*3B" CR LF,
            "$PAIR062,6,0*38" CR LF,
        };

        for (size_t i = 0; i < sizeof(pair062_all_off)/sizeof(*pair062_all_off); i++) {
            strncpy(nmeaSentence, pair062_all_off[i], 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
        }
        busy_wait_us(1500);
    } else {
        // checksum from https://www.meme.au/nmea-checksum.html
        strncpy(nmeaSentence, "$PCAS03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*02" CR LF, 64);
        Serial2.print(nmeaSentence);
        Serial2.flush();
        busy_wait_us(2000);
    }
    V1_printf("disableGpsBroadcast sent %s" EOL, nmeaSentence);
    V1_print(F("disableGpsBroadcast END" EOL));
}

// -----------------------------------------------------------------------------
// my GPTXT on 11/18/24.
// $GPTXT,01,01,02,HW=ATGM336H,0004090746370*1E
// $GPTXT,01,01,02,IC=AT6558-5N-31-0C510800,EF16CKJ-F2-008017*5A
// $GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
// $GPTXT,01,01,02,TB=2020-04-28,13:43:10*40
// $GPTXT,01,01,02,MO=GBR*25
// $GPTXT,01,01,02,BS=SOC_BootLoader,V6.2.0.2*34
// $GPTXT,01,01,02,FI=00856014*71
// $GPTXT,01,01,01,ANTENNAOPEN*25

// -----------------------------------------------------------------------------
// examples of GPTXT messages from spec. we are disabling them now in setGpsBroadcast
// this is not from oiurs
// $GPTXT,01,01,02,MA=CASIC*27
// Indicates the manufacturer's name (CASIC)
// $GPTXT,01,01,02,IC=ATGB03+ATGR201*71
// Indicates the model of the chip or chipset
// (baseband chip model ATGB03, RF chip model ATGR201)
// $GPTXT,01,01,02,SW=URANUS2,V2.2.1.0*1D
// Indicates the software name and version number
// (software name URANUS2, version number V2.2.1.0)
// $GPTXT,01,01,02,TB=2013-06-20,13:02:49*43
// Indicates the code compilation time (June 20, 2013, 13:02:49)
// $GPTXT,01,01,02,MO=GB*77
// Indicates the working mode of the receiver at this startup
// (GB indicates the dual-mode mode of GPS+BDS)
// $GPTXT,01,01,02,CI=00000000*7A
// Indicates the customer number (the customer number is 00000000)

// -----------------------------------------------------------------------------
// re: GSV nmea sentences from SIM65M
// Depending on the number of satellites tracked,
// multiple messages of GSV data may be required.
// In some software versions, maximum number of satellites reported as visible is limited to 12, 
// even though more may be visible
// Hmm: is this fully compatible with max number expected by TinyGPS++ parsing?

// -----------------------------------------------------------------------------
// re: SIM65M comments on the *RMC nmea sentences
// A valid status is derived from all the parameters set in the software.
// This includes the minimum number of satellites required,
// any DOP mask setting,
// presence of DGPS corrections, etc.
// If the default or current software setting requires that a factor is met,
// then if that factor is not met, the solution will be marked as invalid
// Does not support magnetic declination.
// All “course over ground” data are geodetic WGS84 directions relative to true North

// -----------------------------------------------------------------------------
// re: SIM65M 'PAIR' sentences:
// PAIR command is an AIROHA proprietary GNSS data transferring protocol.
// This protocol is used to configure the GNSS module’s parameters,
// to set/get aiding information,
// and to receive notifications from the GNSS module.
// To process data conveniently, the PAIR commands matches with the NMEA sentence format.

// Quectel LC76G spec says:j
// also mentions some weird things in LC86G ?

// GNSS search modes supported by LC26G (AB), LC26G-T (AA), LC76G series and LC86G (LA, PA):
// why does it say QZSS is always enabled by default?
// GPS only
// GPS + QZSS
// GPS + GLONASS
// GPS + GLONASS + QZSS
// GPS + Galileo
// GPS + Galileo + QZSS
// GPS + BDS
// GPS + BDS + QZSS
// GPS + GLONASS + Galileo + BDS
// GPS + GLONASS + Galileo + BDS + QZSS
// =============================================================================
// setGpsConstellations
// -----------------------------------------------------------------------------
// Tells the GPS module which GNSS constellations to search for. The encoding
// is a small integer:
//
//   1 = GPS                 5 = GPS + GLONASS
//   2 = BDS                 6 = BDS + GLONASS
//   3 = GPS + BDS           7 = GPS + BDS + GLONASS
//   4 = GLONASS             default (anything else) = GPS + BDS (i.e. 3)
//
// Future ideas (kept from original):
//   case 0: ; // OPEN: could mean "disable all" — not in CASIC spec; untested
//   case 8: ; // OPEN: could mean "enable all"  — not in CASIC spec; untested
//
// SIM65M reference:
//   Packet Type 066 -- PAIR_COMMON_SET_GNSS_SEARCH_MODE
//   Configure the receiver to start searching for satellites.
//   The setting is available when the NVRAM data is valid.
//   The device restarts when it receives this command.
//   Abbreviations: GPS "G", GLONASS "R", Galileo "E", BeiDou "B", NavIC "I".
//
//   Field meaning (1 = enable, 0 = disable):
//     PAIR066, GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC
//
//   L1 single-frequency supports 5 documented modes G / GR / GE / GB / GREB:
//     PAIR066,1,0,0,0,0,0   GPS only
//     PAIR066,1,1,0,0,0,0   GPS+GLONASS
//     PAIR066,1,0,1,0,0,0   GPS+GALILEO
//     PAIR066,1,0,0,1,0,0   GPS+BEIDOU
//     PAIR066,1,1,1,1,0,0   GPS+GLONASS+GALILEO+BEIDOU
//     PAIR066,1,1,0,1,0,0   GPS+GLONASS+BEIDOU
//   QZSS is always switchable.
//
// OPEN: SIM65M PAIR066 constellation support verified for cases 1,3,5,7 (2026-04-30).
// Cases 2,4,6 (BDS-only, GLONASS-only, BDS+GLONASS) are untested on SIM65M hardware.
// OPEN: consider forcing constellation=3 (GPS+BDS) as the flight default regardless
// of cc._const_group, to simplify the NMEA sentence set seen in flight.
// =============================================================================

// -----------------------------------------------------------------------------
// SIM65M sentence lookup. Returns a static string (not heap-allocated).
// Galileo is field 3. The last two fields are QZSS and NavIC.
// (Are some of these wrong, and is Galileo set in the default?)
// On unknown input, falls back to GPS+BDS and updates *usedConstellations.
// -----------------------------------------------------------------------------
static const char *sim65mConstellationSentence(int *usedConstellations) {
    switch (*usedConstellations) {
        case 1: return "$PAIR066,1,0,0,0,0,0*3B" CR LF;  // GPS
        case 2: return "$PAIR066,0,0,0,1,0,0*3B" CR LF;  // BDS
        case 3: return "$PAIR066,1,0,0,1,0,0*3A" CR LF;  // GPS+BDS
        case 4: return "$PAIR066,0,1,0,0,0,0*3B" CR LF;  // GLONASS
        case 5: return "$PAIR066,1,1,0,0,0,0*3A" CR LF;  // GPS+GLONASS
        case 6: return "$PAIR066,0,1,0,1,0,0*3A" CR LF;  // BDS+GLONASS
        case 7: return "$PAIR066,1,1,0,1,0,0*3B" CR LF;  // GPS+BDS+GLONASS
        default:
            *usedConstellations = 3;
            return "$PAIR066,1,0,0,1,0,0*3A" CR LF;      // GPS+BDS
    }
}

// -----------------------------------------------------------------------------
// PCAS (non-SIM65M) sentence lookup.
// Note: case 0 isn't defined in the CASIC_ProtocolSpecification.pdf.
// On unknown input, falls back to GPS+BDS and updates *usedConstellations.
// -----------------------------------------------------------------------------
static const char *pcasConstellationSentence(int *usedConstellations) {
    switch (*usedConstellations) {
        case 1: return "$PCAS04,1*18" CR LF;   // GPS
        case 2: return "$PCAS04,2*1B" CR LF;   // BDS
        case 3: return "$PCAS04,3*1A" CR LF;   // GPS+BDS
        case 4: return "$PCAS04,4*1D" CR LF;   // GLONASS
        case 5: return "$PCAS04,5*1C" CR LF;   // GPS+GLONASS
        case 6: return "$PCAS04,6*AF" CR LF;   // BDS+GLONASS
        case 7: return "$PCAS04,7*1E" CR LF;   // GPS+BDS+GLONASS
        default:
            *usedConstellations = 3;
            // Note: original default used checksum *1D rather than *1A here. (error)
            return "$PCAS04,3*1A" CR LF;       // GPS+BDS
    }
}

void setGpsConstellations(int desiredConstellations) {
    V1_printf("setConstellations START %d" EOL, desiredConstellations);
    updateStatusLED();
    Watchdog.reset();

    int usedConstellations = desiredConstellations;
    char nmeaSentence[62] = { 0 };

    // Pick the chip-specific sentence. The helper may reset
    // usedConstellations to 3 (GPS+BDS) on unsupported input.
    const char *src = USE_SIM65M
        ? sim65mConstellationSentence(&usedConstellations)
        : pcasConstellationSentence(&usedConstellations);
    strncpy(nmeaSentence, src, sizeof(nmeaSentence));

    // Send to the GPS.
    // Huh -- on SIM65M this PAIR066 command also causes a GPS reset
    // (presumably a hot reset).
    Serial2.print(nmeaSentence);
    Serial2.flush();
    busy_wait_ms(1000);

    V1_printf("setGpsConstellations for usedConstellations %d, sent %s" EOL,
              desiredConstellations, nmeaSentence);
    V1_printf("setGpsConstellations END %d" EOL, desiredConstellations);
}

// =============================================================================
// setGpsBaud
//
// SIM65M PAIR command reference (Source: SIM65M Series_NMEA Message_User Guide_V1.00):
//   PAIR002*38  PAIR_GNSS_SUBSYS_POWER_ON  — power on DSP/RF/Clock; send before location use
//   PAIR003*39  PAIR_GNSS_SUBSYS_POWER_OFF — power off; CM4 still accepts PAIR config commands
//   PAIR004*3E  PAIR_GNSS_SUBSYS_HOT_START  — all NVRAM data valid
//   PAIR005*3F  PAIR_GNSS_SUBSYS_WARM_START — clear ephemeris only
//   PAIR006*3C  PAIR_GNSS_SUBSYS_COLD_START — clear position/almanac/ephemeris
//   PAIR007*3D  PAIR_GNSS_SUBSYS_FULL_COLD_START — factory default reset
//   PAIR023*3B  PAIR_SYSTEM_REBOOT          — reboot entire chip including CM4
//   PAIR062     PAIR_COMMON_SET_NMEA_OUTPUT_RATE — per-sentence interval
//   PAIR864     PAIR_IO_SET_BAUDRATE         — baud rate; must reboot to take effect
//     $PAIR864,0,0,115200*1B  — UART0 to 115200
//   PAIR860     PAIR_IO_OPEN_PORT            — open port with data type bitmap + baud
//     Data_Type bitmap: OUT_NMEA=0x01 OUT_LOG=0x02 OUT_CMD_RSP=0x04 IN_CMD=0x20
//     $PAIR860,0,0,37,9600,0*23  — UART0 NMEA out, 9600, no flow control
//     $PAIR860,0,0,37,115200,0*2B — UART0 NMEA out, 115200, no flow control
//   PAIR866     PAIR_IO_SET_FLOW_CONTROL     — 0=none 1=SW 2=HW
//     $PAIR866,0,2,1*2D  — UART2 SW flow control ON
// -----------------------------------------------------------------------------
// Tells the GPS module to switch to `desiredBaud`, then reconfigures Serial2
// on our side to match.
//
// Assumes we can already talk to the GPS at some agreed-on Serial2 baud
// (set up by init / hot reset / full cold reset). After power-on the GPS
// starts at 9600 baud.
//
// NMEA notes:
//   - Sentences must be terminated with CR + LF.
//   - CR/LF are defined in print_functions.h. They are not part of the
//     checksum, nor is the leading '$'.
//   - Example: $PMTK251,38400*27<CR><LF>
//   - Checksums are pre-calculated and hardwired into the static sentences
//     used here (see https://www.meme.au/nmea-checksum.html). After
//     checkGpsBaudRate() we should only ever be looking up legal bauds.
// =============================================================================

// PAIR864 is preferred over PAIR860 for SIM65M baud-rate changes.
// PAIR860 doesn't list all baud values in spec; PAIR864 does.
// Measured 2026-04-30: both work at 9600; PAIR860 failed to change SIM65M-CB
// from 115200; PAIR864 succeeded. Set false only for targeted debugging.
static const bool SIM65M_USE_PAIR864 = true;

// -----------------------------------------------------------------------------
// Pick the SIM65M baud-change NMEA sentence for a given baud.
// Falls back to 9600 for any unsupported baud (and updates *usedBaud).
// -----------------------------------------------------------------------------
static void buildSIM65MBaudSentence(int *usedBaud, char *out, size_t outSize) {
    // $PAIR860,0,0,37,9600,0*23 means:
    // Open UART0 to NMEA output without flow control.  Baudrate is 9600.
    // PAIR864 lists alternate baudrates only but says min is 115200?

    // Yes! 9600 works after boot with 115200! (no buffer overflow)
    // Did this stop working?
    // This worked if default 115200 originally..for SIM65M module. (not SIM65M-CB?)
    const char *s;
    switch (*usedBaud) {
        // they say the SIM65M-C uses $PAIR864 for setting baud
        // but 115200 minimum?
        // 2.3.147 Packet Type:860 PAIR_IO_OPEN_PORT Open a GNSS data port
        // PAIR860 checksum was wrong! try again 4/30/26
        // the 37 is GNSS_IO_FLAG_OUT_* (all out data response) + GNSS_IN_CMD .. not DATA or RTCM in
        // 0 is disable flow control

        // PAIR864 might only support 115200 and above?
        case 4800:   // supported or ??
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,4800*10"   CR LF
                                   : "$PAIR860,0,0,37,4800,0*20" CR LF; break;
        case 9600:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,9600*13"   CR LF
                                   : "$PAIR860,0,0,37,9600,0*23" CR LF; break;
        case 19200:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,19200*26"  CR LF
                                   : "$PAIR860,0,0,37,19200,0*16" CR LF; break;
        case 38400:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,38400*23"  CR LF
                                   : "$PAIR860,0,0,37,38400,0*13" CR LF; break;
        case 57600:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,57600*28"  CR LF
                                   : "$PAIR860,0,0,37,57600,0*18" CR LF; break;
        case 115200:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,115200*1B" CR LF
                                   : "$PAIR860,0,0,37,115200,0*2B" CR LF; break;
        default:
            *usedBaud = 9600;
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,9600*13"   CR LF
                                   : "$PAIR860,0,0,37,9600,0*23" CR LF; break;
    }
    strncpy(out, s, outSize);
}

// -----------------------------------------------------------------------------
// Pick the non-SIM65M (PCAS / PMTK) baud-change NMEA sentence for a given baud.
// Falls back to 9600 for any unsupported baud (and updates *usedBaud).
//
// History notes (kept from original code):
//   - 9600  $PCAS01,1   didn't work / now it worked? seems OK if chip is in 9600 state.
//   - 9600  $PMTK251    worked once but then broke; needed Arduino restart.
//   - 19200 $PCAS01,2   worked.   $PMTK251,19200 didn't work.
//   - 38400 $PCAS01,3   worked.   $PMTK251,38400 didn't work.
//   - 57600 $PCAS01,4   didn't work. $PMTK251,57600 untested.
//   - 115200 $PCAS01,5  prints, but lots of Rx buffer-overrun ERRORs.
// -----------------------------------------------------------------------------
static void buildPCASBaudSentence(int *usedBaud, char *out, size_t outSize) {
    const char *s;
    switch (*usedBaud) {
        case 4800:   s = "$PCAS01,0*1C" CR LF; break;
        case 9600:   s = "$PCAS01,1*1D" CR LF; break;
        case 19200:  s = "$PCAS01,2*1E" CR LF; break;
        case 38400:  s = "$PCAS01,3*1F" CR LF; break;
        case 57600:  s = "$PCAS01,4*18" CR LF; break;
        case 115200: s = "$PCAS01,5*19" CR LF; break;
        default:
            *usedBaud = 9600;
            // (legacy alternative) "$PMTK251,9600*17" CR LF
            s = "$PCAS01,1*1D" CR LF;
            break;
    }
    // Original used a hard length of 21 here
    strncpy(out, s, outSize);
}

// -----------------------------------------------------------------------------
// Send an NMEA sentence over Serial2 and wait long enough for the GPS to
// finish processing it.
// https://forum.arduino.cc/t/solved-proper-way-to-change-baud-rate-after-initial-setup/419860/5
// -----------------------------------------------------------------------------
static void sendNmeaAndWait(const char *sentence) {
    Serial2.print(sentence);
    Serial2.flush();
    // have to wait for the sentence to get out and complete at the GPS
    busy_wait_ms(1000);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void setGpsBaud(int desiredBaud) {
    V1_printf("setGpsBaud START %d" EOL, desiredBaud);
    updateStatusLED();
    Watchdog.reset();

    // Clamp desiredBaud to a baud the chip actually supports.
    int usedBaud = checkGpsBaudRate(desiredBaud);

    // Build the chip-specific baud-change sentence. usedBaud may be reset
    // to 9600 inside the helper if the requested rate is unsupported.
    char nmeaBaudSentence[64] = { 0 };
    if (USE_SIM65M) {
        buildSIM65MBaudSentence(&usedBaud, nmeaBaudSentence, sizeof(nmeaBaudSentence));
    } else {
        buildPCASBaudSentence(&usedBaud, nmeaBaudSentence, sizeof(nmeaBaudSentence));
    }

    // Drain anything still sitting in the CPU output buffer before we send.
    Serial2.flush();
    busy_wait_ms(1000);

    sendNmeaAndWait(nmeaBaudSentence);
    V1_printf("setGpsBaud for usedBaud %d, sent %s" EOL, usedBaud, nmeaBaudSentence);

    // 7/10/25 spec says to reboot after baud rate change (SIM65M only).
    if (USE_SIM65M) {
        strncpy(nmeaBaudSentence, "$PAIR023*3B" CR LF, sizeof(nmeaBaudSentence));
        sendNmeaAndWait(nmeaBaudSentence);
        V1_printf("setGpsBaud reboot. gps sent %s" EOL, nmeaBaudSentence);
    }

    // Serial2.end() Disables serial communication, allowing the RX and TX
    // pins to be used for general input and output.
    // To re-enable serial communication, call Serial2.begin().

    // OPEN: auto-detect current GPS baud by trying each rate in sequence and
    // checking for NMEA responses. Would recover from an unknown baud state without
    // requiring full power-cycle. VBAT appears to retain the programmed baud rate
    // across resets on ATGM336H — makes anything other than 9600 risky in flight.
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);

    // If the GPS-side command didn't actually change baud, you'd need to
    // change this Serial2.begin() to match what the GPS is really using.
    Serial2.begin(usedBaud);
    V1_printf("setGpsBaud did Serial2.begin(%d)" EOL, usedBaud);
    gpsSleepForMillis(1000, false);
    V1_printf("setGpsBaud END %d" EOL, usedBaud);
}

// =============================================================================
// GpsINIT
// -----------------------------------------------------------------------------
// One-time GPS initialization:
//   1. Configure the GPIOs that control GPS power, reset, and on/off.
//   2. Drive them to known states (power off, NRESET deasserted, ON low).
//   3. Bring up Serial2 at the chip's expected default baud.
//   4. Run a full cold reset, which also sets target baud and balloon mode.
//
// The GPS power pin is floating at boot? Likely GPS is off but unknown.
// Should turn GPS off before doing init if you know it's been initted already.
//
// Equations and schematic for the gate resistor on the mosfet:
//   https://electronics.stackexchange.com/questions/666204/gate-resistors-on-the-mosfet
//
// HEY! can't I control the source impedance of RP2040 GPIO outputs?
// What if I change the GPIO slew rate controls? I could use a 2MA output
// drive and a slow slew rate.
//   https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
//
// GpsPwr is the mosfet control GPIO for the GPS power control.
// A03401A is a p-channel mosfet:
//   https://www.aosmd.com/sites/default/files/res/datasheets/AO3401A.pdf
//
// Doug's inrush-current-limiting design (affecting gate resistance with
// the mosfet):
//   https://docs.google.com/document/d/1b1TdheBbXtl7U7HTO9mjVkThD9QHASqJirI2kSnv4LE/edit?tab=t.0#heading=h.lcp1i04bgm4q
//
// pico-sdk gpio API reference:
//   https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_gpio/include/hardware/gpio.h
// On the RP2040, setting both pulls enables a "bus keep" function, i.e. a
// weak pull to whatever the current high/low state of the GPIO is.
//   void gpio_set_pulls(uint gpio, bool up, bool down);
//   static inline void gpio_pull_down(uint gpio)    { gpio_set_pulls(gpio, false, true);  }
//   static inline void gpio_disable_pulls(uint gpio){ gpio_set_pulls(gpio, false, false); }
//
// hmm.. the weak pullup will change the slew rate down also?
// =============================================================================

// -----------------------------------------------------------------------------
// Configure the GpsPwr (mosfet gate) pin with low drive / slow slew so we
// limit inrush current when powering up the GPS.
// gpio_init defaults to 8mA drive strength; we override it below.
//
// Invariant: GPS_ON_PIN must be LOW and GpsPwr must be HIGH (power off) before
// asserting GpsPwr LOW (power on). Violation causes LNA latch-up on ATGM336H:
// the LNA draws excess current until power is cycled. Root cause: input voltage
// exceeds the VCC rail during the power ramp (classic latch-up).
//
// Scar: Magic sequence to destroy the LNA:
//   1. VCC off; 2. Assert GPS_ON_PIN HIGH; 3. Assert VCC on. -> Hot LNA.
//   Disabling GPS_ON_PIN afterwards does nothing; only power cycling fixes it.
// Source: https://www.eevblog.com/forum/rf-microwave/gps-lna-overheating-on-custom-pcb/
// Fix (2024-11): 100 kΩ gate resistor → 250 µs MOSFET turn-on, limits inrush.
//   Source: https://forum.arduino.cc/t/gps-power-management-reset-loop/529253/5
// -----------------------------------------------------------------------------
static void initGpsPwrPin(void) {
    gpio_init(GpsPwr);  // defaults to 8mA drive strength
    pinMode(GpsPwr, OUTPUT);
    // this will be undone by the next thing that uses gpio_set_pulls()
    gpio_pull_up(GpsPwr);
    gpio_put(GpsPwr, HIGH);  // deassert (power off)
    // if you set them to input after this, they will be high impedance!
    gpio_set_slew_rate(GpsPwr, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(GpsPwr, GPIO_DRIVE_STRENGTH_2MA);
}

// -----------------------------------------------------------------------------
// Configure GPS_NRESET_PIN as an output with a pull-up. Caller drives the
// initial level explicitly later.
// -----------------------------------------------------------------------------
static void initGpsResetPin(void) {
    gpio_init(GPS_NRESET_PIN);
    pinMode(GPS_NRESET_PIN, OUTPUT);
    gpio_pull_up(GPS_NRESET_PIN);
    // gpio_put(GPS_NRESET_PIN, HIGH); // deassert
}

// -----------------------------------------------------------------------------
// Configure GPS_ON_PIN as an output with a pull-down. Caller drives the
// initial level explicitly later.
// OPEN: toggling GPS_ON_PIN for low-power operation (instead of GpsPwr cycling)
// would preserve VBAT and allow hot fixes. Not yet implemented — requires
// verifying LNA latch-up invariant is still met with VBAT always present.
// -----------------------------------------------------------------------------
static void initGpsOnPin(void) {
    gpio_init(GPS_ON_PIN);
    pinMode(GPS_ON_PIN, OUTPUT);
    gpio_pull_down(GPS_ON_PIN);
    // gpio_put(GPS_ON_PIN, LOW); // deassert
}

// -----------------------------------------------------------------------------
// Drive the three GPS control pins to their known initial states and log it.
// Updated: Do a full reset since VBAT may have kept old settings -- don't
// know if that includes baud rate, maybe?
// -----------------------------------------------------------------------------
static void driveGpsPinsToInitialState(void) {
    digitalWrite(GpsPwr, HIGH);
    V1_printf("set GpsPwr %d HIGH (power off)" EOL, GpsPwr);

    digitalWrite(GPS_NRESET_PIN, HIGH);
    V1_printf("set GPS_NRESET_PIN %d HIGH" EOL, GPS_NRESET_PIN);

    digitalWrite(GPS_ON_PIN, LOW);
    V1_printf("set GPS_ON_PIN %d LOW" EOL, GPS_ON_PIN);
}

// -----------------------------------------------------------------------------
// Print which pins we're using for GPS UART and control.
// -----------------------------------------------------------------------------
static void printGpsPinAssignments(void) {
    V1_printf("GPS_UART1_RX_PIN %d" EOL, GPS_UART1_RX_PIN);
    V1_printf("GPS_UART1_TX_PIN %d" EOL, GPS_UART1_TX_PIN);
    V1_printf("GpsPwr %d" EOL, GpsPwr);
    V1_printf("GPS_NRESET_PIN %d" EOL, GPS_NRESET_PIN);
    V1_printf("GPS_ON_PIN %d" EOL, GPS_ON_PIN);
}

// -----------------------------------------------------------------------------
// Bring Serial2 up at the GPS chip's expected boot-time default baud.
// SIM65M defaults to 115200; ATGM (etc.) default to 9600.
//
// For ATGM, we'll first talk at 9600 -- but GpsFullColdReset() will do
// that anyway, so this is a bit redundant.
// -----------------------------------------------------------------------------
static void beginSerial2AtDefaultBaud(void) {
    Serial2.setPollingMode(true);
    // Tried making bigger...seems like 32 is the reality though.
    // Serial2.setFIFOSize(SERIAL2_FIFO_SIZE);
    Serial2.flush();
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);

    if (USE_SIM65M) {
        // default uart baud rate for SIM65
        Serial2.begin(115200);
    } else {
        Serial2.begin(9600);
    }
    gpsSleepForMillis(2000, false);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void GpsINIT(void) {
    V1_println(F(EOL "GpsINIT START"));
    updateStatusLED();
    Watchdog.reset();
    CONSTELLATIONS_GROUP = atoi(cc._const_group);
    // -------------------------------------------------------------------------
    // GPIO setup -- power, reset, on/off
    // -------------------------------------------------------------------------
    initGpsPwrPin();
    initGpsResetPin();
    initGpsOnPin();
    // Drive pins to known starting state (power off, reset deasserted, on=LOW).
    driveGpsPinsToInitialState();
    // -------------------------------------------------------------------------
    // UART setup
    // -------------------------------------------------------------------------
    printGpsPinAssignments();
    // OPEN: unclear whether RX pin being driven while GPS is unpowered causes issues.
    // No failures observed to date. If problems arise, try floating RX via pin mux.
    Serial2.setRX(GPS_UART1_RX_PIN);
    Serial2.setTX(GPS_UART1_TX_PIN);
    beginSerial2AtDefaultBaud();
    // -------------------------------------------------------------------------
    // Full cold reset: also sets baud to the target rate and applies
    // setGpsBalloonMode. After this returns, the GPS is powered up.
    //
    // OPEN: SIM65M baud persistence across cold reset is unclear. ATGM366N retains
    // the programmed baud across hot/cold resets (unusual). SIM65M may revert to
    // 115200. bringUpSerial2AndSetBaud() re-negotiates on each reset to handle both.
    // -------------------------------------------------------------------------
    GpsFullColdReset();
    gpsSleepForMillis(2000, false);
    V1_println(F("GpsINIT END" EOL));
}
// =============================================================================
// pwmGpsPwrOn
// -----------------------------------------------------------------------------
// Soft power-on for GpsPwr (active-low: drives a p-mosfet that gates GPS power).
//
// The point of this routine is to ramp the GPS rail up gradually rather than
// slamming the mosfet fully on, which keeps inrush current low.
//
// Note: vbat doesn't go through the mosfet, so vbat will be high as soon as
// power is available regardless of what we do here.
//
// Two strategies were tried; only the second is used in production.
// =============================================================================

// Strategy A (DISABLED -- doesn't work):
// The RP2040 internal pull-down (50-80 kohm) cannot overcome the 10k external
// pull-up on the PCB. The external pull-up wins; the rail never goes low.
static const bool WEAK_PULLDOWN_FOR_ASSERT = false;

// Set true to log duty_cycle at every 10% boundary.
// Off in production: USB PLL is disabled during cold-reset GPS power-up,
// so V1_printf calls would not reach the host terminal anyway.
static const bool PWM_PRINT_DUTY_CYCLE = false;

// -----------------------------------------------------------------------------
// Strategy A: weak-pulldown soft assert (does not work in this hardware,
// kept for reference). The external 10k pull-up on the PCB overrides the
// RP2040's internal pull-down.
// -----------------------------------------------------------------------------
static void gpsPwrOn_weakPulldown(void) {
    // Assumed current GpsPwr state on entry (the GPIO driving the mosfet
    // for the GPS chip power):
    //   output, driven with 1, with pullup (set by init) -> active deassert.
    pinMode(GpsPwr, INPUT);
    // Pulldown is 50 to 80 kohms on the rp2040 (so is pullup).
    gpio_pull_down(GpsPwr);  // this also disables the pullup
    sleep_ms(5000);          // 5 seconds; should have RC-ramped low by now.

    // Make sure we end with GpsPwr asserted using an active driver.
    pinMode(GpsPwr, OUTPUT);
    gpio_put(GpsPwr, LOW);
}

// -----------------------------------------------------------------------------
// Strategy B: bit-banged PWM that ramps from 0% duty to 100% duty over ~0.5s.
// LOW on the GPIO asserts the mosfet (turns GPS power on).
// PWM period = 5ms (granularity), 100 steps total = 500ms ramp.
// -----------------------------------------------------------------------------
// Bit-banged PWM constants for GPS power soft-start ramp.
// 5 ms period × 100 steps = 500 ms total; 50 µs per 1% duty increment.
// Limits inrush current through the A03401A p-channel mosfet gate.
static const uint64_t GPS_PWM_PERIOD_USECS = 5000UL;
static const uint64_t GPS_PWM_TOTAL_STEPS  = 100UL;
static const uint64_t GPS_PWM_STEP_USECS   = GPS_PWM_PERIOD_USECS / GPS_PWM_TOTAL_STEPS;

static void gpsPwrOn_pwmRamp(void) {
    // Start at 0% duty (mosfet fully off), ramp up to 100% (fully on).
    // Recall: LOW asserts the mosfet, so "on time" means GPIO driven LOW.
    for (uint64_t step = 0; step <= GPS_PWM_TOTAL_STEPS; step++) {
        Watchdog.reset();

        uint64_t on_usecs  = step * GPS_PWM_STEP_USECS;          // asserted (LOW) time
        uint64_t off_usecs = GPS_PWM_PERIOD_USECS - on_usecs;    // deasserted (HIGH) time

        // Drive one PWM period. Skip zero-length halves to avoid
        // unnecessary GPIO toggles at the 0% and 100% endpoints.
        if (on_usecs > 0) {
            digitalWrite(GpsPwr, LOW);    // assert mosfet (GPS power on)
            sleep_us(on_usecs);
        }
        if (off_usecs > 0) {
            digitalWrite(GpsPwr, HIGH);   // deassert mosfet
            sleep_us(off_usecs);
        }

        // duty_cycle as percent (step is already 0..100, so it IS the percent).
        uint64_t duty_cycle = step;
        if (PWM_PRINT_DUTY_CYCLE && (duty_cycle % 10) == 0) {
            V1_printf("pwmGpsPwrOn() duty_cycle (pct) %" PRIu64 EOL, duty_cycle);
        }
    }

    // Make sure we end with GpsPwr fully on (asserted).
    digitalWrite(GpsPwr, LOW);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void pwmGpsPwrOn(void) {
    if (WEAK_PULLDOWN_FOR_ASSERT) {
        gpsPwrOn_weakPulldown();
    } else {
        gpsPwrOn_pwmRamp();
    }
}
// -----------------------------------------------------------------------------
// =============================================================================
// GpsFullColdReset
// -----------------------------------------------------------------------------
// Performs a full cold reset of the GPS module:
//   1. Power down GPS, assert reset, drop Serial2.
//   2. Slow the RP2040 way down (and optionally drop core voltage / disable
//      USB PLL) to minimize our own power draw while the GPS rail is in its
//      high-inrush window.
//   3. Power GPS back on (optionally via PWM soft-start), deassert reset.
//   4. Sleep ~5 sec while the GPS comes up.
//   5. Restore RP2040 clocks/voltage.
//   6. Bring Serial2 up at the right baud, configure constellations,
//      balloon mode, PPS, broadcast filter, etc.
//   7. Read initial NMEA, return whether anything was seen.
//
// BUG: can't seem to reset the baud rate to 9600 when the GPS chip has a
// non-working baud rate.
//
// A full cold reset reverts to 9600 baud (as do standby modes -- don't use).
// =============================================================================

// -----------------------------------------------------------------------------
// Phase 1: Drive the GPS into a known powered-off, in-reset state.
//
// IDEA! since we KNOW the power demand will be high for 1 minute after poweron
// just go into light sleep to reduce rp2040 power demand for 1 minute --
// i.e. guarantee that cold reset takes 1 minute?
//
// Invariant: GPS_ON_PIN must stay LOW until after VCC is stable; see initGpsPwrPin().
// OPEN: experiment with powering on with NRESET held LOW until VCC is stable,
// then deassert NRESET — may reduce TXT broadcast at startup.
// -----------------------------------------------------------------------------
static void powerOffGpsAndAssertReset(void) {
    V1_println(F("Turn off the serial2..float at gps? no UART stuff at poweron?"));
    V1_flush();
    Serial2.end();

    // experiment idea: can we float the rx/tx pins of Serial2? maybe switch it
    // to other pins temporarily?
    // https://github.com/earlephilhower/arduino-pico/discussions/199

    V1_println(F("Doing Gps COLD POWER_ON (GPS_ON_PIN off with first power)"));

    digitalWrite(GPS_ON_PIN, LOW);       // deassert
    digitalWrite(GPS_NRESET_PIN, LOW);   // assert reset
    digitalWrite(GpsPwr, HIGH);          // deassert mosfet (power off)
    // 7/10/25
    // Serial2.end();
    // full 2 secs off?
    gpsSleepForMillis(2000, false);
}

// -----------------------------------------------------------------------------
// Phase 3a: Apply power to the GPS rail. Either soft-PWM ramp the mosfet
// (preferred, lower inrush) or just slam it on.
// -----------------------------------------------------------------------------
static void applyGpsPower(void) {
    // CHANGE: 7/10/25..don't pwm sim65m?
    // if (PWM_GPS_POWER_ON_MODE && !USE_SIM65M) {
    if (PWM_GPS_POWER_ON_MODE) {
        // this is probably at least 2 secs. let's measure
        uint32_t start_millis2 = millis();
        pwmGpsPwrOn();
        uint32_t duration_millis2 = millis() - start_millis2;
        V1_printf("Used pwmGpsPwrOn() and took %lu millisecs" EOL, duration_millis2);
        // soft power-on for GpsPwr (assert low, controls mosfet)
        // note that vbat doesn't have mosfet control, so it will be high right
        // away with availability of power
    } else {
        digitalWrite(GpsPwr, LOW);  // assert to mosfet
        gpsSleepForMillis(500, false);
    }
}

// -----------------------------------------------------------------------------
// Phase 3b: Deassert NRESET after power has come up.
// new 12/7/24 disable Serial2 while powering on!
// should we float the rx/tx also somehow?
// -----------------------------------------------------------------------------
static void deassertGpsReset(void) {
    gpsSleepForMillis(500, false);
    digitalWrite(GPS_NRESET_PIN, HIGH);  // deassert
    gpsSleepForMillis(1000, false);
    Watchdog.reset();
}

// -----------------------------------------------------------------------------
// Phase 2/4: Print which low-power modes are enabled, then drop RP2040 clocks
// (and optionally core voltage / USB PLL) before the GPS power-up window.
//
// IDEA! since we KNOW the power demand will be high for 1 minute after poweron
// just go into light sleep to reduce rp2040 power demand for 1 minute --
// i.e. guarantee that cold reset takes 1 minute?
// hmm. we're stalling things now. maybe only sleep for 15 secs.
// OPEN: slowing the clock makes Serial2 dysfunctional during the GPS bringup window —
// the GPS TXT broadcast (version strings) backs up and causes a power spike when
// the clock is restored. Net benefit unclear. Tested down to 50 MHz; not worth it
// at that level. Left in place for 18 MHz (kazu slow-clock) where savings are larger.
//
// UPDATE: is the broadcast right after power on the issue?
// other power saving: disable usb pll (and restore).
//
// https://github.com/earlephilhower/arduino-pico/discussions/1544
// We had to make sure we reset the watchdog now in gpsSleepForMillis;
// we already wakeup periodically to update led, so fine.
// -----------------------------------------------------------------------------
static void enterLowPowerForGpsBringup(void) {
    Watchdog.reset();
    if (!BALLOON_MODE) measureMyFreqs();

    V1_print(F("GPS power demand high during cold reset..try to minimize rp2040 power" EOL));

    if (ALLOW_LOWER_CORE_VOLTAGE_MODE) {
        V1_print(F("Also lowering core voltage to 0.95v" EOL));
    }
    if (ALLOW_USB_DISABLE_MODE) {
        V1_print(F("No keyboard interrupts will work because will disable USB PLL too" EOL));
    }
    if (ALLOW_KAZU_12MHZ_MODE) {
        V1_print(F("Will stay in 12Mhz using xosc, after things are restored" EOL));
    }
    if (ALLOW_TEMP_12MHZ_MODE) {
        V1_printf("Switch pll_sys PLL_SYS_MHZ %lu to xosc 12Mhz then sleep" EOL, PLL_SYS_MHZ);
    } else {
        V1_printf("Switch pll_sys PLL_SYS_MHZ %lu to pll 18Mhz then sleep" EOL, PLL_SYS_MHZ);
    }
    V1_flush();

    // The global IGNORE_KEYBOARD_CHARS guarantees core1 won't be interrupted
    // while we've messed with clocks during the GPS aggressive power-on control.
    // It should always be re-enabled after 15 secs.
    // Worst case to recover: unplug power and plug in again.
    // hmm core0 has to know to drain garbage chars if we assert this? then deassert?
    IGNORE_KEYBOARD_CHARS = true;

    // DRASTIC measures, do before sleep!
    // Wait another second after flush before shutting down serial —
    // the flush may return before the UART shift register is empty.
    // Sleep may be problematic in this transition.
    // Some more thoughts about low power rp2040 and clocks; was wondering
    // what measureMyFreqs() sees differing ring osc and rtc freqs:
    // https://forums.raspberrypi.com/viewtopic.php?t=342156
    busy_wait_ms(500);

    // remember not to touch Serial if in BALLOON_MODE!!
    if (!BALLOON_MODE) {
        Serial.flush();
        // Serial.end() removed 2026-03-08: caused abort during clock transition.
        busy_wait_ms(500);
    }
    if (!BALLOON_MODE && ALLOW_USB_DISABLE_MODE) {
        // maybe only end if in USB disable mode?
        Serial.end();
        busy_wait_ms(500);
    }

    Watchdog.reset();
    kazuClocksSlow();

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__pll.html
    // There are two PLLs in RP2040:
    //   pll_sys - up to a 133MHz (actually more) system clock
    //   pll_usb - 48MHz USB reference clock
    //
    // pll_deinit(pll) releases/uninitializes the specified PLL, turning off
    // its power. pll_deinit(pll_usb) does NOT check if the PLL is in use
    // before powering it off -- use care.
    //
    // examples: https://sourcevu.sysprogs.com/rp2040/picosdk/symbols/pll_deinit
    // sidenote: pi pico sdk has set_sys_clock_48mhz() which sets sys clk to
    // 48MHz with peripheral clock matched. Example:
    // https://sourcevu.sysprogs.com/rp2040/examples/clocks/hello_48MHz/files/hello_48MHz.c#tok293
    // 18 is the slowest legal I can go for the sys pll.
    //
    // voltage enums:
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h

}

// -----------------------------------------------------------------------------
// Phase 5: Restore RP2040 clocks back to normal after the GPS bringup window.
// -----------------------------------------------------------------------------
static void exitLowPowerAfterGpsBringup(uint32_t pll_sys_mhz_restore) {
    Watchdog.reset();
    busy_wait_ms(500);
    // OPEN: if ALLOW_KAZU_12MHZ_MODE leaves the clock at 12 MHz after restore,
    // PLL_SYS_MHZ will be wrong for PWM div/wrap calculations (which expect the
    // configured clock speed). Consider updating PLL_SYS_MHZ here if 12 MHz is kept.
    int currentGpsBaud = USE_SIM65M ? SIM65M_BAUD_RATE : ATGM336H_BAUD_RATE;
    kazuClocksRestore(pll_sys_mhz_restore, currentGpsBaud);
    // V1_print(F("Restored core voltage back to 1.1v" EOL));
    V1_flush();
    if (!BALLOON_MODE) measureMyFreqs();
    IGNORE_KEYBOARD_CHARS = false;
}

// -----------------------------------------------------------------------------
// Phase 6: Bring Serial2 up and step the GPS to the desired baud rate.
//
// hmm. we get a power surge here then? Is it because the Serial2 data was
// backed up in the gps chip and busy waiting? Drain it? (usually it's the
// TXT stuff (versions) at power on. Don't care.)
//
// We should be able to start talking to it. GPS should come up at 9600 so
// look with our uart at 9600.
//
// OPEN: if the chip is stuck at an unknown baud rate (e.g. 4800 from a prior session),
// beginSerial2AtDefaultBaud() will talk past it. Auto-detect would help here —
// see the OPEN note in setGpsBaud(). In practice, cold reset reverts to 9600.
// -----------------------------------------------------------------------------
static void bringUpSerial2AndSetBaud(void) {
    int BAUD_RATE = USE_SIM65M ? SIM65M_BAUD_RATE : ATGM336H_BAUD_RATE;
    int desiredBaud = checkGpsBaudRate(BAUD_RATE);

    // OPEN: SIM65M baud-at-boot is inconsistent — sometimes retains last programmed
    // baud, sometimes defaults to 115200. beginSerial2AtDefaultBaud() starts at the
    // chip-default, then setGpsBaud() re-programs the target rate.
    beginSerial2AtDefaultBaud();
    setGpsBaud(desiredBaud);

    if (USE_SIM65M) {
        // for old chips that are stuck at 9600 that we want to try faster
        Serial2.begin(9600);
        busy_wait_ms(500);
        setGpsBaud(desiredBaud);
    }

    gpsSleepForMillis(2000, false);
}

// -----------------------------------------------------------------------------
// Phase 6 (cont.): Optionally write the current "no broadcast / GPS only"
// config to GPS flash so subsequent cold resets boot in a calmer state.
//
// From CASIC_ProtocolSpecification_english.pdf page 24:
// CAS00 -- Save the current configuration information to FLASH. Even if the
// receiver is completely powered off, the FLASH contents persist.
//   Format:  $PCAS00*CS<CR><LF>
//   Example: $PCAS00*01<CR><LF>
//
// Could be dangerous, since it's writing a baud rate to the power-off/on
// reset config state -- could change it from 9600 and we'd lose track.
// As long as we stick with 9600 we should be safe.
//
// Will this help us boot in a better config so we don't get the power demand
// peaks we see (on subsequent boots)? Maybe we shouldn't do this all the time
// -- just once. Does the FLASH have a max # of writes issue (100k or ?)?
// We only do gps cold reset at start of day. Don't do it in BALLOON_MODE;
// that should fix the issue.
// -----------------------------------------------------------------------------
static void maybeWriteGpsConfigToFlash(void) {
    if (!ALLOW_UPDATE_GPS_FLASH_MODE || BALLOON_MODE) return;

    if (USE_SIM65M) {
        V1_print(F("SIM65M: Write GPS config"));
        V1_print(F(" (no broadcast, GNSS service disabled)" EOL));
        V1_print(F("SIM65M: (NO) still default constellations? (4)"));
        V1_print(F(" GPS/BDS/GLONASS/GALILEO" EOL));
        V1_print(F("SIM65M: to GPS Flash (for use in next GPS cold reset?)" EOL));
    } else {
        V1_print(F("ATGM336H: Write GPS current config"));
        V1_print(F(" (no broadcast, just GPS constellations" EOL));
        V1_print(F("ATGM336H: to GPS Flash (for use in next GPS cold reset?)" EOL));
    }
    // this will init to just GPS for the right then restore as below
    writeGpsConfigNoBroadcastToFlash();
    // restores to desired constellations and broadcast
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
bool GpsFullColdReset(void) {
    GpsIsOn_state = false;
    PPS_countDisable();

    V1_println(F(EOL "GpsFullColdReset START"));
    uint32_t start_millis = millis();
    Watchdog.reset();

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    // -------------------------------------------------------------------------
    // Phase 1: drive GPS off and into reset
    // -------------------------------------------------------------------------
    powerOffGpsAndAssertReset();

    // -------------------------------------------------------------------------
    // Cold-start command notes (for reference -- we rely on the power cycle
    // above rather than sending these):
    //
    //   $PMTK103*30                -- does this PMTK form work or not?
    //   $PCAS10,3*1F  factory      -- clear all data, reset rcvr
    //   $PCAS10,2*1E  cold start   -- no init info, clear all data except config
    //   $PCAS10,1*1D  warm start   -- no init info, all data valid; clear ephemeris
    //   $PCAS10,0*1C  hot start    -- no init info, all data valid
    //
    // Some say PCAS10 is just a boot-mode configuration (i.e. send before
    // power cycle), and that you have to toggle power off/on to get the
    // full-cold-start effect anyway.
    //
    // We do the full power cycle and don't wait for ack. We always commit
    // to driving Serial2.begin/end correctly rather than relying on
    // GpsIsOn() bookkeeping.
    //
    // OPEN: UART RX/TX pins are driven during GPS power-up. The GPS appears to
    // buffer the TXT broadcast and release it as a burst once powered — this may
    // cause a momentary power spike. Floating the pins until VCC is stable might
    // help; see earlephilhower arduino-pico/discussions/199 for pin-swap approach.
    // -------------------------------------------------------------------------

    // we still have usb pll on, and default clock frequency at this point

    // -------------------------------------------------------------------------
    // Phase 2: cut RP2040 clocks (and possibly voltage / USB PLL) for
    // the high-power GPS bringup window. Save current sys freq so we can
    // undo this temporary lower clock setting.
    // -------------------------------------------------------------------------
    uint32_t PLL_SYS_MHZ_restore = PLL_SYS_MHZ;
    enterLowPowerForGpsBringup();
    // -------------------------------------------------------------------------
    // Phase 3a: power GPS back on
    // -------------------------------------------------------------------------
    applyGpsPower();
    // -------------------------------------------------------------------------
    // Phase 3b: deassert NRESET (okay in both normal and experimental case)
    // -------------------------------------------------------------------------
    deassertGpsReset();
    // Finally turn on the GPS here (if we didn't already above in
    // experimental mode).
    digitalWrite(GPS_ON_PIN, HIGH);  // assert

    // OPEN: intermittent hang observed at 60 MHz during this sleep (2025-xx).
    // No printing here (USB PLL off). May be a clock-domain issue with kazuClocksSlow.
    gpsSleepForMillis(5000, false);  // 5 secs
    // -------------------------------------------------------------------------
    // Phase 4: restore RP2040 clocks
    // -------------------------------------------------------------------------
    exitLowPowerAfterGpsBringup(PLL_SYS_MHZ_restore);
    Watchdog.reset();
    // -------------------------------------------------------------------------
    // Phase 5: Serial2 up, set baud, configure modes
    // -------------------------------------------------------------------------
    bringUpSerial2AndSetBaud();

    if (USE_SIM65M) setGpsBalloonMode();
    if (USE_SIM65M && !BALLOON_MODE) setGpsPPSMode();

    maybeWriteGpsConfigToFlash();

    setGpsConstellations(CONSTELLATIONS_GROUP);
    // no ANT TXT (NMEA sentences) after this:
    setGpsBroadcast();

    // I guess it doesn't power on with location service on
    if (USE_SIM65M) setGnssOn_SIM65M();

    // -------------------------------------------------------------------------
    // Phase 6: read initial NMEA, decide whether the GPS is alive
    // -------------------------------------------------------------------------
    bool sentencesFound = getInitialGpsOutput();

    // flush out any old state in TinyGPSplus, so we don't get a valid fix
    // that's got a big fix_age
    invalidateTinyGpsState();
    GpsStartTime = get_absolute_time();  // usecs

    if (sentencesFound) {
        GpsIsOn_state = true;
        PPS_countEnable(true);  // reset
    }

    uint32_t duration_millis = millis() - start_millis;
    V1_print(F("GpsFullColdReset END"));
    V1_printf(" sentencesFound %u", sentencesFound);
    V1_printf(" duration_millis %lu" EOL, duration_millis);
    return sentencesFound;
}

// =============================================================================
// GpsHotReset
// -----------------------------------------------------------------------------
// Performs a hot reset of the GPS module. Mirrors the structure of
// GpsFullColdReset but is faster and skips the RP2040 low-power dance.
//
//   Hot start:  No initialization information is used and all the data is valid.
//   Warm start: Do not use initialization information and clear ephemeris.
//
// Note: the software-command fallback in sendSoftwareResetCommand() sends a
// WARM start (clears ephemeris) when a hot reset fails, not a true hot start.
// This is intentional — if pin-driven hot reset failed, stale ephemeris is
// likely the cause, and clearing it improves the chance of a fresh fix.
//
// OPEN: SIM65M spec says a power cycle resets settings to factory defaults and
// forces a cold start on next power-up. If true, a hot reset via power-cycle is
// actually a cold start for SIM65M. Switching to GNSS idle mode (PAIR003/PAIR002)
// instead of power-cycling would preserve ephemeris and config — worth testing.
//
// Hot reset doesn't change the baud rate from the prior config -- but what
// if we lost VBAT and config isn't preserved?
// =============================================================================

// -----------------------------------------------------------------------------
// Power-cycle the GPS chip without asserting reset (that's the "hot" part).
// reorganized to match GpsFullColdReset()
//
// Invariant: GPS_ON_PIN must be LOW before VCC is asserted — see initGpsPwrPin().
// OPEN: consider holding NRESET_PIN LOW until VCC is stable before deasserting,
// to further guard against LNA latch-up during the voltage ramp.
// -----------------------------------------------------------------------------
static void hotResetPowerCycle(void) {
    V1_println(F("Doing Gps HOT POWER_ON (GPS_ON_PIN off with power off-on)"));
    // Off (don't assert reset during power off)
    digitalWrite(GPS_ON_PIN, LOW);
    digitalWrite(GPS_NRESET_PIN, HIGH);
    digitalWrite(GpsPwr, HIGH);
    Serial2.end();
    gpsSleepForMillis(1000, false);  // no early out

    // On -- reuse the same timed soft-start used for cold reset.
    // Extracted from three copies (GpsFullColdReset/applyGpsPower, hotResetPowerCycle).
    applyGpsPower();

    gpsSleepForMillis(1000, false);  // no early out

    // now assert the on/off pin
    digitalWrite(GPS_ON_PIN, HIGH);
    gpsSleepForMillis(1000, false);  // no early out
}

// -----------------------------------------------------------------------------
// Bring Serial2 up at the chip's power-on baud and step it to the desired baud.
// Should come up in the last programmed baud rate (from cold reset), but
// reality has been less reliable than that -- so we always re-set baud here.
// -----------------------------------------------------------------------------
static void hotResetBringUpSerial2(void) {
    int BAUD_RATE = USE_SIM65M ? SIM65M_BAUD_RATE : ATGM336H_BAUD_RATE;
    int desiredBaud = checkGpsBaudRate(BAUD_RATE);

    if (USE_SIM65M) {
        // Hmm. did we have failures where it didn't come up in the right baud rate?
        // it either comes up in desiredBaud from some memory (9600?),
        // or comes up in 115200?
        Serial2.begin(115200);
        gpsSleepForMillis(500, false);  // no early out
        setGpsBaud(desiredBaud);
    } else {
        // it either comes up in desiredBaud from some memory, or comes up in 9600?
        // Used to not set ATGM baud rate! now we do..above
        Serial2.begin(9600);
        gpsSleepForMillis(500, false);  // no early out
        setGpsBaud(desiredBaud);
    }
    gpsSleepForMillis(500, false);  // no early out
}

// -----------------------------------------------------------------------------
// Reapply config (constellations, broadcast filter, balloon mode, PPS).
// Optional -- skipped unless HOT_RESET_REDO_CONFIG is set.
//
// Always reconfig balloon mode -- there were known issues with ublox losing
// this, and it's hard to realize if we lost it unless we read the mode.
// Don't reconfig PPS in BALLOON_MODE; it's unnecessary because no prints.
// -----------------------------------------------------------------------------
static void hotResetReapplyConfig(void) {
    if (!HOT_RESET_REDO_CONFIG) return;

    setGpsConstellations(CONSTELLATIONS_GROUP);
    // we don't need no ZDA/TXT
    setGpsBroadcast();

    if (USE_SIM65M) setGpsBalloonMode();
    if (USE_SIM65M && !BALLOON_MODE) setGpsPPSMode();
}

// -----------------------------------------------------------------------------
// SIM65M: read back the navigation mode (sanity check across hot reset) and
// make sure GNSS is enabled. We could change the default config to power up
// with GNSS off, so we always re-enable it here.
// -----------------------------------------------------------------------------
static void hotResetSIM65MPostConfig(void) {
    if (!USE_SIM65M) return;

    // Always read it to make sure it's right thru hot reset.
    V1_println(F("Read the navigation mode: $PAIR081*33"));
    // Packet Type:081 PAIR_COMMON_GET_NAVIGATION_MODE
    Serial2.print("$PAIR081*33" CR LF);
    // 2/16/2025 try faster for faster gps hot reset
    nmeaBufferFastPoll(1500, true);  // duration_millis, printIfFull

    setGnssOn_SIM65M();
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
bool GpsHotReset(void) {
    GpsIsOn_state = false;
    PPS_countDisable();

    V1_println(F(EOL "GpsHotReset START"));
    uint32_t start_millis = millis();

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();
    // turn off the serial
    V1_flush();
    // -------------------------------------------------------------------------
    // Power-cycle the GPS (no reset assertion)
    // -------------------------------------------------------------------------
    hotResetPowerCycle();
    // -------------------------------------------------------------------------
    // Serial2 + baud
    // -------------------------------------------------------------------------
    hotResetBringUpSerial2();
    // -------------------------------------------------------------------------
    // Optional reconfig
    // -------------------------------------------------------------------------
    hotResetReapplyConfig();
    hotResetSIM65MPostConfig();
    // -------------------------------------------------------------------------
    // Read initial NMEA, decide whether the GPS is alive
    // -------------------------------------------------------------------------
    bool sentencesFound = getInitialGpsOutput();

    // flush out any old state in TinyGPSplus, so we don't get a valid fix
    // that's got a big fix_age
    invalidateTinyGpsState();

    uint32_t duration_millis = millis() - start_millis;
    V1_print(F("GpsHotReset END"));
    V1_printf(" sentencesFound %u", sentencesFound);
    V1_printf(" duration_millis %lu" EOL, duration_millis);

    if (sentencesFound) {
        GpsIsOn_state = true;
        PPS_countEnable(true);  // reset
    }
    GpsStartTime = get_absolute_time();  // usecs
    return sentencesFound;
}
// =============================================================================
// writeGpsConfigNoBroadcastToFlash
// -----------------------------------------------------------------------------
// Persist a "quiet" GPS configuration to the chip's flash so that on the
// next cold reset the receiver wakes up with broadcast disabled and only
// GPS constellation enabled. This reduces the power demand spike right
// after power-on.
//
// Sequence:
//   1. Stage the desired persistent config in volatile state:
//        - disable broadcast
//        - reduce constellations to GPS only (1)
//        - SIM65M: turn GNSS subsystem OFF (so the SAVE command is allowed
//          in multi-Hz mode -- spec only permits saving when GNSS is off)
//   2. Send the chip-specific "save current config to flash" NMEA command.
//   3. Restore working runtime state:
//        - SIM65M: turn GNSS back ON
//        - restore configured constellations
//        - re-enable broadcast
//
// Risk note (from original): do we ever power on and not do this full cold
// reset that sets up broadcast? The hot GPS reset shouldn't pull new state
// from config. So changing the FLASH to "no broadcast" should be safe --
// cold reset power-on will come up quiet and we set broadcast back during
// the rest of the cold reset flow.
//
// Reference (SIM65M PAIR commands):
//
//   $PAIR002*38  PAIR_GNSS_SUBSYS_POWER_ON
//     Power on the GNSS system. Include DSP/RF/Clock and other GNSS
//     modules. Please send this command before using any location service.
//
//   $PAIR003*39  PAIR_GNSS_SUBSYS_POWER_OFF
//     Power off GNSS system. Include DSP/RF/Clock and other GNSS modules.
//     CM4 can still receive commands after this (AT, race, PAIR commands
//     not dependent on DSP). Location service is unavailable. The system
//     can still receive configuration PAIR commands.
//
//   $PAIR513*3D  PAIR_NVRAM_SAVE_SETTING
//     Save the current configuration from RTC RAM to flash.
//     In multi-Hz, this can only be set when GNSS is powered off; 1Hz has
//     no such limit. (We power GNSS off above, so OK either way.)
//
//   $PAIR514*3A  PAIR_NVRAM_RESTORE_DEFAULT_SETTING (in case we mess up flash)
//     Clear current config and restore default settings. Does not support
//     run-time restore when GNSS is on -- send PAIR_GNSS_SUBSYS_POWER_OFF
//     first.
//
// OPEN: SIM65M default constellation config on flash save not yet implemented.
// OPEN: constellation=0 (no satellites) at cold-start might further cut peak power.
// =============================================================================

// -----------------------------------------------------------------------------
// Stage the "quiet" config we want to persist into flash.
// -----------------------------------------------------------------------------
static void stageQuietConfigForFlash(void) {
    disableGpsBroadcast();
    setGpsConstellations(1);  // GPS only
    // SIM65M only: turn GNSS subsystem off so the SAVE command is permitted
    // in multi-Hz mode. Also: powering up with GNSS off means a future cold
    // reset boots into a lower-power state.
    if (USE_SIM65M) setGnssOff_SIM65M();
}

// -----------------------------------------------------------------------------
// Send the chip-specific "save current config to flash" NMEA command.
// -----------------------------------------------------------------------------
static void sendSaveConfigToFlashCommand(void) {
    char nmeaSentence[64] = { 0 };
    if (USE_SIM65M) {
        strncpy(nmeaSentence, "$PAIR513*3D" CR LF, 64);
    } else {
        strncpy(nmeaSentence, "$PCAS00*01" CR LF, 64);
    }
    V1_printf("%s" EOL, nmeaSentence);
    Serial2.print(nmeaSentence);
    Serial2.flush();
    sleep_ms(1000);
}

// -----------------------------------------------------------------------------
// Restore working runtime state (GNSS back on, configured constellations,
// broadcast re-enabled).
// -----------------------------------------------------------------------------
static void restoreRuntimeConfig(void) {
    // we could change the default config to power up with GNSS off?
    if (USE_SIM65M) setGnssOn_SIM65M();
    setGpsConstellations(CONSTELLATIONS_GROUP);
    setGpsBroadcast();
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void writeGpsConfigNoBroadcastToFlash(void) {
    V1_println(F("writeGpsConfigNoBroadcastToFlash() START"));
    stageQuietConfigForFlash();
    sendSaveConfigToFlashCommand();
    restoreRuntimeConfig();
    V1_println(F("writeGpsConfigNoBroadcastToFlash END"));
}

// =============================================================================
// GpsON
// -----------------------------------------------------------------------------
// Bring the GPS up. Tries up to 3 times. If `GpsColdReset` is true, uses
// GpsFullColdReset(); otherwise uses GpsHotReset() and falls back to cold
// reset after 3 hot-reset failures.
//
// On each failed attempt (other than the last), also pokes the GPS chip
// directly with the software cold/warm-start NMEA command, in case the pin
// assertions in {Hot,FullCold}Reset didn't take effect.
//
// Assumes GpsINIT was already done (pins etc). The chip could be off or on
// already on entry.
// =============================================================================

// Attempts before escalating hot→cold or giving up on cold reset entirely.
static const uint32_t GPS_RESET_MAX_TRIES = 3;

// -----------------------------------------------------------------------------
// Logging helper -- match the original behavior:
// no print of `GpsColdReset` if it's not set, so you can grep for
// "GpsColdReset" as a special case only.
// -----------------------------------------------------------------------------
static void logGpsOnBoundary(const char *boundary,  // "START" or "END"
                             bool gpsColdReset,
                             const char *trailing) {  // "" or " <EOL>"
    if (!gpsColdReset) {
        V1_printf("GpsON %s GpsIsOn_state %u%s" EOL,
                  boundary, GpsIsOn_state, trailing);
    } else {
        V1_printf("GpsON %s GpsIsOn_state %u GpsColdReset %u%s" EOL,
                  boundary, GpsIsOn_state, gpsColdReset, trailing);
    }
}

// -----------------------------------------------------------------------------
// Send the software-command fallback to nudge the GPS into a cold/warm start
// when pin-driven reset didn't produce any NMEA sentences. Depends on the
// baud setup being right.
//
// Warning: the fallback command for a failed hot reset is a WARM start,
// not a hot start. This clears ephemeris. Intentional — if pin-driven hot
// reset produced no sentences, stale ephemeris is likely the cause.
// -----------------------------------------------------------------------------
static void sendSoftwareResetCommand(bool gpsColdReset) {
    if (USE_SIM65M) {
        if (gpsColdReset) {
            // Packet Type:007 PAIR_GNSS_SUBSYS_FULL_COLD_START
            V1_print(F("ERROR: no sentencesFound, send command for PAIR_GNS_SUBSYS_FULL_COLD_START"));
            Serial2.print("$PAIR001,007,0*3C" CR LF);
        } else {
            // Packet Type:005 PAIR_GNSS_SUBSYS_WARM_START
            // this will clear ephemeris, unlike normal hot starts
            V1_print(F("ERROR: no sentencesFound, send command for PAIR_GNS_SUBSYS_WARM_START (clear ephemeris)"));
            Serial2.print("$PAIR001,005,0*3E" CR LF);
        }
    } else {
        if (gpsColdReset) {
            // PCAS10 factory start
            V1_print(F("ERROR: no sentencesFound, send command PCAS10 factory start"));
            Serial2.print("$PCAS10,3*1F" CR LF);
        } else {
            // PCAS10 warm start.
            // This will clear ephemeris, unlike normal hot starts.
            V1_print(F("ERROR: no sentencesFound, send command PCAS10 warm start (clear ephemeris)"));
            Serial2.print("$PCAS10,1*1D" CR LF);
            Serial2.print("" CR LF);
        }
    }
    Serial2.flush();
    busy_wait_ms(1000);
}

// -----------------------------------------------------------------------------
// Run one reset attempt (cold or hot) and return whether NMEA was seen.
// -----------------------------------------------------------------------------
static bool tryOneGpsReset(bool gpsColdReset) {
    return gpsColdReset ? GpsFullColdReset() : GpsHotReset();
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void GpsON(bool GpsColdReset) {
    logGpsOnBoundary("START", GpsColdReset, "");
    Watchdog.reset();

    // Fast path: if we weren't asked for a cold reset and the GPS is already
    // on, fake `sentencesFound = true` so we skip the hot-reset attempt.
    // (For cold reset we don't care what the initial state is.)
    bool sentencesFound = false;
    if (!GpsColdReset && GpsIsOn()) {
        V1_println(F("do nothing because GpsIsOn()"));
        sentencesFound = true;
    }

    // -------------------------------------------------------------------------
    // Retry loop: up to GPS_RESET_MAX_TRIES attempts. If hot resets keep
    // failing, escalate to cold reset and reset the try counter.
    // -------------------------------------------------------------------------
    uint32_t tryCnt = 0;
    while (!sentencesFound) {
        tryCnt += 1;

        // Hit the retry ceiling -- either give up (cold) or escalate (hot).
        if (tryCnt >= GPS_RESET_MAX_TRIES) {
            if (GpsColdReset) {
                V1_print(F("ERROR: tryCnt 3 on GpsFullColdReset.. not retrying any more "));
                printSystemDateTime();
                V1_print(F(EOL));
                break;
            } else {
                V1_print(F("ERROR: tryCnt 3 on GpsHotReset.. switch to trying GpsColdReset "));
                printSystemDateTime();
                V1_print(F(EOL));
                GpsColdReset = true;
                tryCnt = 0;
            }
        }

        sentencesFound = tryOneGpsReset(GpsColdReset);

        // Software-command fallback: if the pin-driven reset produced
        // nothing and we still have retries left, poke the chip via NMEA.
        if (!sentencesFound && (tryCnt < GPS_RESET_MAX_TRIES)) {
            sendSoftwareResetCommand(GpsColdReset);
        }
    }

    logGpsOnBoundary("END", GpsColdReset, EOL);
    V1_flush();
}

// -----------------------------------------------------------------------------
/*
This used to be in the LightAPRS version of TinyGPSPlus-0.95
instead updated TinyGPSPlus (latest) in libraries to make them public,
not private
< #if defined(ARDUINO_ARCH_RP2040)
< void TinyGPSDate::clear()
< {
<    valid = updated = false;
<    date = 0;
< }b
< #endif
*/

// -----------------------------------------------------------------------------
void invalidateTinyGpsState(void) {
    V1_println(F("invalidateTinyGpsState() START"));
    // TinyGPS++ originally had private flush methods. We added public flush()
    // and fixQualityFlush()/fixModeFlush() to the local library copy.
    // The GpsInvalidAllCnt approach (counting down broadcasts to ignore) was
    // considered but not needed once the flush methods were available.
    // Disabled path kept for reference:
    if (false) {
        GpsInvalidAllCnt = 2;
        GpsInvalidAll = true;
    }

    //***********************************
    // how do we clear a fix from TinyGPS++ ?
    // do we wait until we turn GpsON() on, beforre clearing GPSInvalidAll
    // we can decrement the count only if gps is on? (in setup1())
    // new public methods created in TinyGPS++.h

    // 4/26/25 hmm. altitude and speed doesn't get flushed.
    // maybe we should flush those too, in case a valid 0 gets stuck in there?
    // we don't flush sat count stuff?
    gps.location.flush();
    gps.location.fixQualityFlush();

    gps.speed.flush();
    gps.altitude.flush();

    gps.fix.fixModeFlush();
    gps.date.flush();
    gps.time.flush();
    V1_println(F("invalidateTinyGpsState END"));
}

// -----------------------------------------------------------------------------
void GpsOFF() {
    GpsIsOn_state = false;
    GpsStartTime = 0;
    PPS_countDisable();

    V1_printf("GpsOFF START GpsIsOn_state %u" EOL, GpsIsOn_state);
    digitalWrite(GpsPwr, HIGH);
    // Serial2.end() frees the UART; tx pin goes low (idle high after begin()).
    // Required: without it, the GPS chip sees a driven TX line during power-off
    // which can cause spurious wakeup on chips that monitor the RX line.
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);
    // unlike i2c to vfo, we don't tear down the Serial2 definition...just .end()
    // so we can just .begin() again later
    // have to flush everything.
    // Can't keep enqueued time. not worth saving altitude/lat/lon..we have to
    // wait for time to get set again?
    invalidateTinyGpsState();

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    V1_flush();
    V1_printf("GpsOFF END GpsIsOn_state %u" EOL, GpsIsOn_state);
}

// -----------------------------------------------------------------------------
uint32_t updateGpsDataAndTime(int ms) {
    // to make sure we get some update, even if fix_age is larger than 1 sec.
    V1_println(F("updateGpsDataAndTime START"));
    Watchdog.reset();

    // ms has to be positive?
    // grab data for no more than ms milliseconds
    // stop if no data for 50 milliseconds
    // all the durations below won't start counting until we get the first char
    // (sets start_millis())
    const uint32_t entry_millis = millis();
    uint32_t start_millis       = 0;
    uint32_t last_char_millis   = 0;
    int incomingCharCnt         = 0;

    // inc on '$'
    // static: keep the count since the first time we start doing this function
    // can visually compare these in the stdout...ideally should always be the same
    static int sentenceStartCnt = 0;
    // inc on '*' (comes before the checksum)
    static int sentenceEndCnt = 0;
    static uint32_t last_gcp, last_gswf, last_gfc;

    V1_printf(
        "updateGpsDataAndTime started looking for NMEA current_millis %lu" EOL,
        millis());

    // clear the StampPrintf buffer, in case it had anything.
    if (VERBY[1]) DoLogPrint();

    GpsON(false);

    // Drain any stale bytes already in the UART FIFO before listening.
    // 7/10/25: threshold changed from 30 to 1 to give more headroom for
    // SIM65M-CB parts running at 115200 baud.
    getChar();
    if (s_charsAvailable >= 1) {
        if (VERBY[1])
            StampPrintf("INFO: initially drained NMEA chars because rx has stuff. uart rx initially %d" EOL,
                (int)s_charsAvailable);
        Watchdog.reset();
        // should be at most 31 to drain
        for (int n = s_charsAvailable; n > 0; n--) getChar();
    }

    // don't start sending to TinyGPS until we get $|CR|LF so we know we're aligned
    // can't do in another loop, because of delays getting chars. Have
    // to have it in this main timeout loop
    bool aligned = false;

    // the time of the last $..for setting system time to the secs in the nmea sentence
    // with less variation (rather than time at the end of the checksum)
    uint32_t dollar_millis          = 0;
    // timestamp of the '$' of the GGA sentence, frozen at '*' to prevent
    // overwrite by the next sentence's '$' before TinyGPS++ commits.
    uint32_t sentence_dollar_millis      = 0;
    uint32_t time_sentence_dollar_millis = 0;
    // only one that causes gps.time.updated
    bool     doDelayedTimeUpdate    = false;

    // Fix D: PPS snapshot taken at the '*' character of each sentence — the
    // earliest point at which the sentence body is complete and TinyGPS++ is
    // about to commit. The ISR can still fire between '*' and the commit (CR/LF),
    // but this window is ~4 ms vs the ~400 ms window that existed before Fix D.
    // Used directly as the TimeLib anchor when PPS is valid (see phase 4 below).
    uint32_t snap_PPS_rise_millis = 0;
    bool     snap_PPS_rise_valid  = false;
    // uint32_t timeUpdate_sentences = 0;
    // bool     timeUpdateDone       = false;
    gps.time.updated = false;
    gps.date.updated = false;

    // replaces multi-level break: set when we have enough time updates
    bool     finished             = false;

    while (!finished && (millis() - entry_millis) < (uint64_t)ms) {
        while (!finished && s_charsAvailable > 0) {
            uint32_t now = millis();
            // start the duration timing when we get the first char
            if (start_millis == 0) start_millis = now;
            last_char_millis = now;
            // we count all chars, even CR LF etc
            incomingCharCnt++;
            // UART FIFO is 32 bytes; depth >= 31 means we are falling behind.
            if (VERBY[1] && s_charsAvailable >= 31)
                StampPrintf("ERROR: full. uart rx depth %d incomingCharCnt %d" EOL,
                    (int)s_charsAvailable, incomingCharCnt);
            char c       = s_incomingChar;
            bool isCrlf  = (c == '\r' || c == '\n');
            bool isPrint = !isCrlf && isprint(c);
            // always strip non-printable, non-CRLF chars here and continue the loop
            // CR and LF isprint() is false
            if (!isPrint && !isCrlf) {
                getChar();
                continue;
            }
            // Update sentence-alignment and timing state.
            // crlf falls through to gps.encode() below but isPrint stays false,
            // so it won't be written to the nmea buffer for printing.
            // TinyGPS++ expects the CR LF between sentences.
            switch (c) {
                case '$':
                    aligned = true;
                    sentenceStartCnt++;
                    dollar_millis = now;
                    break;
                case '\r': case '\n':
                    aligned = true;
                    break;
                case '*':
                    sentenceEndCnt++;
                    // clear time updated state, right before any TinyGPS term/commit event
                    // always need checksum before a commit event
                    gps.time.updated = false;
                    // Freeze the '$' timestamp now to prevent the next '$' from
                    // overwriting dollar_millis before gps.time.updated fires.
                    sentence_dollar_millis = dollar_millis;
                    // Fix D (tightened): snapshot PPS state here at '*', not at
                    // gps.time.updated. TinyGPS++ commits on CR/LF, ~4 ms after '*'
                    // at 9600 baud. Snapshotting at '*' eliminates that residual
                    // window in which the ISR could overwrite PPS_rise_millis.
                    // Single-core: no torn-write risk; ISR preempts between C
                    // statements only. This snapshot is unconditional — we only use
                    // it if gps.time.updated fires after gps.encode() below.
                    snap_PPS_rise_millis = PPS_rise_millis;
                    snap_PPS_rise_valid  = PPS_rise_valid;
                    break;
                default: break;
            }

            if (!aligned) { 
                getChar();  
                continue; 
            }

            // unload each char to TinyGPS++ object as it arrives
            // https://arduino.stackexchange.com/questions/13452/tinygps-plus-library
            gps.encode(c);

            // just save the first millis for the GGA sentence with time
            // updated has to transition before we get the next dollar_millis??
            // we use sentence_dollar_millis to make sure no race condition with '$'
            // RMC should be last sentence in the burst? has date.
            // GGA is first in the burst. we use that for time.
            if (gps.time.updated && !doDelayedTimeUpdate) {
                // save the earliest millis from the time update.
                // snap_PPS_rise_millis / snap_PPS_rise_valid were already captured
                // at the '*' case above — the closest point before TinyGPS++ commits.
                doDelayedTimeUpdate          = true;
                time_sentence_dollar_millis  = sentence_dollar_millis;
            }

            // Note we disabled the GPTXT broadcast to reduce the NMEA load (for here)
            // Do we get any unprintable? ignore unprintable chars, just in case.
            if (VERBY[1] && isPrint) nmeaBufferAndPrint(c, false);  // no print if full
            getChar();
        }

        // Trying to synchronize so GGA is always first
        // Should only be one time update trigger now
        // If we get two, we've gone too long on the burst
        // if (timeUpdate_sentences >= 2) finished = true;

        if (finished) break;

        // Break when no char has arrived for GPS_BURST_GAP_MS.
        //
        // The ATGM336H at 9600 baud inserts inter-sentence gaps of ~10-40 ms
        // within a burst (observed between the GSA and GSV sentence groups).
        // The inter-burst silence is ~4500 ms.
        //
        // The gap threshold is NOT the right lever for preventing GSV data loss.
        // The loss occurs at call ENTRY (FIFO overflow during ~500 ms inter-call
        // overhead), not at the gap detector. Increasing this value makes the
        // short call longer, which increases the inter-call overhead for the
        // subsequent call — making the overflow slightly worse, not better.
        //
        // The correct fixes are:
        //   A) Disable GSV in PCAS03 (burst shrinks to ~200 chars, fits cleanly
        //      within one call, no overflow at entry).
        //   B) Increase baud rate to 115200 (burst transmits in ~51 ms, finishes
        //      before the next call starts — FIFO holds only the tail).
        //   19200 baud is insufficient: burst still takes ~300 ms, the overflow
        //   window shrinks by half but the problem persists.
        //
        // 10 ms is kept as a clean exit threshold — enough to detect the end of
        // the burst without adding unnecessary latency between calls.
        static const uint32_t GPS_BURST_GAP_MS = 10;
        uint32_t gapMs = last_char_millis ? (millis() - last_char_millis) : 0;
        if (gapMs >= GPS_BURST_GAP_MS) break;

        gpsSleepForMillis(10, true);   // enableEarlyOut: returns early if char arrives
        getChar();
    }
    // maybe save some time in loop above..don't update led during 1 sec burst?
    updateStatusLED();

    // how long does a burst take? 1 sec? this could be done one sec late relative to gps time
    // gps.date.updated happens in a sentence after the first GGA that sets gps.time.updated
    // gps.time.updated would have been cleared...just need to know we got gps.date.updated also
    // if (doDelayedTimeUpdate && gps.date.updated) {
    if (doDelayedTimeUpdate) {
        // Guard: time_sentence_dollar_millis must be > 0 and within 2000 ms of now.
        // A value of 0 means the '*' for the GGA sentence was never captured — should
        // not happen in normal flow, but if it does, snap_PPS_rise_millis would also
        // be 0 and would be used as the anchor, setting the clock to boot time.
        uint32_t ts_age_ms = millis() - time_sentence_dollar_millis;
        if (time_sentence_dollar_millis == 0 || ts_age_ms > 2000) {
            V1_printf("ERROR: time_sentence_dollar_millis %lu implausible (age %lu ms),"
                      " skipping time update" EOL,
                      time_sentence_dollar_millis, ts_age_ms);
        } else {
            checkUpdateTimeFromGps(time_sentence_dollar_millis,
                                   snap_PPS_rise_millis, snap_PPS_rise_valid);
        }
    }
    
    // Reporting
    uint32_t duration_millis = start_millis ? (millis() - start_millis) : 0;
    if (VERBY[1]) {
        // print/clear any accumulated NMEA sentence stuff
        nmeaBufferPrintAndClear();
        V1_print(F(EOL));
        DoLogPrint();  // dump the StampPrintf if any
    }

    // This will be lower than a peak rate.
    // It includes dead time at start, dead time at end...
    // With some constant rate in the middle? but sentences could be split..
    // start_millis is the first char. so duration_millis will
    // include the end stall detect (25 millis).
    // So it's an average over that period.
    float avgCharRateSec = (duration_millis == 0)
        ? 0.0f
        : 1000.0f * ((float)incomingCharCnt / (float)duration_millis);
    // can it get too big?
    if (avgCharRateSec > 999999.9f) avgCharRateSec = 999999.9f;

    // the time for chars to arrive should never be more than 1 sec?
    // too much stuff in one burst?
    if (duration_millis > 1000)
        V1_printf("ERROR: NMEA sentences duration_millis %lu > 1000 milliseconds" EOL,
            duration_millis);

    V1_printf(
        "NMEA sentences: AvgCharRateSec %.f duration_millis %lu incomingCharCnt %d" EOL,
        avgCharRateSec, duration_millis, incomingCharCnt);

    // checksum errors at TinyGPS?
    uint32_t gcp  = gps.charsProcessed();
    uint32_t gswf = gps.sentencesWithFix();
    uint32_t gfc  = gps.failedChecksum();

    V1_printf("TinyGPS       charsProcessed %10lu sentencesWithFix %5lu failedChecksum %5lu" EOL,
        gcp, gswf, gfc);
    // can visually compare to prior sentences received and printed and see if tinygps
    // is reporting checksum errors that shouldn't exist?
    V1_printf("TinyGPS delta charsProcessed %10lu sentencesWithFix %5lu failedChecksum %5lu" EOL,
        gcp - last_gcp, gswf - last_gswf, gfc - last_gfc);
    last_gcp = gcp; last_gswf = gswf; last_gfc = gfc;

    updateStatusLED();
    uint32_t total_millis = millis() - entry_millis;
    // will be interesting to compare total_millis to duration_millis
    V1_printf("updateGpsDataAndTime END total_millis %lu" EOL EOL, total_millis);
    return total_millis;
}

// =============================================================================
// checkUpdateTimeFromGps
// -----------------------------------------------------------------------------
// Periodically resync the system clock to the GPS time, applying a PPS-based
// skew correction so the resulting system time aligns as closely as possible
// to the NMEA timestamp's true wall-clock instant.
//
// Called once per NMEA burst (rate-limited internally). Returns silently when:
//   - GPS year is out of range, or GPS date/time isn't valid
//   - We were called less than one burst-interval since the last call
//   - The current fix is too stale (fix_age > 250 normally, > 500 if forced)
//   - Hundredths of a second is non-zero (PPS skew is then unreliable)
//   - GPS date/time fields are out of range (and we're not forcing)
//
// On success, calls setTimeWithMillis() with a millis() timestamp adjusted
// backward by the best-guess PPS skew (so the captured "now" represents
// the moment the GPS started transmitting the NMEA sentence, not the
// moment we finished receiving it).
//
// UPDATE: since the first sentence (GNGGA for ATGM336H) has the least
// difference in time from being sent from GPS, it's most accurate.
// The burst takes about 1 sec so the last (GNGST?) might be delayed by
// 1 sec because of uart transmission delays.
// UPDATE: just use GNGGA (TinyGPS mode) ..first one.
// Change the quiet zone to be time of a burst -- that way we always update
// on the last burst before wspr tx?
// Is the quiet zone especially important for a long burst?
// What if the burst interval was 5 secs -- could there be staleness?
// Don't use 5 sec broadcast interval (now we're using?)
// =============================================================================

// Days-per-month lookup (Feb=28; leap years are not validated since we only
// glitch-filter, not date-arithmetic).
static const uint8_t monthDays[] =
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// -----------------------------------------------------------------------------
// Quick prefilter: do we have any business trying to update time?
// Returns true if the GPS year is in a sane range and the date/time fields
// claim to be valid.
// -----------------------------------------------------------------------------
static bool gpsTimeIsUsableForUpdate(void) {
    uint16_t gps_year = gps.date.year();
    bool gps_year_valid = (gps_year >= 2025 && gps_year <= 2035);
    return gps_year_valid && !GpsInvalidAll
           && gps.date.isValid() && gps.time.isValid();
}

// -----------------------------------------------------------------------------
// fix_age gate: refuse to use a stale fix. Tighter limit normally,
// looser limit if we're forcing an update.
// Returns true to keep going, false to abort the whole update.
// -----------------------------------------------------------------------------
static bool fixAgeIsAcceptable(bool forceUpdate, uint32_t timeUpdateCnt) {
    uint32_t fix_age = gps.time.age();
    if (forceUpdate) {
        // always take the first one, to be sure of getting something
        if (timeUpdateCnt != 0) {
            // (caller has already noted forceUpdate elapsed_millis2)
            // loose fix_age constraint, if we're forcing, just in case?
            // with a really big NMEA burst now, the gps.time could have been set 1 sec ago
            // because we wait to use it
            // in fact, with 5 sec waits? we could be waiting 5 secs? 
            // but I think 1 sec is realistic
            if (fix_age > 1000) {
                V1_printf("WARN: bad setTime forceUpdate. fix_age %lu" EOL, fix_age);
                return false;
            }
        }
        return true;
    }
    // 1 sec fix_age is realistic?
    if (fix_age > 1000) {
        V1_printf("WARN: bad try. fix_age %lu" EOL, fix_age);
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// Sanity check the GPS date/time fields. Returns true if anything's bogus.
// Also prints the offending values when bad.
//
// Gate 1 (gpsTimeIsUsableForUpdate) has already verified the year range and
// gps.date.isValid() before this is called — no need to re-check the year here.
// All fields are uint8_t so we don't have to check negatives.
// We don't validate by month-specific leap-year arithmetic — glitches that
// way are unlikely.
// -----------------------------------------------------------------------------
static bool gpsDateTimeIsBad(uint8_t gps_hour, uint8_t gps_minute, uint8_t gps_second,
                             uint8_t gps_day, uint8_t gps_month, uint16_t gps_year) {
    bool bad = false;
    if (gps_hour > 23)   bad = true;
    if (gps_minute > 59) bad = true;
    if (gps_second > 59) bad = true;
    if (gps_day > 31)    bad = true;
    if (gps_month > 12)  bad = true;
    if (gps_month < 1)   bad = true;

    // check the days in a month, only if month is valid for the array
    if (!bad) {
        uint8_t validDays = monthDays[gps_month - 1];  // 0..11 by check above
        if (gps_day > validDays) {
            V1_print(F("ERROR: gps time is bad."));
            V1_printf(" day %u too big. max %u for the month" EOL,
                      gps_day, validDays);
            bad = true;
        }
    }

    if (bad) {
        V1_print(F("ERROR: gps time is bad. Maybe no received gps time yet."));
        V1_printf(" gps_hour %u gps_minute %u gps_second %u",
                  gps_hour, gps_minute, gps_second);
        V1_printf(" gps_day %u gps_month %u gps_year %u" EOL,
                  gps_day, gps_month, gps_year);
    }
    return bad;
}

// -----------------------------------------------------------------------------
// Compare system time to gps time and emit a GOOD/WARN/ERROR drift line.
// secondDelta == 0 is GOOD, == ±1 is WARN, > ±1 is ERROR.
// Skipped on the first update since the first comparison doesn't matter.
//
// sys_subsecond_ms: the millisecond-within-second of the system clock snapshot
// (snap_millis_after % 1000). Logged alongside secondDelta so that a spurious
// WARN caused by a second-boundary crossing (e.g. sys_subsecond_ms near 0 or
// 999) can be distinguished from actual drift in the log.
// -----------------------------------------------------------------------------
static void reportDrift(int secondDelta, bool forceUpdate, uint32_t timeUpdateCnt,
                        uint32_t sys_subsecond_ms) {
    if (timeUpdateCnt == 0) return;
    if (abs(secondDelta) > 1) {
        V1_printf("ERROR: excess clk drift. abs(secondDelta)>1:"
                  "  secondDelta %d sys_subsecond_ms %lu forceUpdate %u ",
                  secondDelta, sys_subsecond_ms, forceUpdate);
    } else if (abs(secondDelta) == 1) {
        V1_printf("WARN: drift. abs(secondDelta)==1:"
                  "  secondDelta %d sys_subsecond_ms %lu forceUpdate %u ",
                  secondDelta, sys_subsecond_ms, forceUpdate);
    } else {
        V1_printf("GOOD: no clk drift."
                  "  secondDelta %d sys_subsecond_ms %lu forceUpdate %u ",
                  secondDelta, sys_subsecond_ms, forceUpdate);
    }
    printSystemDateTime();
    V1_print(F(EOL));
}

// -----------------------------------------------------------------------------
// Diagnostic: how full is the Serial2 RX FIFO right now? Helps us understand
// whether all this print/work is causing characters to back up.
//
// Seems like all this takes 11ms or so max. If chars are arriving 1 per ms,
// the rx fifo needs room for ~11 when we do time, to absorb char backup.
// Should be okay if we started with empty rx fifo, kept up with it, and
// then only do this time update once per burst.
// OPEN: if backup > 30, drain the FIFO here before returning to the caller.
// Currently we only report; caller sees the backup on the next getChar() call.
// -----------------------------------------------------------------------------
static void reportRxFifoBackup(uint32_t fix_age_entry) {
    uint32_t fix_age = gps.time.age();
    // local, not the global. doesn't matter?
    int charsAvailable = Serial2.available();
    if (charsAvailable > 25) {
        V1_print(F("ERROR: rx fifo backup (2): "));
    } else if (charsAvailable > 21) {
        V1_print(F("WARN: rx fifo backup (2): "));
    }
    V1_printf("gps fix_age_entry %lu fix_age now %lu charsAvailable %d" EOL,
              fix_age_entry, fix_age, charsAvailable);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void checkUpdateTimeFromGps(uint32_t sentence_dollar_millis,
                            uint32_t snap_PPS_rise_millis, bool snap_PPS_rise_valid) {
    // okay to have prints here now that we delay the time update more?
    V1_println(F("checkUpdateTimeFromGps START"));
    static uint64_t lastUpdate_millis = 0;
    static uint64_t lastCheck_millis  = 0;
    static uint32_t timeUpdateCnt    = 0;
    uint32_t fix_age_entry = gps.time.age();
    // -------------------------------------------------------------------------
    // Early-out gate 1: GPS year/date/time validity
    // -------------------------------------------------------------------------
    // can't do anything if this isn't good!
    if (!gpsTimeIsUsableForUpdate()) {
        V1_println(F("checkUpdateTimeFromGps: gate1 skip (no valid GPS year/date/time)"));
        return;
    }

    // -------------------------------------------------------------------------
    // Early-out gate 2: rate limit — at most once per burst interval.
    // GPS_WAIT_FOR_NMEA_BURST_MAX must satisfy two constraints:
    //   > burst duration  (~1000 ms of NMEA sentences)  — blocks same-burst double-calls
    //   < call interval   (~5000 ms at 5-sec broadcast)  — allows the next burst through
    // Set to 4500 ms for 5-sec broadcast, 900 ms for 1-sec broadcast.
    // TinyGPS is configured to only commit time on GGA so the first sentence
    // in the burst is always the one we capture.
    // -------------------------------------------------------------------------
    uint32_t elapsed_millis1 = millis() - lastCheck_millis;
    lastCheck_millis = millis();
    // but this burst max time is slightly bigger than the burst interval
    // so we'll only update every other at most?
    if (elapsed_millis1 < GPS_WAIT_FOR_NMEA_BURST_MAX) {
        V1_printf("checkUpdateTimeFromGps: gate2 rate-limit. elapsed_millis1 %lu" EOL,
                  elapsed_millis1);
        return;
    }

    // forceUpdate: true if enough time has passed since the last successful update
    // to warrant ignoring loose gates. Always recomputed — not a static.
    uint32_t elapsed_millis2 = millis() - lastUpdate_millis;
    bool forceUpdate = elapsed_millis2 > GPS_WAIT_FOR_NMEA_BURST_MAX;
    if (forceUpdate && timeUpdateCnt != 0) {
        V1_printf("setTime forceUpdate. elapsed_millis2 %lu" EOL, elapsed_millis2);
    }

    // -------------------------------------------------------------------------
    // Early-out gate 3: fix_age must be acceptable.
    // Try to get as close to the NMEA timestamp as possible.
    // -------------------------------------------------------------------------
    if (!fixAgeIsAcceptable(forceUpdate, timeUpdateCnt)) return;

    // -------------------------------------------------------------------------
    // Early-out gate 4: hundredths must be 0.
    // PPS skew seems wrong often if not -- it goes to zero once we get a fix
    // and stays 0 for the repeated broadcast.
    // -------------------------------------------------------------------------
    uint8_t gps_hundredths = gps.time.centisecond();
    if (gps_hundredths > 0) {
        V1_printf("ERROR: won't setTime ..non-zero gps_hundredths %u ..PPS skew often bad" EOL,
                  gps_hundredths);
        printGpsDateTime(gps.date, gps.time, true);
        V1_print(F(EOL));
        return;
    }

    // -------------------------------------------------------------------------
    // Snapshot all GPS fields.
    // Date fields and hour/minute are stable within a burst — they change at
    // most once per minute and TinyGPS++ does not update them mid-sentence.
    // gps_second is snapshotted last, immediately before now(), to minimise
    // the window in which a new GGA commit could change it between the GPS
    // snapshot and the system clock snapshot (issue 2).
    // -------------------------------------------------------------------------
    uint16_t gps_year   = gps.date.year();
    uint8_t  gps_month  = gps.date.month();
    uint8_t  gps_day    = gps.date.day();
    uint8_t  gps_hour   = gps.time.hour();
    uint8_t  gps_minute = gps.time.minute();

    // -------------------------------------------------------------------------
    // Snapshot system clock and gps_second back-to-back (issue 2 + issue 3).
    //
    // gps_second is read here, as close to now() as possible, so both sides
    // of the drift comparison reflect the same instant.
    //
    // now() returns integer seconds with no sub-second component. If the system
    // clock ticks over a second boundary between the two reads, secondDelta
    // reads as ±1 even with zero real drift. Guard by bracketing now() with
    // millis(): if the counter crossed a 1000 ms boundary, re-read now() once
    // so the snapshot is firmly inside the new second.
    //
    // sys_subsecond_ms is logged alongside secondDelta so a boundary-crossing
    // WARN (sys_subsecond_ms near 0 or 999) can be distinguished from real drift.
    // -------------------------------------------------------------------------
    uint8_t  gps_second      = gps.time.second();  // read immediately before now()
    uint32_t snap_millis_before = millis();
    time_t t = now();
    uint32_t snap_millis_after  = millis();

    bool crossed_second = (snap_millis_after / 1000) != (snap_millis_before / 1000);
    if (crossed_second) {
        // Re-read both — window is now ~microseconds, double-crossing negligible.
        gps_second        = gps.time.second();
        t                 = now();
        snap_millis_after = millis();
        V1_printf("INFO: second boundary crossed during time snapshot, re-read" EOL);
    }

    uint8_t d  = (uint8_t) day(t);
    uint8_t hh = (uint8_t) hour(t);
    uint8_t mm = (uint8_t) minute(t);
    uint8_t ss = (uint8_t) second(t);
    // Sub-second offset of the system clock at snapshot time.
    // Near 0 or near 999 on a WARN means a boundary crossing, not real drift.
    uint32_t sys_subsecond_ms = snap_millis_after % 1000;

    // okay to just compare monthSecs and not roll up into total seconds.
    uint32_t monthSecs =
        (d * 24 * 3600) + (hh * 3600) + (mm * 60) + ss;
    uint32_t gps_monthSecs =
        (gps_day * 24 * 3600) + (gps_hour * 3600) + (gps_minute * 60) + gps_second;

    // UPDATE: always update time if we got this far (we'll only do this once
    // per sentence burst -- might have better skew numbers).
    // if ((!forceUpdate) && y == gps_year && m == gps_month && monthSecs == gps_monthSecs) return;

    // -------------------------------------------------------------------------
    // Early-out gate 5: range-check the GPS fields.
    // Not strictly an early-out if forceUpdate -- we proceed anyway, but
    // solar calcs will be wrong if date is wrong.
    // We proceed on forceUpdate even if the date is bad (solar calcs will be
    // wrong, but we need at least one time set). On non-forced updates, bad
    // date/time is a hard gate — don't set system time from garbage.
    // -------------------------------------------------------------------------
    bool dateTimeBad = gpsDateTimeIsBad(gps_hour, gps_minute, gps_second,
                                        gps_day, gps_month, gps_year);
    if (dateTimeBad && !forceUpdate) return;

    // -------------------------------------------------------------------------
    // Compute anchor timestamp and apply PPS correction (Fixes A, C, D).
    //
    // When PPS is valid and the clock is already close (|secondDelta| <= 1),
    // anchor setTime_millis directly at the PPS edge (snap_PPS_rise_millis).
    // The PPS edge is the GPS second boundary — using it as the anchor means
    // TimeLib's now() tracks the GPS second, not the sentence arrival time.
    //
    // When PPS is not valid or the clock is badly off, anchor at the raw
    // sentence '$' timestamp (sentence_dollar_millis). This is less precise
    // (~fix_age ms into the second) but always correct and never stale.
    //
    // Fix C: snap_PPS_rise_valid gates the correction — if PPS is not running,
    //        the snapshot is zero/stale and must not be used.
    // Fix A: clockIsClose gates the correction — applying a sub-second PPS
    //        anchor on top of a multi-second error makes it worse, not better.
    // Fix D: snap_PPS_rise_millis was captured at '*', before the ISR could
    //        overwrite PPS_rise_millis with the next second's edge.
    // -------------------------------------------------------------------------
    setTime_millis = sentence_dollar_millis;
    int secondDelta  = ((int) monthSecs) - ((int) gps_monthSecs);
    bool clockIsClose = (abs(secondDelta) <= 1);

    if (snap_PPS_rise_valid && clockIsClose && snap_PPS_rise_millis != 0) {
        V1_printf("Anchoring at PPS edge: setTime_millis %lu -> snap_PPS_rise_millis %lu" EOL,
                  setTime_millis, snap_PPS_rise_millis);
        setTime_millis = snap_PPS_rise_millis;
    } else {
        V1_printf("Using raw sentence timestamp: snap_PPS_rise_valid %u clockIsClose %u"
                  " setTime_millis %lu" EOL,
                  snap_PPS_rise_valid, clockIsClose, setTime_millis);
    }

    // setTime_millis is now the best available anchor for the GPS second boundary.
    // It must be <= current millis(); the PPS skew subtraction above ensures this
    // when applied, and sentence_dollar_millis is always in the past when not applied.
    setTimeWithMillis(gps_hour, gps_minute, gps_second,
                      gps_day, gps_month, gps_year,
                      setTime_millis);

    // -------------------------------------------------------------------------
    // Time-update snapshot log.
    // -------------------------------------------------------------------------
    V1_print(F("GOOD: system setTime() with"));
    V1_printf(" gps_day %u gps_hour %u gps_minute %u gps_second %u" EOL,
              gps_day, gps_hour, gps_minute, gps_second);

    V1_print(F("system time before: (should be gps time):"));
    V1_printf(" day %d hour %d minute %d second %d", d, hh, mm, ss);
    V1_printf(" forceUpdate %u time now: ", forceUpdate);
    // this will be current system time
    printSystemDateTime();
    V1_print(F(EOL));

    // -------------------------------------------------------------------------
    // Drift report. secondDelta computed above (before anchor adjustment).
    // -------------------------------------------------------------------------
    reportDrift(secondDelta, forceUpdate, timeUpdateCnt, sys_subsecond_ms);

    // -------------------------------------------------------------------------
    // Diagnostic: rx fifo backup at this point. Might give an indication of
    // how long it takes to do all this work.
    // -------------------------------------------------------------------------
    reportRxFifoBackup(fix_age_entry);

    // -------------------------------------------------------------------------
    // Bookkeeping
    // -------------------------------------------------------------------------
    lastUpdate_millis = millis();
    // forceUpdate is recomputed from elapsed_millis2 at the top of every call
    // that passes gate 2 — the assignment here has no effect on the next call.
    timeUpdateCnt += 1;
    V1_print(EOL);
    // OPEN: the printing in this function (~11 ms measured) may cause the UART RX
    // FIFO to back up. reportRxFifoBackup() above measures this; if consistently
    // > 25, consider deferring the verbose prints until after the next getChar().
}


// -----------------------------------------------------------------------------
void gpsDebug() {
    if (!VERBY[1]) return;
    // am I getting problems with constant strings in ram??
    char debugMsg0[] = "Before any gpsDebug prints";
    realPrintFlush(debugMsg0, true);  // print

    V1_println(F("GpsDebug START"));
    V1_print(F(EOL));
    V1_print(F(EOL));
    bool validA = gps.satellites.isValid() && !GpsInvalidAll;
    bool validB = gps.hdop.isValid() && !GpsInvalidAll;
    bool validC = gps.location.isValid() && !GpsInvalidAll;
    bool validD = gps.altitude.isValid() && !GpsInvalidAll;
    bool validE = gps.course.isValid() && !GpsInvalidAll;
    bool validF = gps.speed.isValid() && !GpsInvalidAll;
    // date and time are not gated by GpsInvalidAll: they remain valid across a
    // GPS power cycle (the chip keeps RTC running on VBAT), so we don't want
    // invalidateTinyGpsState() to suppress a good time reading.
    bool validG = gps.date.isValid();
    bool validH = gps.time.isValid();
    // don't have valid bits, just enum encoded for invalid
    // enum fq {Invalid = '0', GPS = '1', DGPS = '2', PPS = '3', RTK = '4', 
    //  FloatRTK = '5', Estimated = '6', Manual = '7', Simulated = '8' };
    // enum fm { N = 'N', A = 'A', D = 'D', E = 'E'};

    // can use this? gpsDebug will print the char?
    // is there no isValid() for these?
    bool validI = (gps.location.FixQuality() != TinyGPSLocation::Quality::Invalid);
    bool validJ = (gps.fix.FixMode() != TinyGPSFix::Mode::N);

    // can compare char and int? 
    // this includes Estimated as valid?
    // bool validI = gps.location.FixQuality() != 0;
    // bool validJ = gps.fix.FixMode() != 'N';

    V1_printf("gps valids: %u %u %u %u %u %u %u %u %u %u %u" EOL,
        !GpsInvalidAll, validA, validB, validC, validD, validE, validF, validG, validH, validI, validJ);

    printStr("Sats", true, 5);
    printStr("HDOP", true, 5);
    printStr("Latitude", true, 12);
    printStr("Longitude", true, 12);
    printStr("FixAge", true, 7);
    // 2025-01-04 21:46:31
    printStr("Date", true, 11);
    printStr("Time", true, 9);
    printStr("DTAge", true, 6);
    printStr("Alt", true, 8);
    printStr("Degs.", true, 6);
    printStr("Course", true, 7);
    printStr("Speed", true, 6);
    printStr("ChrsRx", true, 10);
    printStr("SentsWfix", true, 10);
    printStr("failCksum", true, 10);
    printStr("FixQual", true, 7);
    printStr("FixMode", true, 7);

    V1_print(F(EOL));

    char debugMsg1[] = "Before printInt/Float/String gpsDebug prints";
    realPrintFlush(debugMsg1, false);  // no print

    // https://github.com/StuartsProjects/GPSTutorial
    // (VERBY[1] already checked at function entry; inner guard removed.)
    // charsProcessed/sentencesWithFix are uint32_t — they wrap at ~4 billion chars,
    // well beyond any single session; printInt truncation is not a concern in practice.
    // HDOP from TinyGPS++ is in hundredths (per NMEA spec), not in the usual
    // 0.0-99.9 float form. Divide by 100 to get the real value.
    // DOP scale: <1=Ideal, 1-2=Excellent, 2-5=Good, 5-10=Moderate, >10=Poor.
    // Source: https://github.com/mikalhart/TinyGPSPlus/issues/8
    printInt(gps.satellites.value(), validA, 5);
    printInt(gps.hdop.value(), validB, 5);
    printFloat(gps.location.lat(), validC, 12, 6);
    printFloat(gps.location.lng(), validC, 12, 6);
    printInt(gps.location.age(), validC, 7);
    // printAge. date & time isValid() is in the function
    printGpsDateTime(gps.date, gps.time, true);
    printFloat(gps.altitude.meters(), validD, 8, 2);
    printFloat(gps.course.deg(), validE, 6, 2);
    printStr(TinyGPSPlus::cardinal(gps.course.value()), validE, 7);
    printFloat(gps.speed.knots(), validF, 6, 2);
    printInt(gps.charsProcessed(), true, 10);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 10);

    // FixQuality enum: Invalid='0' GPS='1' DGPS='2' PPS='3' RTK='4'
    //                  FloatRTK='5' Estimated='6' Manual='7' Simulated='8'
    // FixMode enum:    N='N' A='A' D='D' E='E'
    char fq[2] = { 0 };
    fq[0] = gps.location.FixQuality();
    printStr(fq, true, 7);

    char fm[2] = { 0 };
    fm[0] = gps.fix.FixMode();
    printStr(fm, true, 7);
    V1_print(F(EOL));
    V1_print(F(EOL));
    // am I getting problems with constant strings in ram??
    char debugMsg2[] = "After all gpsDebug prints";
    realPrintFlush(debugMsg2, true);  // print

    V1_println(F("GpsDebug END"));
}
