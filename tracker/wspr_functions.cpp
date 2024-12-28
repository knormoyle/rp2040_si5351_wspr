// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "debug_functions.h"
#include "print_functions.h"
// https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/
#include "hardware/pwm.h"
#include "wspr_functions.h"
#include "led_functions.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// alternatives for hw timers
// April, 2022 article (different now?)
// https://emalliab.wordpress.com/2021/04/18/raspberry-pi-pico-arduino-core-and-timers/

// has code for these cases:
// 1) Mbed supports the concept of Tickers and Timeouts
// can get a microsecond-resolution, periodic function called for any mbed based setup.
// Not less than 60 usec?

// 2) Raspberry Pi Pico Repeating Timer
// is a Pico SDK function for setting up a repeating timer to trigger
// a function each time (see chapter 4.2.12 in the Pico C/C++ SDK datasheet).
// Unfortunately this does not build on the official Arduino RP2040 core.
// It turns out that much of the APIs for the micro-second alarm and timer functions
// are conditional on PICO_TIME_DEFAULT_ALARM_POOL_DISABLED not being set.
// But for the current build this is set (I don’t know why – I haven’t looked into it further) –
// so not only is there no “default alarm pool” to which you can attach 
// repeating timers and alarms,
// but even the functions to handle such things are “compiled out”.

// 3) Raspberry Pi Pico Alarm System
// Looked into the Pico SDK’s alarm system (see chapter 4.2.11 in Pico C/C++ SDK datasheet).
// This provides a number of APIs for setting up alarms based on the system clock and
// includes such APIs as “add_alarm_in_us”.
// But as with the repeated timer system this appears not to be available (at present)
// in the official Arduino mbed based RP2040 core.

// 4) Pico Low-level Timer Hardware API
// Pico SDK’s “low level hardware API” directly 
// (see chapter 4.1.22 in the Pico C/C++ SDK datasheet,
// and chapter 4.6 in the RP2040 datasheet).

// The RP2040 has the concept of alarms which can be triggered on the main system timer,
// and that can be configured as shown in the “timer_lowlevel example” in the Pico GitHub area.
// From the debug code it would appear that alarm[0] is already used somewhere,
// I haven’t investigated where, so I set everything up to use alarm[1].

// Note that the alarm interrupt routine (alarm_irq) must do two things
// in addition to what ever you need it to do:

// -Clear the interrupt by writing to the appropriate bit in INTR.
// -Re-arm the alarm to trigger for its next run.

// This manual “re-arming” process means that if you need a perfectly accurate “tick”
// you won’t get it.
// The next triggering of the alarm will be equal to the requested period plus whatever time
// it has taken to run the code in the interrupt service routine up to the point of
// you re-arming the alarm.

//*******************************************************************************
// So: it appears using the PWM based interrupt mechanism avoids all these issues! (kevin)
//*******************************************************************************
extern bool VERBY[10];
// this can get modified for 18 Mhz operation (from 8 to 1?)
extern uint32_t INTERRUPTS_PER_SYMBOL;

const int WSPR_PWM_SLICE_NUM = 4;
extern volatile bool PROCEED;

//************************************************
void wsprSleepForMillis(int n) {
    // FIX! we could use a sleep_until() thing? but sleep_ms should be fine?
    // if we know watchdog interval is > than the max sleep used here?
    // and ignored led updates
    Watchdog.reset();
    // interesting it says 'attempts'
    // void sleep_ms (uint32_t ms)
    // Wait for the given number of milliseconds before returning.
    // This method attempts to perform a lower power sleep (using WFE) as much as possible.
    // ms the number of milliseconds to sleep

    if (false) {
        // I guess we don't want to use this, because the led isn't update
        // so that 3 short led, start looking like 3 long (longs are config/error cases)
        sleep_ms(n);
    } else {
        int milliDiv = n / 10;
        // sleep approx. n millisecs
        for (int i = 0; i < milliDiv ; i++) {
            // https://docs.arduino.cc/language-reference/en/functions/time/delay/
            // check for update every 10 milliseconds
            if ((milliDiv % 10) == 0) {
                // no prints in this
                updateStatusLED();
                Watchdog.reset();
            }

            // faster recovery with delay?
            // could we get some variation that affects our symbol to symbol time?
            sleep_ms(10);
        }
    }
}

//************************************************
// https://swharden.com/software/FSKview/wspr/
// 110.6 sec continuous wave
// Frequency shifts among 4 tones every 0.683 sec
// Tones are separated by 1.46 Hz
// Total bandwidth is about 6 Hz
// 50 bits of information are packaged into a 162 bit message with FEC
// Transmissions always begin 1 sec after even minutes (UTC)

// There are 162 ((50 + K − 1) * 2) possible symbols.
// Each conveys one sync bit (LSB) and one data bit (MSB).
// Data Transmission Rate and Duration
// Keying rate is: 12000⁄8192 = 1.4648 baud
// Duration is: 162 * 192⁄12000 = 110.6 sec
// Modulation Continuous phase 4 FSK, with 1.4648 Hz tone separation
// Occupied bandwidth is about 6 Hz
// Synchronization is via a 162 bit pseudo-random sync vector.
// Transmissions nominally start one second into an even UTC minute
// (e.g., at hh:00:01, hh:02:01, etc.)

// WSPR Modulation:
// Each symbol represents a frequency shift of 12000 / 8192 Hz (1.46Hz)
// per symbol value giving four-level multi-FSK modulation.
// The transmitted symbol length is the reciprocal of the tone spacing,
// or approximately 0.683 seconds, so the complete message of 162 symbols
// takes around 110.6 seconds to send and occupies a bandwidth of approximately 6Hz.

// WSPR protocol specification states:
// Each tone should last for 8192/12000 = 0.682666667 seconds
// and transitions between tones should be done in a phase-continuous manner.

//*******************************************************
// The RP2040 PWM block has 8 identical slices.
// Each slice can drive two PWM output signals, or
// measure the frequency or duty cycle of an input signal.
// This gives a total of up to 16/24 controllable PWM outputs.

// All 30 GPIOs can be driven by the PWM block.
//
// The PWM hardware functions by continuously comparing the input value to a
// free-running counter.
// This produces a toggling output where the amount of time spent at the high output level
// is proportional to the input value.
//
// The default behaviour of a PWM slice is to count upward until the wrap value
// (ref pwm_config_set_wrap) is reached, and then immediately wrap to 0.

// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h

// If the divide mode is free-running, the PWM counter runs at clk_sys / div.

// FIX! are these delays adjusted for compensation due to code delays? no?
// WSPR_TONE_SPACING = 146;  // ~1.46 Hz
// ths is what we're called with for symbol time (millis)
// WSPR_DELAY = 683;

//*******************************************************
// FIX! what about just using hardware timers on pi pico. and migrating away
// from the PWM interrupt timer? I guess no reason to.
// https://reference.arduino.cc/reference/en/libraries/rpi_pico_timerinterrupt

// https://github.com/khoih-prog/RPI_PICO_TimerInterrupt
// This library enables you to use Interrupt from Hardware Timers on on RP2040-based
// boards using Earle Philhower's arduino-pico core.
// As Hardware Timers are rare, and very precious assets of any board,
// this library now enables you to use up to 16 ISR-based Timers,
// while consuming only 1 Hardware Timer.
// Timers' interval is very long (ulong millisecs).

//*******************************************************
// How is this aligned to 1 sec in off the starting minute?
// can't have parameters?
static uint32_t pwm_interrupt_cnt = 0;
static uint32_t pwm_interrupt_total_cnt = 0;
void PWM4_Handler() {
    // too much output!
    // V1_print/ln(F("PWM4_Handler() START"));
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    pwm_interrupt_cnt++;
    pwm_interrupt_total_cnt++;
    if (pwm_interrupt_cnt >= INTERRUPTS_PER_SYMBOL) {
        pwm_interrupt_cnt = 0;
        // symbol rate is approximately
        // 1.4648 baud (4fsk per sec), or exactly 12,000 Hz / 8192.
        // 0.6826 secs per symbol?
        PROCEED = true;
    }
    // FIX! remove this. redundant and slows down gettng to 'proceed' in tracker.ino
    if (false && VERBY[1]) {
        // we might have two extra 'proceed' synchroniations -> 2 * INTERRUPTS_PER_SYMBOL
        // before we start a message now
        uint32_t pwm_interrupt_cnt_162 = pwm_interrupt_total_cnt % 162;
        if ((pwm_interrupt_total_cnt % 10) == 6) {  // instead of 0
            StampPrintf("sym: %lu %lu" EOL, pwm_interrupt_cnt_162, pwm_interrupt_cnt);
        }

        if ((pwm_interrupt_total_cnt % 162) == 16) {  // instead of 161
            StampPrintf("sym: %lu %lu" EOL, pwm_interrupt_cnt_162, pwm_interrupt_cnt);
            DoLogPrint();
        }
    }
    // V1_print/ln(F("PWM4_Handler() END"));
}

//*******************************************************
void setPwmDivAndWrap(uint32_t PWM_DIV, uint32_t PWM_WRAP_CNT) {
    // valid_params_if(HARDWARE_PWM, div >= 1 && div < 256);
    if (PWM_DIV >= 256)
        V1_printf("ERROR: illegal PWM_DIV  %lu is > 256 ..drops upper bits?" EOL,
            PWM_DIV);
    if (PWM_DIV ==  0)
        V1_print(F("ERROR: illegal PWM_DIV is 0" EOL));  // 0 should be illegal also?

    // gets passed as uint16_t
    if (PWM_WRAP_CNT >= (1 << 16))
        V1_printf("ERROR: illegal PWM_WRAP_CNT %lu is > 2**16 ..drops upper bits?" EOL,
            PWM_WRAP_CNT);
    if (PWM_WRAP_CNT == 0 )
        V1_print(F("ERROR: illegal PWM_WRAP_CNT is 0" EOL));
    V1_flush();

    // side-note: per:
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h

    // don't need to program this.. default is always-on (free-running PWM)
    // static inline void pwm_config_set_clkdiv_mode(pwm_config *c, enum pwm_clkdiv_mode mode)
    // valid_params_if(HARDWARE_PWM, mode == PWM_DIV_FREE_RUNNING ||
    // Configure which event gates the operation of the fractional divider.
    // The default is always-on (free-running PWM).

    // FIX! we could do bounds checking? or ??
    // assumes the values are right for INTERRUPTS_PER_SYMBOL
    // which is now 8 (instead of 500)
    // V1_print/ln(F("zeroTimerSetPeriodMs() START"));

    static pwm_config wspr_pwm_config = pwm_get_default_config();
    // 250 clocks at 125Mhz .008 uSec per clock, is 250 * .008 = 2 uSec
    // 2uS (at 125Mhz? does it need adjusting at other clks?)
    // so if we're count at 2uSec intervals to wrap at 683 millis..
    // that's 500*683 = 341500 counts?

    // the 250 can be a uint? so can be pretty big? Can we make it 5 times bigger
    // (1450 or maybe 1500)
    // to bring down the sie of the wrap count
    // or should we count 8 interrupts, and adjust for 1/8th the total target

    // per https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h#L156
    // can't be bigger than 255!
    // static inline void pwm_config_set_clkdiv_int(pwm_config *c, uint div) {
    // valid_params_if(HARDWARE_PWM, div >= 1 && div < 256);
    pwm_config_set_clkdiv_int(&wspr_pwm_config, PWM_DIV);  // takes uint?

    // Set the highest value the counter will reach before returning to 0.
    // Also known as TOP.
    pwm_config_set_wrap(&wspr_pwm_config, ((uint16_t)PWM_WRAP_CNT - 1));
    pwm_init(WSPR_PWM_SLICE_NUM, &wspr_pwm_config, false);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, PWM4_Handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_clear_irq(WSPR_PWM_SLICE_NUM);

    // global state that's changed on interrupts
    pwm_interrupt_cnt = 0;
    pwm_interrupt_total_cnt = 0;

    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
    // start it counting to interrupt and toggle PROCEED after completion of first symbol?
    // first symbol could be "short" as a result (extra delay in getting first frequency setup)
    // setup the base frequency earlier, so we don't see this extra variation on first symbol!
    // due to the "change" optimization in the i2c writes.
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
    // V1_print/ln(F("zeroTimerSetPeriodMs() END"));
}

//*******************************************************
void disablePwmInterrupts(void) {
    // FIX! do we have to undo all of this? what if we had an parameter that
    // probably cause we have to start with everything at 0 when we start
    // otherwise don't know where we are relative to the divider etc
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    irq_set_enabled(PWM_IRQ_WRAP, false);
    irq_remove_handler(PWM_IRQ_WRAP, PWM4_Handler);

    // global state that's changed on interrupts
    pwm_interrupt_cnt = 0;
    pwm_interrupt_total_cnt = 0;
}

//*******************************************************
// uses PLL_SYS_MHZ, INTERRUPTS_PER_SYMBOL
// to figure a good div and wrap cnt period for the PWM (subtract 1 for TOP)
void calcPwmDivAndWrap(uint32_t *PWM_DIV, uint32_t *PWM_WRAP_CNT,
        uint32_t INTERRUPTS_PER_SYMBOL, uint32_t PLL_SYS_MHZ) {
    V1_print(F("calcPwmDivAndWrap START"));
    V1_printf(" for INTERRUPTS_PER_SYMBOL %lu PLL_SYS_MHZ %lu" EOL,
        INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);

    // FIX! is float enough precision for odd pico frequencies?

    // symbol rate is ~1.4648 baud (4fsk per sec). Exactly: 12000 Hz / 8192.
    // 0.68266666... secs per symbol? (8192/12000). why is 256/375 the same?

    // The WSPR transmission consists of 162 symbols, each has a duration of
    // 256/375 seconds.
    // why do some people quote 110.6 secs?
    // 162 * 256/375 = 110.592 secs total
    // 162 * 8192/12000 = 110.592 secs total

    const float DESIRED_SECS = 110.592;
    const float DESIRED_SECS_MIN = DESIRED_SECS - 0.001;
    const float DESIRED_SECS_MAX = DESIRED_SECS + 0.001;

    // iterate to get the lowest error_so_far, not an absolute comparison
    // return the best you got
    // everything should have less error than this?
    float abs_error_so_far = 999999.99;

    // the wrap count is limited to 16 bits
    // this is enough to make it work.
    // Can't make it work with just 1 interrupt per symbol.
    float interrupts_per_symbol = (float) INTERRUPTS_PER_SYMBOL;

    // interesting frequencies seem to find solutions with 250 divider.
    // and 8 interrupts
    // 1/Mhz is microseconds (1e-6)
    float PLL_SYS_USECS = 1.0 / (float)PLL_SYS_MHZ;
    float wrap_cnt_float;
    int wrap_cnt;
    float totalSymbolsTime = 0.0;

    // we use 250 div now for 125 mhz so check with that first
    // per
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h#L156
    // can't be bigger than 255?
    // 256 would be odd?  (more than 8 bits to past as parameter?,
    // although as a divider it would make sense to support?

    // yes, this says 255 max?
    // static inline void pwm_config_set_clkdiv_int(pwm_config *c, uint div) {
    // valid_params_if(HARDWARE_PWM, div >= 1 && div < 256);
    int DIV_MIN = 250;
    int DIV_MAX = 255;

    int div;
    for (div = DIV_MIN; div <= DIV_MAX; div++) {
        float div_float = (float)div;
        float timePerWrap = 162 * interrupts_per_symbol * div_float * PLL_SYS_USECS * 1e-6;
        float possible_wrap_cnt = DESIRED_SECS / timePerWrap;

        // V1_printf("possible_wrap_cnt %.f div %d" EOL, possible_wrap_cnt, div);
        wrap_cnt_float = possible_wrap_cnt;
        // does a floor..use that and see what we get for total time!
        wrap_cnt = (int) wrap_cnt_float;

        // wrap_cnt can't be too big (how big can the pwm thing count?  16-bit limit?
        if (wrap_cnt >= (1 << 16)) continue;

        // does this come close enough to total time for all symbols
        float symbolTime = INTERRUPTS_PER_SYMBOL *
            div_float * PLL_SYS_USECS * 1e-6 * (float) wrap_cnt;

        totalSymbolsTime = 162 * symbolTime;
        V1_printf("totalSymbolsTime %.3f wrap_cnt %d div %d" EOL,
            totalSymbolsTime, wrap_cnt, div);

        // good enough total range
        float current_abs_error = abs(totalSymbolsTime - DESIRED_SECS);
        if (current_abs_error < abs_error_so_far) {
            V1_printf("BETTER: PLL_SYS_MHZ %lu PWM_DIV %d PWM_WRAP_CNT %d" EOL,
                PLL_SYS_MHZ, div, wrap_cnt);
            V1_printf("BETTER: totalSymbolsTime %.3f" EOL, totalSymbolsTime);
            abs_error_so_far = current_abs_error;
            *PWM_DIV = (uint32_t)div;
            *PWM_WRAP_CNT = (uint32_t)wrap_cnt;
        }

        // If we're within desired bounds, stop trying
        if (totalSymbolsTime >= DESIRED_SECS_MIN && totalSymbolsTime <= DESIRED_SECS_MAX) break;
    }

    // did we go through the full DIV range without getting a break from the bounds compare?
    // we'll still return the best we got
    if (div > DIV_MAX) {
        V1_printf("ERROR: didn't find a great div and wrap_cnt for PLL_SYS_MHZ %lu",
            PLL_SYS_MHZ);
        V1_printf(" PLL_SYS_USECS %.7f" EOL, PLL_SYS_USECS);
    } else {
        V1_printf("GOOD: Found a great div and wrap_cnt for PLL_SYS_MHZ %lu",
            PLL_SYS_MHZ);
        V1_printf(" PLL_SYS_USECS %.7f" EOL, PLL_SYS_USECS);
    }

    V1_printf("GOOD: PLL_SYS_MHZ %lu PWM_DIV %d PWM_WRAP_CNT %d" EOL,
        PLL_SYS_MHZ, div, wrap_cnt);
    V1_printf("GOOD: totalSymbolsTime %.3f" EOL, totalSymbolsTime);
    V1_printf("calcPwmDivAndWrap END for INTERRUPTS_PER_SYMBOL %lu PLL_SYS_MHZ %lu" EOL,
        INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);
}
