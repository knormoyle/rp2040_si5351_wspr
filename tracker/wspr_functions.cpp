// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "defines.h"
#include "debug_functions.h"
#include "print_functions.h"

//*******************************************************
// The RP2040 PWM block has 8 identical slices, the RP2350 has 12.
// Each slice can drive two PWM output signals, or
// measure the frequency or duty cycle of an input signal.
// This gives a total of up to 16/24 controllable PWM outputs.
// All 30 GPIOs can be driven by the PWM block.
//
// The PWM hardware functions by continuously comparing the input value to a free-running counter.
// This produces a // toggling output where the amount of time spent at the high output level
// is proportional to the input value.
// The fraction of time spent at the high signal level is known as the duty cycle of the signal.
//
// The default behaviour of a PWM slice is to count upward until the wrap value (\ref pwm_config_set_wrap)
// is reached, and then immediately wrap to 0.
// PWM slices also offer a phase-correct mode, where the counter starts to count downward after
// reaching TOP, until it reaches 0 again.

// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h

// If the divide mode is free-running, the PWM counter runs at clk_sys / div.

// FIX! are these delays adjusted for compensation due to code delays? no?
// WSPR_TONE_SPACING = 146;  // ~1.46 Hz
// ths is what we're called with for symbol time (millis)
// WSPR_DELAY = 683;

//*******************************************************
// uses PLL_SYS_MHZ to figure a good div and wrap_cnt period for the PWM (subtract 1 for TOP)
void calculateDivAndWrap(int *PWM_DIV, int *PWM_WRAP_CNT, float ms, uint32_t PLL_SYS_MHZ) {
    int div;
    // FIX! is float enough precision for odd pico frequencies?
    // usecs

    // how many interrupts per total delay?
    // the wrap count is limited to 16 bits
    // this is enough to make it work. Can't make it work with just 1 interrupt
    float INTERRUPTS = 8;

    // interesting frequencies seem to find solutions with 250 divider. and 8 interrupts
    // 1/mhz is microseconds (1e-6)
    float PLL_SYS_USECS = 1.0 / (float)PLL_SYS_MHZ;
    float wrap_cnt_float;
    int wrap_cnt;
    // could check if PLL_SYS_PERIOD is integer aligned?
    const float DESIRED_SECS = 110.592;

    // found some ones:


    // we use 250 div now for 125 mhz so check with that first
    int DIV_MIN = 250;
    int DIV_MAX = 351;
    float totalSymbolsTime;

    for (div = DIV_MIN; div <= DIV_MAX; div++) {

        float div_float = (float)div;
        float possible_wrap_cnt = DESIRED_SECS / (162 * INTERRUPTS * div_float * PLL_SYS_USECS * 1e-6);

        // wrap_cnt can't be too big (how big can the pwm thing count?
        // 32-bit limit?
        if (possible_wrap_cnt > 65536) continue; // we -1 this for use

        Serial.printf("possible_wrap_cnt %.f div %d" EOL, possible_wrap_cnt, div);
        wrap_cnt_float = possible_wrap_cnt;
        // does a floor..use that and see what we get for total time!
        wrap_cnt = (int) wrap_cnt_float;

        // does this come close enough to total time for all symbols
        totalSymbolsTime = 162 * INTERRUPTS * div_float * PLL_SYS_USECS * 1e-6 * (float) wrap_cnt;
        Serial.printf("totalSymbolsTime %.3f wrap_cnt %d div %d" EOL, totalSymbolsTime, wrap_cnt, div);

        // good enough total range
        if (totalSymbolsTime > 110.585 && totalSymbolsTime < 110.60) break;

        // The WSPR transmission consists of 162 symbols, each has a duration of 256/375 seconds.
        // 0.68266666666
        // 162 * 256/375 = 110.592 secs total
    }


    if (div > DIV_MAX) {
        Serial.printf("ERROR: didn't find a good div and wrap_cnt for PLL_SYS_MHZ %lu PLL_SYS_USECS %.7f" EOL,
            PLL_SYS_MHZ, PLL_SYS_USECS);
    }
    else {
        *PWM_DIV = div;
        *PWM_WRAP_CNT = wrap_cnt;

        Serial.printf("GOOD: Found a good div and wrap_cnt for PLL_SYS_MHZ %lu PLL_SYS_USECS %.7f" EOL,
            PLL_SYS_MHZ, PLL_SYS_USECS);
        Serial.printf("GOOD: PLL_SYS_MHZ %lu PWM_DIV %d PWM_WRAP_CNT %d" EOL, PLL_SYS_MHZ, div, wrap_cnt);
        Serial.printf("GOOD: totalSymbolsTime %.3f" EOL, totalSymbolsTime);
    }
}
// GOOD: PLL_SYS_MHZ 60 PWM_DIV 250 PWM_WRAP_CNT 20479
// GOOD: totalSymbolsTime 110.587
// GOOD: PLL_SYS_MHZ 100 PWM_DIV 250 PWM_WRAP_CNT 34133
// GOOD: totalSymbolsTime 110.591
// GOOD: PLL_SYS_MHZ 133 PWM_DIV 250 PWM_WRAP_CNT 45397
// GOOD: totalSymbolsTime 110.591
// GOOD: PLL_SYS_MHZ 125 PWM_DIV 250 PWM_WRAP_CNT 42666
// GOOD: totalSymbolsTime 110.590


// The protocol specification states:
// Each tone should last for 8192/12000 = 0.682666667 seconds, and transitions between
// tones should be done in a phase-continuous manner.
