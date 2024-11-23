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
    float PLL_SYS_USECS = 1.0 / (float)PLL_SYS_MHZ; 
    float wrap_cnt_float;
    int wrap_cnt;
    // could check if PLL_SYS_PERIOD is integer aligned?
    const float DESIRED_SECS = 110.592;
    
    // found some ones: 
    // for (div = 200; div<231; div++) {
    // GOOD: PLL_SYS_MHZ 60 PWM_DIV 237 PWM_WRAP_CNT 172827
    // GOOD: PLL_SYS_MHZ 100 PWM_DIV 233 PWM_WRAP_CNT 292990
    // GOOD: PLL_SYS_MHZ 133 PWM_DIV 223 PWM_WRAP_CNT 407151
    // GOOD: PLL_SYS_MHZ 125 PWM_DIV 213 PWM_WRAP_CNT 400626

    int DIV_MIN = 200; 
    int DIV_MAX = 351; 
    float symbolTime_us;
    // float symbolTime_ms;
    float symbolTime;
    float totalSymbolsTime;

    for (div = DIV_MIN; div <= DIV_MAX; div++) {

        float possible_wrap_cnt = DESIRED_SECS / ((162 * div * PLL_SYS_USECS) / 1000000UL);

        // wrap_cnt can't be too big (how big can the pwm thing count?
        // 32-bit limit?
        if (possible_wrap_cnt > 2e9) continue;

        Serial.printf("possible_wrap_cnt %.f div %d" EOL, possible_wrap_cnt, div);
        wrap_cnt_float = possible_wrap_cnt;
        // does a floor..use that and see what we get for total time!
        wrap_cnt = (int) wrap_cnt_float;

        // does this come close enough to total time for all symbols
        
        symbolTime_us = (wrap_cnt * div * PLL_SYS_USECS * 162);
        // symbolTime_ms = symbolTime_us / 1000.0;
        symbolTime    = symbolTime_us / 1000000.0;
        totalSymbolsTime = 162 * symbolTime;
        Serial.printf("totalSymbolsTime %.f wrp_cnt %d div %d" EOL, totalSymbolsTime, wrap_cnt, div);

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

        Serial.printf("GOOD: Found a good div and wrap_cnt for PLL_SYS_MHZ %lu PLL_SYS_USECS %.f" EOL,
            PLL_SYS_MHZ, PLL_SYS_USECS);
        Serial.printf("GOOD: PLL_SYS_MHZ %lu PWM_DIV %d PWM_WRAP_CNT %d" EOL, PLL_SYS_MHZ, div, wrap_cnt);
        Serial.printf("GOOD: symbolTime %.3f" EOL, symbolTime);
        Serial.printf("GOOD: totalSymbolsTime %.3f" EOL, totalSymbolsTime);
    }
}

// The protocol specification states:
// Each tone should last for 8192/12000 = 0.682666667 seconds, and transitions between
// tones should be done in a phase-continuous manner.
