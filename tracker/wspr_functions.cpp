// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "defines.h"
#include "debug_functions.h"
#include "print_functions.h"
// https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/
#include "hardware/pwm.h"
#include "wspr_functions.h"

extern bool VERBY[10];
extern const uint32_t INTERRUPTS_PER_SYMBOL;

const int WSPR_PWM_SLICE_NUM = 4;

extern bool proceed;

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

void calcPwmDivAndWrap(uint32_t *PWM_DIV, uint32_t *PWM_WRAP_CNT, uint32_t INTERRUPTS_PER_SYMBOL, uint32_t PLL_SYS_MHZ) {
    if (VERBY[0])
        Serial.printf("calcPwmDivAndWrap START for INTERRUPTS_PER_SYMBOL %lu PLL_SYS_MHZ %lu" EOL,
        INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);
        // do a StampPrintf, so we can measure usec duration from here to the first symbol

    // FIX! is float enough precision for odd pico frequencies?
    // could check if PLL_SYS_PERIOD is integer aligned?
    const float DESIRED_SECS = 110.592;

    // how many interrupts per total delay?
    // the wrap count is limited to 16 bits
    // this is enough to make it work. Can't make it work with just 1 interrupt
    float interrupts_per_symbol = (float) INTERRUPTS_PER_SYMBOL;

    // interesting frequencies seem to find solutions with 250 divider. and 8 interrupts
    // 1/mhz is microseconds (1e-6)
    float PLL_SYS_USECS = 1.0 / (float)PLL_SYS_MHZ;
    float wrap_cnt_float;
    int wrap_cnt;
    float totalSymbolsTime;

    // we use 250 div now for 125 mhz so check with that first
    // per https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h#L156
    // can't be bigger than 255? or is 256 legal? 256 wouldn't make sense? 
    // static inline void pwm_config_set_clkdiv_int(pwm_config *c, uint div) {
    // valid_params_if(HARDWARE_PWM, div >= 1 && div < 256);
    int DIV_MIN = 250;
    int DIV_MAX = 255;

    int div;
    for (div = DIV_MIN; div <= DIV_MAX; div++) {
        float div_float = (float)div;
        float timePerWrap = 162 * interrupts_per_symbol * div_float * PLL_SYS_USECS * 1e-6;
        float possible_wrap_cnt = DESIRED_SECS / timePerWrap;


        Serial.printf("possible_wrap_cnt %.f div %d" EOL, possible_wrap_cnt, div);
        wrap_cnt_float = possible_wrap_cnt;
        // does a floor..use that and see what we get for total time!
        wrap_cnt = (int) wrap_cnt_float;

        // wrap_cnt can't be too big (how big can the pwm thing count?  16-bit limit?
        if (wrap_cnt >= (1 << 16)) continue;

        // does this come close enough to total time for all symbols
        float symbolTime = INTERRUPTS_PER_SYMBOL * div_float * PLL_SYS_USECS * 1e-6 * (float) wrap_cnt;
        totalSymbolsTime = 162 * symbolTime;
        Serial.printf("totalSymbolsTime %.3f wrap_cnt %d div %d" EOL, 
            totalSymbolsTime, wrap_cnt, div);

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
        *PWM_DIV = (uint32_t)div;
        *PWM_WRAP_CNT = (uint32_t)wrap_cnt;
        Serial.printf("GOOD: Found a good div and wrap_cnt for PLL_SYS_MHZ %lu PLL_SYS_USECS %.7f" EOL,
            PLL_SYS_MHZ, PLL_SYS_USECS);
        Serial.printf("GOOD: PLL_SYS_MHZ %lu PWM_DIV %d PWM_WRAP_CNT %d" EOL, PLL_SYS_MHZ, div, wrap_cnt);
        Serial.printf("GOOD: totalSymbolsTime %.3f" EOL, totalSymbolsTime);
    }
    if (VERBY[0])
        Serial.printf("calcPwmDivAndWrap END for INTERRUPTS_PER_SYMBOL %lu PLL_SYS_MHZ %lu" EOL,
        INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);
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

//*******************************************************
// How is this aligned to 1 sec in off the starting minute?
// can't have parameters?
void PWM4_Handler() {
    // too much output!
    // if (VERBY[0]) Serial.println(F("PWM4_Handler() START"));
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    static uint32_t interrupt_cnt = 0;

    if (++interrupt_cnt >= INTERRUPTS_PER_SYMBOL) {
        // every 500 times PWM4_Handler is called, we toggle proceed to true?
        // is that 500 interrupts?
        interrupt_cnt = 0;
        // if (VERBY[0]) Serial.print(".");
        proceed = true;
    }
    // if (VERBY[0] && ((cnt % 100)==0)) Serial.print(".");
    // if (VERBY[0]) Serial.println(F("PWM4_Handler() END"));
}

//*******************************************************
void setPwmDivAndWrap(uint32_t PWM_DIV, uint32_t PWM_WRAP_CNT) {
    // valid_params_if(HARDWARE_PWM, div >= 1 && div < 256);
    if (PWM_DIV >= 256)
        Serial.printf("ERROR: illegal PWM_DIV  %lu is > 256 ..drops upperr bits?" EOL, PWM_DIV);
    if (PWM_DIV ==  0)
        Serial.print(F("ERROR: illegal PWM_DIV is 0" EOL)); // 0 should be illegal also?

    // gets passed as uint16_t
    if (PWM_WRAP_CNT >= (1 << 16))
        Serial.printf("ERROR: illegal PWM_WRAP_CNT %lu is > 2**16 ..drops upper bits?" EOL, PWM_WRAP_CNT);
    if (PWM_WRAP_CNT == 0 )
        Serial.print(F("ERROR: illegal PWM_WRAP_CNT is 0" EOL));

    // side-note: per:
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h

    // don't need to program this.. default is always-on (free-running PWM)
    // static inline void pwm_config_set_clkdiv_mode(pwm_config *c, enum pwm_clkdiv_mode mode) {
    // valid_params_if(HARDWARE_PWM, mode == PWM_DIV_FREE_RUNNING ||
    // Configure which event gates the operation of the fractional divider.
    // The default is always-on (free-running PWM). 

    // FIX! we could do bounds checking? or ??
    // assumes the values are right for INTERRUPTS_PER_SYMBOL 
    // which is now 8 (instead of 500)
    // if (VERBY[0]) Serial.println(F("zeroTimerSetPeriodMs() START"));

    static pwm_config wspr_pwm_config = pwm_get_default_config();
    // 250 clocks at 125Mhz .008 uSec per clock, is 250 * .008 = 2 uSec
    // 2uS (at 125Mhz? does it need adjusting at other clks?)
    // so if we're count at 2uSec intervals to wrap at 683 millis.. that's 500*683 = 341500 counts?

    // the 250 can be a uint? so can be pretty big? Can we make it 5 times bigger (1450 or maybe 1500)
    // to bring down the sie of the wrap count
    // or should we count 8 interrupts, and adjust for 1/8th the total target

    // per https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h#L156
    // can't be bigger than 256!
    // static inline void pwm_config_set_clkdiv_int(pwm_config *c, uint div) {
    // valid_params_if(HARDWARE_PWM, div >= 1 && div < 256);
    pwm_config_set_clkdiv_int(&wspr_pwm_config, PWM_DIV); // takes uint?

    // Set the highest value the counter will reach before returning to 0. Also known as TOP.
    // this will be the tone_delay we're called with
    // FIX! this doesn't seem right. t should wrap at 683 millis, but * 500 ?
    // so instead of getting 500 interrupts, we get one interrupt?
    // also, subtracting 1 here is ??

    // if div is bigger we could have just 1 interrupt per symbol?
    // less extra processing (interrupts) during symbol transmission?
    pwm_config_set_wrap(&wspr_pwm_config, ((uint16_t)PWM_WRAP_CNT - 1));
    pwm_init(WSPR_PWM_SLICE_NUM, &wspr_pwm_config, false);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, PWM4_Handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
    // if (VERBY[0]) Serial.println(F("zeroTimerSetPeriodMs() END"));
}

//*******************************************************
void disablePwmInterrupts(void) {
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    irq_set_enabled(PWM_IRQ_WRAP, false);
    irq_remove_handler(PWM_IRQ_WRAP, PWM4_Handler);
}
