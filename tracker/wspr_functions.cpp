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

extern volatile bool proceed;

// The protocol specification states:
// Each tone should last for 8192/12000 = 0.682666667 seconds, and transitions between
// tones should be done in a phase-continuous manner.

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
// FIX! what about just using hardware timers on pi pico. and migrating away
// from the PWM interrupt timer. 
// https://reference.arduino.cc/reference/en/libraries/rpi_pico_timerinterrupt
// https://github.com/khoih-prog/RPI_PICO_TimerInterrupt
// This library enables you to use Interrupt from Hardware Timers on on RP2040-based boards 
// such as RASPBERRY_PI_PICO, using Earle Philhower's arduino-pico core. 

// As Hardware Timers are rare, and very precious assets of any board, 
// this library now enables you to use up to 16 ISR-based Timers, 
// while consuming only 1 Hardware Timer. Timers' interval is very long (ulong millisecs).

// Now with these new 16 ISR-based timers, the maximum interval 
// is practically unlimited (limited only by unsigned long milliseconds) 
// while the accuracy is nearly perfect compared to software timers.
// The most important feature is they're ISR-based timers. 
// Therefore, their executions are not blocked by bad-behaving functions / tasks. 
// This important feature is absolutely necessary for mission-critical tasks.
// Functions using normal software timers, relying on loop() and calling millis(), 
// won't work if the loop() or setup() is blocked by certain operation. 

// The catch is your function is now part of an ISR (Interrupt Service Routine), 
// and must be lean / mean, and follow certain rules.
// mportant Notes about ISR
// Inside the attached function, delay() won’t work and the value returned by millis() will not increment. 
// Serial data received while in the function may be lost. 
// You should declare as volatile any variables that you modify within the attached function.

// Typically global variables are used to pass data between an ISR and the main program. 
// To make sure variables shared between an ISR and the main program are updated correctly, 
// declare them as volatile.

//*******************************************************

// How is this aligned to 1 sec in off the starting minute?
// can't have parameters?
static uint32_t pwm_interrupt_cnt = 0;
static uint32_t pwm_interrupt_total_cnt = 0;
void PWM4_Handler() {
    // too much output!
    // if (VERBY[0]) Serial.println(F("PWM4_Handler() START"));
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    pwm_interrupt_cnt++;
    pwm_interrupt_total_cnt++;
    if (pwm_interrupt_cnt >= INTERRUPTS_PER_SYMBOL) {
        // every 500 times PWM4_Handler is called, we toggle proceed to true?
        // is that 500 interrupts?
        pwm_interrupt_cnt = 0;
        // symbol rate is approximately 1.4648 baud (4fsk per sec), or exactly 12,000 Hz / 8192.
        // 0.6826 secs per symbol?

        proceed = true;
    }
    // FIX! remove this. redundant and slows down gettng to 'proceed' in tracker.ino
    if (false and VERBY[0]) {
        uint32_t pwm_interrupt_cnt_162 = pwm_interrupt_total_cnt % 162;
        if ((pwm_interrupt_total_cnt % 10) == 0) 
            StampPrintf("sym: %lu %lu" EOL, pwm_interrupt_cnt_162, pwm_interrupt_cnt);

        if ((pwm_interrupt_total_cnt % 162) == 161) {
            StampPrintf("sym: %lu %lu" EOL, pwm_interrupt_cnt_162, pwm_interrupt_cnt);
            DoLogPrint();
        }
    }
    // if (VERBY[0]) Serial.println(F("PWM4_Handler() END"));

}

//*******************************************************
void setPwmDivAndWrap(uint32_t PWM_DIV, uint32_t PWM_WRAP_CNT) {
    // valid_params_if(HARDWARE_PWM, div >= 1 && div < 256);
    if (PWM_DIV >= 256)
        Serial.printf("ERROR: illegal PWM_DIV  %lu is > 256 ..drops upper bits?" EOL, PWM_DIV);
    if (PWM_DIV ==  0)
        Serial.print(F("ERROR: illegal PWM_DIV is 0" EOL)); // 0 should be illegal also?

    // gets passed as uint16_t
    if (PWM_WRAP_CNT >= (1 << 16))
        Serial.printf("ERROR: illegal PWM_WRAP_CNT %lu is > 2**16 ..drops upper bits?" EOL, PWM_WRAP_CNT);
    if (PWM_WRAP_CNT == 0 )
        Serial.print(F("ERROR: illegal PWM_WRAP_CNT is 0" EOL));
    Serial.flush();

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

    // global state that's changed on interrupts
    pwm_interrupt_cnt = 0;
    pwm_interrupt_total_cnt = 0;

    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
    // does this start it counting to interrupt and toggle 'proceed' after completion of first symbol?
    // first symbol could be "short" as a result (the extra delay in getting first frequency setup)
    // setup the base frequency earlier, so we don't see this extra variation on first symbol!
    // due to the "change" optimization in the i2c writes.
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
    // if (VERBY[0]) Serial.println(F("zeroTimerSetPeriodMs() END"));
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
    // per 
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pwm/include/hardware/pwm.h#L156
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

