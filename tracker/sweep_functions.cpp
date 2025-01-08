// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <stdlib.h>
#include <cstring>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

#include "si5351_functions.h"
#include "print_functions.h"
#include "led_functions.h"
#include "u4b_functions.h"

//****************************************************************************
extern bool VERBY[10];
extern uint32_t XMIT_FREQUENCY;
extern char _Band[3];  // string with 10, 12, 15, 17, 20 legal. null at end
extern char _U4B_chan[4];  // string with 0-599

extern const uint64_t PLL_CALC_SHIFT;
extern uint64_t PLL_FREQ_TARGET;

extern uint32_t PLL_DENOM_OPTIMIZE;

void si5351a_calc_sweep(void) {
    V1_print(F("si5351a_calc_sweep START" EOL));
    // FIX! do we need this
    vfo_calc_cache_flush();

    // global XMIT_FREQUENCY should already be set for band, channel?
    uint32_t xmit_freq = XMIT_FREQUENCY;
    V0_printf("band %s channel %s xmit_freq %lu" EOL, _Band, _U4B_chan, xmit_freq);
    double symbol0desired = calcSymbolFreq(xmit_freq, 0, true);  // print

    V1_printf("Now: sweep calc 5351a programming starting at %.6f" EOL, symbol0desired);
    V1_print(F(EOL "partial sweep at 0.25hz increment" EOL));
    uint32_t pll_num_last = 0;
    // integer start freq.
    // should be no fractional part in the double? (since it's the base symbol 0 freq?)
    // start at middle of the bin, u4b channel 0, symbol 0
    uint32_t freq = (uint32_t) symbol0desired;
    uint64_t freq_xxx;

    double actual;
    double actual_pll_freq;
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num;
    uint32_t pll_denom;
    uint32_t r_divisor;
    // sweep 200 * 0.25 hz = 50hz (1/4th the passband)
    // sweep 80 * 1 hz = 80hz
    for (int i = 0; i < 80; i++) {
        if (false) {
            // use this for 0.25 steps
            freq_xxx = (freq << PLL_CALC_SHIFT) +
                (i << (PLL_CALC_SHIFT - 2));  // + i/4
            switch (i % 4) {
                case 0: break;
                // adds 0.25 (shifted)
                case 1: freq_xxx += ((1 << PLL_CALC_SHIFT) >> 2); break;
                // adds 0.50 (shifted)
                case 2: freq_xxx += ((2 << PLL_CALC_SHIFT) >> 2); break;
                // adds 0.75 (shifted)
                case 3: freq_xxx += ((3 << PLL_CALC_SHIFT) >> 2); break;
            }
        } else {
            // use for 1.0 steps
            freq_xxx = (freq + i) << PLL_CALC_SHIFT;  // + i
        }

        // note this will include any correction to SI5351_TCXO_FREQ (already done)
        // everything is 32 bit in and out of this, but 64-bit calcs inside it.
        vfo_calc_div_mult_num(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num, &pll_denom, &r_divisor,
            freq_xxx, true); // FIX! do_farey for now

        V1_printf("actual %.6f actual_pll_freq %.6f", actual, actual_pll_freq);
        V1_printf(" pll_mult %lu pll_num %lu pll_denom %lu ms_div %lu r_divisor %lu",
            pll_mult, pll_num, pll_denom, ms_div, r_divisor);

        // not so good if two pll_nums are the same (sequentially)
        if (pll_num == pll_num_last) {
            // V1_print(F("UNDESIREABLE: pll_num and pll_num_last same"));
        }
        pll_num_last = pll_num;
    }
    V1_print(F(EOL));
    // FIX! maybe not needed if we didn't load anything into the cache
    vfo_calc_cache_flush();
    V1_print(F("si5351a_calc_sweep END" EOL));
}
//****************************************************************************
// does this work? no. Hans method on 144Mhz is to fix the numerator and
// step the denominator causes non-uniform symbol shift
// alternate way to get steps of 1 on 144Mhz
// with 26Mhz
// c = 26e6 / 1.4648 in a + b/c equation
// 17749863. too large
// or  to require 3 steps:
// c = (3 * 26e6) / 1.4648 in a + b/c equation

// kbn: with the added choice of 1, 2 or 3 numerator steps
// to get the desired single wspr transition
// per: https://groups.io/g/QRPLabs/topic/si5351a_issues_with_frequency/96467329
//
// divide the XTAL frequency by my output divider and then divide again
// by my desired step,
// wpsr: 1.4648 and use that number for my value for c
// in the a + b/c equation.
// What this does is make each increment of b  in the equation result in
// the output frequency changing by the desired step,
// and then I have manipulated b directly to send the WSPR signals.

//****************************************************************************
// In your case if you divide 25mhz by 6 and then by 6.66666667
// you get the somewhat magic value (rounded) of 625000.

// 0.6080016 * 625000 is 380001 exactly.

// Downside of this method is that setting your base frequency will have error,
// but the steps up from that frequency will be accurate.
// (must be careful using this approach  as c needs to be below 1048575,
// but works for 6.6667 steps on 144mhz ).
//
// Looked at the Adafruit library. Interesting way to approach things.
// It does use float calculations for the divisions so
// I think it will have some error.
//
// My WSPR code using this method is here: https://github.com/roncarr880/QRP_LABS_WSPR
// The Si5351 routines would need changes to work on 144mhz
// with the fixed divider of 6.
// Project also used an R divider for the transmit. Would need to be removed.

//*********************************************************************************
// sweep the bands for the current PLL_FREQ_TARGET to get the mult/div
// for the spreadsheet to compute optimum denom for the PLL_FREQ_TARGET
void si5351a_calc_sweep_band() {
    V1_print(F("si5351a_calc_sweep_band START" EOL));
    double actual;
    double actual_pll_freq;
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num_here;
    uint32_t pll_denom;
    uint32_t r_divisor;
    uint64_t freq_xxx;

    char band[3];

    // FIX! do we need this?
    vfo_calc_cache_flush();
    for (uint8_t i = 0; i <= 4 ; i++) {
        switch (i) {
            case 0: snprintf(band, sizeof(band), "10"); break;
            case 1: snprintf(band, sizeof(band), "12"); break;
            case 2: snprintf(band, sizeof(band), "15"); break;
            case 3: snprintf(band, sizeof(band), "17"); break;
            case 4: snprintf(band, sizeof(band), "20"); break;
        }
        // will pick the _lane we're using for the current u4b channel config
        uint8_t symbol = 0;
        char lane[2] = { 0 };  // '1', '2', '3', '4'
        lane[0] = '1';  // first freq bin

        set_PLL_DENOM_OPTIMIZE(band);
        uint32_t xmit_freq = init_rf_freq(band, lane);
        calcSymbolFreq_xxx(&freq_xxx, xmit_freq, symbol);
        // This will use the current PLL_DENOM_OPTIMIZE now in its calcs?
        vfo_calc_div_mult_num(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num_here, &pll_denom, &r_divisor,
            freq_xxx, true); // FIX! do_farey for now

        V1_print(F(EOL));
        V1_printf("sweep band %s xmit_freq %lu PLL_FREQ_TARGET %" PRIu64 " r_divisor %lu" EOL,
            band, xmit_freq, PLL_FREQ_TARGET, r_divisor);
        V1_printf("sweep band %s lane %s symbol %u", band, lane, symbol);
        V1_printf(" pll_mult %lu ms_div %lu actual_pll_freq %.6f" EOL,
            pll_mult, ms_div, actual_pll_freq);
        V1_printf("sweep band %s lane %s symbol %u", band, lane, symbol);
        V1_printf(" pll_num %lu pll_denom %lu actual %.6f" EOL,
            pll_num_here, pll_denom, actual);
        V1_print(F(EOL));
    }
    // FIX! do we need this if we didn't load anything into the cache?
    vfo_calc_cache_flush();
    V1_print(F("si5351a_calc_sweep_band END" EOL));
}
//*********************************************************************************
void si5351a_calc_optimize(double *symbolShiftError, double *symbolAbsoluteError,
    uint32_t *pll_num, bool print) {
    V1_print(F(EOL "si5351a_calc_optimize START" EOL));
    // don't flush the VCC cache here, so we keep the results for the 4 symbols

    // just to see what we get, calculate the si5351 stuff for
    // all the 0.25 Hz variations for possible tx in a band.
    // all assuming u4b channel 0 freq bin.
    // stuff that's returned by vfo_calc_div_mult_num()
    double actual;
    double actual_pll_freq;
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num_here;
    uint32_t pll_denom;
    uint32_t r_divisor;

    // stuff that's input to vfo_calc_div_mult_num()
    uint64_t freq_xxx;
    // should already be set for band, channel? (XMIT_FREQUENCY)
    uint32_t xmit_freq = XMIT_FREQUENCY;
    if (print) {
        V0_printf("band %s channel %s xmit_freq %lu" EOL, _Band, _U4B_chan, xmit_freq);
    }

    // compute the actual shifts too, which are the more important thing
    // as opposed to actual freq (since tcxo causes fixed error too
    // Assume no drift thru the tx.
    // this is floats, because it's for printing only (accuracy)
    double symbol0desired = calcSymbolFreq(xmit_freq, 0, false);  // no print
    double symbol1desired = calcSymbolFreq(xmit_freq, 1, false);  // no print
    double symbol2desired = calcSymbolFreq(xmit_freq, 2, false);  // no print
    double symbol3desired = calcSymbolFreq(xmit_freq, 3, false);  // no print
    // will give the freq you should see on wsjt-tx if hf_freq is the xmit_freq
    // for a channel. symbol can be 0 to 3.
    // Can subtract 20 hz to get the low end of the bin
    // (assume freq calibration errors of that much, then symbol the 200hz passband?

    // in calcSymbolFreq(), could compare these offsets from the symbol0desired to expected?
    // (offset 1.46412884334 Hz)
    if (print) {
        V1_print(F(EOL));
        // + 0 Hz
        // +1*(12000/8192) Hz [1.4648 Hz]
        // +2*(12000/8192) Hz [2.9296 Hz]
        // +3*(12000/8192) Hz [4.3945 Hz]
        V1_printf("band %s channel %s desired symbol 0 freq %.6f" EOL,
            _Band, _U4B_chan, symbol0desired);
        V1_printf("band %s channel %s desired symbol 1 freq %.6f" EOL,
            _Band, _U4B_chan, symbol1desired);
        V1_printf("band %s channel %s desired symbol 2 freq %.6f" EOL,
            _Band, _U4B_chan, symbol2desired);
        V1_printf("band %s channel %s desired symbol 3 freq %.6f" EOL,
            _Band, _U4B_chan, symbol3desired);
        V1_print(F(EOL));
    }

    // check what pll_num gets calced in the freq_xxx (shifted) domain
    // and also, the fp respresentation (actual) of the actual frequency after /128
    // of the *128 'shifted domain' integer representation
    // actual returned is now a double
    double symbol0actual;
    double symbol1actual;
    double symbol2actual;
    double symbol3actual;
    for (uint8_t symbol = 0; symbol <= 3; symbol++) {
        calcSymbolFreq_xxx(&freq_xxx, xmit_freq, symbol);
        // This will use the current PLL_DENOM_OPTIMIZE now in its calcs?
        // or will use Farey if that's enabled
        vfo_calc_div_mult_num(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num_here, &pll_denom, &r_divisor,
            freq_xxx, true);

        if (print) {
            V1_printf("channel %s symbol %u", _U4B_chan, symbol);
            V1_printf(" actual %.6f actual_pll_freq %.6f", actual, actual_pll_freq);
            V1_printf(" pll_mult %lu pll_num %lu pll_denom %lu ms_div %lu r_divisor %lu" EOL,
                pll_mult, pll_num_here, pll_denom, ms_div, r_divisor);
        }
        switch (symbol) {
            case 0: symbol0actual = actual; break;
            case 1: symbol1actual = actual; break;
            case 2: symbol2actual = actual; break;
            case 3: symbol3actual = actual; break;
        }
    }

    if (print) {
        V1_print(F(EOL));
        V1_print(F("Showing shifts in symbol frequencies, rather than absolute error" EOL));
        V1_printf("channel %s symbol 0 actual %.6f" EOL,
            _U4B_chan, symbol0actual);
        V1_printf("channel %s symbol 1 actual %.6f shift0to1 %.6f Hz" EOL,
            _U4B_chan, symbol1actual, symbol1actual - symbol0actual);
        V1_printf("channel %s symbol 2 actual %.6f shift0to2 %.6f Hz" EOL,
            _U4B_chan, symbol2actual, symbol2actual - symbol0actual);
        V1_printf("channel %s symbol 3 actual %.6f shift0to3 %.6f Hz" EOL,
            _U4B_chan, symbol3actual, symbol3actual - symbol0actual);
    }

    // just one absolute error

    // use the biggest absolute Error
    double symbolAbsoluteError_0 = abs(symbol0actual - symbol0desired);
    double symbolAbsoluteError_1 = abs(symbol1actual - symbol1desired);
    double symbolAbsoluteError_2 = abs(symbol2actual - symbol2desired);
    double symbolAbsoluteError_3 = abs(symbol3actual - symbol3desired);

    double symbolAbsoluteError_here = symbolAbsoluteError_0;
    if (symbolAbsoluteError_1 > symbolAbsoluteError_here)
        symbolAbsoluteError_here = symbolAbsoluteError_1;
    if (symbolAbsoluteError_2 > symbolAbsoluteError_here)
        symbolAbsoluteError_here = symbolAbsoluteError_2;
    if (symbolAbsoluteError_3 > symbolAbsoluteError_here)
        symbolAbsoluteError_here = symbolAbsoluteError_3;

    // just 0->1 0->2 0->3 incremental shift errors. compared to expected 12000/8192
    double expectedShift = 12000.0 / 8192.0;
    // assume it's a positive shift

    // just look at adjacent shifts
    double symbolShiftError_1 = abs(symbol1actual - symbol0actual) - expectedShift;
    double symbolShiftError_2 = abs(symbol2actual - symbol1actual) - expectedShift;
    double symbolShiftError_3 = abs(symbol3actual - symbol2actual) - expectedShift;
    V1_print(F(EOL));
    V1_printf("Expected shift 0 to 1: %.6f Hz" EOL, 1 * expectedShift);
    V1_printf("Expected shift 0 to 2: %.6f Hz" EOL, 2 * expectedShift);
    V1_printf("Expected shift 0 to 3: %.6f Hz" EOL, 3 * expectedShift);

    double symbolShiftError_here = symbolShiftError_1;
    if (symbolShiftError_2 > symbolShiftError_here)
        symbolShiftError_here = symbolShiftError_2;
    if (symbolShiftError_3 > symbolShiftError_here)
        symbolShiftError_here = symbolShiftError_3;

    *symbolShiftError = symbolShiftError_here;
    *symbolAbsoluteError = symbolAbsoluteError_here;
    *pll_num = pll_num_here;

    if (print) {
        V1_print(F(EOL));
        V1_printf("symbolAbsoluteError: %.6f Hz" EOL, symbolAbsoluteError_here);
        V1_printf("symbolShiftError: %.6f Hz" EOL, symbolShiftError_here);
        V1_print(F(EOL));
        V1_print(F("si5351a_calc_optimize END" EOL));
    }
    // no VCC cache flush here!
}

//*********************************************************************************
void si5351a_denom_optimize_search() {
    // no need to do the calcs if we're not going to print them!
    // the optimized values should be baked into si5351_functions.cpp()
    // so we don't have to calc. one magic denom per band?
    // check the freq bin boundaries. (channel 0 and 599?)
    if (true && VERBY[1]) {
        double symbolShiftError;
        double symbolAbsoluteError;
        double last_symbolShiftError = 1e6;
        double last_symbolAbsoluteError = 1e6;
        uint32_t last_PLL_DENOM_OPTIMIZE;
        uint32_t pll_num;
        uint32_t STEP;
        int sse;

        // Instead: use the best initial values per band in this function
        // (from spreadsheet or prior runs for a band)
        set_PLL_DENOM_OPTIMIZE(_Band);
        // print
        uint32_t default_PLL_DENOM_OPTIMIZE = PLL_DENOM_OPTIMIZE;
        si5351a_calc_optimize(&symbolShiftError, &symbolAbsoluteError, &pll_num, true);
        V1_printf("SEED values: PLL_DENOM_OPTIMIZE %lu pll_num %lu", PLL_DENOM_OPTIMIZE, pll_num);
        V1_printf(" symbolAbsoluteError %.8f symbolShiftError %.8f" EOL,
            symbolAbsoluteError, symbolShiftError);
        // check 4 digits of precision
        sse = (int)10000 * symbolShiftError;
        if (sse != 0) {
            V1_printf("WARN: SEED symbolShiftError != 0 to 4 digits of precision. sse %d" EOL, sse);
        }

        last_symbolShiftError = symbolShiftError;
        last_symbolAbsoluteError = symbolAbsoluteError;
        last_PLL_DENOM_OPTIMIZE = PLL_DENOM_OPTIMIZE;
        STEP = PLL_DENOM_OPTIMIZE >> 1; // divide-by-4
        V1_print(F(EOL));

        //**********
        uint8_t iter;
        int stepDir;
        // FIX! what's the max # of iterations that the algo could take over the range
        // have to account for some iters not changing the STEP !!
        for (iter = 1; iter <= 30; iter++) {
            if (STEP == 0) break;
            // iter 1. Assume convex curve on the error function, over the whole range?
            last_PLL_DENOM_OPTIMIZE = PLL_DENOM_OPTIMIZE;
            uint32_t PLL_DENOM_OPTIMIZE_pos;
            uint32_t PLL_DENOM_OPTIMIZE_neg;

            PLL_DENOM_OPTIMIZE_pos = PLL_DENOM_OPTIMIZE + STEP;
            if (PLL_DENOM_OPTIMIZE_pos > 1048575) PLL_DENOM_OPTIMIZE_pos = 1048575;
            if (STEP > PLL_DENOM_OPTIMIZE) PLL_DENOM_OPTIMIZE_neg = 0;
            else PLL_DENOM_OPTIMIZE_neg = PLL_DENOM_OPTIMIZE - STEP;

            // try positive step. one liner print
            V1_print(F("***********************"));
            V1_printf(" try PLL_DENOM_OPTIMIZE_pos %lu with pos STEP %lu", PLL_DENOM_OPTIMIZE_pos, STEP);
            V1_print(F(" ***********************" EOL));
            PLL_DENOM_OPTIMIZE = PLL_DENOM_OPTIMIZE_pos;
            si5351a_calc_optimize(&symbolShiftError, &symbolAbsoluteError, &pll_num, false);  // don't print
            V1_printf("best pll_num %lu for pll_denom %lu -> symbolAbsoluteError %.8f symbolShiftError %.8f" EOL,
                pll_num, PLL_DENOM_OPTIMIZE, symbolAbsoluteError, symbolShiftError);

            // both should improve
            stepDir = 0;
            // if ((symbolShiftError < last_symbolShiftError) && (symbolAbsoluteError < last_symbolAbsoluteError)) {
            if (symbolShiftError < last_symbolShiftError) {
                stepDir = 1;
                last_symbolShiftError = symbolShiftError;
                last_symbolAbsoluteError = symbolAbsoluteError;
            }
            // try negative step. one liner print
            V1_print(F("***********************"));
            V1_printf(" try PLL_DENOM_OPTIMIZE_neg %lu with neg STEP %lu", PLL_DENOM_OPTIMIZE_neg, STEP);
            V1_print(F(" ***********************" EOL));
            PLL_DENOM_OPTIMIZE = PLL_DENOM_OPTIMIZE_neg;
            si5351a_calc_optimize(&symbolShiftError, &symbolAbsoluteError, &pll_num, false);  // don't print
            V1_printf("best pll_num %lu for pll_denom %lu -> symbolAbsoluteError %.8f symbolShiftError %.8f" EOL,
                pll_num, PLL_DENOM_OPTIMIZE, symbolAbsoluteError, symbolShiftError);
            // both should improve
            // if ((symbolShiftError < last_symbolShiftError) && (symbolAbsoluteError < last_symbolAbsoluteError)) {
            if (symbolShiftError < last_symbolShiftError) {
                stepDir = -1;
                last_symbolShiftError = symbolShiftError;
                last_symbolAbsoluteError = symbolAbsoluteError;
            }
            if ((stepDir == 1) || (stepDir == -1)) {
                if (stepDir == 1) PLL_DENOM_OPTIMIZE = PLL_DENOM_OPTIMIZE_pos;
                else PLL_DENOM_OPTIMIZE = PLL_DENOM_OPTIMIZE_neg;
                V1_print(F(EOL));
                V1_printf("FOUND BETTER: iter %u stepDir %d PLL_DENOM_OPTIMIZE %lu last_symbolAbsoluteError %.8f" EOL,
                    iter, stepDir, PLL_DENOM_OPTIMIZE, last_symbolAbsoluteError);
                V1_printf("FOUND BETTER: iter %u stepDir %d PLL_DENOM_OPTIMIZE %lu last_symbolShiftError %.8f" EOL,
                    iter, stepDir, PLL_DENOM_OPTIMIZE, last_symbolShiftError);
                // note how we don't change the step size until we confirm we're stuck at the new place
            } else {
                // stepDir 0, stuck at this place, change step size
                PLL_DENOM_OPTIMIZE = last_PLL_DENOM_OPTIMIZE;
                V1_printf("iter %u stepDir %d PLL_DENOM_OPTIMIZE %lu" EOL,
                    iter, stepDir, PLL_DENOM_OPTIMIZE);
                // reduce the step size (1/2)
                STEP = STEP >> 1;
            }
        }

        //************************
        // Final report.
        V1_print(F(EOL "***********************"));
        PLL_DENOM_OPTIMIZE = last_PLL_DENOM_OPTIMIZE;
        V1_printf("BEST FOUND: PLL_DENOM_OPTIMIZE: %lu", PLL_DENOM_OPTIMIZE);
        V1_print(F(" ***********************" EOL));
        si5351a_calc_optimize(&symbolShiftError, &symbolAbsoluteError, &pll_num, true);  // print
        V1_printf("BEST FOUND: PLL_DENOM_OPTIMIZE %lu pll_num %lu", PLL_DENOM_OPTIMIZE, pll_num);
        V1_printf(" symbolAbsoluteError %.8f symbolShiftError %.8f" EOL,
            symbolAbsoluteError, symbolShiftError);

        // check 4 digits of precision
        sse = (int)10000 * symbolShiftError;
        if (sse != 0) {
            V1_printf("WARN: BEST FOUND symbolShiftError != 0 to 4 digits of precision. sse %d" EOL, sse);
        }

        if (PLL_DENOM_OPTIMIZE == default_PLL_DENOM_OPTIMIZE) {
            V1_printf("GOOD: couldn't improve on hard-wired PLL_DENOM_OPTIMIZE" EOL);
        }
        else {
            V1_printf("ERROR: shouldn't have improved on hard-wired PLL_DENOM_OPTIMIZE" EOL);
            V1_printf("ERROR: default_PLL_DENOM_OPTIMIZE %lu PLL_DENOM_OPTIMIZE %lu " EOL,
                default_PLL_DENOM_OPTIMIZE, PLL_DENOM_OPTIMIZE);
        }
    }
    // FIX! do we need this if we didn't load anything into the cache?
    vfo_calc_cache_flush();
}
//*********************************************************************************
