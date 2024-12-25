
#include <Arduino.h>
#include <stdlib.h>
// #include <cstring>
#include "new_functions.h"
#include "print_functions.h"

// THIS IS CURRENTLY UNUSED

extern bool VERBY[10];
extern const int PLL_CALC_SHIFT;
// I was playing with R output divisor, but you can see there's no benefit to
// making this 1,2,4 for divide-by-2,4,8. 0 causes divide-by-1
extern const uint8_t R_DIVISOR_SHIFT;
// I use a 26Mhz tcxo
extern uint32_t SI5351_TCXO_FREQ;

//****************************************************
// I do ppb correction on the tcxo from a user-entered configuration
// SI5351_TCXO_FREQ = doCorrection(SI5351_TCXO_FREQ);

// https://math.stackexchange.com/questions/4302370/how-do-you-find-good-rational-approximations-to-a-decimal-number
// https://groups.io/g/picoballoon/topic/110236980
// Usual way to find the best rational approximations is continued fractions. 
// https://mathcenter.oxford.emory.edu/site/math125/continuedFractions/

// Converting Rational Values to Continued Fractions
// Every real number can be written as a simple continued fraction.
// proof here:
// https://www.quora.com/Can-all-numbers-be-written-as-continued-fractions

// The inverse algorithm to produce the cfe from a rational number. 
// Euclidean algorithm:
// 1) Find the integer part.
// 2) subtract the integer part from the original fraction to obtain a remainder (expressed as a fraction).
// 3) repeating this process. 
// 4) The process terminates for every rational number, but we can have early outs for our goals
// and si5351a programming constraints

// https://blogs.sas.com/content/iml/2018/09/06/continued-fraction-rational-numbers.html
// algo: https://hsinhaoyu.github.io/cont_frac/

// Interesting that our numerator and denominator is constrained from 0 to 1048575
// I suppose just generate and check for error afterwards. (or early out on running fraction sum?)
// Likely will get good-enough answer if we limit iterations?

//****************************************************

// Find the quotient and remainder of a rational number.
void qr(uint64_t *q, uint64_t *r, uint64_t a, uint64_t b, uint8_t iteration) {
    // a: The numerator of the rational number
    // b: The denominator of the rational number
    // returns: q quotient, r remainder
    // i.e. a = (b * q) + r; return (q, r).

    // integer floor divide
    uint64_t q_here = (a / b);     // the integer quotient
    uint64_t r_here = a - (b * q_here); // the integer remainder
    *q = q_here;
    *r = r_here;
}

// Turn a rational number into a continued fraction.
void r2cf( uint64_t *last_denom, uint32_t *q, double rn) {
    // Euclidean algorithm.
    // param rn: The rational number
    // returns: iterator of the old denominator b and the quotient q
    // maximum of 5 iterations

    // If your number is W.D (Whole.Decimal)
    // To get W just do (int)W.D. To get D you can do W.D - (int) W.D
    double a = (uint64_t) rn;
    a = (double) a;
    double b = rn - a;
    // integers that represent a and b, just getting 16 decimal digits
    // of precision for b
    // since b is double fp, shouldn't overflow
    uint64_t a_int = (uint64_t) a;
    uint64_t b_int = (uint64_t) (b * 1e16); 

    uint64_t q_int;
    uint64_t r_int;
    uint8_t iter;
    for (iter = 1; iter < 5; iter++) {
	    qr(&q_int, &r_int, a_int, b_int, iter);
        V1_printf(" q_int %" PRIu64 " r_int %" PRIu64 " a_int %" PRIu64 " b_int %" PRIu64 " iter %u" EOL,
            q_int, r_int, a_int, b_int, iter);

	    // if (r == 0.0) break;
        // Instead: break if r is 'close enough'
        // We started with 16 decimal digits in r_int
        // break if just 8 decimal digits
	    if (r_int < 100000000) {
            V1_printf("Stopping on iter %u because r_int %" PRIu64 EOL, iter, r_int);
            break;
        }
	    a_int = b_int;
        b_int = r_int;
        iter++;
        // hmm. should we exactly sum the continued fractions (to a new fraction)
        // and stop when the denominator is past our legal max for si5351a?
        // we don't really need to keep the list of continued fractions?
    }
    *last_denom = b_int; // integer
    *q = q_int; // integer
    
}

//****************************************************
// experimental: new algo,
// The desired freq input here is freq_x128, the desired symbol freq
// shifted left by PLL_CALC_SHIFT. Probably should have a better variable name,
// because x128 only makes sense for PLL_CALC_SHIFT=7
//
// If the param denom_to_use is non-zero, use it rather than re-calculating, to save time
// the initial calc that generates it, will be done during vfo turn-on time, so won't
// affect the latency for starting symbol rf from the pwm interrupt time synchronization.
// hopefully all 4 symbols can use the same denom? (could save 4 denoms?)
void new_vfo_calc(double *actual, double *actual_pll_freq,
    uint32_t *ms_div, uint32_t *pll_mult, uint32_t *pll_num, uint32_t *pll_denom,
    uint32_t *r_divisor, uint32_t freq_x128, uint32_t use_this_denom) {

    // const int PLL_MAX_FREQ  = 900000000;
    // const int PLL_MIN_FREQ  = 600000000;

    uint64_t PLL_FREQ_TARGET;
    PLL_FREQ_TARGET  = 900000000;

    uint64_t PLL_DENOM;
    if (use_this_denom != 0) {
        PLL_DENOM = use_this_denom;
    } else if (true) {
        const uint64_t PLL_DENOM_MAX = 0x000fffff; // 1048575
        PLL_DENOM = PLL_DENOM_MAX;
    } else {
        // this was sort of okay on 12/21/24. could it be better with PLL_DENOM_MAX above though?
        PLL_DENOM = 1000000;
    }

    uint64_t PLL_DENOM_x128 = PLL_DENOM << PLL_CALC_SHIFT;

    // we hardwire in a divide-by-4 in the R0 and R1 output dividers
    // so this ms_div is 1/4th what it would be for a divide-by-1 R0 and R1
    // the << 2 in the divisor
    uint64_t ms_div_here = 1 + (
        (PLL_FREQ_TARGET << PLL_CALC_SHIFT) /
        ((uint64_t)freq_x128 << R_DIVISOR_SHIFT)
        );
    ms_div_here &= 0xfffffffe;   // make it even number

    if (ms_div_here < 4 || ms_div_here > 900)
        V1_printf("ERROR: ms_div %" PRIu64 " is out of range 4 to 900" EOL, ms_div_here);

    // NEW: *4 for the R0 and R1 output divider. don't lose bits beyond 32-bits
    uint64_t pll_freq_x128 = ((uint64_t)freq_x128 * ms_div_here) << R_DIVISOR_SHIFT;
    // this is just integer. not useful!
    uint64_t pll_freq_here = pll_freq_x128 >> PLL_CALC_SHIFT;

    uint64_t tcxo_freq = (uint64_t) SI5351_TCXO_FREQ;  // 26 mhz?
    uint64_t tcxo_freq_x128 = tcxo_freq << PLL_CALC_SHIFT;

    // remember: floor division (integer)
    // tcxo_freq is integer..
    uint64_t pll_mult_here = pll_freq_x128 / tcxo_freq_x128;

    // mult has to be in the range 15 to 90
    if (pll_mult_here < 15 || pll_mult_here > 90) {
        V1_printf("ERROR: pll_mult %" PRIu64 " is out of range 15 to 90" EOL, pll_mult_here);
        V1_printf("integer pll_freq_here %" PRIu64 " tcxo_freq %" PRIu64 EOL, pll_freq_here, tcxo_freq);
    }

    // pll_num can be 0 to 1048575
    // it's interesting these are done in the non-scaled domain (not *128)
    // since pll_freq_here is what we want to get to, shouldn't we be scaled here?
    uint64_t pll_remain_x128 = pll_freq_x128 - (pll_mult_here * tcxo_freq_x128);
    uint64_t pnh_x128 = (pll_remain_x128 * PLL_DENOM_x128) / tcxo_freq_x128;
    // here's how we add 0.5 (in the scaled domain) to get rounding effect before shift down
    // the r divisor reduces
    uint64_t pll_num_x128 = pnh_x128 + (1 << (PLL_CALC_SHIFT - 1));
    uint64_t pll_num_here = pll_num_x128 >> PLL_CALC_SHIFT;
    if (pll_num_here > 1048575)
        V1_printf("ERROR: pll_num %" PRIu64 " is out of range 0 to 1048575" EOL, pll_num_here);

    // https://rfzero.net/tutorials/si5351a/
    double actual_pll_freq_here = (double)tcxo_freq *
        ((double)pll_mult_here + ((double)pll_num_here / (double)PLL_DENOM));

    // note we return a double here...only for printing
    double actual_here = actual_pll_freq_here / (double)(ms_div_here << R_DIVISOR_SHIFT);

    // output so we can print or use
    *ms_div    = (uint32_t)ms_div_here;
    *pll_mult  = (uint32_t)pll_mult_here;
    *pll_num   = (uint32_t)pll_num_here;
    *pll_denom = (uint32_t)PLL_DENOM;
    *r_divisor = (uint32_t)pow(2, R_DIVISOR_SHIFT);
    *actual = actual_here;
    *actual_pll_freq = actual_pll_freq_here;
}

