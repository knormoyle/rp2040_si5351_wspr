// Farey sequence-based rational approximation of numbers.
// Per Magnusson, 2024
// MIT licence, http://www.opensource.org/licenses/mit-license.php

// from
// https://axotron.se/blog/fast-algorithm-for-rational-approximation-of-floating-point-numbers/

#include <cstdint>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <random>
#include <chrono>
#include <math.h>

// one or the other. same methods/classes so can drop in
// if (DO_BEST_RAT) will evaluate true if 1
#define DO_BEST_RAT 0
#define DO_BEST_RAT_CONTINUED_FRACTIONS 0

#if DO_BEST_RAT_CONTINUED_FRACTIONS == 1
#include "ad_rat_by_cont_frac.cpp"
#else
#include "ad_rat_by_fast_farey.cpp"
#endif

using namespace std;

typedef struct {
  uint32_t numerator;
  uint32_t denominator;
  uint32_t iterations; // kbn
  double target; // kbn
  double actual_real; // kbn
  double actual_error; // kbn

  // for the continued fractions algo
  uint32_t niter;
  uint32_t n;
  uint32_t d;
  uint32_t error;

} rational_t;


// Find the best rational approximation to a number between 0 and 1.
//
// target - a number between 0 and 1 (inclusive)
// maxdenom - the maximum allowed denominator
//
// The algorithm is based on Farey sequences/fractions. See
// https://web.archive.org/web/20181119092100/https://nrich.maths.org/6596

// a, b, c, d notation from
// https://en.wikipedia.org/wiki/Farey_sequence is used here (not
// from the above reference). I.e. narrow the interval between a/b
// and c/d by splitting it using the mediant (a+c)/(b+d) until we are
// close enough with either endpoint, or we have a denominator that is
// bigger than what is allowed.

// Start with the interval 0 to 1 (i.e. 0/1 to 1/1).

// A simple implementation of just calculating the mediant (a+c)/(b+d) and
// iterating with the mediant replacing the worst value of a/b and c/d is very
// inefficient in cases where the target is close to a rational number
// with a small denominator, like e.g. when approximating 10^-6.

// The straightforward algorithm would need about 10^6 iterations as it
// would try all of 1/1, 1/2, 1/3, 1/4, 1/5 etc. To resolve this slow
// convergence, at each step, it is calculated how many times the
// interval will need to be narrowed from the same side and all those
// steps are taken at once.
rational_t rational_approximation(double target, uint32_t maxdenom)
{
  rational_t retval;
  double mediant;  // float does not have enough resolution
                      // to deal with single-digit differences
                      // between numbers above 10^8.
  double N, Ndenom, Ndenom_min;
  double epsilon = 1e-10; // To handle rounding issues in conjunction with floor
  uint32_t a = 0, b = 1, c = 1, d = 1, ac, bd, Nint;
  const int maxIter = 100;

  if(target > 1) {
    // Invalid
    retval.iterations = 0; // kbn
    retval.numerator = 1;
    retval.denominator = 1;
    return retval;
  }
  if(target < 0) {
    // Invalid
    retval.iterations = 0; // kbn
    retval.numerator = 0;
    retval.denominator = 1;
    return retval;
  }
  if(maxdenom < 1) {
    maxdenom = 1;
  }

  mediant = 0;
  Ndenom_min = 1/((double) 10*maxdenom);

  int ii = 0;
  // Farey approximation loop
  while(true) {
    ac = a+c;
    bd = b+d;
    if(bd > maxdenom || ii > maxIter) {
      // The denominator has become too big, or too many iterations.
      // Select the best of a/b and c/d.
      if(target - a/(double)b < c/(double)d - target) {
        ac = a;
        bd = b;
      } else {
        ac = c;
        bd = d;
      }
      if (true) {
        // kbn doesn't help
        // if (ii > maxIter) {
        printf("break 1\n");
        break;
      }
    }
    mediant = ac/(double)bd;
    if(target < mediant) {
      // Discard c/d as the mediant is closer to the target.
      // How many times in a row should we do that?
      // N = (c - target*d)/(target*b - a), but need to check for division by zero
      Ndenom = target * (double)b - (double)a;
      if(Ndenom < Ndenom_min) {
        // Division by zero, or close to it!
        // This means that a/b is a very good approximation
        // as we would need to update the c/d side a
        // very large number of times to get closer.
        // Use a/b and exit the loop.
        ac = a;
        bd = b;
        printf("break 2\n");
        break;
      }
      N = (c - target * (double)d)/Ndenom;
      Nint = floor(N + epsilon);
      // Check if the denominator will become too large
      if(d + Nint*b > maxdenom) {
        // Limit N, as the denominator would otherwise become too large
        N = (maxdenom - d)/(double)b;
        Nint = floor(N);
      }
      // Fast forward to a good c/d.
      c = c + Nint*a;
      d = d + Nint*b;

    } else {

      // Discard a/b as the mediant is closer to the target.
      // How many times in a row should we do that?
      // N = (target*b - a)/(c - target*d), but need to check for division by zero
      Ndenom = (double)c - target * (double)d;
      if(Ndenom < Ndenom_min) {
        // Division by zero, or close to it!
        // This means that c/d is a very good approximation
        // as we would need to update the a/b side a
        // very large number of times to get closer.
        // Use c/d and exit the loop.
        ac = c;
        bd = d;
        printf("break 3\n");
        break;
      }
      N = (target * (double)b - a)/Ndenom;
      Nint = floor(N + epsilon);
      // Check if the denominator will become too large
      if(b + Nint*d > maxdenom) {
        // Limit N, as the denominator would otherwise become too large
        N = (maxdenom - b)/(double)d;
        Nint = floor(N);
      }
      // Fast forward to a good a/b.
      a = a + Nint*c;
      b = b + Nint*d;
    }
    ii++;
  }

  retval.iterations = ii;
  retval.numerator = ac;
  retval.denominator = bd;
  return retval;
}

//*************************************************************
rational_t retval;

char str[40];
void doit(double target, bool bumped) {
    uint32_t maxdenom = 1048575;
    retval.target = target;
    retval = rational_approximation(target, maxdenom);

    double actual_real = ((double) retval.numerator) / (double) retval.denominator;
    double actual_error = target - actual_real;
    retval.actual_real = actual_real;
    retval.actual_error = actual_error;

    // don't print small error cases
    if (retval.actual_error >= 20e-10) {
        printf("\n");
        printf("target %.16f\n", target);
        printf("numerator %u\n", retval.numerator);
        printf("denominator %u\n", retval.denominator);
        printf("iterations %u\n", retval.iterations);
        printf("actual_real %.16f\n", actual_real);
        printf("actual_error %.16f\n", actual_error);
    }
}

//*************************************************************
RationalResult result;

void doit2(double target) {
    // this just makes denom bigger
    // double eps = 1e-10;
    double eps = -1;

    double t = target;
    result.target = target;

    uint32_t maxdenom = 1048575;
    // uint32_t maxdenom = 1048575 * 2;

    // HACK! kbn
    // uint32_t maxdenom = 1000000;
    // uint32_t maxdenom = 1000;
    // uint32_t maxdenom = 2097151; // 2**21 - 1
    // uint32_t maxdenom = 65535; // 2**16 - 1

    // does continued fractions use an int here?
    long l = maxdenom;
    if (eps == -1.0) {

        result = find_best_rat(l, t);
        // printf("target= %f best_rat= %ld / %ld max_denom= %ld err= %g abs_err= %g niter= %d\n",
        //        t, result.numerator, result.denominator, l, result.error,
        //        fabs(result.error), result.iterations);
    }
    else {
        result = find_best_rat_with_err_bound(eps, l, t);
        // printf("target= %f best_rat= %ld / %ld max_denom= %ld err= %g abs_err= %g abs_err/error= %g niter= %d\n",
        //        t, result.numerator, result.denominator, l, result.error,
        //        fabs(result.error), fabs(result.error)/eps, result.iterations);
    }


    if (DO_BEST_RAT_CONTINUED_FRACTIONS==1) {
        // kbn ..for consistency with other algo
        result.error = result.err;
        result.numerator = result.n;
        result.denominator = result.d;
        result.iterations = result.niter;
    }

    double actual_real = ((double) result.numerator) / (double) result.denominator;
    double actual_error = target - actual_real;

    result.actual_real = actual_real;
    result.actual_error = actual_error;

    printf("\n");
    printf("target %.16f\n", target);
    printf("numerator %lu\n", result.numerator);
    printf("denominator %lu\n", result.denominator);
    printf("iterations %u\n", result.iterations);
    printf("actual_real %.16f\n", actual_real);
    printf("actual_error %.16e\n", actual_error);
    }



//*************************************************************
void histo_reals(double* data, int num_data) {
    int num_bins = 21; // Number of bins
    double min_val = data[0], max_val = data[0]; // Find data range
    for (int i = 1; i < num_data; i++) {
        // printf("%.16f\n", data[i]);
        if (data[i] < min_val) min_val = data[i];
        if (data[i] > max_val) max_val = data[i];
    }
    printf("min_val %.4f max_val %.4f\n", min_val, max_val);

    // FIX! if equal the number of bins is zero!
    // Calculate bin width (one fewer than num_bins..allows roundup
    double bin_width = (max_val - min_val) / (num_bins - 1.0);
    printf("min_val %.4f max_val %.4f bin_width %.4f\n", min_val, max_val, bin_width);
    // one more bin in case of round up?
    int bin_cnts[num_bins] = {0}; // Array to store counts per bin

    for (int i = 0; i < num_data; i++) {
        // Calculate bin index
        int bin_index = (int)((data[i] - min_val) / bin_width);
        // Increment count for that bin
        bin_cnts[bin_index]++;
    }
    // Print histogram (example output)
    printf("\n");
    for (int i = 0; i < num_bins; i++) {
        printf("[%.5f - %.5f]: ", min_val + i * bin_width, min_val + (i + 1) * bin_width);
        if (false) {
            for (int j = 0; j < bin_cnts[i]; j++) printf("*");
            printf("\n");
        }
        printf("bin_cnt %d pct %.3f\n",
            bin_cnts[i], (100.0 * ((double) bin_cnts[i])) / num_data);
    }
    printf("\n");
}
//*************************************************************
// will fail if we try to save more than this?
#define ARRAY_SIZE 200000
    
// globals so we can loop
double maxerr_actual_error = 0.0;
double maxerr_actual_real = 0.0;
double maxerr_target = 0.0;
uint32_t maxerr_num = 0;
uint32_t maxerr_denom = 0;
uint32_t maxerr_iter = 0;
uint32_t maxerr_i = 0;

int main30() {
    uint32_t nums[ARRAY_SIZE] = { 0 };
    uint32_t denoms[ARRAY_SIZE] = { 0 };
    uint32_t iters[ARRAY_SIZE] = { 0 };
    double targets[ARRAY_SIZE] = { 0.0 };
    double actual_reals[ARRAY_SIZE] = { 0.0 };
    double actual_errors[ARRAY_SIZE] = { 0.0 };

    double lower_bound = 0.0 + 1e13;
    double upper_bound = 1.0 - 1e13;

    bool DO_RANDOM_CENTER = false;

    // Seed the random number engine using the current time
    unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator1(seed1);
    std::uniform_real_distribution<double> unif1(lower_bound, upper_bound);
    // default_random_engine re;

    // Declaring the upper and lower bounds
    // do we care about 0? no unlikely
    // interesting how wide the variation is between low and high
    // 2e-6
    // since the change in output freq is just the divisor and the tcxo
    // this is directly related to the change in pll freq
    // depending on the divisor we need a bigger shift in pll freq.
    // larger and we need more. smaller and we need less

    if (false) {
        lower_bound = 0.55;
        upper_bound = 0.57;
        lower_bound = 0.14484461538462 - 1e-7;
        upper_bound = 0.14484698377404 + 1e-7;

        lower_bound = 0.14484000000000;
        upper_bound = 0.14484999900000;

        lower_bound = 0.10004000000000;
        upper_bound = 0.10004999900000;

        lower_bound = 0.14000000000000;
        upper_bound = 0.14000010000000;

        lower_bound = 0.54000000000000;
        upper_bound = 0.54000010000000;

        lower_bound = 0.1;
        upper_bound = 0.9;

        lower_bound = 0.4;
        upper_bound = 0.7;

        // a random center
        lower_bound = 0 + 1e-6;
        upper_bound = 1.0 - 1e-6;

        if (DO_RANDOM_CENTER) {
            double center = unif1(generator1);
            lower_bound = center - 1e-6;
            upper_bound = center + 1e-6;
        }

        // interesting straight lines
        lower_bound = 0.50000014911;
        upper_bound = 0.50000044911;

        lower_bound = 0.50000004000;
        upper_bound = 0.50000050000;

        lower_bound = 0.50000000500;
        upper_bound = 0.50000050000;
    }

    if (false) {
        lower_bound = 0.50000000000;
        upper_bound = 0.50000050000;
    }

    if (false) {
        // Expected shift 0 to 1: 1.464844 Hz
        // Expected shift 0 to 2: 2.929688 Hz
        // Expected shift 0 to 3: 4.394531 Hz
     
        // what about using the interpolative divider for glitchless frequency shift?
        // it's a percentage of the target output freq
        lower_bound = 1.464844 / 29e6;
        upper_bound = 4.394531 / 29e6;
    }

    bool DO_FULL_INTERVAL = false;
    if (DO_FULL_INTERVAL) {
        lower_bound = 0 + 1e-16;
        upper_bound = 1 - 1e-16;

        printf("lower_bound %.16f\n", lower_bound);
        printf("upper_bound %.16f\n", upper_bound);

        // lower_bound = 0+1e-16;
        // upper_bound = 1-1e-16;
        // Seed the random number engine using the current time
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        // don't do this again?:
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    }

    // Seed the random number generator
    // https://linux.die.net/man/3/random
    // srand(time(NULL));
    // srandom(time(NULL));

    // our 393mhz 10M mul/div choices have reals  (including CW freq)
    //     struct real_freqs_t { double f; };
    //     real_freqs_t real_freqs[] = {
    //         {0.14484461538462},
    //         {0.14484540625000},
    //         {0.14484619290865},
    //         {0.14484698377404},
    //         {0.10390000000000}
    //     };

    bool STEP_REAL = true;

    double STEP_INTERVAL;
    uint64_t NUM_TO_DO;
    if (STEP_REAL) {
        NUM_TO_DO = 1e10 - 2;
        NUM_TO_DO = 1e6;
        STEP_INTERVAL = 1.0 / 1e10;
    }

    // don't start at 0, because we inc first
    static double multiplier_fraction = 0.0;

    for (uint32_t i = 0; i < NUM_TO_DO; i++) {
        // Generate a random double between 0 and 1
        // multiplier_fraction = (double)rand() / (double)RAND_MAX;

        // Getting a random double value
        // only > 0.5?
        // had a bad case at 0.46

        if (STEP_REAL) {
            multiplier_fraction += STEP_INTERVAL; // doesn't start with 0.0
        } else {
            if (false) {
                int j = i % 5;
                // multiplier_fraction = real_freqs[j].f;
            } else {
                // multiplier_fraction = unif(generator);
            }
        }

        // multiplier_fraction = (double)random() / (double)RAND_MAX;

        // this keeps the max error at:
        // 0.0000000067122029
        // while  (multiplier_fraction < 0.5)
        // while  (multiplier_fraction > 0.5)
        // while  (multiplier_fraction > 0.7)
        //   multiplier_fraction = unif(re);

        // multiplier_fraction = 0.4642856924664300;
        // multiplier_fraction = 0.4642856924664400;
        // multiplier_fraction = 0.47194409375000;
        double target = multiplier_fraction;
        
        // Using a si5351a fractional feedback, With a 26mhz tcxo, 
        // and maybe a 400mhz PLL with a minimal divisor of 15 after the pll,
        // it seems like we need decimal 11 digits of precision
        // on the fractional real being approximated.
        // Because the effect of the 10th digit on on the wspr symbol freq is at most:
        //    (1e-11 * 26e6) / 15 = 1.73e-5 Hz..
        // which is plenty of precision (uHz level)

        // So we only need 11 digits of precision  in the fractional real which is 0 to 1

        // Ideally means integer-scaled shifting should save log2(10**11) == 36 bits
        // ..but can we shift 32 bits into 64 bit? with 900Mhz max? yes we could?

        // Alternative: to the sprintf/sscanf:
        // Could multiply by 2**32 and cast it to a uint32_t ,  
        // then put it back in the double and divide by 2**32. 
        // So then we just have 32 bits of precision (less than 36 bits, but close?)

        char str[40];
        sprintf(str, "%.11f", target);
        sscanf(str, "%lf", &target);
        printf("target %.16f\n", target);

        // ad_rat doesn't like this case
        // maxerr_target 0.285714341919480
        // maxerr_num 2
        // maxerr_denom 7
        // maxerr_iter 4
        // maxerr_i 52668
        // target = 0.285714341919480;

        if (DO_BEST_RAT==1 || DO_BEST_RAT_CONTINUED_FRACTIONS==1) {
            doit2(target);

            targets[i] = target;
            nums[i]   = result.numerator;
            denoms[i] = result.denominator;
            iters[i] = result.iterations;
            // targets[i] = result.target;
            actual_reals[i] = result.actual_real;
            actual_errors[i] = result.actual_error * 1e10;

            // compare
            doit(target, false);  // not bumped
            if ((result.numerator != retval.numerator) ||
                (result.denominator != retval.denominator)) {
                printf("ERROR: doit and doit2 don't agree\n");
                printf("ERROR: target %.16f", target);
                printf("ERROR: result.numerator %lu", result.numerator);
                printf("ERROR: result.denominator %lu", result.denominator);
                printf("ERROR: retval.numerator %u", retval.numerator);
                printf("ERROR: retval.denominator %u", retval.denominator);
            }
                
        } else {
            // original target saved!
            doit(target, false);  // not bumped

            // don't print out the low error cases
            if (retval.actual_error < 20e-10) {
                continue;
            }
            targets[i] = target;
            nums[i]   = retval.numerator;
            denoms[i] = retval.denominator;
            iters[i] = retval.iterations;
            // this would be possibly an adjusted target
            // targets[i] = retval.target;
            actual_reals[i] = retval.actual_real;
            actual_errors[i] = retval.actual_error * 1e10;
        }

        if (abs(actual_errors[i]) > abs(maxerr_actual_error)) {
            maxerr_actual_error = actual_errors[i];
            maxerr_actual_real = actual_reals[i];
            maxerr_target = targets[i];
            maxerr_num = nums[i];
            maxerr_denom = denoms[i];
            maxerr_iter = iters[i];
            maxerr_i = i;
        }
    }

    if (DO_BEST_RAT_CONTINUED_FRACTIONS)
        printf("\nUsed continued fractions find_best_rat(l, t)\n");
    else if (DO_BEST_RAT)
        printf("\nUsed fast farey find_best_rat(l, t)\n");
    else
        printf("\nUsed magnusson rational_approximation(target, maxdenom)");

    printf("maxerr_actual_error(*1e10) %.15f\n", maxerr_actual_error);
    printf("maxerr_actual_real %.15f\n", maxerr_actual_real);
    printf("maxerr_target %.15f\n", maxerr_target);
    printf("maxerr_num %u\n", maxerr_num);
    printf("maxerr_denom %u\n", maxerr_denom);
    printf("maxerr_iter %u\n", maxerr_iter);
    printf("maxerr_i %u\n", maxerr_i);
    printf("\n");
    if (!STEP_REAL) {
        printf("\nhisto_reals(actual_errors, NUM_TO_DO)\n");
        histo_reals(actual_errors, NUM_TO_DO);
    }

    return 0;
}

/*
why big error?
target 0.0726858828202311
numerator 74332
denominator 1022647
iterations 12
actual_real 0.0726858828119576
actual_error 0.0000000000082735
*/



/*
uint32_t maxdenom = 1048575;

why is this only getting 4 iterations? it's close to denom max
target 0.4642856924664329
numerator 486830
denominator 1048557
iterations 4
actual_real 0.4642856802253001
actual_error 0.0000000122411328
break 1
*/

/*
Another big error case
target 0.4642856924664300
numerator 486830
denominator 1048557
iterations 4
actual_real 0.4642856802253001
actual_error 0.0000000122411299
break 1
*/


/*
same as prior 2
Another big error case
remember maxdenom = 1048575;
Used find_best_rat(l, t)
maxerr_actual_error(*1e10) 122.411298875135799
maxerr_actual_real 0.464285680225300
maxerr_target 0.464285692466430
maxerr_num 486830
maxerr_denom 1048557
maxerr_iter 5
maxerr_i 6703
*/

/*
why did he stop after 4 iterations?
Used find_best_rat(l, t)
maxerr_actual_error(*1e10) 562.051942765151580
maxerr_actual_real 0.285714285714286
maxerr_target 0.285714341919480
maxerr_num 2
maxerr_denom 7
maxerr_iter 4
maxerr_i 52668
*/

/*
Ndenom_min wasn't assign (I had broken magnusson):
after fixing that, always got same answer 3 like ad_rat..
magnusson oscillated between 3 answers, when tried repeatedly with same hard target
ad_rat stuck to the third answer, which is the better answer.

target 0.2857143419194800
numerator 0
denominator 1
iterations 0
actual_real 0.0000000000000000
actual_error 0.2857143419194800
break 3

target 0.2857143419194800
numerator 1
denominator 3
iterations 1
actual_real 0.3333333333333333
actual_error -0.0476189914138533
break 2

target 0.2857143419194800
numerator 2
denominator 7
iterations 2
actual_real 0.2857142857142857
actual_error 0.0000000562051943
break 2
*/

/*
continued fractions algo got same 2/7 result
target 0.2857143419194800
numerator 2
denominator 7
iterations 3
actual_real 0.2857142857142857
actual_error 0.0000000562051943
*/


/*
continued fractions can go off in the weeds, with denom too large!
Used continued fractions find_best_rat(l, t)
maxerr_actual_error(*1e10) 26563395018.484191894531250
maxerr_actual_real -1.800060783625919
maxerr_target 0.856278718222500
maxerr_num 2038667857
maxerr_denom 3162412287
maxerr_iter 15
maxerr_i 25792
*/


/*
    multiplier_fraction = 0.61663664871234;
    doit();
    multiplier_fraction = 0.6166384515;
    doit();
    multiplier_fraction = 0.6166402544;
    doit();
    multiplier_fraction = 0.6166420573;
    doit();
*/


int main() {
    for (int iterations = 0; iterations < 2000; iterations++) {
        maxerr_num = 0;
        maxerr_denom = 0;
        maxerr_iter = 0;
        maxerr_i = 0;

        main30();
        // if (maxerr_actual_error > 500) break;
    }
    return 0;
}
