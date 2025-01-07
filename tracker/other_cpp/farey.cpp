// Farey sequence-based rational approximation of numbers.
// Per Magnusson, 2024
// MIT licence, http://www.opensource.org/licenses/mit-license.php

// from
// https://axotron.se/blog/fast-algorithm-for-rational-approximation-of-floating-point-numbers/


#include <cstdint>
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
#define DO_BEST_RAT 1
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
      printf("break 1\n");
      break;
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
    if (false) {
        // round it to 14 digits of precision
        // we know target is between 0 and 1 to start
        sprintf(str, "%.14f", target);
        sscanf(str, "%lf", &target);
    }

    retval.target = target;
    retval = rational_approximation(target, maxdenom);

    double actual_real = ((double) retval.numerator) / (double) retval.denominator;
    double actual_error = target - actual_real;
    retval.actual_real = actual_real;
    retval.actual_error = actual_error;
    
    printf("\n");
    if (bumped) {
        printf("bumped target %.16f\n", target);
        printf("bumped numerator %u\n", retval.numerator);
        printf("bumped denominator %u\n", retval.denominator);
        printf("bumped iterations %u\n", retval.iterations);
        printf("bumped actual_real %.16f\n", actual_real);
        printf("bumped actual_error %.16f\n", actual_error);
    } else {
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

    // round it to 14 digits of precision
    // we know target is between 0 and 1 to start
    if (false) {
        // round it to 14 digits of precision
        // we know target is between 0 and 1 to start
        sprintf(str, "%.14f", target);
        sscanf(str, "%lf", &target);
    }

    double t = target;
    result.target = target;

    uint32_t maxdenom = 1048575;
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
    printf("actual_error %.16f\n", actual_error);
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
// #define NUM_TO_DO 100000
#define NUM_TO_DO 200000

int main() {
    uint32_t nums[NUM_TO_DO] = { 0 };
    uint32_t denoms[NUM_TO_DO] = { 0 };
    uint32_t iters[NUM_TO_DO] = { 0 };
    double targets[NUM_TO_DO] = { 0.0 };
    double actual_reals[NUM_TO_DO] = { 0.0 };
    double actual_errors[NUM_TO_DO] = { 0.0 };

    double maxerr_actual_error = 0.0;
    double maxerr_actual_real = 0.0;
    double maxerr_target = 0.0;
    uint32_t maxerr_num = 0;
    uint32_t maxerr_denom = 0;
    uint32_t maxerr_iter = 0;
    uint32_t maxerr_i = 0;

    double lower_bound = 0;
    double upper_bound = 1.0;

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

    double center = unif1(generator1);
    lower_bound = center - 1e-6;
    upper_bound = center + 1e-6;
    printf("lower_bound %.16f\n", lower_bound);
    printf("upper_bound %.16f\n", upper_bound);


    // Seed the random number engine using the current time
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // don't do this again?:
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);


    // Seed the random number generator
    // https://linux.die.net/man/3/random
    // srand(time(NULL));
    srandom(time(NULL));

    // our 393mhz 10M mul/div choices have reals  (including CW freq)
    struct real_freqs_t { double f; };
    real_freqs_t real_freqs[] = {
        {0.14484461538462},
        {0.14484540625000},
        {0.14484619290865},
        {0.14484698377404},
        {0.10390000000000}
    };

    for (uint32_t i = 0; i < NUM_TO_DO; i++) {
        // Generate a random double between 0 and 1
        // multiplier_fraction = (double)rand() / (double)RAND_MAX;

        // Getting a random double value
        // only > 0.5?
        // had a bad case at 0.46

        double multiplier_fraction;
        if (true) {
            multiplier_fraction = unif(generator);
        } else {
            int j = i % 5;
            multiplier_fraction = real_freqs[j].f;
        }

        // double multiplier_fraction = unif(re);

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
        printf("multiplier_fraction %.16f\n", multiplier_fraction);
        double target = multiplier_fraction;
        // ad_rat doesn't like this case 
        // maxerr_target 0.285714341919480
        // maxerr_num 2
        // maxerr_denom 7
        // maxerr_iter 4
        // maxerr_i 52668
        // target = 0.285714341919480;

        if (DO_BEST_RAT==1 || DO_BEST_RAT_CONTINUED_FRACTIONS==1) {
            targets[i] = target;
            doit2(target);
            nums[i]   = result.numerator;
            denoms[i] = result.denominator;
            iters[i] = result.iterations;
            // targets[i] = result.target;
            actual_reals[i] = result.actual_real;
            actual_errors[i] = result.actual_error * 1e10;
        } else {
            // original target saved!
            targets[i] = target;
            doit(target, false);  // not bumped
            //**********************************************************
            if (false) {
                // kbn enhancement for the rare cases of larger error than typical
                // bump increment! up to 5x by 1e-14..looking to meet error goals
                // the actual error will be the sum of the bump add, plus final error
                double actual_error_xe10 = retval.actual_error * 1e10;
                int tries = 0;
                // so we don't change the original target when we save it to compute our
                // error from the original goal
                double new_target = target;
                char str[40];
                while (abs(actual_error_xe10) > 10) {
                    tries += 1;
                    if (tries > 5) {
                        printf("ERROR: bump tries > 5..using target %.14f\n", target);
                        break;
                    }
                    // increase the bump amount by 10**tries * 1e-14
                    // new_target = new_target + (pow(10, tries) * 1e-14);
                
                    switch (tries) {
                        case(1): sprintf(str, "%.12f", new_target);
                        case(2): sprintf(str, "%.11f", new_target);
                        case(3): sprintf(str, "%.10f", new_target);
                        case(4): sprintf(str, "%.9f", new_target);
                        case(5): sprintf(str, "%.8f", new_target);
                    }
            
                    sscanf(str, "%lf", &new_target);
                    printf("ERROR: actual_error_xe10 %.4f tries (%d) bump to new_target %.14f chopping off precision\n", 
                        actual_error_xe10, tries, new_target);
                    // could try plus or minus direction randomly?
                    printf("-> doit with new_target %.14f\n", new_target);
                    doit(new_target, true);  // bumped
                    actual_error_xe10 = retval.actual_error * 1e10;
                }
            }
            //**********************************************************
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
    printf("\nhisto_reals(actual_errors, NUM_TO_DO)\n");
    histo_reals(actual_errors, NUM_TO_DO);

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
}
