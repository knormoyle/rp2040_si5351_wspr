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
#include <math.h>

using namespace std;
 
typedef struct {
  uint32_t numerator;
  uint32_t denominator;
  uint32_t iterations; // kbn
  double target; // kbn
  double actual_real; // kbn
  double actual_error; // kbn

} rational_t;

#include "ad_rat_by_fast_farey.cpp"
 
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
  // Ndenom_min = 1/((double) 10*maxdenom);
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
void doit(double target) { 
    uint32_t maxdenom = 1048575;
    // round it to 14 digits of precision
    // we know target is between 0 and 1 to start
    sprintf(str, "%.14f", target);
    sscanf(str, "%lf", &target);

    retval = rational_approximation(target, maxdenom);
    retval.target = target;

    double actual_real = ((double) retval.numerator) / (double) retval.denominator;
    double actual_error = target - actual_real;

    retval.actual_real = actual_real;
    retval.actual_error = actual_error;

    printf("\n");
    printf("target %.16f\n", target);
    printf("numerator %u\n", retval.numerator);
    printf("denominator %u\n", retval.denominator);
    printf("iterations %u\n", retval.iterations);
    printf("actual_real %.16f\n", actual_real);
    printf("actual_error %.16f\n", actual_error);
}

//*************************************************************
RationalResult result;

void doit2(double target) { 

    // this just makes denom bigger
    // double eps = 1e-10;
    double eps = -1;

    // round it to 10 digits of precision
    // we know target is between 0 and 1 to start
    sprintf(str, "%.10f", target);
    sscanf(str, "%lf", &target);

    double t = target;
    result.target = target;

    uint32_t maxdenom = 1048575;
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
    int num_bins = 20; // Number of bins
    double min_val = data[0], max_val = data[0]; // Find data range

    for (int i = 1; i < num_data; i++) {
        // printf("%.16f\n", data[i]);
        if (data[i] < min_val) min_val = data[i];
        if (data[i] > max_val) max_val = data[i];
    }
    printf("min_val %.f max_val %.f\n", min_val, max_val);
    // FIX! if equal the number of bins is zero!
    double bin_width = (max_val - min_val) / num_bins;  // Calculate bin width
    printf("min_val %.2f max_val %.2f bin_width %.2f\n", min_val, max_val, bin_width);
    int bin_counts[num_bins] = {0}; // Array to store counts per bin

    for (int i = 0; i < num_data; i++) {
        int bin_index = (data[i] - min_val) / bin_width;  // Calculate bin index
        bin_counts[bin_index]++;  // Increment count for that bin
    }
    // Print histogram (example output)
    for (int i = 0; i < num_bins; i++) {
        printf("[%f - %f]: ", min_val + i * bin_width, min_val + (i + 1) * bin_width);
        for (int j = 0; j < bin_counts[i]; j++) {
            printf("*");
        }
        printf("\n");
    }
}
//*************************************************************
#define NUM_TO_DO 100000

int main() {
    uint32_t nums[NUM_TO_DO] = { 0 };
    uint32_t denoms[NUM_TO_DO] = { 0 };
    uint32_t iters[NUM_TO_DO] = { 0 };
    double targets[NUM_TO_DO] = { 0.0 };
    double actual_reals[NUM_TO_DO] = { 0.0 };
    double actual_errors[NUM_TO_DO] = { 0.0 };

    // Declaring the upper and lower bounds
    double lower_bound = 0.0;
    double upper_bound = 1.0;
    uniform_real_distribution<double> unif(lower_bound, upper_bound);
    default_random_engine re;

    // Seed the random number generator
    srand(time(NULL)); 
    for (int i = 0; i < NUM_TO_DO; i++) {
        // Generate a random double between 0 and 1
        // multiplier_fraction = (double)rand() / (double)RAND_MAX;
 
        // Getting a random double value
        // only > 0.5?
        // had a bad case at 0.46

        double multiplier_fraction;
        multiplier_fraction = unif(re);
        // this keeps the max error at:
        // 0.0000000067122029
        // while  (multiplier_fraction < 0.5) 
        // while  (multiplier_fraction > 0.5) 
        // while  (multiplier_fraction > 0.7) 
        //   multiplier_fraction = unif(re);
        
        // multiplier_fraction = 0.4642856924664300;
        // multiplier_fraction = 0.4642856924664400;

        if (false) {
            doit2(multiplier_fraction);
            nums[i]   = retval.numerator;
            denoms[i] = retval.denominator;
            iters[i] = retval.iterations;
            targets[i] = retval.target;
            actual_reals[i] = retval.actual_real;
            actual_errors[i] = retval.actual_error * 1e10;
        } else {
            doit2(multiplier_fraction);
            nums[i]   = result.numerator;
            denoms[i] = result.denominator;
            iters[i] = result.iterations;
            targets[i] = result.target;
            actual_reals[i] = result.actual_real;
            actual_errors[i] = result.actual_error * 1e10;
        }

    }
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
