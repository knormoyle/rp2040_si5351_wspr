// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// Farey sequence-based rational approximation of numbers.
// Per Magnusson, 2024
// MIT licence, http://www.opensource.org/licenses/mit-license.php
// https://axotron.se/blog/fast-algorithm-for-rational-approximation-of-floating-point-numbers/

#include <cstdint>
#include <cmath>
#include "farey_functions.h"

// Background:
// Farey's work as a geologist is forgotten. His contribution to math continues on,
// but it's crazy how it started:
// https://mathshistory.st-andrews.ac.uk/Biographies/Farey/

// Farey's article ..was also published in the Philosophical Magazine and appeared in 1816.
// It was called 'On a curious property of vulgar fractions' and it was sent to the editor
// from Howland Street in London, the residence of Farey's eldest son where he spent the
// last years of his life (in fact he died in that house).

// The article consists of only four paragraphs.
// In the first paragraph Farey says that he noted the "curious property"
// while examining the tables of Complete decimal quotients produced by Henry Goodwin.
// In the second paragraph he defines the Farey series and states the "curious property".

// The final paragraph of [5]
//
//   I am not acquainted, whether this curious property of vulgar fractions has been before 
//   pointed out?; or whether it may admit of some easy or general demonstration?;
//   which are points on which I should be glad to learn the sentiments of some of
//   your mathematical readers ...

// [5] J Farey, On a curious property of vulgar fractions, Philos. Mag. J. 47 (1816), 385-386.


// In 1816 the British geologist John Farey defined the Farey sequence Fn as the list, written
// in increasing order,of all the rational numbers between 0 and 1 that have only the
// numbers 1,2,3,...,n as denominators.

// We can do the same thing for rational numbers between any two positive numbers.
// For example we can consider sequences betweeen 1 and 2 where we have:
// F1 = 1/1, 2/1
// F2 = 1/1, 3/2, 2/1 .
//
// What would F3 and F4 be in this case?
// For the two positive rational numbers b/d and a/c the mediant is defined as (a+b) / (c+d).
// The mediant has the nice property that it is always in between the two fractions giving rise
// to it:
//     if 0 < (b/d) < (a/c) then (b/d)< (a+b)/(c+d) < (a/c).
//
// Clearly each Farey sequence Fn+1 must contain all of the terms of Fn, along with some new terms.
// Mediants also have the nice property that each 'new' term in the Farey sequence Fn+1 is the
// mediant // of two consecutive terms in Fn.


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

rational_t rational_approximation(double target, uint32_t maxdenom) {
    rational_t retval;
    // float does not have enough resolution
    // to deal with single-digit differences
    // between numbers above 10^8.
    double mediant;
    double N, Ndenom, Ndenom_min;
    double epsilon = 1e-10;  // To handle rounding issues in conjunction with floor
    uint32_t a = 0, b = 1, c = 1, d = 1, ac, bd, Nint;
    // kbn: max it at 100
    // saw iterations going to 1001 for a 10M wspr freq .
    const int maxIter = 100;

    if (target > 1) {
        // Invalid
        retval.iterations = 0;
        retval.numerator = 1;
        retval.denominator = 1;
        return retval;
    }
    if (target < 0) {
        // Invalid
        retval.iterations = 0;
        retval.numerator = 0;
        retval.denominator = 1;
        return retval;
    }
    if (maxdenom < 1) {
        maxdenom = 1;
    }

    mediant = 0;
    Ndenom_min = 1 / ((double) 10 * maxdenom);
    int ii = 0;
    // Farey approximation loop
    while (true) {
        ac = a+c;
        bd = b+d;
        if (bd > maxdenom || ii > maxIter) {
            // The denominator has become too big, or too many iterations.
            // Select the best of a/b and c/d.
            if ((target - a / (double)b) < (c / (double)d - target)) {
                ac = a;
                bd = b;
            } else {
                ac = c;
                bd = d;
            }
            break;
        }
        mediant = ac / (double)bd;
        if (target < mediant) {
            // Discard c/d as the mediant is closer to the target.
            // How many times in a row should we do that?
            // N = (c - target*d)/(target*b - a), but need to check for division by zero
            Ndenom = target * (double)b - (double)a;
            if (Ndenom < Ndenom_min) {
                // Division by zero, or close to it!
                // This means that a/b is a very good approximation
                // as we would need to update the c/d side a
                // very large number of times to get closer.
                // Use a/b and exit the loop.
                ac = a;
                bd = b;
                break;
            }
            N = (c - target * (double)d) / Ndenom;
            Nint = floor(N + epsilon);
            // Check if the denominator will become too large
            if ((d + Nint * b) > maxdenom) {
                // Limit N, as the denominator would otherwise become too large
                N = (maxdenom - d) / (double)b;
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
            if (Ndenom < Ndenom_min) {
                // Division by zero, or close to it!
                // This means that c/d is a very good approximation
                // as we would need to update the a/b side a
                // very large number of times to get closer.
                // Use c/d and exit the loop.
                ac = c;
                bd = d;
                break;
            }
            N = (target * (double)b - a)/Ndenom;
            Nint = floor(N + epsilon);
            // Check if the denominator will become too large
            if ((b + Nint*d) > maxdenom) {
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
