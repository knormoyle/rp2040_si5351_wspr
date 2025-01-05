// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef FAREY_FUNCTIONS_H
#define FAREY_FUNCTIONS_H
#include <stdint.h>

typedef struct {
  uint32_t numerator;
  uint32_t denominator;
  uint32_t iterations;
} rational_t;

rational_t rational_approximation(double target, uint32_t maxdenom);

#endif
