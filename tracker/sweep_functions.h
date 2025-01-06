// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024

// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#ifndef SWEEP_FUNCTIONS_H
#define SWEEP_FUNCTIONS_H
#include <stdint.h>

void si5351a_calc_optimize(double *sumShiftError, double *sumAbsoluteError, uint32_t *pll_num, bool print);
void si5351a_calc_sweep(void);
void si5351a_calc_sweep_band(void);
void si5351a_denom_optimize_search();

#endif  // SWEEP_FUNCTIONS_H
