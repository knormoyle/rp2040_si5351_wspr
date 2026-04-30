// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef SLOW_CLOCK_FUNCTIONS_H
#define SLOW_CLOCK_FUNCTIONS_H
#include <stdint.h>

void kazuClocksSlow(void);
void kazuClocksRestore(uint32_t PLL_SYS_MHZ_restore, int currentGpsBaud);

#endif // SLOW_CLOCK_FUNCTIONS_H
