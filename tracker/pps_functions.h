// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef PPS_FUNCTIONS_H
#define PPS_FUNCTIONS_H
#include <stdint.h>

void gpsPPS_init();
void gpsPPS_callback(uint gpio, uint32_t events);

void setGpsPPSMode(void);
void PPS_countEnable(bool reset);
void PPS_countDisable(void);

#endif // PPS_FUNCTIONS_H

