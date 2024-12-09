// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef KEYBOARD_FUNCTIONS_H
#define KEYBOARD_FUNCTIONS_H
#include <stdint.h>

char drainSerialTo_CRorNL (uint32_t millis_max);
char getOneChar (uint32_t millis_max);
uint32_t get_sie_status(void);
uint32_t get_sie_connected(void);

#endif
