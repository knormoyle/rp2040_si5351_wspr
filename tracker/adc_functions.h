// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef ADC_FUNCTIONS_H
#define ADC_FUNCTIONS_H
// #include <stdint.h>

float readVoltage(void);
float readTemp(void);
void adc_INIT(void);

#endif
