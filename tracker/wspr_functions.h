// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef WSPR_FUNCTIONS_H
#define WSPR_FUNCTIONS_H
#include <stdint.h>

void PWM4_Handler();
void calcPwmDivAndWrap(uint32_t *PWM_DIV, uint32_t *PWM_WRAP_CNT, uint32_t INTERRUPTS_PER_SYMBOL, uint32_t PLL_SYS_MHZ);
void setPwmDivAndWrap(uint32_t PWM_DIV, uint32_t PWM_WRAP_CNT);
void disablePwmInterrupts(void);

#endif
