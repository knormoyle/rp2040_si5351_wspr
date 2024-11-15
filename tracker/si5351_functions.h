// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#ifndef SI5351_FUNCTIONS_H
#define SI5351_FUNCTIONS_H
#include <stdint.h>

// FIX! why were these static?
// static would mean you can only call it from within this translation unit
void vfo_init(void);  // removed static
void vfo_set_power_on(bool turn_on);  // removed static

bool vfo_is_on(void);
void vfo_turn_on(uint8_t clk_number);
void vfo_turn_off(void);

void si5351a_setup_PLLB(uint8_t mult, uint32_t num, uint32_t denom);
void si5351a_setup_multisynth1(uint32_t div);  // removed static
void si5351a_setup_multisynth0(uint32_t div);

void vfo_set_freq_x16(uint8_t clk_number, uint32_t freq);
void vfo_turn_on_clk_out(uint8_t clk_number);
void vfo_turn_off_clk_out(uint8_t clk_number);

void vfo_set_drive_strength(uint8_t clk_number, uint8_t strength);

#endif  // SI5351_FUNCTIONS_H
