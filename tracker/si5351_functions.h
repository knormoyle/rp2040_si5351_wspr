// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

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
