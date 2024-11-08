/* 
 * si5351_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#ifndef SI5351_FUNCTIONS_H
#define SI5351_FUNCTIONS_H
#include <stdint.h>

// FIX! why were these static?
// The keyword static has different meanings depending on where it is applied. In the context of a function, static means that the function has internal linkage. This means that the function can only be called from within the same translation unit (i.e., the same source file). It is not visible to other files.

void vfo_init(void); // removed static
void vfo_set_power_on(bool turn_on); // removed static
void si5351a_reset_PLLB(void); // removed static

bool vfo_is_on(void);
void vfo_turn_on(uint8_t clk_number);
void vfo_turn_off(void);

void si5351a_setup_PLLB(uint8_t mult, uint32_t num, uint32_t denom);
void si5351a_setup_multisynth1(uint32_t div); // removed static
void si5351a_setup_multisynth0(uint32_t div);

void vfo_set_freq_x16(uint8_t clk_number, uint32_t freq);
void vfo_turn_on_clk_out(uint8_t clk_number);
void vfo_turn_off_clk_out(uint8_t clk_number);

void vfo_set_drive_strength(uint8_t clk_number, uint8_t strength);

#endif

