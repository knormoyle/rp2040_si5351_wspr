// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#ifndef SI5351_FUNCTIONS_H
#define SI5351_FUNCTIONS_H
#include <stdint.h>

// so config_functions.cpp can get them and do a test
const int SI5351_TCXO_FREQ =                26000000;

const int SI5351A_OUTPUT_ENABLE_CONTROL =   3;
const int SI5351A_CLK0_CONTROL =            16;
const int SI5351A_CLK1_CONTROL =            17;

// FIX! is there a clk2? does CLK7 drive it?
const int SI5351A_CLK7_CONTROL =            23;
const int SI5351A_PLLB_BASE =               34;
const int SI5351A_MULTISYNTH0_BASE =        42;
// FIX! how do we turn off multisynth 1?
const int SI5351A_MULTISYNTH1_BASE =        50;
const int SI5351A_PLL_RESET =               177;

const int SI5351A_CLK0_MS0_INT =            (1 << 6);
const int SI5351A_CLK0_MS0_SRC_PLLB =       (1 << 5);
const int SI5351A_CLK1_MS1_INT =            (1 << 6);
const int SI5351A_CLK1_MS1_SRC_PLLB =       (1 << 5);

const int SI5351A_CLK0_SRC_MULTISYNTH_0 =   (3 << 2);
const int SI5351A_CLK0_CLK0_INV =           (1 << 4);
// FIX! is using 2 undocumented? 
const int SI5351A_CLK1_SRC_MULTISYNTH_1 =   (3 << 2);
const int SI5351A_CLK1_CLK1_INV =           (1 << 4);

const int SI5351A_CLK0_IDRV_8MA =           (3 << 0);
const int SI5351A_CLK0_IDRV_6MA =           (2 << 0);
const int SI5351A_CLK0_IDRV_4MA =           (1 << 0);
const int SI5351A_CLK0_IDRV_2MA =           (0 << 0);

const int SI5351A_CLK1_IDRV_8MA =           (3 << 0);
const int SI5351A_CLK1_IDRV_6MA =           (2 << 0);
const int SI5351A_CLK1_IDRV_4MA =           (1 << 0);
const int SI5351A_CLK1_IDRV_2MA =           (0 << 0);

const int SI5351A_PLL_RESET_PLLB_RST =      (1 << 7);

// FIX! why were these static?
// static would mean you can only call it from within this translation unit
int i2cWrite(uint8_t reg, uint8_t val);     // write reg via i2c
int i2cReadTest(uint8_t reg, uint8_t val);  // read reg via i2c

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
