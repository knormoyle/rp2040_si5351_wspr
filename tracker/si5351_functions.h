// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024

// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#ifndef SI5351_FUNCTIONS_H
#define SI5351_FUNCTIONS_H
#include <stdint.h>

const int WSPR_TX_CLK_0_NUM =               0;
const int WSPR_TX_CLK_1_NUM =               1;

const int SI5351A_DEVICE_STATUS =           0;
const int SI5351A_INTERRUPT_STATUS_STICKY = 1;
const int SI5351A_OUTPUT_ENABLE_CONTROL =   3;

const int SI5351A_CLK0_CONTROL =            16;
const int SI5351A_CLK1_CONTROL =            17;
const int SI5351A_CLK2_CONTROL =            18;
const int SI5351A_CLK7_CONTROL =            23;

// FIXed 6/24/25. was 24! should be 26
const int SI5351A_CLK3_0_DISABLE_STATE =    24;
const int SI5351A_CLK7_4_DISABLE_STATE =    25;
const int SI5351A_PLLA_BASE =               26; // 8 regs
const int SI5351A_PLLB_BASE =               34; // 8 regs
const int SI5351A_MULTISYNTH0_BASE =        42; // 8 regs
const int SI5351A_MULTISYNTH1_BASE =        50; // 8 regs
const int SI5351A_MULTISYNTH2_BASE =        58; // 8 regs
const int SI5351A_PLL_RESET =               177;

const int SI5351A_CLK0_MS0_INT =            (1 << 6);
const int SI5351A_CLK0_MS0_SRC_PLLB =       (1 << 5);

const int SI5351A_CLK1_MS1_INT =            (1 << 6);
const int SI5351A_CLK1_MS1_SRC_PLLB =       (1 << 5);

const int SI5351A_CLK2_MS2_INT =            (1 << 6);
const int SI5351A_CLK2_MS2_SRC_PLLB =       (1 << 5);

const int SI5351A_CLK0_SRC_MULTISYNTH_0 =    (3 << 2);
const int SI5351A_CLK0_INV =                 (1 << 4);
const int SI5351A_CLK0_PDN =                 (1 << 7);

const int SI5351A_CLK1_SRC_MULTISYNTH_1 =    (3 << 2);
const int SI5351A_CLK1_INV =                 (1 << 4);
const int SI5351A_CLK1_PDN =                 (1 << 7);

const int SI5351A_CLK2_SRC_MULTISYNTH_2 =    (3 << 2);
const int SI5351A_CLK2_INV =                 (1 << 4);
const int SI5351A_CLK2_PDN =                 (1 << 7);

const int SI5351A_CLK01_IDRV_8MA =           (3 << 0);
const int SI5351A_CLK01_IDRV_6MA =           (2 << 0);
const int SI5351A_CLK01_IDRV_4MA =           (1 << 0);
const int SI5351A_CLK01_IDRV_2MA =           (0 << 0);

const int SI5351A_CLK2_IDRV_8MA =            (3 << 0);
const int SI5351A_CLK2_IDRV_6MA =            (2 << 0);
const int SI5351A_CLK2_IDRV_4MA =            (1 << 0);
const int SI5351A_CLK2_IDRV_2MA =            (0 << 0);

const int SI5351A_PLL_RESET_PLLB_RST =       (1 << 7);
const int SI5351A_PLL_RESET_PLLA_RST =       (1 << 5);

bool reserved_reg(uint8_t reg);
int i2cWrite(uint8_t reg, uint8_t val);   // write reg via i2c
int i2cWrRead(uint8_t reg, uint8_t val);  // read reg via i2c

void vfo_init(void);
void vfo_set_power_on(bool turn_on);

bool vfo_is_on(void);
void vfo_turn_on();
void vfo_turn_off(void);

void si5351a_setup_PLL(uint8_t mult, uint32_t num, uint32_t denom, bool do_pllb);
void si5351a_setup_multisynth012(uint32_t div);

// this will give the freq you should see on wsjt-x if
// hf_freq is the XMIT_FREQ for a channelsymbol can be 0 to 3.
// Can subtract 20 hz to get the low end of the bin
// (assume freq calibration errors of that much,
// then symbol the 200hz passband?
double calcSymbolFreq(uint32_t hf_freq, uint8_t symbol, bool print);

// BUG? this is the only thing I have that returns a 64-bit 
// occasionaly wrong (ptr to wrong 32-bit ?)
// https://stackoverflow.com/questions/28432224/returned-uint64-t-seems-truncated
// is it because si5351_functions.cpp doesn't include it's own .h?
// should always do that and not be dependent on Arduino doing it right?

void calcSymbolFreq_xxx(uint64_t *freq_xxx, uint32_t hf_freq, uint8_t symbol);

uint8_t startSymbolFreq(uint32_t hf_freq, uint8_t symbol, bool only_pll_num, bool just_do_calcs);

// used to print (not change) for walking a range in setup,
// just to see what changes
uint8_t vfo_calc_div_mult_num(double *actual, double *actual_pll_freq, uint32_t *ms_div, uint32_t *pll_mult, uint32_t *pll_num, uint32_t *pll_denom, uint64_t freq_xxx, bool use_PLL_DENOM_OPTIMIZE);
uint8_t vfo_set_freq_xxx(uint8_t clk_number, uint32_t freq, bool only_pll_num, bool just_do_calcs);

void vfo_turn_on_clk_out(uint8_t clk_number, bool print);
// FIX! ...this doesn't seem to work on ms5351m
void vfo_turn_off_clk_out(uint8_t clk_number, bool print);
void vfo_set_drive_strength(uint8_t clk_number, uint8_t strength);

uint32_t doCorrection(uint32_t freq);
void set_PLL_DENOM_OPTIMIZE(char *band);

void si5351a_reset_PLLA(bool print);
void si5351a_reset_PLLB(bool print);

// just flip the PDN bit..using previously set _prev state 
void si5351a_power_up_clk01(bool print);
void si5351a_power_down_clk01(bool print);

// 0: invalidate, 1: lookup, 2: install
uint8_t vfo_calc_cache(
    double *actual, double *actual_pll_freq,
    uint32_t *ms_div, uint32_t *pll_mult, uint32_t *pll_num, uint32_t *pll_denom,
    uint64_t freq_xxx, uint8_t operation);

// shorthand for calling a flush
void vfo_calc_cache_flush();
// in the tracker.ino, we can print valid entries and validate that valid entries don't have 0 values
uint8_t vfo_calc_cache_print_and_check();

void init_PLL_freq_target(uint64_t *PLL_FREQ_TARGET, char *band);

#endif  // SI5351_FUNCTIONS_H
