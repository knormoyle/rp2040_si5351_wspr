/* 
 * si5351_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#include "hardware/gpio.h"

extern const int VFO_VDD_ON_N_PIN;
extern const int VFO_I2C0_SDA_PIN;
extern const int VFO_I2C0_SCL_PIN;

// when we set both?
extern const int WSPR_TX_CLK_NUM;
extern const int WSPR_TX_CLK_1_NUM;
// this is the other differential clock for wspr? (was aprs)
extern const int WSPR_TX_CLK_0_NUM;

extern const int SI5351A_CLK_IDRV_8MA;
extern const int SI5351A_CLK_IDRV_6MA;
extern const int SI5351A_CLK_IDRV_4MA;
extern const int SI5351A_CLK_IDRV_2MA;

const int PLL_CALCULATION_PRECISION=4;
#define VFO_I2C_INSTANCE i2c0
const int VFO_I2C0_SCL_HZ=(1000 * 1000);

static void vfo_init(void)
{
  // turn ON VFO VDD
  gpio_init(VFO_VDD_ON_N_PIN);
  gpio_pull_up(VFO_VDD_ON_N_PIN);
  gpio_put(VFO_VDD_ON_N_PIN, 0);

  // init I2C0 for VFO
  i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);

  gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
  gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

  gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
}

static void vfo_set_power_on(bool turn_on)
{
  static bool s_is_on = false;
  if (turn_on == s_is_on) return;
  s_is_on = turn_on;
  gpio_set_dir(VFO_VDD_ON_N_PIN, (turn_on ? GPIO_OUT : GPIO_IN));
}

constant int SI5351A_I2C_ADDR=0x60;

static uint8_t s_i2c_buf[16];

int i2cWrite(uint8_t reg, uint8_t val){    // write reg via i2c
  s_i2c_buf[0] = reg;
  s_i2c_buf[1] = val;

  int res;
  res = i2c_write_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, s_i2c_buf, 2, false, 1000);

  if (res < PICO_ERROR_NONE) {
    #if defined(DEVMODE)
    printf("I2C error %d: reg:%02x val:%02x\n", res, reg, val);
    #endif
  }
  return res;
}

int i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt){   // write array
  s_i2c_buf[0] = reg;
  memcpy(&s_i2c_buf[1], vals, vcnt);

  int res;
  res = i2c_write_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, s_i2c_buf, (vcnt + 1), false, 10000);

  if (res < PICO_ERROR_NONE) {
    #if defined(DEVMODE)
    printf("I2C error %d: reg:%02x\n", res, reg);
    #endif
  }
  return res;
}

/*
    Si5351A related functions
    Developed by Kazuhisa "Kazu" Terasaki AG6NS
    https://github.com/kaduhi/AFSK_to_FSK_VFO
    This code was developed originally for QRPGuys AFP-FSK Digital Transceiver III kit
    https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii
    https://qrpguys.com/wp-content/uploads/2022/09/ft8_v1.4_092522-1.zip
 */

// The Si5351 consists of two main stages: two PLLs which are locked to the reference oscillator (a 25/27 MHz crystal) and which can be set from 600 to 900 MHz, and the output (multisynth) clocks which are locked to a PLL of choice and can be set from 500 kHz to 200 MHz (per the datasheet, although it does seem to be possible to set an output up to 225 MHz).

// Calibration
// There will be some inherent error in the reference oscillator's actual frequency, so we can account for this by measuring the difference between the uncalibrated actual and nominal output frequencies, then using that difference as a correction factor in the library. The init() and set_correction() methods use a signed integer calibration constant measured in parts-per-billion. The easiest way to determine this correction factor is to measure a 10 MHz signal from one of the clock outputs (in Hz, or better resolution if you can measure it), scale it to parts-per-billion, then use it in the set_correction() method in future use of this particular reference oscillator. Once this correction factor is determined, it should not need to be measured again for the same reference oscillator/Si5351 pair unless you want to redo the calibration. With an accurate measurement at one frequency, this calibration should be good across the entire tuning range.

// The calibration method is called like this:
// si5351.set_correction(-6190, SI5351_PLL_INPUT_XO);

// However, you may use the third argument in the init() method to specify the frequency correction and may not actually need to use the explict set_correction() method in your code.

// A handy calibration program is provided with the library in the example folder named si5351_calibration. To use it, simply hook up your Arduino to your Si5351, then connect it to a PC with the Arduino IDE. Connect the CLK0 output of the Si5351 to a frequency counter capable of measuring at 10 MHz (the more resolution, the better). Load the sketch then open the serial terminal window. Follow the prompts in the serial terminal to change the output frequency until your frequency counter reads exactly 10.000 000 00 MHz. The output from the Arduino on your serial terminal will tell you the correction factor you will need for future use of that reference oscillator/Si5351 combination.

// One thing to note: the library is set for a 25 MHz reference crystal

// correction is parts per billion for the frequency used/measured
// Could try a number of correction values and decide which to use (try 10 WSPR with correction 10/20/50/100/500/1000?)

constant int SI5351_TCXO_FREQ=                26000000;

constant int SI5351A_OUTPUT_ENABLE_CONTROL=   3;
constant int SI5351A_CLK0_CONTROL=            16;
constant int SI5351A_CLK1_CONTROL=            17;
constant int SI5351A_CLK7_CONTROL=            23;
constant int SI5351A_PLLB_BASE=               34;
constant int SI5351A_MULTISYNTH0_BASE=        42;
constant int SI5351A_MULTISYNTH1_BASE=        50;
constant int SI5351A_PLL_RESET=               177;

constant int SI5351A_CLK0_MS0_INT=            (1 << 6);
constant int SI5351A_CLK0_MS0_SRC_PLLB=       (1 << 5);
constant int SI5351A_CLK1_MS1_INT=            (1 << 6);
constant int SI5351A_CLK1_MS1_SRC_PLLB=       (1 << 5);

constant int SI5351A_CLK0_SRC_MULTISYNTH_0=   (3 << 2);
constant int SI5351A_CLK1_SRC_MULTISYNTH_0=   (2 << 2);

constant int SI5351A_CLK1_CLK1_INV=           (1 << 4);
constant int SI5351A_CLK1_SRC_MULTISYNTH_1=   (3 << 2);

constant int SI5351A_CLK0_IDRV_8MA=           (3 << 0);
constant int SI5351A_CLK0_IDRV_6MA=           (2 << 0);
constant int SI5351A_CLK0_IDRV_4MA=           (1 << 0);
constant int SI5351A_CLK0_IDRV_2MA=           (0 << 0);

constant int SI5351A_CLK1_IDRV_8MA=           (3 << 0);
constant int SI5351A_CLK1_IDRV_6MA=           (2 << 0);
constant int SI5351A_CLK1_IDRV_4MA=           (1 << 0);
constant int SI5351A_CLK1_IDRV_2MA=           (0 << 0);

constant int SI5351A_PLL_RESET_PLLB_RST=      (1 << 7);

static uint32_t prev_ms_div = 0;
static uint8_t s_regs[8];
static uint8_t s_vfo_drive_strength[3];  // 0:2mA, 1:4mA, 2:6mA, 3:8mA

void si5351a_setup_PLLB(uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t p1 = 128 * mult + ((128 * num) / denom) - 512;
  uint32_t p2 = 128 * num - denom * ((128 * num) / denom);
  uint32_t p3 = denom;

  s_regs[0] = (uint8_t)(p3 >> 8);
  s_regs[1] = (uint8_t)p3;
  s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
  s_regs[3] = (uint8_t)(p1 >> 8);
  s_regs[4] = (uint8_t)p1;
  s_regs[5] = ((uint8_t)(p3 >> 12) & 0xf0) | ((uint8_t)(p2 >> 16) & 0x0f);
  s_regs[6] = (uint8_t)(p2 >> 8);
  s_regs[7] = (uint8_t)p2;

  static uint8_t s_regs_prev[8];
  uint8_t start = 0;
  uint8_t end = 7;
  if (prev_ms_div != 0) {
    for (; start < 8; start++) {
      if (s_regs[start] != s_regs_prev[start]) break;
    }
    if (start == 8) return;
    for (; end > start; end--) {
      if (s_regs[end] != s_regs_prev[end]) break;
    }
  }
  uint8_t reg = SI5351A_PLLB_BASE + start;
  uint8_t len = end - start + 1;
  i2cWriten(reg, &s_regs[start], len);
  *((uint64_t *)s_regs_prev) = *((uint64_t *)s_regs);

}

// div must be even number
void si5351a_setup_multisynth0(uint32_t div)
{
  uint32_t p1 = 128 * div - 512;

  s_regs[0] = 0;
  s_regs[1] = 1;
  s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
  s_regs[3] = (uint8_t)(p1 >> 8);
  s_regs[4] = (uint8_t)p1;
  s_regs[5] = 0;
  s_regs[6] = 0;
  s_regs[7] = 0;
  i2cWriten(SI5351A_MULTISYNTH0_BASE, s_regs, 8);
  i2cWrite(SI5351A_CLK0_CONTROL, (SI5351A_CLK0_MS0_INT |
                                  SI5351A_CLK0_MS0_SRC_PLLB |
                                  SI5351A_CLK0_SRC_MULTISYNTH_0 |
                                  s_vfo_drive_strength[0]));

#ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
  i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);
  i2cWrite(SI5351A_CLK1_CONTROL, (SI5351A_CLK1_MS1_INT |
                                  SI5351A_CLK1_MS1_SRC_PLLB |
                                  SI5351A_CLK1_CLK1_INV |
                                  SI5351A_CLK1_SRC_MULTISYNTH_1 |
                                  s_vfo_drive_strength[0]));
#endif

#ifdef TEST_ONLY
  printf("VFO_DRIVE_STRENGTH: %d\n", (int)s_vfo_drive_strength[0]);
#endif //TEST_ONLY

}

static void si5351a_setup_multisynth1(uint32_t div)
{
  uint32_t p1 = 128 * div - 512;

  s_regs[0] = 0;
  s_regs[1] = 1;
  s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
  s_regs[3] = (uint8_t)(p1 >> 8);
  s_regs[4] = (uint8_t)p1;
  s_regs[5] = 0;
  s_regs[6] = 0;
  s_regs[7] = 0;
  i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);

  i2cWrite(SI5351A_CLK1_CONTROL, (SI5351A_CLK1_MS1_INT |
                                  SI5351A_CLK1_MS1_SRC_PLLB |
                                  SI5351A_CLK1_SRC_MULTISYNTH_1 |
                                  s_vfo_drive_strength[1]));
#ifdef TEST_ONLY
  printf("VFO_DRIVE_STRENGTH: %d\n", (int)s_vfo_drive_strength[1]);
#endif //TEST_ONLY
}

// FIX! compare to https://github.com/etherkit/Si5351Arduino
// Si5351 Library (V2) for Arduino 
static void si5351a_reset_PLLB(void)
{
  i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
}

// freq is in 28.4 fixed point number, 0.0625Hz resolution
void vfo_set_freq_x16(uint8_t clk_number, uint32_t freq)
{
  constant int PLL_MAX_FREQ=        900000000;
  constant int PLL_MIN_FREQ=        600000000;
  constant int PLL_MID_FREQ=        ((PLL_MAX_FREQ + PLL_MIN_FREQ) / 2); // divide by 2 result must be integer
  constant int PLL_DENOM_MAX=       0x000fffff;

  uint32_t ms_div = PLL_MID_FREQ / (freq >> PLL_CALCULATION_PRECISION) + 1;
  ms_div &= 0xfffffffe;   // make it even number

  uint32_t pll_freq = ((uint64_t)freq * ms_div) >> PLL_CALCULATION_PRECISION;

  uint32_t tcxo_freq = SI5351_TCXO_FREQ;
  uint32_t pll_mult   = pll_freq / tcxo_freq;
  uint32_t pll_remain = pll_freq - (pll_mult * tcxo_freq);
  uint32_t pll_num    = (uint64_t)pll_remain * PLL_DENOM_MAX / tcxo_freq;
  si5351a_setup_PLLB(pll_mult, pll_num, PLL_DENOM_MAX);

  if (ms_div != prev_ms_div) {
    prev_ms_div = ms_div;
    if (clk_number == 0) {
      si5351a_setup_multisynth0(ms_div);
    }
    else {
      // this was for setting up the aprs clock on clk_num == 1?
      si5351a_setup_multisynth1(ms_div);
    }
    si5351a_reset_PLLB();
  }
}

static uint8_t  si5351bx_clken = 0xff;

void vfo_turn_on_clk_out(uint8_t clk_number)
{
  uint8_t enable_bit = 1 << clk_number;

#ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
  if (clk_number == 0) {
    enable_bit |= 1 << 1;
  }
#endif
  si5351bx_clken &= ~enable_bit;
  i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
}

void vfo_turn_off_clk_out(uint8_t clk_number)
{
  uint8_t enable_bit = 1 << clk_number;
#ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
  if (clk_number == 0) {
    enable_bit |= 1 << 1;
  }
#endif
  si5351bx_clken |= enable_bit;
  i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
}

void vfo_set_drive_strength(uint8_t clk_number, uint8_t strength)
{
  s_vfo_drive_strength[clk_number] = strength;

  // reset the prev_ms_div to force vfo_set_freq_x16() to call si5351a_setup_multisynth1() next time
  prev_ms_div = 0;
}

bool vfo_is_on(void)
{
  return gpio_is_dir_out(VFO_VDD_ON_N_PIN);
}

// what is vfo_clk2 ? is that another PLL? is that used for calibration?
void vfo_turn_on(uint8_t clk_number)
{
  if (vfo_is_on()) return;    // already on

  gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
  vfo_set_power_on(true);
  // sleep_ms(100);
  busy_wait_us_32(100000);

  // output 7MHz on CLK0
  uint8_t reg;
  while (i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff) < PICO_ERROR_NONE) {   // Disable all CLK output drivers
    i2c_deinit(VFO_I2C_INSTANCE);
    // sleep_ms(10);
    busy_wait_us_32(10000);
    i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);

    gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
    gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);

    // sleep_ms(10);
    busy_wait_us_32(10000);
  }

  for (reg = SI5351A_CLK0_CONTROL; reg <= SI5351A_CLK7_CONTROL; reg++) i2cWrite(reg, 0xCC);    // Powerdown CLK's

  static const uint8_t s_ms_values[] = { 0, 1, 0x0C, 0, 0, 0, 0, 0 };
  i2cWriten(42, (uint8_t *)s_ms_values, 8);   // set MS0 for div_4 mode (minimum division)
  i2cWriten(50, (uint8_t *)s_ms_values, 8);   // set MS1 for div_4 mode (minimum division)
  i2cWriten(58, (uint8_t *)s_ms_values, 8);   // set MS2 for div_4 mode (minimum division)

  static const uint8_t s_pll_values[] = { 0, 0, 0, 0x05, 0x00, 0, 0, 0 };
  i2cWriten(26, (uint8_t *)s_pll_values, 8);  // set PLLA for div_16 mode (minimum even integer division)
  i2cWriten(34, (uint8_t *)s_pll_values, 8);  // set PLLB for div_16 mode (minimum even integer division)

  i2cWrite(149, 0x00);  // Disable Spread Spectrum
  i2cWrite(177, 0xA0);  // Reset PLLA and PLLB
  i2cWrite(187, 0x00);  // Disable all fanout

  prev_ms_div = 0;

  // uint32_t freq = 7040000UL << PLL_CALCULATION_PRECISION;

  // FIX! make this the base freq for the band?
  uint32_t freq = 14097000UL << PLL_CALCULATION_PRECISION;
  vfo_set_freq_x16(clk_number, freq);

  si5351bx_clken = 0xff;
  vfo_turn_on_clk_out(clk_number);
}

void vfo_turn_off(void)
{
  if (!vfo_is_on()) return;   // already off

  // disable all clk output
  si5351bx_clken = 0xff;
  i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
  // sleep_ms(10);
  busy_wait_us_32(10000);

  vfo_set_power_on(false);
  gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_NULL);
  gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_NULL);
}