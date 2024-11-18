// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.


// besides comparing to the arduino library, can compare to:
// "Multipurpose signal generator with SI5351"
// https://github.com/pu2clr/SI5351

// for busy_wait_us_32()
#include <Arduino.h>
#include "si5351_functions.h"
#include <stdlib.h>
#include <cstring>
#include "hardware/gpio.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog
#include "defines.h"

// for i2c0
#include "hardware/i2c.h"
#define VFO_I2C_INSTANCE i2c0

// FIX! should these be in tracker.ino (for consistency?)
extern const int SI5351A_I2C_ADDR;
extern const int VFO_I2C0_SCL_HZ;

extern uint32_t XMIT_FREQUENCY;
extern bool DEVMODE;
// decode of _verbose 0-9
extern bool VERBY[10];

extern char _tx_high[2];  // 0 is 2mA si5351. 1 is 8mA si5351
extern char _correction[7];  // parts per billion -3000 to 3000. default 0

extern const int Si5351Pwr;
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

extern const int PLL_CALCULATION_PRECISION;

static bool vfo_turn_on_completed = false;
static bool vfo_turn_off_completed = false;


//****************************************************
// removed static
void vfo_init(void) {
    if (VERBY[0]) Serial.println(F("vfo_init START"));
    // turn ON VFO VDD
    // pin 4 ?
    pinMode(Si5351Pwr, OUTPUT);

    // this is also pin 4
    gpio_init(VFO_VDD_ON_N_PIN);
    gpio_pull_up(VFO_VDD_ON_N_PIN);
    gpio_put(VFO_VDD_ON_N_PIN, 0);

    // init I2C0 for VFO
    i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);

    gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
    gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
    if (VERBY[0]) Serial.println(F("vfo_init END"));
}

//****************************************************
// removed static
void vfo_set_power_on(bool turn_on) {
    if (VERBY[0]) Serial.printf("vfo_set_power_on START %u" EOL, turn_on);
    static bool s_is_on = false;
    // if (turn_on == s_is_on) return;

    if ( turn_on) {
        Serial.printf("set VDD_ON_N_PIN%d LOW (power on)" EOL, VFO_VDD_ON_N_PIN);
        digitalWrite(VFO_VDD_ON_N_PIN, LOW);
    } else {
        Serial.printf("set VDD_ON_N_PIN%d HIGH (power off)" EOL, VFO_VDD_ON_N_PIN);
        digitalWrite(VFO_VDD_ON_N_PIN, HIGH);
    }

    // always just turn it on!
    s_is_on = turn_on;
     
    // FIX! kevin 11/18/24
    // huh? don't change the direction when it's on vs off
    // we can just read the level
    // gpio_set_dir(VFO_VDD_ON_N_PIN, (turn_on ? GPIO_OUT : GPIO_IN));

    if (VERBY[0]) Serial.printf("vfo_set_power_on END %u" EOL, s_is_on);
}


//****************************************************
int i2cWrite(uint8_t reg, uint8_t val) {  // write reg via i2c
    if (VERBY[0]) Serial.printf("i2cWrite START reg %02x val %02x" EOL, reg, val);
    // FIX! shouldn't this be local ? or does it setup data for i2cWriten
    // moved here to be local, and not static (shared) anymore
    // only need length 2!
    uint8_t s_i2c_buf[2];
    s_i2c_buf[0] = reg;
    s_i2c_buf[1] = val;

    int res;
    res = i2c_write_timeout_us(VFO_I2C_INSTANCE,
        SI5351A_I2C_ADDR, s_i2c_buf, 2, false, 1000);

    if (res < PICO_ERROR_NONE) {
        if (VERBY[0]) Serial.printf("I2C write error %d: reg:%02x val:%02x" EOL, res, reg, val);
    }
    if (VERBY[0]) Serial.printf("i2cWrite END reg %02x val %02x" EOL, reg, val);
    return res;
}

    
//****************************************************
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
// i2c_read_timeout_us
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html#ga9662f16f2e0def852f8fc051e695528d

// i2c_read_timeout_us()
// static int i2c_read_timeout_us (
// i2c_inst_t * i2c,
// uint8_t 	addr,
// uint8_t * dst,
// size_t 	len,
// bool 	nostop,
// uint 	timeout_us 
// )		
//
// Attempt to read specified number of bytes from address, with timeout.
// 
// Parameters
// i2c	Either i2c0 or i2c1
// addr	7-bit address of device to read from
// dst	Pointer to buffer to receive data
// len	Length of data in bytes to receive
// nostop	If true, master retains control of the bus at the end of the transfer (no Stop is issued), and the next transfer will begin with a Restart rather than a Start.
// timeout_us	The time that the function will wait for the entire transaction to complete
// Returns
// Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged, no device present, or PICO_ERROR_TIMEOUT if a timeout occurred.

// just reads two byte2
int i2cReadTest(uint8_t reg, uint8_t val) {  // read reg via i2c
    if (VERBY[0]) Serial.printf("i2cReadTest START reg %02x val %02x" EOL, reg, val);
    // FIX! shouldn't this be local ? or does it setup data for i2cWriten
    // moved here to be local, and not static (shared) anymore
    // only need length 2!
    uint8_t s_i2c_buf[2];
    s_i2c_buf[0] = reg;
    s_i2c_buf[1] = 254; // a fixed value that should be overwritten?
    
    uint8_t val_orig = val;
    int res;
    res = i2c_read_timeout_us(VFO_I2C_INSTANCE,
        SI5351A_I2C_ADDR, s_i2c_buf, 2, false, 1000);

    val = s_i2c_buf[1];

    if (res < PICO_ERROR_NONE) {
        if (VERBY[0]) Serial.printf("I2C read error %d: reg:%02x val:%02x" EOL, res, reg, val);
    }
    else {
        if (VERBY[0]) Serial.printf("I2C read okay %d: reg:%02x val:%02x" EOL, res, reg, val);
    }

    if (VERBY[0]) Serial.printf("i2cReadTest END reg %02x val %02x" EOL, reg, val);
    return res;
}




//****************************************************
int i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt) {   // write array
    if (VERBY[0]) {
        Serial.printf("i2cWriten START reg %02x vcnt %u" EOL, reg, vcnt);
        for (uint8_t i = 0; i < vcnt; i++) {
            Serial.printf("val i %d %u" EOL, i, *(vals + i));
        }
    }
        
    // FIX! shouldn't this be local ? or does it use the data from i2cWrite
    uint8_t s_i2c_buf[16];
    // moved here to be local, and not static (shared) anymore
    s_i2c_buf[0] = reg;

    // because of the large vcnt, the buf is length 16?
    memcpy(&s_i2c_buf[1], vals, vcnt);

    int res;
    res = i2c_write_timeout_us(VFO_I2C_INSTANCE,
        SI5351A_I2C_ADDR, s_i2c_buf, (vcnt + 1), false, 10000);

    if (res < PICO_ERROR_NONE) {
        if (VERBY[0]) Serial.printf("I2C error %d: reg:%02x" EOL, res, reg);
    }

    if (VERBY[0]) Serial.printf("i2cWriten START reg %02x vcnt %u" EOL, reg, vcnt);
    return res;
}

//****************************************************
/*
    Si5351A related functions
    Developed by Kazuhisa "Kazu" Terasaki AG6NS
    https://github.com/kaduhi/AFSK_to_FSK_VFO ..last update afsk_to_fsk_vfo.ino 6/30/21
    This code was developed originally for QRPGuys AFP-FSK Digital Transceiver III kit
    https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii
    https://qrpguys.com/wp-content/uploads/2022/09/ft8_v1.4_092522-1.zip
    description:
    https://qrpguys.com/wp-content/uploads/2021/06/afp_fsk_061921.pdf

 */


static uint32_t prev_ms_div = 0;
static uint8_t s_regs[8];
// updated with config _tx_high during vfo_turn_on()
// 0:2mA, 1:4mA, 2:6mA, 3:8mA
static uint8_t s_vfo_drive_strength[3];

//****************************************************
// FIX! removed static. hmm maybe add back..should only call from this file?
void si5351a_setup_PLLB(uint8_t mult, uint32_t num, uint32_t denom) {
    if (VERBY[0]) Serial.printf("si5351a_setup_PLLB START mult %u num %lu denom %lu" EOL, mult, num, denom);
    static uint8_t s_regs_prev[8];

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

    // start and end are looked at below, if prev_ms_div = 0
    // so these are out of the for loops
    uint8_t start = 0;
    uint8_t end = 7;
    // FIX! why always use the s_regs_prev buffer to copy to s_regs? (overwrite?)
    // it means this is a one shot kind of deal? just one call?
    // but what if we change bands?
    // FIX! disabled this from original Kazu. not sure why you would restore from s_regs_prev
    if (prev_ms_div != 0) {
        for (; start < 8; start++) {
            if (s_regs[start] != s_regs_prev[start]) break;
        }
        // FIX! is this detecting a non-change?
        if (start == 8) return;

        for (; end > start; end--) {
            // is this so we just write the start to end that has changed?
            if (s_regs[end] != s_regs_prev[end]) break;
        }
    }

    uint8_t reg = SI5351A_PLLB_BASE + start;
    uint8_t len = end - start + 1;
    i2cWriten(reg, &s_regs[start], len);
    *((uint64_t *)s_regs_prev) = *((uint64_t *)s_regs);
    if (VERBY[0]) Serial.printf("si5351a_setup_PLLB END mult %u num %lu denom %lu" EOL, mult, num, denom);
}

//****************************************************
// div must be even number
// removed static
void si5351a_setup_multisynth0(uint32_t div) {
    if (VERBY[0]) Serial.printf("si5351a_setup_multisynth0 START div %lu" EOL, div);
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


    // old #ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
    i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);
    i2cWrite(SI5351A_CLK1_CONTROL, (SI5351A_CLK1_MS1_INT |
                                  SI5351A_CLK1_MS1_SRC_PLLB |
                                  SI5351A_CLK1_CLK1_INV |
                                  SI5351A_CLK1_SRC_MULTISYNTH_1 |
                                  s_vfo_drive_strength[0]));
    // old #endif
    if (VERBY[0]) Serial.printf("VFO_DRIVE_STRENGTH: %d" EOL, (int)s_vfo_drive_strength[0]);
    if (VERBY[0]) Serial.printf("si5351a_setup_multisynth0 END div %lu" EOL, div);
}

//****************************************************
void si5351a_setup_multisynth1(uint32_t div) {
    if (VERBY[0]) Serial.printf("si5351a_setup_multisynth1 START div %lu" EOL, div);
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
    if (VERBY[0]) {
        Serial.printf("VFO_DRIVE_STRENGTH: %d" EOL, (int)s_vfo_drive_strength[1]);
    }
    if (VERBY[0]) Serial.printf("si5351a_setup_multisynth1 END div %lu" EOL, div);
}

// FIX! compare to https://github.com/etherkit/Si5351Arduino
// libraries: wget https://github.com/etherkit/Si5351Arduino/archive/refs/heads/master.zip
// but not used? could use here with:
// #include <si5351.h>
// #include <Wire.h>
// Si5351 si5351;

//****************************************************
// we don't user PLLA ?
void si5351a_reset_PLLB(void) {
  if (VERBY[0]) Serial.println(F("si5351a_reset_PLLB START"));
  i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
  if (VERBY[0]) Serial.println(F("si5351a_reset_PLLB END"));
}

//****************************************************
// freq is in 28.4 fixed point number, 0.0625Hz resolution
void vfo_set_freq_x16(uint8_t clk_num, uint32_t freq) {
  if (VERBY[0]) Serial.printf("vfo_set_freq_x16 START clk_num %u freq %lu" EOL, clk_num, freq);
    const int PLL_MAX_FREQ  = 900000000;
    const int PLL_MIN_FREQ  = 600000000;

    // divide by 2 result must be integer
    const int PLL_MID_FREQ  = ((PLL_MAX_FREQ + PLL_MIN_FREQ) / 2);
    const int PLL_DENOM_MAX = 0x000fffff;

    uint32_t ms_div = PLL_MID_FREQ / (freq >> PLL_CALCULATION_PRECISION) + 1;
    ms_div &= 0xfffffffe;   // make it even number

    uint32_t pll_freq = ((uint64_t)freq * ms_div) >> PLL_CALCULATION_PRECISION;

    uint32_t tcxo_freq = SI5351_TCXO_FREQ;
    uint32_t pll_mult   = pll_freq / tcxo_freq;
    uint32_t pll_remain = pll_freq - (pll_mult * tcxo_freq);
    uint32_t pll_num    = (uint64_t)pll_remain * PLL_DENOM_MAX / tcxo_freq;

    // FIX! this has sticky s_regs_prev state that it uses if called multiple times?
    si5351a_setup_PLLB(pll_mult, pll_num, PLL_DENOM_MAX);

    // only if it changes
    if (ms_div != prev_ms_div) {
        prev_ms_div = ms_div;
        if (clk_num == 0) si5351a_setup_multisynth0(ms_div);
        // this used to be for setting up the aprs clock on clk_num == 1?
        else si5351a_setup_multisynth1(ms_div);
        si5351a_reset_PLLB();
    }
    if (VERBY[0]) Serial.printf("vfo_set_freq_x16 END clk_num %u freq %lu" EOL, clk_num, freq);
}

//****************************************************
static uint8_t  si5351bx_clken = 0xff;
void vfo_turn_on_clk_out(uint8_t clk_num) {
    if (VERBY[0]) Serial.printf("vfo_turn_on_clk_out START clk_num %u" EOL, clk_num);
    uint8_t enable_bit = 1 << clk_num;

    #ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
    if (clk_num == 0) {
        enable_bit |= 1 << 1;
    }
    #endif
    si5351bx_clken &= ~enable_bit;
    i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
    if (VERBY[0]) Serial.println(F("vfo_turn_on_clk_out END"));
}

void vfo_turn_off_clk_out(uint8_t clk_num) {
    if (VERBY[0]) Serial.println(F("vfo_turn_off_clk_out START"));
    uint8_t enable_bit = 1 << clk_num;
    // always now
    // #ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
    if (clk_num == 0) {
        enable_bit |= 1 << 1;
    }
    // #endif
    si5351bx_clken |= enable_bit;
    i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
    if (VERBY[0]) Serial.printf("vfo_turn_on_clk_out END clk_num %u" EOL, clk_num);
}

//****************************************************
void vfo_set_drive_strength(uint8_t clk_num, uint8_t strength) {
    if (VERBY[0]) Serial.printf("vfo_set_drive_strength START clk_num %u" EOL, clk_num);
    // only called during the initial vfo_turn_on()
    s_vfo_drive_strength[clk_num] = strength;
    // reset the prev_ms_div to force vfo_set_freq_x16()
    // to call si5351a_setup_multisynth1() next time
    prev_ms_div = 0;
    if (VERBY[0]) Serial.printf("vfo_set_drive_strength END clk_num %u" EOL, clk_num);
}

//****************************************************
bool vfo_is_on(void) {
    // power on and completed successfully
    // FIX! in vs out doesn't make sense

    // static bool gpio_is_dir_out	(	uint 	gpio	)	
    // Check if a specific GPIO direction is OUT.
    // gpio	GPIO number
    // returns
    // true if the direction for the pin is OUT

    // can use gpio_get_out_level()

    // gpio_get_out_level()
    // static bool gpio_get_out_level	(uint gpio )	
    // Determine whether a GPIO is currently driven high or lowThis function returns the high/low output level most recently assigned to a GPIO via gpio_put() or similar. This is the value that is presented outward to the IO muxing, not the input level back from the pad (which can be read using gpio_get()).
    // 
    // To avoid races, this function must not be used for read-modify-write sequences when driving GPIOs – instead functions like gpio_put() should be used to atomically update GPIOs. This accessor is intended for debug use only.

    // gpio	GPIO number
    // Returns
    // true if the GPIO output level is high, false if low.
    return (!gpio_get_out_level(VFO_VDD_ON_N_PIN) && vfo_turn_on_completed);
}

//****************************************************
bool vfo_is_off(void) {
    // power on and completed successfully
    // return (gpio_is_dir_out(VFO_VDD_ON_N_PIN) && vfo_turn_off_completed);
    return (gpio_get_out_level(VFO_VDD_ON_N_PIN) && vfo_turn_off_completed);
}

// what is vfo_clk2 ? is that another PLL? is that used for calibration?


//****************************************************
void vfo_turn_on(uint8_t clk_num) {
    if (VERBY[0]) Serial.printf("vfo_turn_on START clk_num %u" EOL, clk_num);

    // already on successfully
    if (vfo_is_on()) return;
    vfo_turn_on_completed = false;
    vfo_turn_off_completed = false;
    // sets state to be used later
    if (_tx_high[0] == '0') {
        vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_2MA);
        vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_2MA);
    } else {
        vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_8MA);
        vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_8MA);
    }

    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);

    vfo_set_power_on(true);

    // sleep_ms(100);
    busy_wait_us_32(100000);

    // output 7MHz on CLK0
    uint8_t reg;
    // Disable all CLK output drivers
    if (VERBY[0]) Serial.print(F("vfo_turn_on trying to i2cWrite SI5351A_OUTPUT_ENABLE_CONTROL with 0xff"));
    while (i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff) < PICO_ERROR_NONE) {
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
        if (VERBY[0]) Serial.print(F("vfo_turn_on trying to init the I2C0 pins inside loop"));
    }
    if (VERBY[0]) Serial.print(F("vfo_turn_on done trying to init the I2C0 pins in loop"));

    // Powerdown CLK's
    for (reg = SI5351A_CLK0_CONTROL; reg <= SI5351A_CLK7_CONTROL; reg++) {
        i2cWrite(reg, 0xCC);
    }
    static const uint8_t s_ms_values[] = { 0, 1, 0x0C, 0, 0, 0, 0, 0 };
    i2cWriten(42, (uint8_t *)s_ms_values, 8);   // set MS0 for div_4 mode (min. division)
    i2cWriten(50, (uint8_t *)s_ms_values, 8);   // set MS1 for div_4 mode (min. division)
    i2cWriten(58, (uint8_t *)s_ms_values, 8);   // set MS2 for div_4 mode (min.division)

    static const uint8_t s_pll_values[] = { 0, 0, 0, 0x05, 0x00, 0, 0, 0 };
    // set PLLA for div_16 mode (minimum even integer division)
    i2cWriten(26, (uint8_t *)s_pll_values, 8);
    // set PLLB for div_16 mode (minimum even integer division)
    i2cWriten(34, (uint8_t *)s_pll_values, 8);

    i2cWrite(149, 0x00);  // Disable Spread Spectrum
    i2cWrite(177, 0xA0);  // Reset PLLA and PLLB
    i2cWrite(187, 0x00);  // Disable all fanout

    prev_ms_div = 0;

    // do a parts per billion correction?
    uint32_t freq = XMIT_FREQUENCY;
    if (atoi(_correction) != 0) {
        // this will be a floor divide
        // FIX! what range _correction will be ? -3000 to 3000
        uint32_t orig_freq = freq;
        freq = freq + (atoi(_correction) * freq / 1000000000UL);
        if (VERBY[0]) {
            Serial.printf("correction shifts %d orig freq %lu, to new freq, %lu" EOL,
                atoi(_correction), orig_freq, freq);
        }
    }

    freq = (uint32_t) freq << PLL_CALCULATION_PRECISION;
    vfo_set_freq_x16(clk_num, freq);

    si5351bx_clken = 0xff;
    vfo_turn_on_clk_out(clk_num);
    vfo_turn_on_completed = true;
    if (VERBY[0]) Serial.printf("vfo_turn_on END clk_num %u" EOL, clk_num);
}

//****************************************************
void vfo_turn_off(void) {
    if (VERBY[0]) Serial.println(F("vfo_turn_off START"));
    // already off successfully?
    if (vfo_is_off()) return;
    vfo_turn_on_completed = false;
    vfo_turn_off_completed = false;

    // disable all clk output
    si5351bx_clken = 0xff;
    i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
    // sleep_ms(10);
    busy_wait_us_32(10000);

    vfo_set_power_on(false);
    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_NULL);
    vfo_turn_off_completed = true;
    if (VERBY[0]) Serial.println(F("vfo_turn_off END"));
}

//*****************************************************
// random notes for reference, from other code

// The Si5351 consists of two main stages:
// two PLLs which are locked to the reference oscillator (a 25/27 MHz crystal)
// and which can be set from 600 to 900 MHz,
// and the output (multisynth) clocks which are locked to a PLL of choice
// and can be set from 500 kHz to 200 MHz (per the datasheet,
// although it does seem to be possible to set an output up to 225 MHz).

// Calibration
// There will be some inherent error in the reference oscillator's actual frequency,
// so we can account for this by measuring the difference between the uncalibrated
// actual and nominal output frequencies, then using that difference as a correction
// factor in the library.

// The init() and set_correction() methods use a signed integer calibration constant
// measured in parts-per-billion.
// The easiest way to determine this correction factor is to measure a 14 MHz signal
// from one of the clock outputs (in Hz, or better resolution if you can measure it),
// scale it to parts-per-billion,
// then use it in the set_correction() method in future use of this particular
// reference oscillator.

// Once this correction factor is determined, it should not need to be measured again
// for the same reference oscillator/Si5351 pair unless you want to redo the
// calibration.
// With an accurate measurement at one frequency,
// this calibration should be good across the entire tuning range.

// The calibration method is called like this:
// si5351.set_correction(-6190, SI5351_PLL_INPUT_XO);

// However, you may use the third argument in the init() method to specify
// the frequency correction and may not actually need to use the
// explict set_correction() method in your code.

// One thing to note: the library is set for a 25 MHz reference crystal

// correction is parts per billion for the frequency used/measured
// Could try a number of correction values and decide which to use
// (try 10 WSPR with correction 10/20/50/100/500/1000?)

// another source for programming comparison
// https://dk7ih.de/a-simple-software-to-control-the-si5351a-generator-chip/
// https://cdn-shop.adafruit.com/datasheets/Si5351.pdf
