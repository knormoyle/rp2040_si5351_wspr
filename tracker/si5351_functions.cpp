// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// https://rfzero.net/documentation/rf/
// The table below shows the typical output power vs. current in the output stages
// running in push-pull with a T1 transformer
// Current [mA]
// 137 kHz    1 MHz     10 MHz    30 MHz    50 MHz    200 MHz
// 8 9,5 dBm  14,2 dBm  14,5 dBm  15,0 dBm  14,5 dBm  13,3 dBm
// 6 9,2 dBm  12,8 dBm  13,3 dBm  13,7 dBm  13,0 dBm  11,8 dBm
// 4 8,3 dBm  10,3 dBm  10,7 dBm  11,0 dBm  10,5 dBm   9,7 dBm
// 2 5,2 dBm   4,7 dBm   5,0 dBm   5,5 dBm   5,0 dBm   4,5 dBm

// Note running at 4ma output is just 4 dBm reduction. so maybe that should be our low power?

// nice api/functions (for comparison) at:
// https://github.com/pu2clr/SI5351 "Multipurpose signal generator with SI5351"
// https://github.com/pu2clr/SI5351/blob/master/source/si5351_signal_generator2/si5351wire.cpp
// the Arduino si5351 library is also useful
// https://github.com/etherkit/Si5351Arduino

// https://jeremyclark.ca/wp/telecom/si5351-clock-generator-programming-2/

// https://rfzero.net/tutorials/si5351a/
// https://dk7ih.de/a-simple-software-to-control-the-si5351a-generator-chip/
// also, examples from hans
// https://qrp-labs.com/synth/si5351ademo.html

// https://cdn-shop.adafruit.com/datasheets/Si5351.pdf

// do we set crystal load capacitance anywhere? Nope. default 10pf is good
// (the tcxo is spec'ed for 10pf? Default is 10pf in the chip, so don't need to program it.

// Manually Generating an Si5351 Register Map for 10-MSOP and 20-QFN Devices
// https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
// AN1234 Manually Generating an Si5351 Register Map for 16QFN Devices
// https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/an1234-si5351-16qfn-register-map.pdf
// CRYSTAL SELECTION GUIDE FOR Si5350/51 DEVICES
// https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN551.pdf

// an alternative flowchart for programming si5351
// https://nt7s.com/2018/02/si5351-programming-flowchart/

// overview and programming
// http://www.wa5bdu.com/programming-the-si5351a-synthesizer/

// interesting thoughts on using gcd to determine things?
// https://www.reddit.com/r/amateurradio/comments/lpdfpx/tutorial_how_to_find_the_exact_divider_ratios_fo/

//****************************************************
// Si5351A related functions
// forked from:
// Kazuhisa "Kazu" Terasaki AG6NS
// https://github.com/kaduhi/AFSK_to_FSK_VFO ..last update afsk_to_fsk_vfo.ino 6/30/21
// https://github.com/kaduhi/AFSK_to_FSK_VFO/tree/main
// This code was developed originally for QRPGuys AFP-FSK Digital Transceiver III kit
// https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii
// https://qrpguys.com/wp-content/uploads/2022/09/ft8_v1.4_092522-1.zip
// description:
// https://qrpguys.com/wp-content/uploads/2021/06/afp_fsk_061921.pdf
//****************************************************

#include <Arduino.h>
#include <stdlib.h>
#include <cstring>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
// https://github.com/raspberrypi/pico-sdk/blob/master/src/common/pico_divider_headers/include/pico/divider.h
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__pico__divider.html
// #include "pico.h"
// #include "hardware/divider.h"
// div_u64u64()

#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

#include "si5351_functions.h"
#include "print_functions.h"
#include "led_functions.h"
#include "u4b_functions.h"
#include "farey_functions.h"
#include "global_structs.h"

// These are in arduino-pio core
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries
// #include <SPI.h>
// #include <Wire.h>

// this is the target PLL freq when making muliplier/divider initial calculations
// set in tracker.ino
extern uint64_t PLL_FREQ_TARGET;
extern bool USE_FAREY_WITH_PLL_REMAINDER;
extern bool USE_FAREY_CHOPPED_PRECISION;
extern bool TEST_FAREY_WITH_PLL_REMAINDER;
extern bool DISABLE_VFO_CALC_CACHE;
extern uint64_t PLL_CALC_SHIFT;
extern bool USE_SI5351A_CLK_POWERDOWN_FOR_WSPR_MODE;

extern bool USE_MFSK16_SHIFT;

// from tracker.ino
extern uint32_t SI5351_TCXO_FREQ;  // 26 mhz with correction already done
extern const int SI5351A_I2C_ADDR;
extern const int VFO_I2C_SCL_HZ;

// Tried a concave optimization (stepwise) on this
// now that we have optimum denoms from a spreadsheet, (for given mult/div per band)
// this is really just a double-check in case pll_mult/pll_div change with
// a different algo, and I need the hard-wired denom adjusted..
// Assuming pll_num and pll_denom can optimize independently one at a time?

// starts with the max;
extern uint32_t PLL_DENOM_OPTIMIZE;
extern uint32_t PLL_DENOM_OPTIMIZE_calced;

extern uint32_t XMIT_FREQUENCY;
// decode of _verbose 0-9
extern bool VERBY[10];

// FIX! update this based on solar elevation
extern uint8_t SOLAR_SI5351_TX_POWER;

extern ConfigStruct cc;

extern const int Si5351Pwr;
// FIX! are these just used on the Wire.begin?
// FIX! should this be a extern const. Or: only used here?
#define VFO_I2C_INSTANCE i2c0
extern const int VFO_I2C_SDA_PIN;
extern const int VFO_I2C_SCL_PIN;

static bool vfo_turn_on_completed = false;
static bool vfo_turn_off_completed = false;

static uint8_t si5351bx_clken = 0xff;
static bool s_is_on = false;

// updated with config cc._tx_high during vfo_turn_on()
// 0:2mA, 1:4mA, 2:6mA, 3:8mA
static uint8_t s_vfo_drive_strength[3] = { 0 };

// so we can just remember the state of the CLK control, and flip the PDN
// power down bit..because the clock enable doesn't seem to work
// on the ms5351m ..always on. Have to PLL reset if we flip PDN
// to maintain 180 phase relationship..see hans
static uint8_t s_CLK0_control_prev = { 0 };
static uint8_t s_CLK1_control_prev = { 0 };
static uint8_t s_CLK2_control_prev = { 0 };

static uint8_t s_PLLB_regs_prev[8] = { 0 };
static uint32_t s_PLLB_ms_div_prev = 0;
static uint32_t s_PLLB_pll_mult_prev = 0;
static uint32_t s_PLLB_pll_denom_prev = 0;

// only used because we set PLLA the same as PLLB, so it can lock?
// FIX! but if we set it up, does that cause it to be on when it was off before?

static uint8_t s_PLLA_regs_prev[8] = { 0 };
static uint32_t s_PLLA_ms_div_prev = 0;
static uint32_t s_PLLA_pll_mult_prev = 0;
static uint32_t s_PLLA_pll_denom_prev = 0;

//****************************************************
// https://wellys.com/posts/rp2040_arduino_i2c/
// If thinking about using Wire/Wire1
// There can only be two Wire interfaces, Wire and Wire1
// should have unique addresses for everything on a Wire*
// pi pico can have two Wire
// only one Wire and Wire pair can be active at a time
// https://github.com/lkoepsel/I2C/blob/main/Arduino/Pico/I2C_Scanner/I2C_Scanner.ino
//****************************************************
void vfo_init(void) {
    V1_println(F("vfo_init START"));
    Watchdog.reset();

    // clear any old remembered state we have outside of the si5351
    // hmm. maybe not this though
    si5351bx_clken = 0xff;
    s_PLLB_ms_div_prev = 0;
    memset(s_PLLB_regs_prev, 0, 8);
    s_PLLA_ms_div_prev = 0;
    memset(s_PLLA_regs_prev, 0, 8);

    memset(s_vfo_drive_strength, 0, 3);

    // turn ON VFO VDD. pin 4 ?
    // controls mosfet which controls both the tcxo and the si5351a vdd/vcc
    // do the init first
    gpio_init(Si5351Pwr);
    pinMode(Si5351Pwr, OUTPUT);
    // slow transition on the mosfet gate should help limit turn-on
    // current surge (whatever the default pll behavior is for si5351a)
    // if you set them to input after this, they will be high impedance!
    // again, we can't let a rp2040 pulldown do the assert transition
    // because the external R of 10k ohm on the gate of the mosfet
    // is stronger than the rp2040 pulldown (50k was it?)
    gpio_set_slew_rate(Si5351Pwr, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(Si5351Pwr, GPIO_DRIVE_STRENGTH_2MA);

    // was 12/26/24
    // FIX! remove pull_up to save power. Don't need it?
    // gpio_pull_up(Si5351Pwr);

    // was 12/26/24
    // no reason to turn it on during the setup init?
    // but need it on during the repeated inits
    // just get rid of the vfo_init in setup!
    gpio_put(Si5351Pwr, 0);

    // init I2C for VFO
    uint32_t actualRate = i2c_init(VFO_I2C_INSTANCE, VFO_I2C_SCL_HZ);
    V1_printf("vfo_init i2c actual rate: %lu" EOL, actualRate);
    // let it float if we don't drive the i2c?
    gpio_set_pulls(VFO_I2C_SDA_PIN, false, false);
    gpio_set_pulls(VFO_I2C_SCL_PIN, false, false);
    gpio_set_function(VFO_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C_SCL_PIN, GPIO_FUNC_I2C);
    V1_println(F("vfo_init END"));
    Watchdog.reset();
}

//****************************************************
void vfo_set_power_on(bool turn_on) {
    // this works even if s_is_on is wrong relative to current on state
    // doesn't turn off/on first
    V1_printf("vfo_set_power_on START %u" EOL, turn_on);
    Watchdog.reset();
    V1_flush();

    // don't make it dependent on s_is_on state
    // if (turn_on == s_is_on) return;
    if (turn_on) {
        V1_printf("set Si5351Pwr %d LOW (power on) before" EOL, Si5351Pwr);
        gpio_init(Si5351Pwr);
        pinMode(Si5351Pwr, OUTPUT_4MA);
        digitalWrite(Si5351Pwr, LOW);
        V1_printf("set Si5351Pwr %d LOW (power on) after" EOL, Si5351Pwr);
    } else {
        V1_printf("set VDD_ON_N_PIN %d HIGH (power off)" EOL, Si5351Pwr);
        digitalWrite(Si5351Pwr, HIGH);
    }

    // FIX! always? or should we only do it if was not on?
    // clear any old remembered state we have outside of the si5351
    // hmm. maybe not this though
    si5351bx_clken = 0xff;
    s_PLLB_ms_div_prev = 0;
    memset(s_PLLB_regs_prev, 0, 8);
    s_PLLA_ms_div_prev = 0;
    memset(s_PLLA_regs_prev, 0, 8);
    memset(s_vfo_drive_strength, 0, 3);

    V1_flush();

    // always just turn it on!
    s_is_on = turn_on;

    // don't change the direction when it's on vs off. LightAPRS code did
    // we can just read the level
    // gpio_set_dir(Si5351Pwr, (turn_on ? GPIO_OUT : GPIO_IN));

    V1_printf("vfo_set_power_on END %u" EOL, s_is_on);
    V1_flush();
}

//****************************************************
bool reserved_reg(uint8_t reg) {
    // hung on read of d7 (215)
    // hung on read of d8 (216)
    bool bad = false;
    switch (reg) {
        case 4: {}  // default mask state is not masking interrupt (0)
        case 5: {}
        case 6: {}
        case 7: {}
        case 8: {}
        case 10: {}
        case 11: {}
        case 12: {}
        case 13: {}
        case 14: {}
        case 173: {}
        case 174: {}
        case 175: {}
        case 176: {}
        case 178: {}
        case 179: {}
        case 180: {}
        case 181: {}
        case 182: bad = true; break;
        // possible range is already constrained to 255 by uint8_t size
        default: if (reg >= 184) bad = true;
    }
    switch (reg) {
        case 0:
            // SI5351A_DEVICE_STATUS can only read
            // reg 0: Device Status
            // 7: SYS_INIT System Initialization. 0: complete
            // 6: LOL_B PLLB Loss of Lock (PLL out of table 3 lock range or bad ref. clk). 0: locked
            // 5: LOL_A PLLB Loss of Lock (PLL out of table 3 lock range or bad ref. clk). 0: locked
            // 4: CLKIN Loss of Signal (Si5351C only) 0: normal
            // 3:2 Reserved
            // 1:0 Revision ID.
            bad = false; break;
        case 1:
            // SI5351A_INTERRUPT_STATUS_STICKY can read and write
            // reg 1: Interrupt Status Sticky 1: if interrupt since last cleared.
            // 7: SYS_INIT_STKY System Calibration 1: if interrupt since last cleared.
            // 6: LOL_B_STKY PLLB Loss of Lock 1: if interrupt since last cleared.
            // 5: LOL_A_STKY PLLB Loss of Lock 1: if interrupt since last cleared.
            // 4: LOS_STKY CLKIN Loss of Signal (Si5351C only) 1: if interrupt since last cleared.
            // 3:0  Reserved
            bad = false; break;

        // FIX! we should check these rules for bad multisynth?
        // multisynth3 thru multisynth7 ??
        // do invalid numbers write ? what happens?
        // This 8-bit number is the Multisynth divide ratio.
        // can only be even integers greater than or equal to 6.
        // All other divide values are invalid.
        // Si5351B and C are 8-outputs. Si5351a we use is only 3.
        // (there is 8 output version) that's why.
        default: if (reg >= 66 && reg <= 91) bad = true;
    }
    // for i2c test: reg that don't return exact pattern just written are considered "reserved"
    // terms of this test can't just do write than read and compare data.
    // i guess in normal balloon operation, that's just status bits, which we don't check
    // so okay to call them reserved here
    return bad;
}

//****************************************************
int i2cWrite(uint8_t reg, uint8_t val) {  // write reg via i2c
    // V1_printf("i2cWrite START reg %02x val %02x" EOL, reg, val);
    // FIX! shouldn't this be local ? or does it setup data for i2cWriten
    // moved here to be local, and not static (shared) anymore
    // only need length 2!
    uint8_t i2c_buf[2];
    i2c_buf[0] = reg;
    i2c_buf[1] = val;

    int res;
    if (reserved_reg(reg)) {
        // don't want to hang on a reserved reg. so don't send
        V1_printf("ERROR: i2cWrite reserved reg %u" EOL, reg);
        // make this a unique error to recognize my reserved reg detection
        res = 127;
    } else {
        // V1_printf("i2cWrite doing i2c_write_blocking reg %02x val %02x" EOL, reg, val);
        // res = i2c_write_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false, 1000);
        res = i2c_write_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false);

        if (res == 127) {
            V1_printf("BAD: reserved reg %d detected on i2cWrite" EOL, reg);
        } else if (res == 2) {
            // V1_printf("GOOD: res %d after i2cWrite" EOL, res);
        } else if (res == PICO_ERROR_GENERIC) {
            V1_printf("ERROR: res %d after i2cWrite %d" EOL, res, reg);
        } else {
            V1_printf("UNEXPECTED: res %d after i2cWrite %d" EOL, res, reg);
        }
    }
    // V1_printf("i2cWrite END reg %02x val %02x" EOL, reg, val);
    return res;
}

//****************************************************
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
// i2c_read_timeout_us
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html#ga9662f16f2e0def852f8fc051e695528d

// i2c_read_timeout_us()
// static int i2c_read_timeout_us (
// i2c_inst_t * i2c,
// uint8_t addr,
// uint8_t * dst,
// size_t len,
// bool nostop,
// uint timeout_us
// )
//
// Attempt to read specified number of bytes from address, with timeout.
//
// Parameters
// i2c Either i2c0 or i2c1
// addr 7-bit address of device to read from
// dst Pointer to buffer to receive data
// len Length of data in bytes to receive
// nostop If true, master retains control of the bus at
// the end of the transfer (no Stop is issued),
// and the next transfer will begin with a Restart rather than a Start.
// timeout_us The time that the function will wait for the entire transaction to complete

// Returns
// Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged,
// no device present, or PICO_ERROR_TIMEOUT if a timeout occurred.

// per https://github.com/earlephilhower/arduino-pico/discussions/1059

// uint8_t _address = SI5351A_I2C_ADDR;
// void writeRegister(uint8_t myRegister, uint8_t myValue) {
//     uint8_t error;
//     Wire.beginTransmission(_address);
//     Wire.write(myRegister);
//     Wire.write(myValue);
//     error = Wire.endTransmission();
//     if (error == 0) V1_print(F("I2C device found at address 0x"));
//     if (error == 4) V1_print(F("Unknown error at address 0x"));
// }
//
// byte readRegister(uint8_t myRegister) {
//     uint8_t error;
//     byte returnValue;
//
//     Wire.beginTransmission(_address);
//     Wire.write(myRegister);
//     error = Wire.endTransmission();
//     if (error == 0) V1_print(F("I2C device found at address 0x"));
//     if (error == 4) V1_print(F("Unknown error at address 0x"));
//
//     Wire.requestFrom(_address, 1, true);
//     // blocking
//     while (Wire.available()) {
//         returnValue = Wire.read();
//     }
//     return returnValue;
// }

//****************************************************
// do a read like this
// uint8_t val = 0xaa;
// static uint8_t reg = 0;
// res = i2cWrRead(reg, &val) ;

// FIX! don't need to read a stream of bytes? so no i2cWrReadn()?
// read reg via i2c
int i2cWrRead(uint8_t reg, uint8_t *val) {
    V1_printf("i2cWrRead START reg %02x val %02x" EOL, reg, *val);
    uint8_t i2c_buf[2];
    i2c_buf[0] = reg;
    i2c_buf[1] = 254;  // a fixed value that should be overwritten?

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
    int res;

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/error_8h_source.html
    // enum pico_error_codes {
    //     PICO_OK = 0,
    //     PICO_ERROR_NONE = 0,
    //     PICO_ERROR_TIMEOUT = -1,
    //     PICO_ERROR_GENERIC = -2,
    //     PICO_ERROR_NO_DATA = -3,
    //     PICO_ERROR_NOT_PERMITTED = -4,
    //     PICO_ERROR_INVALID_ARG = -5,
    //     PICO_ERROR_IO = -6,
    //     PICO_ERROR_BADAUTH = -7,
    //     PICO_ERROR_CONNECT_FAILED = -8,
    // };
    if (reserved_reg(reg)) {
        // don't want to hang on a reserved reg. so don't send
        V1_printf("ERROR: i2cWrRead reserved reg %u" EOL, reg);
        // make this a unique error to recognize my reserved reg detection
        res = 127;
    } else {
        // V1_print(F("i2cWrRead doing i2c_write_blocking then i2c_read_blocking" EOL));
        // new: 1/1/25 let these have timeout?
        V1_print(F("i2cWrRead doing i2c_write then i2c_read with 100ms timeout" EOL));
        int res1, res2;
        // FIX! should these be _timeout_us instead?
        // we currently don't use the read data for anything..just printing
        // res1 = i2c_write_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, &reg, 1, true);
        res1 = i2c_write_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, &reg, 1, true, 100000);
        // res2 = i2c_read_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 1, false);
        res2 = i2c_read_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 1, false, 100000);
        // copy the data we got to val location to return it.
        *val = i2c_buf[0];

        // see enums for other errors (all negative) at
        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/error_8h_source.html
        if (res1 == PICO_ERROR_GENERIC || res1 != 1)
            res = PICO_ERROR_GENERIC;
        else if (res2 == PICO_ERROR_GENERIC || res2 != 1)
            res = PICO_ERROR_GENERIC;
        else if (res1 == 1 && res2 == 1)
            res = 1;  // good one byte read! both parts
    }

    // cover all - errors above
    if (res == 127) {
        // my decode for reserved
    } else if (res == PICO_ERROR_GENERIC || res < 0) {
        V1_printf("ERROR: i2cWrRead() got bad res %d reg %02x val %02x" EOL,
            res, reg, *val);
    } else if (res == 1) {
        V1_printf("i2cWrRead() got good res %d reg %02x val %02x" EOL,
            res, reg, *val);
    } else {
        V1_printf("UNEXPECTED: i2cWrRead() got unexpected res %d reg %02x val %02x" EOL,
            res, reg, *val);
    }

    V1_printf("i2cWrRead END reg %02x val %02x" EOL, reg, *val);
    return res;

    // https://pschatzmann.github.io/pico-arduino/doc/html/class_pico_hardware_i2_c.html
    // https://github.com/earlephilhower/arduino-pico/discussions/1059
}

//****************************************************
int i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt) {   // write array
    if (false && VERBY[1]) {
        V1_printf("i2cWriten START reg %02x vcnt %u" EOL, reg, vcnt);
        for (uint8_t i = 0; i < vcnt; i++) {
            V1_printf("val i %d %u" EOL, i, *(vals + i));
        }
    }

    uint8_t i2c_buf[16];
    i2c_buf[0] = reg;
    // because of the large vcnt, the buf is length 16?
    memcpy(&i2c_buf[1], vals, vcnt);

    int res;
    if (reserved_reg(reg)) {
        // don't want to hang on a reserved reg. so don't send
        V1_printf("i2cWriten reserved reg %u", reg);
        // make this a unique error to recognize my reserved reg detection
        res = 127;
    } else {
        // V1_printf("i2cWriten doing i2c_write_blocking reg %02x " EOL, reg);
        // res = i2c_write_timeout_us(VFO_I2C_INSTANCE,
        res = i2c_write_blocking(VFO_I2C_INSTANCE,
            SI5351A_I2C_ADDR, i2c_buf, (vcnt + 1), false);  // addr + data (byte)
        if (res == 127) {
            V1_printf("BAD: reserved reg %d detected on i2cWriten" EOL, reg);
        } else if (res == (1 + (int) vcnt)) {
            // V1_printf("GOOD: res %d after i2cWriten" EOL, res);
        } else if (res == PICO_ERROR_GENERIC) {
            V1_printf("ERROR: res %d after i2cWriten" EOL, res);
        } else {
            V1_printf("UNEXPECTED: res %d after i2cWriten %d" EOL, res, reg);
        }
    }

    // V1_printf("i2cWriten START reg %02x vcnt %u" EOL, reg, vcnt);
    return res;
}

//****************************************************
void si5351a_setup_PLL(uint8_t mult, uint32_t num, uint32_t denom, bool do_pllb) {
    Watchdog.reset();
    // V1_printf("si5351a_setup_PLL START do_pllb %u mult %u num %lu denom %lu" EOL, 
    //     do_pllb, mult, num, denom);
    uint8_t PLL_regs[8] = { 0 };

    // see MSNx_p1, p2 p3 in 
    // https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
    uint32_t p1 = 128 * mult + ((128 * num) / denom) - 512;
    uint32_t p2 = 128 * num - denom * ((128 * num) / denom);
    uint32_t p3 = denom;
    // V1_printf("setup_PLL do_pllb %u mult %u num %lu denom %lu" EOL, do_pllb, mult, num, denom);

    PLL_regs[0] = (uint8_t)(p3 >> 8);
    PLL_regs[1] = (uint8_t)p3;
    PLL_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    PLL_regs[3] = (uint8_t)(p1 >> 8);
    PLL_regs[4] = (uint8_t)p1;
    PLL_regs[5] = ((uint8_t)(p3 >> 12) & 0xf0) | ((uint8_t)(p2 >> 16) & 0x0f);
    PLL_regs[6] = (uint8_t)(p2 >> 8);
    PLL_regs[7] = (uint8_t)p2;

    // Hans says registers are double-buffered,
    // and the last-of-8 triggers an update of all the changes
    // Always include in your block of register writes, the final one of the block of 8.
    // By all means chop some unnecessary writes off the start of the block,
    // but never the end!
    // https://groups.io/g/picoballoon/message/19155
    // Maybe this works, because p2 is affected by both numerator and denominator
    // I'm doing numerator-change-only for symbol shift now, so p2 should always change
    // unless the symbol doesn't change!

    // Looking for a range of bits that changed?
    // the i2cWriten writes as burst below.
    // s_PLLB[AB]_ms_div_prev:
    // used basically as a 'valid' bit: implies s_PLLB_regs_prev has good data?

    //*************************
    bool ALWAYS_WRITE_8 = true;
    bool ALWAYS_WRITE_SINGLES = false;

    // start and end are looked at below, 
    // if PLLB_ms_div_prev != 0 which
    // says we can look at last saved state (s_PLL[AB]_regs_prev)
    uint8_t start = 0;
    uint8_t end = 7;

    if (!ALWAYS_WRITE_8) {
        uint8_t PLL_regs_prev[8];
        uint8_t PLL_ms_div_prev;
        if (do_pllb) {
            memcpy(PLL_regs_prev, s_PLLB_regs_prev, 8);
            PLL_ms_div_prev = s_PLLB_ms_div_prev;
        } else {
            memcpy(PLL_regs_prev, s_PLLA_regs_prev, 8);
            PLL_ms_div_prev = s_PLLA_ms_div_prev;
        }

        if (PLL_ms_div_prev != 0) {
            for (; start < 8; start++) {
                if (PLL_regs[start] != PLL_regs_prev[start]) break;
            }
            // detect no change of anything?
            if (start == 8) return;
            for (; end > start; end--) {
                // so so we just write the start to end that has changed?
                if (PLL_regs[end] != PLL_regs_prev[end]) break;
            }
        }
    }
    //*************************
    // HACK: 1/9/24. always do the last one, per Hans!
    // p2 isn't guaranteed to change if you change both num and denom
    // this is the last in the block of 8, reg 7.
    // p2 = 128 * num - denom * ((128 * num) / denom);

    uint8_t reg;
    uint8_t len;
    if (!ALWAYS_WRITE_8) {
        if (do_pllb) 
            reg = SI5351A_PLLB_BASE + start;
        else 
            reg = SI5351A_PLLA_BASE + start;

        len = end - start + 1;
        i2cWriten(reg, &PLL_regs[start], len);
        busy_wait_us(50);
        // per hans: write the last one, separately? to trigger update of double-buffer?
        i2cWriten(reg + 7, &PLL_regs[7], 1);

    } else {
        if (do_pllb) 
            reg = SI5351A_PLLB_BASE;
        else 
            reg = SI5351A_PLLA_BASE;

        len = 8;
        start = 0;
        if (!ALWAYS_WRITE_SINGLES) {
            i2cWriten(reg, &PLL_regs[start], len);
        } else {
            for (uint8_t i = 0; i < len ; i++) {
                uint8_t data = PLL_regs[i];
                i2cWrite(reg+i, data); 
                busy_wait_us(50);
            }
        }
    }

    // this can't be swap..so how could it have worked?
    // was it a pointer copy?
    // maybe PLLB_regs memory always got reallocated on the next call?
    // wouldn't if it was static?
    // was: *((uint64_t *)s_PLLB_regs_prev) = *((uint64_t *)PLLB_regs);

    if (do_pllb) 
        memcpy(s_PLLB_regs_prev, PLL_regs, 8);
    else
        memcpy(s_PLLA_regs_prev, PLL_regs, 8);

    // V1_printf("si5351a_setup_PLL END do_pllb %u mult %u num %lu denom %lu" EOL, 
    //    do_pllb, mult, num, denom);
}

//****************************************************
// swapping pointers instead of memcpy:
// both should be static or global, so no mem allocaiton issue
// https://stackoverflow.com/questions/8403447/swapping-pointers-in-c-char-int
// best/obvious to just memcpy for just 8 bytes

//****************************************************
// div must be even number
void si5351a_setup_multisynth012(uint32_t div) {
    // this ignores the state of PDN. PDN==0 will always allow clocks on!
    V1_printf("si5351a_setup_multisynth012 START div %lu" EOL, div);
    Watchdog.reset();
    if ((div % 2) != 0) {
        V1_printf("ERROR: si5351a_setup_multisynth012 div %lu isn't even" EOL, div);
    }

    uint8_t s_regs[8] = { 0 };
    uint32_t p1 = 128 * div - 512;

    // was: s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    // But what are the magic groups of eight?  where updates only
    // happen when you write the last of the group?
    // https://groups.io/g/picoballoon/message/19164

    // Just these 4 groups of 8? (starting at these base addresses)
    // SI5351A_PLLA_BASE =               26; // 8 regs
    // SI5351A_PLLB_BASE =               34; // 8 regs
    // SI5351A_MULTISYNTH0_BASE =        42; // 8 regs
    // SI5351A_MULTISYNTH1_BASE =        50; // 8 regs

    // R output divider is 0 (divide by 1)
    s_regs[0] = 0;
    s_regs[1] = 1;
    s_regs[2] = ((uint8_t)(p1 >> 16) & 0x03);
    s_regs[3] = (uint8_t)(p1 >> 8);
    s_regs[4] = (uint8_t)p1;
    s_regs[5] = 0;
    s_regs[6] = 0;
    s_regs[7] = 0;

    // if we're causing a change, the clocks better be off for cleanliness, 
    // and we'll be turning them on later? We can check the powerdown mode
    // and force PDN in here, so it doesn't turn on the clocks?

    uint8_t force_clk0_powerdown;
    uint8_t force_clk1_powerdown;
    // okay if we start CW stuff with it not powered down
    // since we'll power it down separately at first
    if (USE_SI5351A_CLK_POWERDOWN_FOR_WSPR_MODE) {
        force_clk0_powerdown = SI5351A_CLK0_PDN;
        force_clk1_powerdown = SI5351A_CLK1_PDN;
    } else {
        force_clk0_powerdown = 0x00;
        force_clk1_powerdown = 0x00;
    }

    if (cc._monopole[0] == '1') {
        force_clk1_powerdown = SI5351A_CLK1_PDN;
    }

    i2cWriten(SI5351A_MULTISYNTH0_BASE, s_regs, 8);
    // was 1/10/25 why was this here? it was sort of working?
    //    SI5351A_CLK0_MS0_INT |
    uint8_t CLK0_control_data =
        force_clk0_powerdown |
        SI5351A_CLK0_MS0_SRC_PLLB |
        SI5351A_CLK0_SRC_MULTISYNTH_0 |
        s_vfo_drive_strength[0];  // should just be 2 bits 0,1,2,3 = 2,4,6,8mA

    i2cWrite(SI5351A_CLK0_CONTROL, CLK0_control_data);
    s_CLK0_control_prev = CLK0_control_data;

    // this is used for antiphase (differential tx: clk0/clk1)
    // ms5351m: the output enable doesn't seem to disable..
    // whether or not we use INV!!  both always enabled?
    // which is why we're using the PDN bit! (means we'll need a pll reset
    // when we PDN=0 later!
    i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);
    // was 1/10/25
    // SI5351A_CLK1_MS1_INT |
    uint8_t CLK1_control_data =
        force_clk1_powerdown |
        SI5351A_CLK1_MS1_SRC_PLLB |
        SI5351A_CLK1_SRC_MULTISYNTH_1 |
        SI5351A_CLK1_INV |
        s_vfo_drive_strength[1];  // should just be 2 bits 0,1,2,3 = 2,4,6,8mA

    i2cWrite(SI5351A_CLK1_CONTROL, CLK1_control_data);
    s_CLK1_control_prev = CLK1_control_data;

    // power down the CLK2. just make it 2MA drive
    // right now: we don't use it for freq calibration (to rp2040 on pcb)
    i2cWriten(SI5351A_MULTISYNTH2_BASE, s_regs, 8);
    // was 1/10/25
    // SI5351A_CLK2_MS2_INT |
    uint8_t CLK2_control_data =
        SI5351A_CLK2_PDN |
        SI5351A_CLK2_MS2_SRC_PLLB |
        SI5351A_CLK2_SRC_MULTISYNTH_2 |
        SI5351A_CLK2_IDRV_2MA;  // always just 2MA for clk2, if ever used

    i2cWrite(SI5351A_CLK2_CONTROL, CLK2_control_data);
    s_CLK2_control_prev = CLK2_control_data;

    V1_printf("s_vfo_drive_strength[0] %02x" EOL, (int)s_vfo_drive_strength[0]);
    V1_printf("s_vfo_drive_strength[1] %02x" EOL, (int)s_vfo_drive_strength[1]);
    V1_printf("si5351a_setup_multisynth012 END div %lu" EOL, div);
}

//****************************************************
void si5351a_power_up_clk01(bool print) {
    // Only used for keying cw since the clk output enable doesn't seem
    // to turn off clk on ms5351m??
    // uses the previously set s_CLKn_control_prev
    if (print)
        V1_print(F("si5351a_power_up_clk01 START" EOL));
    // Watchdog.reset();

    // boolean inversion. Clear PDN
    uint8_t CLK0_control_data = s_CLK0_control_prev & ~SI5351A_CLK0_PDN;
    i2cWrite(SI5351A_CLK0_CONTROL, CLK0_control_data);
    s_CLK0_control_prev = CLK0_control_data;

    uint8_t CLK1_control_data;
    if (cc._monopole[0] == '1') {
        CLK1_control_data = s_CLK1_control_prev | SI5351A_CLK1_PDN;
    } else {
        CLK1_control_data = s_CLK1_control_prev & ~SI5351A_CLK1_PDN;
    }
    i2cWrite(SI5351A_CLK1_CONTROL, CLK1_control_data);
    s_CLK1_control_prev = CLK1_control_data;

    // always need to reset after change of CLKn_PDN to maintain phase relationship (Hans)
    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
    if (print)
        V1_print(F("si5351a_power_up_clk01 END" EOL));
    // hans picture says 1.46ms for effect?
    // don't add delay here though. will change wpm timing
}
//****************************************************
void si5351a_power_down_clk01(bool print) {
    // Only used for keying cw since the clk output enable doesn't seem
    // to turn off clk on ms5351m??
    // uses the previously set s_CLKn_control_prev
    if (print)
        V1_print(F("si5351a_power_down_clk01 START" EOL));
    // Watchdog.reset();

    // set PDN
    uint8_t CLK0_control_data = s_CLK0_control_prev | SI5351A_CLK0_PDN;
    i2cWrite(SI5351A_CLK0_CONTROL, CLK0_control_data);
    s_CLK0_control_prev = CLK0_control_data;

    uint8_t CLK1_control_data = s_CLK1_control_prev | SI5351A_CLK1_PDN;
    i2cWrite(SI5351A_CLK1_CONTROL, CLK1_control_data);
    s_CLK1_control_prev = CLK1_control_data;

    // always need to reset after change of CLKn_PDN to maintain phase
    // relationship (Hans)
    // HACK: wait one millisecond before doing the pll reset?
    // FIX! do we really need the pll reset to turn it off too?
    busy_wait_ms(1);
    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);

    // hans picture says 1.46ms for effect?
    // don't add delay here though. will change wpm timing
    // sleep_ms(2);
    if (print)
        V1_print(F("si5351a_power_down_clk01 END" EOL));
}

// FIX! compare to https://github.com/etherkit/Si5351Arduino
// libraries: wget https://github.com/etherkit/Si5351Arduino/archive/refs/heads/master.zip
// but not used? could use here with:
// #include <si5351.h>
// #include <Wire.h>
// Si5351 si5351;

//****************************************************
// we don't user PLLA ?
// The "Si5351A/B/C Data Sheet" says to apply a PLLA and PLLB soft reset
// before enabling the outputs [1]. This is required to get a deterministic
// phase relationship between the output clocks.
// Without the PLL reset, the phase offset beween the clocks is unpredictable.
//
// References: (no longer exists)
// [1] https://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351-B.pdf
// https://patchwork.kernel.org/project/linux-clk/patch/1501010261-7130-1-git-send-email-sergej@taudac.com/
// I am using _one_ PLL for all my clocks, still, the phase relationship
// between the clocks is random on each activation.
// The only way I was able to fix it, is to reset the corresponding PLL.

// FIX! will the i2cwrite timeout if we're in the middle of a pll reset/system init event?
// how long does a pll reset take? (B or default A?)
void si5351a_reset_PLLB(bool print) {
    if (print) {
        V1_println(F(EOL "si5351a_reset_PLLB START"));
        V1_flush();
    }
    // why does example say to write reg 177 = 0xAC for 'soft' reset of PLLA and PLLB?
    // that's more bits then I woudl expect?
    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);

    if (print) {
        Watchdog.reset();
        // only wait for the pll reset if we say to print?
        sleep_ms(2000);
        uint8_t reg;
        uint8_t val;
        int res;
        // FIX! is this mucking up anything
        // clear the status, then read it
        reg = SI5351A_DEVICE_STATUS;
        i2cWrite(reg, 0);
        sleep_ms(500);

        val = 0xff;  // initial value for the read to overwrite
        res = i2cWrRead(reg, &val);
        V1_printf("after PLLB reset (1): SI5351A_DEVICE_STATUS reg %02x val %02x res %d" EOL,
            reg, val, res);
        V1_flush();
        sleep_ms(4000);
        Watchdog.reset();

        val = 0xff;
        res = i2cWrRead(reg, &val);
        V1_printf("after PLLB reset (2): SI5351A_DEVICE_STATUS reg %02x val %02x res %d" EOL,
            reg, val, res);

        // clear the status, then read it
        reg = SI5351A_INTERRUPT_STATUS_STICKY;
        i2cWrite(reg, 0);
        sleep_ms(500);

        val = 0xff;   // initial value for the read to overwrite
        res = i2cWrRead(reg, &val);
        V1_print(F("after PLLB reset (3): SI5351A_INTERRUPT_STATUS_STICKY"));
        V1_printf(" reg %02x val %02x res %d" EOL, reg, val, res);
        V1_flush();

        V1_println(F("si5351a_reset_PLLB END" EOL));
    }
}

//****************************************************
// FIX! will the i2cwrite timeout if we're in the middle of a pll reset/system init event?
// so maybe we shouldn't do reset_PLLA, then reset_PLLB ?
void si5351a_reset_PLLA(bool print) {
    if (print) {
        V1_println(F(EOL "si5351a_reset_PLLA START"));
        V1_flush();
    }

    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLA_RST);

    if (print) {
        Watchdog.reset();
        // only wait for the pll reset if we say to print?
        sleep_ms(2000);
        uint8_t reg;
        uint8_t val;
        int res;

        // clear the status, then read it
        reg = SI5351A_DEVICE_STATUS;
        i2cWrite(reg, 0);
        sleep_ms(500);

        val = 0xff;
        res = i2cWrRead(reg, &val);
        // was seeing A0 here after 0.5 secs. halve to wait longer?
        V1_printf("after PLLA reset (1): SI5351A_DEVICE_STATUS reg %02x val %02x res %d" EOL,
            reg, val, res);
        V1_flush();

        sleep_ms(4000);
        Watchdog.reset();

        val = 0xff;
        res = i2cWrRead(reg, &val);
        V1_print(F("after PLLA reset (2): SI5351A_INTERRUPT_STATUS_STICKY"));
        V1_printf(" reg %02x val %02x res %d" EOL, reg, val, res);

        //******************
        // clear the status, then read it
        reg = SI5351A_INTERRUPT_STATUS_STICKY;
        i2cWrite(reg, 0);
        sleep_ms(500);

        val = 0xff;
        res = i2cWrRead(reg, &val);
        V1_print(F("after PLLA reset (3): SI5351A_INTERRUPT_STATUS_STICKY"));
        V1_printf(" reg %02x val %02x res %d" EOL, reg, val, res);
        V1_flush();

        V1_println(F("si5351a_reset_PLLA END" EOL));
    }
}

//****************************************************
// good for doing calc only, so see what changes with freq changes
uint8_t vfo_calc_div_mult_num(double *actual, double *actual_pll_freq,
    uint32_t *ms_div, uint32_t *pll_mult, uint32_t *pll_num, uint32_t *pll_denom,
    uint64_t freq_xxx, bool use_PLL_DENOM_OPTIMIZE) {
    Watchdog.reset();

    if ((PLL_FREQ_TARGET % 1000000) != 0) {
        V1_printf("ERROR: only support multiples of 1e6 for PLL_FREQ_TARGET %" PRIu64 EOL,
            PLL_FREQ_TARGET);
    }
    if ((SI5351_TCXO_FREQ % 1000000) != 0) {
        V1_printf("ERROR: only support multiples of 1e6 for SI5351_TCXO_FREQ %lu " EOL,
            SI5351_TCXO_FREQ);
    }

    // background:
    // https://rfzero.net/documentation/tools/si5351a-frequency-tool/
    // hans code from 2015:
    // https://qrp-labs.com/images/synth/demo6/si5351a.c

    // http://www.wa5bdu.com/programming-the-si5351a-synthesizer/
    // http://www.wa5bdu.com/si5351a-quadrature-vfo/

    // Following the VCO is another divider stage that divides the VCO frequency
    // by a value of ‘d + e/f’
    // and can be used to take the frequency down into the low MHz range.
    // The chip will provide an output with lower jitter if this value is an integer
    // and better still if it is an even integer.
    // So we let e/f be zero and select a value for d that’s an even number.

    const uint64_t PLL_DENOM_MAX = 0x000fffff;  // 1048575

    // if doing numerator-shift algo (not Farey)
    // set from either optimize algo, or known best (per band) in tracker.ino
    // either by set_PLL_DENOM_OPTIMIZE() or by si5351a_calc_optimize() which can be
    // overwritten by the former

    uint64_t tcxo_freq = (uint64_t) SI5351_TCXO_FREQ;  // 26 mhz?
    uint64_t tcxo_freq_xxx = tcxo_freq << PLL_CALC_SHIFT;

    //*****************************
    // the divider is 'a + b/c' or "Feedback Multisynth Divider"
    // c is PLL_DENOM

    // Following the VCO is another divider stage
    // that divides the VCO frequency by a value of ‘d + e/f’ and
    // can be used to take the frequency down into the low MHz range.
    // The chip will provide an output with lower jitter if this value is an integer.
    // better still if it is an even integer.
    // So we let e/f be zero and select a value for d that’s an even number.
    // Remember that there is enough resolution in the PLL/VCO stage to provide
    // fine tuning.
    // The data sheet calls the ‘d + e/f’ divider the Output Multisynth Divider,
    // because it acts on the output of the PLL/VCO.
    // we don't have to worry about the divider R. we use 1 for that

    // So we’re down to six values to calculate which are the ‘a, b, c, d, e and f’
    // of the dividers ‘a + b/c’ and ‘d + e/f’.
    // But it will actually be a lot simpler.
    // We said that the second divider d + e/f will be an even integer,
    // so e and f are not needed.
    // Then in the first divider a + b/c, we will make c a constant
    // so we are now down to three required values: a, b and d.

    // R divisor is 1 

    // hmm what does rp2040 have for 64-bit integer divided.
    // Is it sometimes getting the wrong answers with big PLL_CALC_SHIFT?
    // https://lorenz-ruprecht.at/docu/pico-sdk/1.4.0/html/group__pico__divider.html
    // FIX! is the compiler messing up when I have big shifts?
    // does it optimize the the two shifts and lose precision as a result


    // if we don't like, or get too big of a pll_remain for Farey, 
    // can change the PLL_FREQ_TARGET a little, and try again.
    // Could also look for ranges (<0.5) for Farey, since they don't seem to have error outliers
    // in certain ranges.
    uint8_t trial = 0;
    double target;
    double pll_remain;
    uint64_t pll_remain_xxx;
    uint64_t pll_freq_xxx;
    uint64_t pll_mult_here;
    uint64_t ms_div_here;
    bool DEBUG = true;
    uint8_t MAXTRIAL = 5; // might have to go up 100Mhz?

    uint8_t retcode = 0;
    // will get called for all 4 symbol freqs. should redo if any fail? 
    // (will be during calcs-only phase)
    while (trial < MAXTRIAL) {
        retcode = 0;
        // the choice of 25Mhz for stepping pll target is arbitrary. 
        // (relates to mul/div/tcxo and output freq)
        // no need to do 1e6 shift down mode here
        uint64_t pll_freq_target_trial = PLL_FREQ_TARGET + (25000000 * trial);
        uint64_t aaa = pll_freq_target_trial << PLL_CALC_SHIFT;

        // we're only outputting freqs up to 29Mhz, so no chance of losing bits here!
        uint64_t bbb = freq_xxx;

        // does divide use these? I can't seem to call them directly though..
        // https://lorenz-ruprecht.at/docu/pico-sdk/1.4.0/html/group__pico__divider.html
        // divider.h
        // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_divider/include/hardware/divider.h

        // The pico_divider library provides a more user friendly set of APIs over the divider
        // (and support for 64 bit divides),
        // You can just use C level `/` and `%` operators
        // and gain the benefits of the fast hardware divider.
        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__pico__divider.html
        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__divider.html
        // uint64_t div_u64u64 (uint64_t a, uint64_t b)

        // wrong answer case? 19 bit shift
        // DEBUG: PLL_FREQ_TARGET shifted: aaa 209715200000000 // correct
        // DEBUG: freq_xxx shifted: bbb 1512046592 < shift of 2884? seems like missing bits? >
        // DEBUG: ms_div_here = aa / bbb: 138696 <correct. so is the shifted PLL_FREQ_TARGET wrong?
        // DEBUG: bits_aaa: 47.58
        // DEBUG: bits_bbb: 30.49

        ms_div_here = aaa / bbb;

        if (DEBUG) {
            uint64_t bits_shifted_out = pll_freq_target_trial >> (64 - PLL_CALC_SHIFT);
            double bits_pft = log2(pll_freq_target_trial);
            double bits_aaa = log2(aaa);
            double bits_bbb = log2(bbb);

            V1_printf(EOL "DEBUG: trial %u pll_freq_target_trial %" PRIu64 EOL, 
                trial, pll_freq_target_trial);
            V1_printf("DEBUG: freq: %" PRIu64 EOL, freq_xxx >> PLL_CALC_SHIFT);
            V1_printf("DEBUG: pll_freq_target_trial shifted: aaa %" PRIu64 EOL, aaa);
            V1_printf("DEBUG: bits_shifted_out: %" PRIu64 EOL, bits_shifted_out);
            V1_printf("DEBUG: PLL_CALC_SHIFT %" PRIu64 EOL, PLL_CALC_SHIFT);
            V1_printf("DEBUG: freq_xxx shifted: bbb %" PRIu64 EOL, bbb);
            V1_printf("DEBUG: ms_div_here = aa / bbb: %" PRIu64 EOL, ms_div_here);
            V1_printf("DEBUG: bits_pft: %.2f" EOL, bits_pft);
            V1_printf("DEBUG: bits_aaa: %.2f" EOL, bits_aaa);
            V1_printf("DEBUG: bits_bbb: %.2f" EOL, bits_bbb);
        }

        // make it even. odd bumps up.
        // has to be a bump up? pll freq will increase..so mult may up too?
        if ((ms_div_here % 2) == 1) ms_div_here += 1;
        if (DEBUG) {
            V1_printf("DEBUG: ms_div_here post-inc?: %" PRIu64 EOL, ms_div_here);
        }

        // is 900 really the upper limiter or ?? we have 908 for 24Mhz
        // do we need lower pll freq?
        if (ms_div_here < 4 || ms_div_here > 900) {
            V1_printf(EOL "ERROR: vfo_calc_div_mult_num (1) ms_div %" PRIu64, ms_div_here);
            V1_print(F(" is out of range 4 to 900" EOL));
            V1_print(F("ERROR: vfo_calc_div_mult_num (1) no recalc done."));
            V1_print(F(" Should never happen! rf output is wrong" EOL));
            retcode = -1;
        }

        pll_freq_xxx = freq_xxx * ms_div_here;

        // this is just integer. only useful for printing/rough error check
        uint64_t int_pll_freq_here = pll_freq_xxx >> PLL_CALC_SHIFT;

        // remember: floor division (integer)
        // tcxo_freq is integer..
        pll_mult_here = pll_freq_xxx / tcxo_freq_xxx;

        //*************************************************************
        // good info on out-of-spec behavior
        // https://rfzero.net/documentation/rf/
        // https://rfzero.net/tutorials/si5351a/
        // mult has to be in the range 15 to 90

        // I was getting pll_mult of 14 for 10M when I targeted 390Mhz pll
        // need 15 minimum

        // FIX! not clear if this attempt to "fix" adds any value
        // should never have this problem with good algo + pll target?
        if (pll_mult_here < 15 || pll_mult_here > 90) {
            V1_printf(EOL "ERROR: pll_mult %" PRIu64 " is out of range 15 to 90." EOL,
                pll_mult_here);
            V1_print(F("ERROR: Need to pick another target pll freq" EOL));
            V1_printf("int_pll_freq_here %" PRIu64 " tcxo_freq %" PRIu64 EOL,
                int_pll_freq_here, tcxo_freq);

            // Would this work. forcing the boundary cases?
            if (pll_mult_here < 15) pll_mult_here = 15;
            else pll_mult_here = 90;

            // Have to recompute the divisor if we change pll_mult? i.e. it's forced..
            // recompute the pll_freq based on the forced multiplier

            // 64 bit calcs so don't lose bits beyond 32-bits
            // mult will be min or max
            // This is no longer correct compared to creating it from the output freq
            uint64_t pll_freq_xxx_2 = tcxo_freq_xxx * pll_mult_here;
            // this is just integer. for printing/quick compare only!
            int_pll_freq_here = pll_freq_xxx_2 >> PLL_CALC_SHIFT;

            ms_div_here = pll_freq_xxx_2 / bbb;  // bbb from above
            // make it even. odd bumps up.
            if ((ms_div_here % 2) == 1) ms_div_here += 1;

            V1_print(F("vfo_calc_div_mult_num pll_freq implied by new ms_div, pll_mult" EOL));
            V1_print(F("ms_div implied by that pll_freq" EOL));

            if (ms_div_here < 4 || ms_div_here > 900) {
                V1_printf(EOL "ERROR: vfo_calc_div_mult_num (2) ms_div %" PRIu64, ms_div_here);
                V1_print(F(" is out of range 4 to 900" EOL));
                V1_print(F("ERROR: vfo_calc_div_mult_num (2) no recalc done."));
                V1_print(F(" Should never happen! rf output is wrong" EOL));
                retcode = -1;
            }

            V1_printf("ERROR: Now (2) pll_mult %" PRIu64 " ms_div %" PRIu64 EOL,
                pll_mult_here, ms_div_here);

            // flag this case as bad?
            retcode = -1;
        }

        //*************************************************************
        // check the range of "legal" pll frequencies
        if (false && (int_pll_freq_here < 600000000 || int_pll_freq_here > 900000000)) {
            V1_printf("WARN: integer pll_freq %" PRIu64, int_pll_freq_here);
            V1_print(F(" is outside datasheet legal range" EOL));
            retcode = -1;
        }
        // live with it. maybe upgrade to ERROR to not miss with grep?
        if (int_pll_freq_here < 390000000 || int_pll_freq_here > 900000000) {
            V1_printf("WARN: integer pll_freq %" PRIu64, int_pll_freq_here);
            V1_print(F(" is outside my 'legal' range" EOL));
            retcode = -1;
        }

        // pll_num max 20 bits (0 to 1048575)?

        // also look at https://github.com/etherkit/Si5351Arduino
        // a new method. (greatest common divisor?)
        // https://github.com/etherkit/Si5351Arduino/issues/79

        //*******************************************************************
        // it's interesting these are done in the non-scaled domain (not *128)
        // since pll_freq_here is what we want to get to, shouldn't we be scaled here?
        // FIX! we should calc this as real for Farey!
        pll_remain_xxx = pll_freq_xxx - (pll_mult_here * tcxo_freq_xxx);
        double bits_pll_remain_xxx = log2(pll_remain_xxx);

        // this is for Farey target, but is good for non-Farey algo too?
        // this cast of pll_remain_xxx can only have 52 bits of precision?
        pll_remain = ((double) pll_remain_xxx) / pow(2, PLL_CALC_SHIFT);
        // we want a fraction to multiple the txco_freq with
        target = pll_remain / (double)tcxo_freq;

        if (DEBUG) {
            V1_printf("DEBUG: target: %.16f" EOL, target);
            V1_printf("DEBUG: pll_remain_xxx: %" PRIu64 EOL, pll_remain_xxx);
            V1_printf("DEBUG: bits_pll_remain_xxx: %.2f" EOL, bits_pll_remain_xxx);
        }

        // Do we have a good remainder to target?
        // there are other known error-prone regions (small) like around 0.5
        if ((retcode == 0) && target > 0.001 && target < 0.99) {
            V1_printf("GOOD: trial %u found achievable target %.16f" EOL, 
                trial, target);
            break;
        }
        trial += 1;
    }

    if (trial>=MAXTRIAL) {
        V1_print(F(EOL "ERROR: fatal..didn't find good pll_remain. Change PLL_TARGET_FREQ" EOL));
        retcode = -1;
    }

    //*******************************************************************
    rational_t retval;
    // should never use these initial values?
    retval.numerator = 0;
    retval.denominator = 0;
    // what we really need is an 64-bit int version of Farey so we wouldn't need reals!
    // then we could increase our shift for scaled-integer?
    // so we won't be restricted by just 52 bits in fp mantissa here?
    // eventually the 20-bit limit on num/denom should be the limiter
    if (TEST_FAREY_WITH_PLL_REMAINDER | USE_FAREY_WITH_PLL_REMAINDER) {
        // Farey algo wants double reals. So here we go
        // luckily all my integer *_xxx is shifts..i.e. powers of 2!
        // so no loss of precision going back and forth

        // lets see what farey/magnusson get with the remainder
        // we can't use this as we need to adjust the remainder to get
        // the wspr shifts after the divide by divisor and R shift
        // I suppose we'd just need to redo this algo and create 4 num/denom pairs
        // calculate the first pll freq/output freq, then generate 3 freqs from that
        // and calculate their num/denom pairs.
        uint32_t maxdenom = (uint32_t)PLL_DENOM_MAX;
        // FIX! why are we limiting ourselves to the PLL_CALC_SHIFT precision on the remainder!!

        // FIX! if we zero out some of the lower digits of the mantissa, does it help reduce error?
        // with 26mhz tcxo, seems like we need 12 digits of precision
        // on the real being approximated.
        // (1e-12 * 26e6 (tcxo)) / 15  (divisor) = 1.73e-6
        // so we need to shift at least 12 bits for the _xxx shifted-integer stuff?
        V1_print(F(EOL));
        V1_printf("Farey target %.16f" EOL, target);

        // Using a si5351a fractional feedback, With a 26mhz tcxo,
        // and maybe a 400mhz PLL with a minimal divisor of 15 after the pll,

        // it seems like we need decimal 11 digits of precision
        // on the fractional real being approximated.
        // Because the effect of the 10th digit on on the wspr symbol freq is at most:
        //    (1e-11 * 26e6) / 15 = 1.73e-5 Hz..
        // which is plenty of precision (uHz level)

        // So we only need 11 digits of precision  in the fractional real which is 0 to 1

        // Ideally means integer-scaled shifting should save log2(10**11) == 36 bits
        // ..but can we shift 32 bits into 64 bit? with 900Mhz max? yes we could?

        // Alternative to the sprintf/sscanf:
        // Could multiply by 2**32 and cast it to a uint32_t ,  then put it
        // back in the double and divide by 2**32.
        // So then we just have 32 bits of precision (less than 36 bits, but close?)

        // hmm.. does this help Farey?
        // we are already limited in terms of how much we can shift
        // so precision is already limited
        // from case: 10M 400Mhz PLL_CALC_SHIFT 16
        // DEBUG: bits_pll_remain_xxx: 37.84
        // but the conversion to real is another base (fp double)
        // Unclear how many bits there this is doing a roundoff in the decimal digit domain...
        // interesting. that's different
        // it seems to help in the farey.cpp test file..which does random reals 0 to 1
        // and looks at errors. This seems to help the errors bunch? no outliers?

        // If we have 1-e-11 accuracy in the fractional real multiplier that goes into
        // the pll, and a 26Mhz tcxo, and maybe a small divider like 15:
        //   (1e-11 * 26e6) / 15 = 1.7e-5. So that's in the tens of uHz for accuracy.
        // don't need more than that. So should be able to chop the Farley input (target)
        // this still can be more than 36 bits of precision! Single precision float won't do!
        // Oh: is 36 + 16 integer-shift(* 2**16) = 52 (double fp mantissa bits), the reason
        // we can't integer shift scale more than 16?

        if (USE_FAREY_CHOPPED_PRECISION) {
            char str[40];
            snprintf(str, sizeof(str), "%.11f", target);
            sscanf(str, "%lf", &target);
            // the Farey algo uses doubles
            // but starting with lower precision can be a good thing?
            V1_print(F("Farey after .11f digit sprintf/sscanf:"));
            V1_printf(" Farey new target %.16f" EOL, target);
        }

        retval = rational_approximation(target, maxdenom);
        double actual_real = ((double)retval.numerator) / (double)retval.denominator;

        // results
        V1_printf("Farey numerator %lu" EOL, retval.numerator);
        V1_printf("Farey denominator %lu" EOL, retval.denominator);
        V1_printf("Farey iterations %lu" EOL, retval.iterations);
        V1_printf("Farey actual real %.16f" EOL, actual_real);
        V1_printf("Farey actual real error %.4e" EOL, target - actual_real);
        V1_print(F(EOL));
    }

    //*******************************************************************
    uint64_t pll_denom_to_use = 0;

    if (!use_PLL_DENOM_OPTIMIZE && !USE_FAREY_WITH_PLL_REMAINDER) {
        // new: calculate the optimum denominator for numerator-shift-by-1
        uint64_t wspr_shift_xxx = (12000L << PLL_CALC_SHIFT) / 8192L;
        // 15.625 Hz is 15625/1000 or 125 / 8
        uint64_t mfsk16_shift_xxx = (15L << PLL_CALC_SHIFT) / 8L;
        uint64_t shift_xxx;
        if (USE_MFSK16_SHIFT) {
            shift_xxx = mfsk16_shift_xxx;
            V1_printf(EOL "Numerator-Shift algo using mfsk16 shift_xxx %" PRIu64 EOL, shift_xxx);
        } else {
            shift_xxx = wspr_shift_xxx;
            V1_printf(EOL "Numerator-Shift algo using wspr shift_xxx %" PRIu64 EOL, shift_xxx);
        }

        // also divide by the r divisor
        // we will lose some bits here, depending on how big the PLL_CALC_SHIFT was
        // optimal denom = (tcxo / (divider * wspr shift)
        // add 0.5 shifted, to get rounding effect on the floor divide
        // could shift some more here, to see if it's exact or not, but don't care
        // this is a floor, not a round ..hmmm
        // shift tcxo_freq_xxx left 1, add 1, then shift right 1?
        pll_denom_to_use = (tcxo_freq_xxx / ms_div_here) / shift_xxx;

        V1_printf(EOL "Numerator-Shift algo pll_denom_to_use %" PRIu64 EOL, 
            pll_denom_to_use);

        // this denom might not be exact.

        // FIX! no benefit to 2x or 3x the denom if it doesn't exceed max
    }

    //*******************************************************************
    uint64_t pll_num_here = 0;
    uint64_t pll_denom_here = 0;

    if (USE_FAREY_WITH_PLL_REMAINDER) {
        pll_num_here = (uint64_t)retval.numerator;
        pll_denom_here = (uint64_t)retval.denominator;
    } else {
        if (use_PLL_DENOM_OPTIMIZE) {
            // old hardwired, per-band, constant. assume these are correct (not too big
            pll_denom_here = PLL_DENOM_OPTIMIZE;
            V1_printf(EOL "Numerator-Shift algo using hard-wired PLL_DENOM_OPTIMIZE, pll_denom_here %" PRIu64 EOL, 
                pll_denom_here);
        } else {
            // we can multiply it by 2 or 3, if the result is < PLL_DENOM_MAX
            // will this be better for absolute error/round off issues?
            // don't go right up to the edge!
            // the pll_num will be calculated below and be correct for 
            // these choices
            if ((pll_denom_to_use * 3) < (PLL_DENOM_MAX - 100)) 
                pll_denom_here = 3 * pll_denom_to_use;
            else if ((pll_denom_to_use * 2) < (PLL_DENOM_MAX - 100)) 
                pll_denom_here = 2 * pll_denom_to_use;
            else 
                pll_denom_here = pll_denom_to_use;
            V1_printf(EOL "Numerator-Shift algo using calculated pll_denom, pll_denom_here %" PRIu64 EOL, 
                pll_denom_here);
        }

        // should never be negative
        // shouldn't need this comparision here?
        // if (pll_denom_here > PLL_DENOM_MAX) pll_denom_here = PLL_DENOM_MAX;

        // FIX! the R divisor needs to be in here somehow. but it's always one 
        // for us, so no problem for now
        pll_num_here = (pll_remain_xxx * pll_denom_here) / tcxo_freq_xxx;
        V1_printf(EOL "Numerator-Shift algo calcs pll_num %" PRIu64 EOL, pll_num_here);

    }
    //*******************************************************************
    if (pll_num_here == 0 || pll_num_here >= PLL_DENOM_MAX) {  // 1048575
        V1_printf(EOL "ERROR: pll_num %" PRIu64 " is out of range 1 to 1048575-1" EOL,
            pll_num_here);
        retcode = -1;
    }
    if (pll_denom_here == 0 || pll_denom_here >= PLL_DENOM_MAX) {  // 1048575
        V1_printf(EOL "ERROR: pll_denom %" PRIu64 " is out of range 1 to 1048575-1" EOL,
            pll_denom_here);
        retcode = -1;
    }

    // https://rfzero.net/tutorials/si5351a/
    // When we're done, we can calc what the fout should be ?
    // Ah. this has more precision in it..can't just shift down to print it!
    // Doug (traquito) has WSPR_TONE_SPACING_HUNDREDTHS_HZ = 146 (1.4648 Hz)
    // hmm. looking at the sweep of "actual" seems like they are 2 hz steps?
    // need to make that a real number
    double actual_pll_freq_here = (double)tcxo_freq *
        ((double)pll_mult_here + ((double)pll_num_here / (double)pll_denom_here));

    // note we return a double here...only for printing
    double actual_here = actual_pll_freq_here / (double) ms_div_here;

    // for use by some of the sweep functions?
    PLL_DENOM_OPTIMIZE_calced = pll_denom_here;

    // output so we can print or use
    *ms_div    = (uint32_t)ms_div_here;  // these uint64_t turn into uint32_t here
    *pll_mult  = (uint32_t)pll_mult_here;
    *pll_num   = (uint32_t)pll_num_here;
    *pll_denom = (uint32_t)pll_denom_here;
    *actual = actual_here;
    *actual_pll_freq = actual_pll_freq_here;
    
    // make the caller adjust PLL_FREQ_TARGET and redo everything for 4 symbols
    // if we took more than one trial here
    if (trial != 0) return -1;
    else return retcode;
}

//****************************************************
// freq is in 28.4 fixed point number, 0.0625Hz resolution
// only_pll_num allows num and denom, when doing Farey
uint8_t vfo_set_freq_xxx(uint8_t clk_num, uint64_t freq_xxx, bool only_pll_num, bool just_do_calcs) {
    Watchdog.reset();
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num;
    uint32_t pll_denom;
    double actual_pll_freq;
    double actual;
    if (clk_num != 0) {
        V1_println("ERROR: vfo_set_freq_xxx should only be called with clk_num 0");
        // I guess force clk_num, although code is broken somewhere
        clk_num = 0;
        // note we only have one s_PLLB_ms_div_prev copy state also
    }

    // lookup in cache, if not there, compute/fill it
    uint8_t retval = 0;
    uint8_t retcode = 0;
    if (DISABLE_VFO_CALC_CACHE) {
        retcode = vfo_calc_div_mult_num(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num, &pll_denom, freq_xxx, false);
    } else {
        // lookup. get values if in cache already!
        // don't do any prints on the lookup
        retval = vfo_calc_cache(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num, &pll_denom, freq_xxx, 1);
        if (retval != 1) {  // cache miss?
            V1_print(F(EOL "WARN: vfo_set_freq_xxx must redo vfo_calc_div_mult_num()" EOL));
            retcode = vfo_calc_div_mult_num(&actual, &actual_pll_freq,
                &ms_div, &pll_mult, &pll_num, &pll_denom, freq_xxx, false);
            // install. If we install it when already there, we'll flag the double
            // entry on lookup later
            vfo_calc_cache(&actual, &actual_pll_freq,
                &ms_div, &pll_mult, &pll_num, &pll_denom, freq_xxx, 2);
            V1_printf("vfo_set_freq_xxx after redo: pll_mult %lu ms_div %lu" EOL,
                pll_mult, ms_div);
        }
    }

    if (just_do_calcs) {
        V1_print(F("vfo_set_freq_xxx just_do_calcs" EOL));
        return retcode;
    }

    //*****************************************************
    // hmm. does ms5351m need this when both numerator and denominator change a lot?
    // do we always need it when we do the Farey num/denom? why?
    // bool do_pll_reset = ((pll_denom != s_PLLB_pll_denom_prev) || USE_FAREY_WITH_PLL_REMAINDER);

    // don't need to turn off?
    // actually better not to, no popping?
    if (false) {
        vfo_turn_off_clk_out(WSPR_TX_CLK_0_NUM, false);
    }

    // for numerator-shift algo:
    // calcs were done to show that only pll_num needs to change for symbols 0 to 3
    // we always do an early turn-on with symbol 0 that gets all other state right
    // and it stays that way for the whole message. Guarantee no pll reset needed!

    // turns out there's no speedup to make use of this bool
    si5351a_setup_PLL(pll_mult, pll_num, pll_denom, true); // PLLB

    // make PLLA the same (so it locks? Is that better power than unlocked?)
    // maybe PLLA is disabled if not used? unknown
    // HACK. don't setup PLLA 1/10/25
    if (false) {
        si5351a_setup_PLL(pll_mult, pll_num, pll_denom, false); // PLLA
    }

    // we don't check if the pll_mult has changed
    // FIX! should we OR these cases into do_pll_reset ??
    if (pll_mult != s_PLLB_pll_mult_prev) {
        if (only_pll_num) {
            V1_print(F("ERROR: only_pll_num true"));
            V1_printf(" but pll_mult %lu changed. s_PLLB_pll_mult_prev %lu",
                pll_mult, s_PLLB_pll_mult_prev);
            V1_print(F(" need to do si5351a_reset_PLLB() after this?" EOL));
        }
    }
    if (ms_div != s_PLLB_ms_div_prev) {
        // s_PLLB_ms_div_prev is global
        // only reset pll if ms_div changed?
        if (only_pll_num) {
            V1_print(F("ERROR: only_pll_num true"));
            V1_printf(" but ms_div %lu changed. s_PLLB_ms_div_prev %lu",
                ms_div, s_PLLB_ms_div_prev);
            V1_print(F(" need to do si5351a_reset_PLLB() after this?" EOL));
        }
        // setting up multisynth0/1/2
        // if we're causing a change, the clocks better be off, and 
        // we'll be turning them on later? We can check the powerdown mode
        // and force PDN in here, so it doesn't turn on the clocks?
        si5351a_setup_multisynth012(ms_div);

        // FIX! should we change the multisynth setup to start with output clock PDN=1?
        // then power it up later? (or here)

        // I guess when we first turn change ms_div, we'll do a reset_PLLB (elsewhere)
        // si5351a_reset_PLLB(true);

    }
    // V1_printf("vfo_set_freq_xxx END clk_num %u freq %lu" EOL, clk_num, freq);

    // hmm. does ms5351m need this when both numerator and denominator change (a lot?)
    // if (do_pll_reset)
    // FIX! do we always need this? why?
    if (true) {
        // we might need this if doing Farey with numerator plus denominator change

        if (false && USE_FAREY_WITH_PLL_REMAINDER) si5351a_reset_PLLB(false);
        // apparently don't need to write this if on and just changing numerator?
        // it's weird that I seem to need clk turn on, but turn off doesn't always disable?
        vfo_turn_on_clk_out(WSPR_TX_CLK_0_NUM, false);
        
    }

    s_PLLB_pll_mult_prev = pll_mult;
    s_PLLA_pll_mult_prev = pll_mult;

    s_PLLB_ms_div_prev = ms_div;
    s_PLLA_ms_div_prev = ms_div;

    s_PLLB_pll_denom_prev = pll_denom;
    s_PLLA_pll_denom_prev = pll_denom;

    return retcode;
}

//****************************************************
// no pll reset needed for maintaining 180 degree clk0/clk1
// only pll reset if the clk PDN bit used
void vfo_turn_on_clk_out(uint8_t clk_num, bool print) {
    Watchdog.reset();
    if (print) {
        V1_printf("vfo_turn_on_clk_out START clk_num %u" EOL, clk_num);
    }
    if (clk_num != 0) {
        V1_printf("ERROR: vfo_turn_on_clk_out should only have clk_num 0 not %u"
            EOL, clk_num);
    } else {
        // enable clock 0 and 1. 0 is enabled
        uint8_t disable_bits = 1 << clk_num;
        // clk0 implies clk1 also. so do a 3!
        disable_bits |= 1 << 1;
        // don't do anything if no bits change
        // note the use of bitwise inversion ~
        if ((si5351bx_clken & ~disable_bits) != si5351bx_clken) {  // 0 is enabled
        // HACK: always do it?
        // if (true) {
            si5351bx_clken &= ~disable_bits;
            if (print) {
                V1_printf("vfo_turn_on_clk_out si5351bx_clken %02x" EOL,
                    si5351bx_clken);
            }
            i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
        }
    }

    if (print) {
        V1_println(F("vfo_turn_on_clk_out END"));
        V1_flush();
    }
}

//****************************************************
// hans used to say we need pll reset whenever we turn clocks off/on
// to retain 180 degree phase relationship (CLK0/CLK1)
// I think it's just if the power down bit is used. I can't 
// seem to disable the clk with this anyhow?

// Hmm. this is never used now?
void vfo_turn_off_clk_out(uint8_t clk_num, bool print) {
    Watchdog.reset();
    // are these enable bits inverted? Yes, 1 is disable
    if (print) {
        V1_printf(EOL "vfo_turn_off_clk_out START clk_num %u" EOL, clk_num);
    }
    uint8_t disable_bits = 1 << clk_num;

    // clk0 implies clk1 also
    if (clk_num != 0) {
        V1_printf("ERROR: vfo_turn_off_clk_out should only have clk_num 0 not %u"
            EOL, clk_num);
    } else {
        disable_bits |= 1 << 1;
        si5351bx_clken |= disable_bits;  // 1 is disable
        // if si5351a power is off we'll get ERROR: res -1 after i2cWrite 3
        if (print) {
            V1_printf("vfo_turn_off_clk_out si5351bx_clken %02x" EOL,
                si5351bx_clken);
        }
        i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
    }

    if (print) {
        V1_printf("vfo_turn_off_clk_out END clk_num %u" EOL EOL, clk_num);
        V1_flush();
    }
}
//****************************************************
void vfo_set_drive_strength(uint8_t clk_num, uint8_t strength) {
    Watchdog.reset();
    V1_printf("vfo_set_drive_strength START clk_num %u strength %u" EOL, 
        clk_num, strength);
    s_vfo_drive_strength[clk_num] = 0x3 & strength;

    //**********************
    // reset the s_PLLB_ms_div_prev to force vfo_set_freq_xxx()
    // to call si5351a_setup_multisynth012() next time
    s_PLLB_ms_div_prev = 0;
    s_PLLA_ms_div_prev = 0;

    // new 11/24/24 ..maybe clear all this old state too!
    memset(s_PLLB_regs_prev, 0, 8);
    memset(s_PLLA_regs_prev, 0, 8);

    // No change on these though?
    // si5351bx_clken = 0xff;

    // Triggering a pll reset requires some delay afterwards
    // so we don't want that to happen or be required (to retain 180 degree antiphase)
    // if we change enough, and don't pll reset,
    // we can lose the 180 degree phase relationship between CLK0 and CLK1.
    // see comments from Hans.
    // Not well documented when a pll reset is required.
    // Not needed/desired for the small Hz shifts during symbol tx
    V1_printf("vfo_set_drive_strength END clk_num %u strength %u" EOL, clk_num, strength);
}

//****************************************************
bool vfo_is_on(void) {
    // power on and completed successfully
    // FIX! switching in/out direction on the gpio power pin doesn't make sense.
    // old code changed the dir of the gpio
    // static bool gpio_is_dir_out ( uint gpio )

    // Check if a specific GPIO direction is OUT.
    // gpio GPIO number

    // returns
    // true if the direction for the pin is OUT
    // can use this? decided not to.

    // gpio_get_out_level()
    // static bool gpio_get_out_level (uint gpio )
    // Determine whether a GPIO is currently driven high or low

    // This function returns the high/low output level most recently assigned to
    // a GPIO via gpio_put() or similar.
    // This is the value that is presented outward to the IO muxing,
    // not the input level back from the pad (which can be read using gpio_get()).
    //
    // To avoid races, this function must not be used for read-modify-write sequences
    // when driving GPIOs.. instead functions like gpio_put()
    // should be used to atomically update GPIOs.
    // This accessor is intended for debug use only.
    // gpio GPIO number
    // Returns
    // true if the GPIO output level is high, false if low.
    // return (!gpio_get_out_level(Si5351Pwr) && vfo_turn_on_completed);

    // can do this sample of input on an output gpio?
    return (!gpio_get(Si5351Pwr) && vfo_turn_on_completed);
}

//****************************************************
bool vfo_is_off(void) {
    // power on and completed successfully
    // return (gpio_is_dir_out(Si5351Pwr) && vfo_turn_off_completed);
    // FIX! should we get rid of vfo_turn_off_completed?
    // assumes pin is configured correctly so we can read value?
    return (gpio_get(Si5351Pwr) && vfo_turn_off_completed);
}
// we don't deal with clk2 in any of this. Goes to rp2040 for calibration.

//****************************************************
void vfo_write_clock_en_with_retry(uint8_t val) {
    Watchdog.reset();
    V1_print(F("vfo_write_clock_en_with_retry START" EOL));
    // This can power the vfo off/on on failure! and we'll lose state
    V1_flush();
    // set all CLKx output disable. will re-enable clk0/1 later
    int tries = 0;
    // any error?
    // FIX! what about timeout?
    int res = i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, val);

    V1_printf("vfo_turn_on first res %d of i2cWrite SI5351A_OUTPUT_ENABLE_CONTROL" EOL , res);
    while (res != 2) {
        if (tries > 5) {
            V1_println("Rebooting because couldn't init VFO_I2C_INSTANCE after 5 tries");
            V1_flush();
            Watchdog.enable(1000);
            // milliseconds
            for (;;) {
                // FIX! put a bad status in the leds?
                updateStatusLED();
            }
        }
        Watchdog.reset();
        tries++;
        V1_println("VFO_I2C_INSTANCE trying re-init, after trying a i2cWrite and it failed");
        V1_flush();

        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
        i2c_deinit(VFO_I2C_INSTANCE);
        busy_wait_ms(1000);

        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
        i2c_init(VFO_I2C_INSTANCE, VFO_I2C_SCL_HZ);
        busy_wait_ms(1000);

        // gpio_pull_up(VFO_I2C_SDA_PIN);
        // gpio_pull_up(VFO_I2C_SCL_PIN);
        // don't use pullups or pulldowns since there are external pullups
        gpio_set_pulls(VFO_I2C_SDA_PIN, false, false);
        gpio_set_pulls(VFO_I2C_SCL_PIN, false, false);
        gpio_set_function(VFO_I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(VFO_I2C_SCL_PIN, GPIO_FUNC_I2C);
        i2c_set_slave_mode(VFO_I2C_INSTANCE, false, 0);
        busy_wait_ms(1000);

        // power the vfo off/on
        digitalWrite(Si5351Pwr, HIGH);
        busy_wait_ms(1000);
        digitalWrite(Si5351Pwr, LOW);
        busy_wait_ms(2000);
        V1_printf("vfo_turn_on re-iinit the I2C pins inside loop. tries %d" EOL, tries);

        // all clocks off?
        // FIX! what about timeout?
        Watchdog.reset();
        res = i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, val);
        V1_flush();
    }
    if (res == 2) si5351bx_clken = val;
    V1_printf("vfo_write_clock_en_with_retry res %d END" EOL, res);
}
//****************************************************
void vfo_turn_on() {
    V1_print(F(EOL "vfo_turn_on START" EOL));
    Watchdog.reset();

    // already on successfully?
    // FIX! ..always turn it on now? for debug
    // if (vfo_is_on()) return;

    // could there be reset problems ..we need to off then on?
    // FIX! we could remove the explict extra vfo_set_power_on(true)
    // when calling vfo_turn_on()
    vfo_set_power_on(false);
    sleep_ms(1000);
    vfo_set_power_on(true);
    sleep_ms(3000);
    vfo_turn_on_completed = false;
    vfo_turn_off_completed = false;

    // this should have been done once in setup
    // but we disable the i2c now on turn off? so have to turn it on again?
    vfo_init();
    Watchdog.reset();
    // 2 secs enough?
    sleep_ms(2000);

    // Disable all CLK output drivers
    V1_println(F("vfo_turn_on trying to i2cWrite SI5351A_OUTPUT_ENABLE_CONTROL with 0xff"));
    vfo_write_clock_en_with_retry(0xff);
    V1_print(F("vfo_turn_on done trying to init the I2C pins in loop" EOL));

    // could disable all OEB pin enable control??
    // (OEB/SSEN pins don't exist on si5351a 3 output)
    i2cWrite(9, 0xFF);

    // 12/28/24 new writes below here are good
    // new: pll input source is XTAL for PLLA and PLLB
    i2cWrite(15, 0x00);

    // init:
    // Power Down, Fractional Mode. PLLB source to MultiSynth. Not inverted.
    // Src Multisynth. 2mA drive
    uint8_t reg;
    for (reg = SI5351A_CLK0_CONTROL; reg <= SI5351A_CLK7_CONTROL; reg++) {
        Watchdog.reset();
        i2cWrite(reg, 0xAC);
    }

    memset(s_vfo_drive_strength, 0, 0);
    // Moved this down here ..we don't know if we'll hang earlier
    // sets state to be used later

    //*********************
    uint8_t power;
    if (cc._solar_tx_power[0] == '1') {
        switch (SOLAR_SI5351_TX_POWER)  {
            case 0: power = SI5351A_CLK01_IDRV_2MA; break;
            case 1: power = SI5351A_CLK01_IDRV_4MA; break;
            case 2: power = SI5351A_CLK01_IDRV_6MA; break;
            case 3: power = SI5351A_CLK01_IDRV_8MA; break;
            default: power = SI5351A_CLK01_IDRV_8MA;
        }
    } else {
        if (cc._tx_high[0] == '1') power = SI5351A_CLK01_IDRV_8MA;
        else power = SI5351A_CLK01_IDRV_4MA;
    }
    // this also clears the "prev" state, so we know we'll reload all state in the si5351
    // i.e. no optimization based on knowing what we had sent before!
    vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, power);
    vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, power);

    //*********************
    // new 12/27/24: have clock 0:3 disabled state be high impedance
    i2cWrite(24, 0b10101010);
    // clk 4-7 ? shouldn't hurt..doesn't exist on si5351a 3 output
    i2cWrite(25, 0b10101010);

    // FIX! are these just initial values?
    // set PLLA-B for div_16 mode (minimum even integer division)
    const uint8_t s_plla_values[] = { 0, 0, 0, 0x05, 0x00, 0, 0, 0 };
    const uint8_t s_pllb_values[] = { 0, 0, 0, 0x05, 0x00, 0, 0, 0 };
    // write 8 regs
    // set PLLA for div_16 mode (minimum even integer division)
    Watchdog.reset();
    i2cWriten(26, (uint8_t *)s_plla_values, 8);
    // we use pllb only? (write 8 regs)
    // set PLLB for div_16 mode (minimum even integer division)
    i2cWriten(34, (uint8_t *)s_pllb_values, 8);

    // set MS0-2 for div_4 mode (min. division)
    const uint8_t s_ms_01_values[] = { 0, 1, 0x0C, 0, 0, 0, 0, 0 };
    const uint8_t s_ms_2_values[]  = { 0, 1, 0x0C, 0, 0, 0, 0, 0 };
    // write 8 regs
    // set MS0 for div_4 mode (min. division)
    Watchdog.reset();
    i2cWriten(42, (uint8_t *)s_ms_01_values, 8);
    // set MS1 for div_4 mode (min. division)
    i2cWriten(50, (uint8_t *)s_ms_01_values, 8);
    // set MS2 for div_4 mode (min.division)
    i2cWriten(58, (uint8_t *)s_ms_2_values, 8);

    // disable spread spectrum
    // FIX! what is 149? doesn't exist?
    // only a feature on PLLA?
    // in AN619
    // https://github.com/adafruit/Adafruit_Si5351_Library/pull/16
    i2cWrite(149, 0x00);

    // leave phase offsets = default (0)
    // new 2/28/24 now we're going to write 0's to clk0/1/2 offsets, just in case
    Watchdog.reset();
    i2cWrite(165, 0x00);
    i2cWrite(166, 0x00);
    i2cWrite(167, 0x00);

    // leave 183 crystal load capacitance at default 11 10pF
    // FIX! we're using default 10pf load capacitance?
    // write it just in case
    // Register 183. Crystal Internal Load Capacitance
    // Bit
    // Reset value = 11xx xxxx
    // Type R/WR/W
    // 7:6 XTAL_CL[1:0]
    // 5:0 Reserved
    // Bits 5:0 should be written to 010010b. (FIX! hmm. we write 0)
    //
    // Crystal Load Capacitance Selection.
    // These 2 bits determine the internal load capacitance value for the crystal.
    // See the Crystal Inputs section in the Si5351 data sheet.
    // 00: Reserved. Do not select this option.
    // 01: Internal CL = 6 pF.
    // 10: Internal CL = 8 pF.
    // 11: Internal CL = 10 pF (default).
    // new 12/28/24
    // better? low by 4hz of middle?
    // did this cause variance over time?
    // seeing it vary between 133 and 136 hz
    // i2cWrite(183, 0x80);

    // off. high by 16 hz on 1 tracker
    i2cWrite(183, 0xC0);

    // FIX! is 187 this a reserved address?
    // in AN1234 register map, this is shown as CLKIN_FANOUT_EN and XO_FANOUT_EN
    // also MS_FANOUT_EN ? Rev  0.6 of spec (newer is 0.95?)
    // https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
    // also in AN619 "Manually Generating an Si5351 Register map" covers Si5351A/B/C
    // for Si5351 16QFN device only?
    // but what enables it?
    // I think it just doesn't exist in the 0.95 datasheet. let's leave it out!
    // i2cWrite(187, 0x00);
    // Disable all fanout

    //***********************
    // clear all the old state
    s_PLLB_ms_div_prev = 0;
    s_PLLA_ms_div_prev = 0;
    s_PLLB_pll_mult_prev = 0;
    s_PLLA_pll_mult_prev = 0;
    memset(s_PLLB_regs_prev, 0, 8);
    memset(s_PLLA_regs_prev, 0, 8);
    si5351bx_clken = 0xff;  // all disabled

    Watchdog.reset();
    // start with symbol 0 to match what we leave it to, during warm up time later?
    uint32_t hf_freq = XMIT_FREQUENCY;
    uint64_t freq_xxx_with_symbol;
    calcSymbolFreq_xxx(&freq_xxx_with_symbol, hf_freq, 0);
    vfo_set_freq_xxx(WSPR_TX_CLK_0_NUM, freq_xxx_with_symbol, false, false);

    // does PLLA matter? these print the lock status by doing i2c reads
    if (false) si5351a_reset_PLLA(true);

    si5351a_reset_PLLB(true);

    // the final state is clk0/clk1 running and clk outputs on?
    // don't need this if it was done in vfo_set_freq_xx (at end)
    // if it wasn't, the clocks won't be on until you get one of these?
    // vfo_turn_on_clk_out(clk_num, true);  // print

    // could get clocks off with this, but we're going to get the clks
    // rf'ing soon anyhow.
    // si5351a_power_down_clk01();

    vfo_turn_on_completed = true;
    V1_print(F("vfo_turn_on END" EOL));
}
//****************************************************
// do this on the tcxo 26Mhz freq as ppb effect to get actual tcxo freq
uint32_t doCorrection(uint32_t freq) {
    uint32_t freq_corrected = freq;
    if (atoi(cc._correction) != 0) {
    // this will be a floor divide
    // https://user-web.icecube.wisc.edu/~dglo/c_class/constants.html
        V1_printf("Correcting tcxo %lu freq using ppb multiplier (* 1e-9) to get actual tcxo freq", freq);
        V1_printf(" correction %s" EOL, cc._correction);
        freq_corrected = freq + (atoi(cc._correction) * freq / 1000000000UL);
    }
    V1_printf("doCorrection freq %lu freq_corrected %lu (actual tcxo freq?)" EOL,
        freq, freq_corrected);
    return freq_corrected;
}
//****************************************************
void vfo_turn_off(void) {
    V1_println(F(EOL "vfo_turn_off START"));
    if (vfo_is_off()) { 
        V1_println(F("vfo_turn_off END already off"));
        return;
    }
    vfo_turn_on_completed = false;
    vfo_turn_off_completed = false;

    // disable all clk output
    // FIX! this will fail if i2c is not working. hang if vfo is powered off?
    // we ride thru it with a -2 return?
    si5351bx_clken = 0xff;

    // FIX! if si5351a power is actually off
    // we'll get ERROR: res -1 after i2cWrite 3
    // do we care about cleanly stopping clocks before power off?
    // 12/14/24 don't do, for now
    // i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
    // sleep_ms(10);

    // void busy_wait_us_32 (uint32_t delay_us)
    // Busy wait wasting cycles for the given (32 bit) number
    // of microseconds using the default timer instance.
    busy_wait_us_32(10000);
    vfo_set_power_on(false);

    // disable the i2c
    gpio_set_function(VFO_I2C_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(VFO_I2C_SCL_PIN, GPIO_FUNC_NULL);
    vfo_turn_off_completed = true;
    V1_println(F("vfo_turn_off END"));
}

//**********************************
// calculate the actual tx frequency for a symbol,
// given the base xmit freq for the u4b channel config'ed
// WARNING: THIS MUST NOT BE USED FOR SI5351A calcs..just for printing!
double calcSymbolFreq(uint32_t hf_freq, uint8_t symbol, bool print) {
    if (symbol > 3) {
        V1_printf("ERROR: calcSymbolFreq symbol %u is not 0 to 3 ..using 0"
            EOL, symbol);
        symbol = 0;
    }
    // the frequency shift is 12000 / 8192 (approx 1.46hz)
    // 1.46..Hz per symbol
    double symbolOffset = ((double) symbol * 12000.0) / 8192.0;
    double symbolFreq = symbolOffset + (double) hf_freq;
    if (print) {
        V1_printf("For hf_freq %lu symbol %u symbolFreq is %.6f",
            hf_freq, symbol, symbolFreq);
        V1_printf(" symbolOffset %.6f" EOL, symbolOffset);
    }
    return symbolFreq;
}

//**********************************
// uint64_t so we can get more precision!
// 10M -> 29Mhz. Want 10e-4 precision on freq.
// so log2(29e6*10e) = 38 or 39 bits of precision needed.
// so want 64-bits for the shifted *_xxx
// was: Change to using ptr. seems like I lose half of the return 64-bit sometimes?

// was: uint64_t calcSymbolFreq_xxx(uint32_t hf_freq, uint8_t symbol) {
void calcSymbolFreq_xxx(uint64_t *freq_xxx, uint32_t hf_freq, uint8_t symbol) {
    // not expensive to always recalc the symbol freq
    // don't want printing though (too slow)
    // if we add 0.5 effectively, then it's like a round up?
    // was: 1/7/25
    //    ((symbol * (12000L << PLL_CALC_SHIFT) + 4096L) / 8192L);
    // let's add nothing. Just have more precision (64 bits)
    uint64_t wspr_shift_xxx = (12000L << PLL_CALC_SHIFT) / 8192L;
    uint64_t freq_xxx_here =
        (((uint64_t) hf_freq) << PLL_CALC_SHIFT) +
        (wspr_shift_xxx * (uint64_t) symbol);

    *freq_xxx = freq_xxx_here;
    // return freq_xxx;
}

//**********************************
uint8_t startSymbolFreq(uint32_t hf_freq, uint8_t symbol, bool only_pll_num, bool just_do_calcs) {
    // Calculate the frequency for a symbol
    // Note all the shifting so integer arithmetic is used everywhere,
    // and precision is not lost.
    static bool oneShotDebugPrinted = false;
    if (!oneShotDebugPrinted) {
        // if it's the first startSymbolFreq with only_pll_num false,
        // we have time to print the 4 symbol freqs that will be used
        // we can shift down, so they will be what the sdr sees
        // AH this has more precision in it..can't just shift down to print it!
        // Doug has WSPR_TONE_SPACING_HUNDREDTHS_HZ = 146 (1.4648 Hz)
        // don't do the >> PLL_CALC_SHIFT here, as that's integer roundoff
        // do float division by 16!
        // print all info
        double symbol_0_freq = calcSymbolFreq(hf_freq, 0, true);
        double symbol_1_freq = calcSymbolFreq(hf_freq, 1, true);
        double symbol_2_freq = calcSymbolFreq(hf_freq, 2, true);
        double symbol_3_freq = calcSymbolFreq(hf_freq, 3, true);
        V1_print(F(EOL));
        V1_printf("symbol_0_freq %.6f" EOL, symbol_0_freq);
        V1_printf("symbol_1_freq %.6f" EOL, symbol_1_freq);
        V1_printf("symbol_2_freq %.6f" EOL, symbol_2_freq);
        V1_printf("symbol_3_freq %.6f" EOL, symbol_3_freq);
        V1_print(F(EOL));
        oneShotDebugPrinted = true;
    }

    // FIX! we should cache the last four symbol results with hf_freq and symbol?
    // inside freq_xxx_with_symbol? so then if you recalc and PLL_FREQ_TARGET
    // is the same, I shouldn't have to recalculate? use the saved result?
    // if any of the saved PLL_FREQ_TARGET are not the same as current, ignore them
    // or if the saved hf_freq is not the same.. Oh save the denom, since PLL_DENOM_OPTIMIZE also
    // affects the numerator-shift algo

    // not expensive to always recalc the symbol freq
    uint64_t freq_xxx_with_symbol;
    calcSymbolFreq_xxx(&freq_xxx_with_symbol, hf_freq, symbol);
    // FIX! does this change the state of the clock output enable?
    // no..
    // changes both clk0 and clk1
    uint8_t retcode = 0;
    retcode = vfo_set_freq_xxx(WSPR_TX_CLK_0_NUM, freq_xxx_with_symbol, only_pll_num, just_do_calcs);
    return retcode;

    // Note: Remember to do setup with the base frequency and symbol == 0,
    // so the i2c writes have seeded the si5351
    // so the first symbol won't have different delay than the subsequent symbols
    // hmm wonder what the typical "first symbol" is for a real wspr message?
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
// Then use it in the set_correction() method in future use of this particular
// reference oscillator.
// Once this correction factor is determined, it should not need to be measured again
// for the same reference oscillator/Si5351 pair unless you want to redo the
// calibration.

// With an accurate measurement at one frequency,
// This calibration should be good across the entire tuning range.
// The calibration method is called like this:
// si5351.set_correction(-6190, SI5351_PLL_INPUT_XO);

// However, you may use the third argument in the init() method to specify
// the frequency correction and may not actually need to use the
// explict set_correction() method in your code.

// One thing to note: the library is set for a 25 MHz reference crystal
// Correction is parts per billion for the frequency used/measured
// Could try a number of correction values and decide which to use
// (try 10 WSPR with correction 10/20/50/100/500/1000?)

// re: 25 or 26 or 27mhz into si5351a
// Re: Si5351A Freq Synth- 25MHz or 27MHz Reference Oscillator and Why?
// here's a quick overview:
// The '5351 has a PLL with a range from 600 to 900 MHz, and two dividers.
// These are "fractional dividers", where the divisor is in the form of A + (B / C),
// and in most cases can provide a high degree of resolution.
// One divider is in the PLL feedback loop,
// and the other divides the PLL output to provide the desired clock output.
// This output fractional divider has a minimum value of 8,
// which limits the output frequency to 900/8 or 112.5 MHz.
//
// But there are also integer output divisors available: 4, and 6.
// Setting the output divider to 4 gives you a max output freq of 225 MHz.
// You are now limited to using the PLL feedback divider for frequency adjustment,
// and while there are tricks you can play with the divider fractions,
// some reference frequencies make this easier than others.
//
// So 27 MHz is a good frequency, but unless you care about
// exact WSPR tone step frequencies
// 25 MHz is probably just as good.
//
// BTW, there are two identical PLL/divider sections in this chip
// and additional dividers used to drop the output frequency
// beyond what the fractional divider can do by itself.
// And The PLL range typically goes beyond the guaranteed range, by quite a bit.
// It also has settable output delay (useful for phase control),
// settable output drive strength, and some very effective jitter attenuators.

//**************************************************
// comparing to traquito 12/21/24:
// https://groups.io/g/picoballoon/topic/110236980#msg18787
//
// details:
// We both operate in a shifted integer domain,
// when calculating what to init si5351a with.
// You work in a *100 domain so you maintain precision to
// the hundredths for symbol frequency
// To go back and forth from integer domain to the *100 domain,
// you probably use *10 and /10
//
// I work in the shifted integer domain of *16...
// you can get there and back with shifts ..i.e. << 4 and >> 4
// (arguably shifts were better to do with older slower microcontrollers)
//
// but that means my precision is in 1/16ths, not 1/100ths like you
//
// so when I did the printing correctly (the config was already correct),
// casting things to float doubles, so no precision lost on the print,
// I get the following:
//
// Comparing the actual symbol freqs I use,
// versus what the most accurate should be (to 4 decimal places)
// assuming each expected offset is 1200/8192 per the wspr spec
// so I'm pretty close..symbolOffset compared to expectedSymbolOffset
// but could be better slightly.
//
// If instead  I work in a shifted domain of 7 bit shift..i.e. *128 ..
// I can still shift left and right  rather than multiply and divide
// for uint32_t datatype that leaves 25 bits of room for calculations..
// estimating  enough up to 32Mhz worth of non-shifted target freq
// I do have some intermediate calcs in uint64_t to make sure I don't lose bits.
// If I have to, i can move more calcs to uint64_t (64-bit integer, unsigned)
// So I think if i increase my shifted domain to be in the *128 domain,
// I should get about as accurate as your *100 integer domain calcs
// (for symbol freqs)

// other ideas for creating programming values
// https://groups.io/g/QRPLabs/topic/si5351a_issues_with_frequency/96467329
// https://github.com/roncarr880/QRP_LABS_WSPR
// he does this
//   When using even output divider, PLL c parameter...
//   For 1 hz resolution,  c = CLOCK / divider.
//   For 1.46 hz resolution, c = CLOCK / ( 1.46 * divider ).
//     Can then send WSPR by just adding to the b parameter 0,1,2,or3.
//   Smallest divider for 1 hz resolution, about 26.
//   Max value of c is 1048575
//   Use of the R dividers will complicate this.

// forum poster notes difficulties at 144Mhz
// (hans uses denominator shift method though)

// You might be able to generate a particular frequency
// with the right crystal and fractional divider,
// but you can't then step the frequency in uniform sub-Hz steps.

// This would make WSPR on 144 a difficult task,
// though maybe you can get close enough to the 1.465Hz tone spacing to
// get it to work (sometimes?).
// It looks as though to generate signals in the 144mhz range,
// the Si5351 will be using a fixed output divider of 6.
// A simple Si5351 library that uses fixed point and does nothing fancy
// but does allow fractional frequencies should have a 1/6 hz accuracy,
// in that the PLL frequency is calculated to the nearest hz using fixed point.
// To get a one hz change at 144mhz the PLL will change by 6 hz.  ( divider is 6 ).

// To do better would require calculating the PLL frequency to
// a fractional frequency.

// hans says:
// The Si5351A is capable of very high precision,
// you can achieve better than a milli-Hz at 144MHz and
// I suspect a few orders of magnitude higher precision than that.
// The PLL feedback and the Division down from the internal
// PLL VCO are both fractional, of the form a + b / c.
// Both b and c are 20-bit integers.
// Most code and libraries you see just fix c at 0xFFFFF and just vary b.
// This is what limits the resolution.

//*********************************************************************************
void set_PLL_DENOM_OPTIMIZE(char *band) {
    V1_println(F("set_PLL_DENOM_OPTIMIZE START"));
    // FIX! hack! we should have fixed values per band? do they vary by freq bin?
    uint32_t PLL_DENOM_MAX = 1048575;
    V1_print(F("Sets PLL_DENOM_OPTIMIZE with hard-wired values. May not be used." EOL));
    // this is the target PLL freq when making muliplier/divider initial
    // calculations set in tracker.ino
    // NOTE: this is only if we don't calc the Numerator-Shift algo
    // i.e. this is old spreadsheet data. The denom is calculated now (if not Farey)
    // so should be no reason to use these. They are rounded up when necessary though (better)
    // since the spreadsheet did fp math, not scaled integer
    int b = atoi(band);
    switch (PLL_FREQ_TARGET) {
        case 900000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 522039; break; // div 34 mul 36
                case 12: PLL_DENOM_OPTIMIZE = 493037; break; // div 36 mul 34
                case 15: PLL_DENOM_OPTIMIZE = 422603; break; // div 42 mul 34
                case 17: PLL_DENOM_OPTIMIZE = 328691; break; // div 54 mul 37
                case 20: PLL_DENOM_OPTIMIZE = 253562; break; // div 70 mul 37
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; // 2m? (no)
            }
            break;
        case 700000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 739556; break; // div 24 mul 25
                case 12: PLL_DENOM_OPTIMIZE = 633905; break; // div 28 mul 26
                case 15: PLL_DENOM_OPTIMIZE = 522039; break; // div 34 mul 27
                case 17: PLL_DENOM_OPTIMIZE = 467088; break; // div 38 mul 26
                case 20: PLL_DENOM_OPTIMIZE = 354987; break; // div 50 mul 27
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; // 2m? (no)
            }
            break;
        case 600000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 806788; break; // div 22 mul 23
                case 12: PLL_DENOM_OPTIMIZE = 739556; break; // div 24 mul 23
                case 15: PLL_DENOM_OPTIMIZE = 633905; break; // div 28 mul 22
                case 17: PLL_DENOM_OPTIMIZE = 522039; break; // div 34 mul 23
                case 20: PLL_DENOM_OPTIMIZE = 422603; break; // div 42 mul 22
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; // 2m? (no)
            }
            break;
        case 500000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 986074; break; // div 18 mul 19
                case 12: PLL_DENOM_OPTIMIZE = 887467; break; // div 20 mul 19
                case 15: PLL_DENOM_OPTIMIZE = 739556; break; // div 24 mul 19
                case 17: PLL_DENOM_OPTIMIZE = 633905; break; // div 28 mul 19
                case 20: PLL_DENOM_OPTIMIZE = 493037; break; // div 36 mul 19
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; // this algo can't do 2m
            }
            break;
        default:
            if (!USE_FAREY_WITH_PLL_REMAINDER) {
                V1_print(F("ERROR: using unoptimized PLL_DENOM_OPTIMIZE for"));
                V1_printf(" PLL_FREQ_TARGET %" PRIu64 EOL, PLL_FREQ_TARGET);
            }
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; break;
                case 12: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; break;
                case 15: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; break;
                case 17: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; break;
                case 20: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX; break;
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX;
            }
    }
    V1_println(F("set_PLL_DENOM_OPTIMIZE END"));
}

//****************************************************
// 1/11/25 NEW: set PLL_FREQ_TARGET by band
void init_PLL_freq_target(uint64_t *PLL_FREQ_TARGET, char *band) {
    uint64_t pll_freq_target;
    if (USE_FAREY_WITH_PLL_REMAINDER) {
        switch (atoi(band)) {
            case 20: pll_freq_target = 600000000; break;
            case 17: pll_freq_target = 600000000; break; // 425 didn't get uHz error
            case 15: pll_freq_target = 600000000; break;
            case 12: pll_freq_target = 600000000; break;
            case 10: pll_freq_target = 600000000; break;
            case  2: pll_freq_target = 600000000; break;
            // default to 20M in case of error cases
            default: pll_freq_target = 600000000;
        }
    } else {
        switch (atoi(band)) {
            case 20: pll_freq_target = 600000000; break;
            case 17: pll_freq_target = 600000000; break;
            case 15: pll_freq_target = 600000000; break;
            case 12: pll_freq_target = 600000000; break;
            case 10: pll_freq_target = 600000000; break;
            case  2: pll_freq_target = 900000000; break;
            // default to 20M in case of error cases
            default: pll_freq_target = 600000000;
        }
    }
    *PLL_FREQ_TARGET = pll_freq_target;
}   

//****************************************************
// vfo_calc_cache operation (VCC)
// 0: flush cache
// 1: lookup cache and return data (ptrs) and true if match, false if not
// 2: install into cache at stack ptr and increment ptr modulo 4 (cache size)
// 3: print current cache contents

// cache size is fixed at 5 (four wspr freqs plus 1 cw freq, for configured band/ u4b channel
// if any constants like PLL_FREQ_TARGET are changed, there are hardwired constants
// so no issue in flushing the cache. We do band sweeps and freq sweeps, but
// the freq should be enough to managing cache validity.
// cache is only used to save Farey results? could save fixed PLL_DENOM_OPTIMIZE results also?
// we'll recompute acut and actual_pll_freq wherever this is used
void vfo_calc_cache_flush() {
    uint32_t junku[1] = { 0 };
    uint64_t junku64[1] = { 0 };
    double junkd[1] = { 0 };
    vfo_calc_cache(junkd, junkd, junku, junku, junku, junku, junku64[0], 0);
}

// this will do a print of current valid entries
// will force a reboot if any valid entry has any values that are 0
uint8_t vfo_calc_cache_print_and_check() {
    uint32_t junku[1] = { 0 };
    uint64_t junku64[1] = { 0 };
    double junkd[1] = { 0 };
    uint8_t retval = vfo_calc_cache(junkd, junkd, junku, junku, junku, junku, junku64[0], 3);
    return retval;
}

// actual and actual_pll_freq are just saved, so we can total bypass the normal vfo_calc* routine
// and reproduce it's data. simplest.
// FIX! for testing, can change this to other sizes and you'll see that normal wspr
// starts doing Farney algo
const uint8_t VCC_SIZE = 5;
uint8_t vfo_calc_cache(double *actual, double *actual_pll_freq,
    uint32_t *ms_div, uint32_t *pll_mult, uint32_t *pll_num, uint32_t *pll_denom,
    uint64_t freq_xxx, uint8_t operation) {

    // no prints on the lookup
    if (operation != 1) {
        V1_print(F("vfo_calc_cache START" EOL));
    }

    static bool cache_valid[VCC_SIZE] = { 0 };
    static double cache_actual[VCC_SIZE] = { 0 };
    static double cache_actual_pll_freq[VCC_SIZE] = { 0 };
    static uint32_t cache_ms_div[VCC_SIZE] = { 0 };
    static uint32_t cache_pll_mult[VCC_SIZE] = { 0 };
    static uint32_t cache_pll_num[VCC_SIZE] = { 0 };
    static uint32_t cache_pll_denom[VCC_SIZE] = { 0 };
    static uint64_t cache_freq_xxx[VCC_SIZE] = { 0 };

    static uint8_t ptr = 0;

    double actual_here = 0.0;
    double actual_pll_freq_here = 0.0;
    uint32_t ms_div_here = 0;
    uint32_t pll_mult_here = 0;
    uint32_t pll_num_here = 0;
    uint32_t pll_denom_here = 0;
    uint64_t freq_xxx_here = 0;

    bool found = false;
    uint8_t retval = 0;
    uint8_t found_i = 0;
    switch (operation) {
        case 0: {   // invalidate cache
            // just valid clear should be fine, but all for safety/bug finding! no stale uses!
            memset(cache_valid, 0, sizeof(cache_valid));
            memset(cache_actual, 0, sizeof(cache_actual));
            memset(cache_actual_pll_freq, 0, sizeof(cache_actual_pll_freq));
            memset(cache_ms_div, 0, sizeof(cache_ms_div));
            memset(cache_pll_mult, 0, sizeof(cache_pll_mult));
            memset(cache_pll_num, 0, sizeof(cache_pll_num));
            memset(cache_pll_denom, 0, sizeof(cache_pll_denom));
            memset(cache_freq_xxx, 0, sizeof(cache_freq_xxx));
            retval = 1;
            break;
        }
        case 1:  {  // search in cache
            for (uint8_t i = 0; i < VCC_SIZE; i++) {
                freq_xxx_here = cache_freq_xxx[i];
                if (cache_valid[i] && (freq_xxx_here == freq_xxx)) {
                    actual_here = cache_actual[i];
                    actual_pll_freq_here = cache_actual_pll_freq[i];
                    ms_div_here = cache_ms_div[i];
                    pll_mult_here = cache_pll_mult[i];
                    pll_num_here = cache_pll_num[i];
                    pll_denom_here = cache_pll_denom[i];
                    // shouldn't >1 hit? will use last if so
                    if (found) {
                        V1_printf("ERROR: vfo_calc_cache multi hits: i %u prior found_i %u" EOL,
                            i, found_i);
                        V1_print(F("MULTI HIT: "));
                        V1_printf(" vfo_calc_cache hit on i %u freq_xxx %" PRIu64,
                            i, freq_xxx_here);
                        V1_printf(" actual %.6f actual_pll_freq %.6f",
                            actual_here, actual_pll_freq_here);
                        V1_printf(" pll_mult %lu pll_num %lu pll_denom %lu",
                            pll_mult_here, pll_num_here, pll_denom_here);
                        V1_printf(" ms_div %lu" EOL, ms_div_here);
                    }

                    bool badCache =
                        (actual_here == 0.0) ||
                        (actual_pll_freq_here == 0.0) ||
                        (ms_div_here == 0) ||
                        (pll_mult_here == 0) ||
                        (pll_num_here == 0) ||    // should never happen? very unlikely
                        (pll_denom_here == 0) ||
                        (freq_xxx_here == 0);    // we should never lookup freq_xxx

                        // is 900 really the upper limiter or ?? we have 908 for 24Mhz
                        // do we need lower pll freq?
                        if (ms_div_here < 4 || ms_div_here > 900) {
                            V1_printf("ERROR: cached ms_div %lu is out of range 4 to 900" EOL,
                                ms_div_here);
                            // FIX! how should we recalc
                            badCache = true;
                        }
                        if (pll_mult_here < 15 || pll_mult_here > 90) {
                            V1_printf("ERROR: cached pll_mult %lu is out of range 15 to 90." EOL,
                                pll_mult_here);
                            badCache = true;
                        }
                        if (freq_xxx_here == 0) {
                            V1_printf("ERROR: cached freq_xxx %" PRIu64 " is 0." EOL,
                                freq_xxx_here);
                            badCache = true;
                        }
                        // FIX! we could check the num/denum for range also?

                    if (badCache) {
                        V1_println(F("ERROR: fatal. valid cache entry had 0 or freq_xxx 0."));
                        V1_println(F("ERROR: won't use anything from this cache entry"));
                        found = false;
                    } else {
                        found = true;
                        found_i = i;
                        retval = 1;
                        *actual = actual_here;
                        *actual_pll_freq = actual_pll_freq_here;
                        *ms_div = ms_div_here;
                        *pll_mult = pll_mult_here;
                        *pll_num = pll_num_here;
                        *pll_denom = pll_denom_here;
                    }
                }
            }
            break;
        }
        case 2: {  // write to cache and make valid
            cache_valid[ptr] = true;
            cache_actual[ptr] = *actual;
            cache_actual_pll_freq[ptr] = *actual_pll_freq;
            cache_ms_div[ptr] = *ms_div;
            cache_pll_mult[ptr] = *pll_mult;
            cache_pll_num[ptr] = *pll_num;
            cache_pll_denom[ptr] = *pll_denom;
            cache_freq_xxx[ptr] = freq_xxx;
            ptr = ((ptr+1) % VCC_SIZE);
            retval = 1;
            break;
        }
        case 3: {  // print valid cache contents for debug
            // we can print and validate the cache every tracker.ino loop?
            uint8_t VCC_valid_cnt = 0;
            for (uint8_t i = 0; i < VCC_SIZE; i++) {
                if (cache_valid[i]) {
                    VCC_valid_cnt += 1;
                    actual_here = cache_actual[i];
                    actual_pll_freq_here = cache_actual_pll_freq[i];
                    ms_div_here = cache_ms_div[i];
                    pll_mult_here = cache_pll_mult[i];
                    pll_num_here = cache_pll_num[i];
                    pll_denom_here = cache_pll_denom[i];
                    freq_xxx_here = cache_freq_xxx[i];
                    V1_printf("vfo_calc_cache valid i %u freq_xxx %" PRIu64,
                        i, freq_xxx_here);
                    V1_printf(" actual %.6f actual_pll_freq %.6f",
                        actual_here, actual_pll_freq_here);
                    V1_printf(" pll_mult %lu pll_num %lu pll_denom %lu",
                        pll_mult_here, pll_num_here, pll_denom_here);
                    V1_printf(" ms_div %lu" EOL, ms_div_here);

                    bool badCache =
                        (actual_here == 0.0) ||
                        (actual_pll_freq_here == 0.0) ||
                        (ms_div_here == 0) ||
                        (pll_mult_here == 0) ||
                        (pll_num_here == 0) ||    // should never happen? very unlikely
                        (pll_denom_here == 0) ||
                        (freq_xxx_here == 0);    // we should never lookup freq_xxx

                    if (badCache) {
                        V0_println(F("ERROR: fatal, reboot. valid cache had 0 or freq_xxx 0." EOL));
                        V0_flush();
                        Watchdog.enable(5000);  // milliseconds
                        while (true) tight_loop_contents();
                    }
                }
            }
            retval = VCC_valid_cnt;
            break;
        }

        default: {
            V1_printf("ERROR: illegal vfo_cache_cache operation %u" EOL, operation);
            retval = 1;
        }
    }

    // count the current valid entries (after any update)
    uint8_t totalValid = 0;
    for (uint8_t i = 0; i < VCC_SIZE; i++) {
        if (cache_valid[i]) totalValid += 1;
    }
// no print on lookup
    if (operation != 1) {
        V1_printf("vfo_calc_cache END totalValid %u" EOL, totalValid);
    }
    return retval;
}

