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
// Developed by Kazuhisa "Kazu" Terasaki AG6NS
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
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

#include "si5351_functions.h"
#include "print_functions.h"
#include "led_functions.h"
#include "u4b_functions.h"

// These are in arduino-pio core
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries
// #include <SPI.h>
// #include <Wire.h>

// this is the target PLL freq when making muliplier/divider initial calculations
// set in tracker.ino
extern uint32_t PLL_FREQ_TARGET;

// from tracker.ino
extern uint32_t SI5351_TCXO_FREQ;  // 26 mhz with correction already done
extern const int SI5351A_I2C_ADDR;
extern const int VFO_I2C0_SCL_HZ;

// Tried a concave optimization (stepwise) on this
// now that we have optimum denoms from a spreadsheet, (for given mult/div per band)
// this is really just a double-check in case pll_mult/pll_div change with 
// a different algo, and I need the hard-wired denom adjusted..
// Assuming pll_num and pll_denom can optimize independently one at a time?

// starts with the max;
extern uint32_t PLL_DENOM_OPTIMIZE;
extern uint32_t XMIT_FREQUENCY;
// decode of _verbose 0-9
extern bool VERBY[10];

extern char _tx_high[2];  // 0 is 4mA si5351. 1 is 8mA si5351
extern char _correction[7];  // parts per billion -3000 to 3000. default 0
extern char _Band[3];  // string with 10, 12, 15, 17, 20 legal. null at end
extern char _U4B_chan[4];  // string with 0-599

extern const int Si5351Pwr;
// FIX! are these just used on the Wire.begin?
// FIX! should this be a extern const. Or: only used here?
#define VFO_I2C_INSTANCE i2c0
extern const int VFO_I2C0_SDA_PIN;
extern const int VFO_I2C0_SCL_PIN;

extern const int PLL_CALC_SHIFT;

static bool vfo_turn_on_completed = false;
static bool vfo_turn_off_completed = false;

static uint8_t si5351bx_clken = 0xff;
static bool s_is_on = false;

// updated with config _tx_high during vfo_turn_on()
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

// only used because we set PLLA the same as PLLB, so it can lock?
// FIX! but if we set it up, does that cause it to be on when it was off before?

static uint8_t s_PLLA_regs_prev[8] = { 0 };
static uint32_t s_PLLA_ms_div_prev = 0;

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

    // init I2C0 for VFO
    uint32_t actualRate = i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
    V1_printf("vfo_init i2c actual rate: %lu" EOL, actualRate);
    // let it float if we don't drive the i2c?
    gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
    gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);
    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
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
    V1_println(F("vfo_set_power_on After V1_flush()"));

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
        V1_printf("GOOD: i2cWrRead() got good res %d reg %02x val %02x" EOL,
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
void si5351a_setup_PLLB(uint8_t mult, uint32_t num, uint32_t denom) {
    Watchdog.reset();
    // V1_printf("si5351a_setup_PLLB START mult %u num %lu denom %lu" EOL, mult, num, denom);
    uint8_t PLLB_regs[8] = { 0 };

    uint32_t p1 = 128 * mult + ((128 * num) / denom) - 512;
    uint32_t p2 = 128 * num - denom * ((128 * num) / denom);
    uint32_t p3 = denom;

    PLLB_regs[0] = (uint8_t)(p3 >> 8);
    PLLB_regs[1] = (uint8_t)p3;
    PLLB_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    PLLB_regs[3] = (uint8_t)(p1 >> 8);
    PLLB_regs[4] = (uint8_t)p1;
    PLLB_regs[5] = ((uint8_t)(p3 >> 12) & 0xf0) | ((uint8_t)(p2 >> 16) & 0x0f);
    PLLB_regs[6] = (uint8_t)(p2 >> 8);
    PLLB_regs[7] = (uint8_t)p2;

    // Hans says registers are double-buffered, and the last-of-8 triggers an update of all the changes
    // 'Always include in your block of register writes, the final one of the block of 8. 
    // So by all means chop some unnecessary writes off the start of the block, but never the end!'
    // https://groups.io/g/picoballoon/message/19155
    // Maybe this works, because p2 is affected by both numerator and denominator
    // I'm doing numerator-change-only for symbol shift now, so p2 should always change
    // unless the symbol doesn't change!

    // start and end are looked at below, if s_PLLB_ms_div_prev != 0 which 
    // says we can look at last saved state (s_PLLB_regs_prev)
    // so these are out of the for loops
    uint8_t start = 0;
    uint8_t end = 7;

    // Looking for a range of bits that changed?
    // the i2cWriten writes as burst below.
    if (s_PLLB_ms_div_prev != 0) {  // global. basically a 'valid' bit: implies s_PLLB_regs_prev has data
        for (; start < 8; start++) {
            if (PLLB_regs[start] != s_PLLB_regs_prev[start]) break;
        }
        // detect no change of anything?
        if (start == 8) return;

        for (; end > start; end--) {
            // so so we just write the start to end that has changed?
            if (PLLB_regs[end] != s_PLLB_regs_prev[end]) break;
        }
    }

    uint8_t reg = SI5351A_PLLB_BASE + start;
    uint8_t len = end - start + 1;
    i2cWriten(reg, &PLLB_regs[start], len);

    // this can't be swap..so how could it have worked?
    // was it a pointer copy?
    // maybe PLLB_regs memory always got reallocated on the next call?
    // wouldn't if it was static?
    // was:
    // *((uint64_t *)s_PLLB_regs_prev) = *((uint64_t *)PLLB_regs);
    memcpy(s_PLLB_regs_prev, PLLB_regs, 8);

    // V1_printf("si5351a_setup_PLLB END mult %u num %lu denom %lu" EOL, mult, num, denom);
}

//****************************************************
void si5351a_setup_PLLA(uint8_t mult, uint32_t num, uint32_t denom) {
    Watchdog.reset();
    // straight copy of *_setup_PLLB. 
    // has the global s_PLLA_regs_prev to compare to
    // V1_printf("si5351a_setup_PLLA START mult %u num %lu denom %lu" EOL, mult, num, denom);
    uint8_t PLLA_regs[8] = { 0 };

    uint32_t p1 = 128 * mult + ((128 * num) / denom) - 512;
    uint32_t p2 = 128 * num - denom * ((128 * num) / denom);
    uint32_t p3 = denom;

    PLLA_regs[0] = (uint8_t)(p3 >> 8);
    PLLA_regs[1] = (uint8_t)p3;
    PLLA_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    PLLA_regs[3] = (uint8_t)(p1 >> 8);
    PLLA_regs[4] = (uint8_t)p1;
    PLLA_regs[5] = ((uint8_t)(p3 >> 12) & 0xf0) | ((uint8_t)(p2 >> 16) & 0x0f);
    PLLA_regs[6] = (uint8_t)(p2 >> 8);
    PLLA_regs[7] = (uint8_t)p2;

    // start and end are looked at below, if s_PLLA_ms_div_prev != 0 which 
    // says we can look at last saved state (s_PLLA_regs_prev)
    // so these are out of the for loops
    uint8_t start = 0;
    uint8_t end = 7;

    // Looking for a range of bits that changed?
    // the i2cWriten writes as burst below.
    if (s_PLLA_ms_div_prev != 0) {  // global. basically a 'valid' bit: implies s_PLLA_regs_prev has data
        for (; start < 8; start++) {
            if (PLLA_regs[start] != s_PLLA_regs_prev[start]) break;
        }
        // detect no change of anything?
        if (start == 8) return;

        for (; end > start; end--) {
            // so so we just write the start to end that has changed?
            if (PLLA_regs[end] != s_PLLA_regs_prev[end]) break;
        }
    }

    uint8_t reg = SI5351A_PLLA_BASE + start;
    uint8_t len = end - start + 1;
    i2cWriten(reg, &PLLA_regs[start], len);

    memcpy(s_PLLA_regs_prev, PLLA_regs, 8);

    // V1_printf("si5351a_setup_PLLA END mult %u num %lu denom %lu" EOL, mult, num, denom);
}

//****************************************************
// swapping pointers instead of memcpy:
// both should be static or global, so no mem allocaiton issue
// https://stackoverflow.com/questions/8403447/swapping-pointers-in-c-char-int
// best/obvious to just memcpy for just 8 bytes

//****************************************************
// experiment with R divisor
// divide-by-4. to add >1Hz accuracy for symbol freqs on 10M (easier on 20M)
// was getting +- 0.5Hz. This will give +- 0.125 hz ?
// maybe increase it to 3 for divide-by-8
// divide-by-4
// const uint8_t R_DIVISOR_SHIFT = 2;
// divide-by-2
// const uint8_t R_DIVISOR_SHIFT = 1;

// divide-by-1
const uint8_t R_DIVISOR_SHIFT = 0;

//****************************************************
// div must be even number
void si5351a_setup_multisynth012(uint32_t div) {
    V1_printf("si5351a_setup_multisynth012 START div %lu" EOL, div);
    Watchdog.reset();
    if ((div % 2) != 0) {
        V1_printf("ERROR: si5351a_setup_multisynth012 div %lu isn't even" EOL, div);
    }

    uint8_t s_regs[8] = { 0 };
    uint32_t p1 = 128 * div - 512;
    // R_DIVISOR_SHIFT is hardwired constant  (/4 => shift 2)

    uint8_t R_OUTPUT_DIVIDER_ENCODE;
    switch (R_DIVISOR_SHIFT) {
        case 0: R_OUTPUT_DIVIDER_ENCODE = 0b000; break;  // divide-by-1
        case 1: R_OUTPUT_DIVIDER_ENCODE = 0b001; break;  // divide-by-2
        case 2: R_OUTPUT_DIVIDER_ENCODE = 0b010; break;  // divide-by-4
        case 3: R_OUTPUT_DIVIDER_ENCODE = 0b011; break;  // divide-by-8
        case 4: R_OUTPUT_DIVIDER_ENCODE = 0b100; break;  // divide-by-16
        case 5: R_OUTPUT_DIVIDER_ENCODE = 0b101; break;  // divide-by-32
        case 6: R_OUTPUT_DIVIDER_ENCODE = 0b110; break;  // divide-by-64
        case 7: R_OUTPUT_DIVIDER_ENCODE = 0b111; break;  // divide-by-128
        default: R_OUTPUT_DIVIDER_ENCODE = 0b000;        // divide-by-1
    }

    // bits [2:0] are R0 Output Divider
    // 000b: Divide by 1
    // 001b: Divide by 2
    // 010b: Divide by 4
    // 011b: Divide by 8
    // 100b: Divide by 16
    
    // was: s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    // But what are the magic groups of eight?  where updates only
    // happen when you write the last of the group?
    // https://groups.io/g/picoballoon/message/19164

    // Just these 4 groups of 8? (starting at these base addresses)
    // SI5351A_PLLA_BASE =               26; // 8 regs
    // SI5351A_PLLB_BASE =               34; // 8 regs
    // SI5351A_MULTISYNTH0_BASE =        42; // 8 regs
    // SI5351A_MULTISYNTH1_BASE =        50; // 8 regs    

    // R_OUTPUT_DIVIDER_ENCODE is bits 6:4
    s_regs[0] = 0;
    s_regs[1] = 1;
    s_regs[2] = (R_OUTPUT_DIVIDER_ENCODE << 4) | ((uint8_t)(p1 >> 16) & 0x03);
    s_regs[3] = (uint8_t)(p1 >> 8);
    s_regs[4] = (uint8_t)p1;
    s_regs[5] = 0;
    s_regs[6] = 0;
    s_regs[7] = 0;

    i2cWriten(SI5351A_MULTISYNTH0_BASE, s_regs, 8);
    uint8_t CLK0_control_data =
        SI5351A_CLK0_MS0_INT |
        SI5351A_CLK0_MS0_SRC_PLLB |
        SI5351A_CLK0_SRC_MULTISYNTH_0 |
        s_vfo_drive_strength[0];  // should just be 2 bits 0,1,2,3 = 2,4,6,8mA
    i2cWrite(SI5351A_CLK0_CONTROL, CLK0_control_data);
    s_CLK0_control_prev = CLK0_control_data;

    // this is used for antiphase (differential tx: clk0/clk1)
    // the output enable doesn't seem to disable..whether or not we use INV!!
    // both always enabled?
    i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);
    uint8_t CLK1_control_data =
        SI5351A_CLK1_MS1_INT |
        SI5351A_CLK1_MS1_SRC_PLLB |
        SI5351A_CLK1_SRC_MULTISYNTH_1 |
        SI5351A_CLK1_INV |
        s_vfo_drive_strength[1];  // should just be 2 bits 0,1,2,3 = 2,4,6,8mA
    i2cWrite(SI5351A_CLK1_CONTROL, CLK1_control_data);
    s_CLK1_control_prev = CLK1_control_data;

    // power down the CLK2. just make it 2MA drive 
    // right now: we don't use it for freq calibration (to rp2040 on pcb)
    i2cWriten(SI5351A_MULTISYNTH2_BASE, s_regs, 8);
    uint8_t CLK2_control_data =
        SI5351A_CLK2_MS2_INT |
        SI5351A_CLK2_MS2_SRC_PLLB |
        SI5351A_CLK2_SRC_MULTISYNTH_2 |
        SI5351A_CLK2_PDN |
        SI5351A_CLK2_IDRV_2MA; // always just 2MA for clk2, if ever used
    i2cWrite(SI5351A_CLK2_CONTROL, CLK2_control_data);
    s_CLK2_control_prev = CLK2_control_data;

    V1_printf("VFO_DRIVE_STRENGTH CLK0: %d" EOL, (int)s_vfo_drive_strength[0]);
    V1_printf("VFO_DRIVE_STRENGTH CLK1: %d" EOL, (int)s_vfo_drive_strength[1]);
    V1_printf("si5351a_setup_multisynth012 END div %lu" EOL, div);
}

//****************************************************
void si5351a_power_up_clk01() {
    // Only used for keying cw since the clk output enable doesn't seem
    // to turn off clk on ms5351m??
    // uses the previously set s_CLKn_control_prev
    // V1_print(F("si5351a_power_up_clk01 START" EOL));
    // Watchdog.reset();

    // boolean inversion. Clear PDN
    uint8_t CLK0_control_data = s_CLK0_control_prev & ~SI5351A_CLK0_PDN;
    i2cWrite(SI5351A_CLK0_CONTROL, CLK0_control_data);
    s_CLK0_control_prev = CLK0_control_data;

    uint8_t CLK1_control_data = s_CLK1_control_prev & ~SI5351A_CLK1_PDN;
    i2cWrite(SI5351A_CLK1_CONTROL, CLK1_control_data);
    s_CLK1_control_prev = CLK1_control_data;

    // always need to reset after change of CLKn_PDN to maintain phase relationship (Hans)
    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
    // V1_print(F("si5351a_power_up_clk01 END" EOL));
    // hans picture says 1.46ms for effect?
    // don't add delay here though. will change wpm timing
}
//****************************************************
void si5351a_power_down_clk01() {
    // Only used for keying cw since the clk output enable doesn't seem
    // to turn off clk on ms5351m??
    // uses the previously set s_CLKn_control_prev
    // V1_print(F("si5351a_power_down_clk01 START" EOL));
    // Watchdog.reset();

    // set PDN
    uint8_t CLK0_control_data = s_CLK0_control_prev | SI5351A_CLK0_PDN;
    i2cWrite(SI5351A_CLK0_CONTROL, CLK0_control_data);
    s_CLK0_control_prev = CLK0_control_data;

    uint8_t CLK1_control_data = s_CLK1_control_prev | SI5351A_CLK1_PDN;
    i2cWrite(SI5351A_CLK1_CONTROL, CLK1_control_data);
    s_CLK1_control_prev = CLK1_control_data;

    // always need to reset after change of CLKn_PDN to maintain phase relationship (Hans)
    // HACK: wait one millisecond before doing the pll reset?
    busy_wait_ms(1);
    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
    // hans picture says 1.46ms for effect?
    // don't add delay here though. will change wpm timing
    // sleep_ms(2);
    // V1_print(F("si5351a_power_down_clk01 END" EOL));
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

        val = 0xff; // initial value for the read to overwrite
        res = i2cWrRead(reg, &val);
        V1_printf("after PLLB reset (1): SI5351A_DEVICE_STATUS reg %02x val %02x res %d" EOL,
            reg, val, res);
        V1_flush();
        sleep_ms(4000);
        Watchdog.reset();

        val = 0xff;
        res = i2cWrRead(reg, &val);
        V1_printf("after PLLB reset(2): SI5351A_DEVICE_STATUS reg %02x val %02x res %d" EOL,
            reg, val, res);

        // clear the status, then read it
        reg = SI5351A_INTERRUPT_STATUS_STICKY;
        i2cWrite(reg, 0);
        sleep_ms(500);

        val = 0xff; // initial value for the read to overwrite
        res = i2cWrRead(reg, &val);
        V1_printf("after PLLB reset (1): SI5351A_INTERRUPT_STATUS_STICKY reg %02x val %02x res %d" EOL,
            reg, val, res);
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

        // FIX! is this mucking up anything?
        //******************
        // clear the status, then read it
        reg = SI5351A_DEVICE_STATUS;
        i2cWrite(reg, 0);
        sleep_ms(500);

        val = 0xff;
        res = i2cWrRead(reg, &val);
        // was seeing A0 here after 0.5 secs. halve to wait longer?
        V1_printf("after PLLA reset(1): SI5351A_DEVICE_STATUS reg %02x val %02x res %d" EOL,
            reg, val, res);
        V1_flush();

        sleep_ms(4000);
        Watchdog.reset();

        val = 0xff;
        res = i2cWrRead(reg, &val);
        V1_printf("after PLLA reset(2): SI5351A_DEVICE_STATUS reg %02x val %02x res %d" EOL,
            reg, val, res);

        //******************
        // clear the status, then read it
        reg = SI5351A_INTERRUPT_STATUS_STICKY;
        i2cWrite(reg, 0);
        sleep_ms(500);

        val = 0xff;
        res = i2cWrRead(reg, &val);
        V1_printf("after PLLA reset (1): SI5351A_INTERRUPT_STATUS_STICKY reg %02x val %02x res %d" EOL,
            reg, val, res);
        V1_flush();

        V1_println(F("si5351a_reset_PLLA END" EOL));
    }
}

//****************************************************
// good for doing calc only, so see what changes with freq changes
void vfo_calc_div_mult_num(double *actual, double *actual_pll_freq,
    uint32_t *ms_div, uint32_t *pll_mult, uint32_t *pll_num, uint32_t *pll_denom,
    uint32_t *r_divisor, uint32_t freq_x128) {
    Watchdog.reset();

    // interesting they had relatively low pll freq (620Mhz) for 15M case
    // https://rfzero.net/documentation/tools/si5351a-frequency-tool/
    // this is hans code from 2015:
    // https://qrp-labs.com/images/synth/demo6/si5351a.c

    // const int PLL_MAX_FREQ  = 900000000;
    // const int PLL_MIN_FREQ  = 600000000;

    // all my spread sheet stuff was with band mult/div targeting 900Mhz
    // PLL_FREQ_TARGET  = 900000000;
    // new: what if we target 700Mhz?
    // PLL_FREQ_TARGET  = 700000000;
    // set in tracker.ino now. Might change per band!

    // http://www.wa5bdu.com/programming-the-si5351a-synthesizer/
    // http://www.wa5bdu.com/si5351a-quadrature-vfo/
    // Following the VCO is another divider stage that divides the VCO frequency
    // by a value of ‘d + e/f’
    // and can be used to take the frequency down into the low MHz range.
    // The chip will provide an output with lower jitter if this value is an integer
    // and better still if it is an even integer.
    // So we let e/f be zero and select a value for d that’s an even number.

    const uint64_t PLL_DENOM_MAX = 0x000fffff;  // 1048575
    // set from either optimize algo, or known best (per band) in tracker.ino
    // either by set_PLL_DENOM_OPTIMIZE() or by si5351a_calc_optimize() which can be
    // overwritten by the former
    uint64_t PLL_DENOM = PLL_DENOM_OPTIMIZE;

    // search algo may have gone out of bounds?
    // should never negative
    if (PLL_DENOM > PLL_DENOM_MAX) PLL_DENOM = PLL_DENOM_MAX;
    uint64_t PLL_DENOM_x128 = PLL_DENOM << PLL_CALC_SHIFT;

    //*****************************
    // the divider is 'a + b/c' or "Feedback Multisynth Divider"
    // c is PLL_DENOM

    // Following the VCO is another divider stage
    // that divides the VCO frequency by a value of ‘d + e/f’ and
    // can be used to take the frequency down into the low MHz range.
    // The chip will provide an output with lower jitter if this value is an integer.
    // better still if it is an even integer.
    // So we let e/f be zero and select a value for d that’s an even number.
    // Remember that there is enough resolution in the PLL/VCO stage to provide fine tuning.
    // The data sheet calls the ‘d + e/f’ divider the Output Multisynth Divider,
    // because it acts on the output of the PLL/VCO.
    // we don't have to worry about the divider R. we use 1 for that

    // So we’re down to six values to calculate which are the ‘a, b, c, d, e and f’ of
    // the dividers ‘a + b/c’ and ‘d + e/f’.
    // But it will actually be a lot simpler.
    // We said that the second divider d + e/f will be an even integer,
    // so e and f are not needed.
    // Then in the first divider a + b/c, we will make c a constant
    // so we are now down to three required values: a, b and d.

    // we can hardwire in a divide-by-4 in the R0 and R1 output dividers
    // so the ms_div would be 1/4th what it would be for a divide-by-1 R0 and R1
    // the << 2 in the divisor
    // not doing that any more. R_DIVISOR_SHIFT is zero.
    uint64_t ms_div_here = 1 + (
        (((uint64_t)PLL_FREQ_TARGET) << PLL_CALC_SHIFT) /
        (((uint64_t)freq_x128) << R_DIVISOR_SHIFT));
    ms_div_here &= 0xfffffffe;   // make it even number

    // is 900 really the upper limiter or ?? we have 908 for 24Mhz
    // do we need lower pll freq?
    if (ms_div_here < 4 || ms_div_here > 900) {
        V1_printf("ERROR: ms_div %" PRIu64 " is out of range 4 to 900" EOL, ms_div_here);
        // FIX! how should we recalc
        V1_print(F("ERROR: no recalc done. Should never happen! rf output is wrong" EOL));
    }

    // R_DIVISOR_SHIFT: possible *4 for the R0 and R1 output divider.
    // 64 bit calcs so don't lose bits beyond 32-bits
    uint64_t pll_freq_x128 = ((uint64_t)freq_x128 * ms_div_here) << R_DIVISOR_SHIFT;
    // this is just integer. not useful!
    uint64_t pll_freq_here = pll_freq_x128 >> PLL_CALC_SHIFT;

    // FIX! should we just apply correction to the crystal frequency? yes.
    // SI5351_TXCO_FREQ is calculated in tracker.ino set so correction calc
    // is just one once

    uint64_t tcxo_freq = (uint64_t) SI5351_TCXO_FREQ;  // 26 mhz?
    uint64_t tcxo_freq_x128 = tcxo_freq << PLL_CALC_SHIFT;

    // remember: floor division (integer)
    // tcxo_freq is integer..
    uint64_t pll_mult_here = pll_freq_x128 / tcxo_freq_x128;

    //*************************************************************
    // good info on out-of-spec behavior
    // https://rfzero.net/documentation/rf/
    // https://rfzero.net/tutorials/si5351a/
    // mult has to be in the range 15 to 90

    // I was getting pll_mult of 14 for 10M when I targeted 390Mhz pll
    // need 15 min
    if (pll_mult_here < 15 || pll_mult_here > 90) {
        V1_printf("ERROR: pll_mult %" PRIu64 " is out of range 15 to 90." EOL,
            pll_mult_here);
        V1_print(F("ERROR: Need to pick a high target pll freq" EOL));
        V1_printf("integer pll_freq_here %" PRIu64 " tcxo_freq %" PRIu64 EOL,
            pll_freq_here, tcxo_freq);
        // doesn't work to try to force it in terms of understanding calcs
        // if (pll_mult_here < 15) pll_mult_here = 15;
        // else pll_mult_here = 90;

        // Have to recompute the divisor if we change pll_mult? i.e. it's forced..
        // recompute the pll_freq based on the forced multiplier
        // R_DIVISOR_SHIFT: possible *4 for the R0 and R1 output divider.
        // 64 bit calcs so don't lose bits beyond 32-bits
        // mult will be min or max
        pll_freq_x128 = tcxo_freq_x128 * pll_mult_here;
        // HACK bad
        // this is just integer. not useful!
        pll_freq_here = pll_freq_x128 >> PLL_CALC_SHIFT;

        // this alternate strategy will imply required ms_div
        ms_div_here = 1 + (
            (((uint64_t)PLL_FREQ_TARGET) << PLL_CALC_SHIFT) /
            (((uint64_t)freq_x128) << R_DIVISOR_SHIFT));

        // make sure it is even number
        ms_div_here &= 0xfffffffe;
        V1_print(F("ERROR: pll_freq implied by new min or max pll_mult" EOL));
        V1_print(F("ms_div implied by that pll_freq" EOL));
        if (ms_div_here < 4 || ms_div_here > 900) {
            V1_printf("ERROR: ms_div %" PRIu64 " is out of range 4 to 900" EOL, ms_div_here);
            V1_print(F("ERROR: no recalc. Should never happen! rf output is wrong" EOL));
        }
        V1_printf("ERROR: Now integer pll_freq_here %" PRIu64 " integer tcxo_freq %" PRIu64 EOL,
            pll_freq_here, tcxo_freq);
        V1_printf("ERROR: Now pll_mult %" PRIu64 " ms_div %" PRIu64 EOL,
            pll_mult_here, ms_div_here);
    }

    //*************************************************************
    // check the range of "legal" pll frequencies
    // change to use my "legal" pll range
    if (false) {
        if (pll_freq_here < 600000000 || pll_freq_here > 900000000) {
            V1_printf("WARN: integer pll_freq %" PRIu64 " is outside datasheet legal range" EOL, 
                pll_freq_here);
        }
    }
    else {
        // (16 * 25) - (1 * 25)  = 375. 
        // So I think I shouldn't see less than 375? with 400 target?
        // That's if I use a 25Mhz tcxo rather than 26, 
        // which should have 390 min also?
        // check for 390 min
        if (pll_freq_here < 390000000 || pll_freq_here > 900000000) {
            V1_printf("WARN: integer pll_freq %" PRIu64 " is outside my 'legal' range" EOL, 
                pll_freq_here);
        }
    }
    // live with it. maybe upgrade to ERROR to not miss with grep?    }
    // pll_num max 20 bits (0 to 1048575)?

    // In the Si5351A, the "PLL num" refers to a 20-bit register value
    // used to set the numerator of the fractional PLL multiplier,
    // allowing for fine-tuning of the internal PLL frequency within a specified range;

    // essentially, it provides the fractional part of the multiplication factor
    // with 20 bits of precision.
    // also look at https://github.com/etherkit/Si5351Arduino
    // a new method. (greatest common divisor?)
    // https://github.com/etherkit/Si5351Arduino/issues/79
    // he says to choose INTEGER_FACTOR1 to be 10e*
    // y1 = Fout/INTEGER_FACTOR1
    // x1 = 900e6*/INTEGER_FACTOR1  <-- needs to be an integer.

    // If you choose INTEGER_FACTOR1 to be 10^something, this will be an integer too!
    // Multisynth output equation:
    // A+B/C
    // A = floor(x1, y1)
    // B = x1 % y1
    // C = y1
    // it's interesting these are done in the non-scaled domain (not *128)
    // since pll_freq_here is what we want to get to, shouldn't we be scaled here?

    uint64_t pll_remain_x128 = pll_freq_x128 - (pll_mult_here * tcxo_freq_x128);
    uint64_t pnh_x128 = (pll_remain_x128 * PLL_DENOM_x128) / tcxo_freq_x128;

    // here's how we add 0.5 (in the scaled domain) to get rounding effect before shift down
    // the r divisor reduces
    uint64_t pll_num_x128 = pnh_x128 + (1 << (PLL_CALC_SHIFT - 1));
    uint64_t pll_num_here = pll_num_x128 >> PLL_CALC_SHIFT;
    if (pll_num_here > 1048575) {
        V1_printf("ERROR: pll_num %" PRIu64 " is out of range 0 to 1048575" EOL,
            pll_num_here);
    }

    // https://rfzero.net/tutorials/si5351a/
    // When we're done, we can calc what the fout should be ?
    // Ah. this has more precision in it..can't just shift down to print it!
    // Doug (traquito) has WSPR_TONE_SPACING_HUNDREDTHS_HZ = 146 (1.4648 Hz)
    // hmm. looking at the sweep of "actual" seems like they are 2 hz steps?
    // need to make that a real number
    double actual_pll_freq_here = (double)tcxo_freq *
        ((double)pll_mult_here + ((double)pll_num_here / (double)PLL_DENOM));

    // note we return a double here...only for printing
    double actual_here = actual_pll_freq_here / 
        (double)(ms_div_here << R_DIVISOR_SHIFT);

    // output so we can print or use
    *ms_div    = (uint32_t)ms_div_here;
    *pll_mult  = (uint32_t)pll_mult_here;
    *pll_num   = (uint32_t)pll_num_here;
    *pll_denom = (uint32_t)PLL_DENOM;
    *r_divisor = (uint32_t)pow(2, R_DIVISOR_SHIFT);
    *actual = actual_here;
    *actual_pll_freq = actual_pll_freq_here;
}

//****************************************************
// freq is in 28.4 fixed point number, 0.0625Hz resolution
void vfo_set_freq_x128(uint8_t clk_num, uint32_t freq_x128, bool only_pll_num) {
    Watchdog.reset();
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num;
    uint32_t pll_denom;
    uint32_t r_divisor;
    double actual_pll_freq;
    double actual;
    if (clk_num != 0) {
        V1_println("ERROR: vfo_set_freq_16() should only be called with clk_num 0");
        // I guess force clk_num, although code is broken somewhere
        clk_num = 0;
        // note we only have one s_PLLB_ms_div_prev copy state also
    }

    // we get pll_denom to know what was used in the calc
    // R_DIVISOR_SHIFT is hardwired constant (/4 => shift 2)
    vfo_calc_div_mult_num(&actual, &actual_pll_freq,
        &ms_div, &pll_mult, &pll_num, &pll_denom, &r_divisor,
        freq_x128);

    // if (only_pll_num) {
    // calcs were done to show that only pll_num needs to change for symbols 0 to 3
    // we always do an early turn-on with symbol 0 that gets all other state right
    // and it stays that way for the whole message. Guarantee no pll reset needed!
    // turns out there's no speedup to make use of this bool
    // this has sticky s_regs_prev state that it uses if called multiple times?
    si5351a_setup_PLLB(pll_mult, pll_num, pll_denom);
    if (ms_div != s_PLLB_ms_div_prev) {
        // s_PLLB_ms_div_prev is global
        // only reset pll if ms_div changed?
        if (only_pll_num) {
            V1_printf("ERROR: only_pll_num but ms_div %lu changed. s_PLLB_ms_div_prev %lu" EOL,
                ms_div, s_PLLB_ms_div_prev);
            V1_print(F("ERROR: will cause si5351a_reset_PLLB()" EOL));
        }
        // setting up multisynth0/1/2
        si5351a_setup_multisynth012(ms_div);
        // FIX! should we rely on the clock turn on to reset pll?
        // normally we should only change ms_div while the clock outputs are off?
        // so there will be a turn on after this?

        // si5351a_reset_PLLB(true);

        // static global? for comparison next time
        s_PLLB_ms_div_prev = ms_div;
    }
    // make PLLA the same (so it locks? Is that better power than unlocked?)
    si5351a_setup_PLLA(pll_mult, pll_num, pll_denom);
    s_PLLA_ms_div_prev = ms_div;

    // V1_printf("vfo_set_freq_x128 END clk_num %u freq %lu" EOL, clk_num, freq);
}

//****************************************************
// FIX! hans says we need pll reset whenever we turn clocks off/on
// to retain 180 degree phase relationship (CLK0/CLK1)
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
            si5351bx_clken &= ~disable_bits;
            if (print) {
                V1_printf("vfo_turn_on_clk_out si5351bx_clken %02x" EOL, 
                    si5351bx_clken);
            }
            i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
        }
    }

    // FIX! this should always have a pll reset after it?
    if (print) {
        V1_println(F("vfo_turn_on_clk_out END"));
        V1_flush();
    }
}

//****************************************************
// FIX! hans says we need pll reset whenever we turn clocks off/on
// to retain 180 degree phase relationship (CLK0/CLK1)
// Suppose we don't do this without a pll reset for some reason
// when clocks are turned back on

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
        if ((si5351bx_clken | disable_bits) != si5351bx_clken) { // 0 is enabled
            si5351bx_clken |= disable_bits; // 1 is disable
            // if si5351a power is off we'll get ERROR: res -1 after i2cWrite 3
            if (print) {
                V1_printf("vfo_turn_off_clk_out si5351bx_clken %02x" EOL, 
                    si5351bx_clken);
            }
            i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
        }
    }
    
    if (print) {
        V1_printf("vfo_turn_off_clk_out END clk_num %u" EOL EOL, clk_num);
        V1_flush();
    }
}
//****************************************************
void vfo_set_drive_strength(uint8_t clk_num, uint8_t strength) {
    Watchdog.reset();
    V1_printf("vfo_set_drive_strength START clk_num %u" EOL, clk_num);

    // only called during the initial vfo_turn_on()
    s_vfo_drive_strength[clk_num] = 0x3 && strength;

    //**********************
    // reset the s_PLLB_ms_div_prev to force vfo_set_freq_x128()
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
    V1_printf("vfo_set_drive_strength END clk_num %u" EOL, clk_num);
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

    //*********************************
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
    V1_print(F("vfo_write_clock_en_with_retry START"));
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
        V1_println("i2c_deinit() start");
        i2c_deinit(VFO_I2C_INSTANCE);
        V1_println("i2c_deinit() complete");
        busy_wait_ms(1000);

        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
        V1_println("i2c_init() start");
        i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
        V1_println("i2c_init() complete");
        busy_wait_ms(1000);

        // gpio_pull_up(VFO_I2C0_SDA_PIN);
        // gpio_pull_up(VFO_I2C0_SCL_PIN);
        // don't use pullups or pulldowns since there are external pullups
        gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
        gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);
        gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
        i2c_set_slave_mode(VFO_I2C_INSTANCE, false, 0);
        busy_wait_ms(1000);

        // power the vfo off/on
        digitalWrite(Si5351Pwr, HIGH);
        busy_wait_ms(1000);
        digitalWrite(Si5351Pwr, LOW);
        busy_wait_ms(2000);
        V1_printf("vfo_turn_on re-iinit the I2C0 pins inside loop. tries %d" EOL, tries);

        // all clocks off?
        // FIX! what about timeout?
        Watchdog.reset();
        res = i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, val);
        V1_flush();
    }
    if (res==2) si5351bx_clken = val;
    V1_printf("vfo_write_clock_en_with_retry res %d END" EOL, res);
}
//****************************************************
void vfo_turn_on(uint8_t clk_num) {
    // FIX! what if clk_num is not zero?
    // turn on of 0 turns on 0 and 1 now    clk_num = 0;
    V1_printf("vfo_turn_on START clk_num %u" EOL, clk_num);
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
    V1_print(F("vfo_turn_on done trying to init the I2C0 pins in loop" EOL));

    // could disable all OEB pin enable control??
    // (OEB/SSEN pins don't exist on si5351a 3 output)
    i2cWrite(9, 0xFF);

    // 12/28/24 new writes below here are good
    // new: pll input source is XTAL for PLLA and PLLB
    i2cWrite(15, 0x00);

    // Power Down, Integer Mode. PLLB source to MultiSynth. Not inverted. Src Multisynth.
    // hmm. why Integer Mode. means we have to switch to fractional mode?

    // 12/28/24 change to fractional mode
    // 2mA drive
    // changed from PLLA source
    // starts at 16?
    // orig:   i2cWrite(reg, 0xAC);

    uint8_t reg;
    for (reg = SI5351A_CLK0_CONTROL; reg <= SI5351A_CLK7_CONTROL; reg++) {
        Watchdog.reset();
        i2cWrite(reg, 0xAC);
    }
    // was:   i2cWrite(reg, 0xCC);

    memset(s_vfo_drive_strength, 0, 0);
    // Moved this down here ..we don't know if we'll hang earlier
    // sets state to be used later
    if (_tx_high[0] == '0') {
        // this also clears the "prev" state, so we know we'll reload all state in the si5351
        // i.e. no optimization based on knowing what we had sent before!
        // 4 dBM reduction?
        vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK01_IDRV_4MA);
        vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK01_IDRV_4MA);
    } else {
        // FIX! make sure this is default in config
        // 15 dBm?
        vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK01_IDRV_8MA);
        vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK01_IDRV_8MA);
    }

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
    s_PLLB_ms_div_prev = 0;
    s_PLLA_ms_div_prev = 0;
    // new 11/24/24 ..maybe clear all this old state
    memset(s_PLLB_regs_prev, 0, 8);
    memset(s_PLLA_regs_prev, 0, 8);
    si5351bx_clken = 0xff; // all disabled

    uint32_t freq = XMIT_FREQUENCY;

    // this is aligned to integer. (symbol 0)
    V1_printf("initial freq for vfo_set_freq_x128() is %lu" EOL, freq);
    uint32_t freq_x128 = freq << PLL_CALC_SHIFT;

    Watchdog.reset();
    // FIX! should we get rid of pll reset here and rely on the turn_on_clk
    // to do it?
    vfo_set_freq_x128(clk_num, freq_x128, false);

    // The final state is clk0/clk1 running and clk outputs on?
    // this doesn't cause a reset_PLLB
    vfo_turn_on_clk_out(clk_num, true); // print

    // it was done in vfo_set_freq_x128 if we didn't have initial state
    // or didn't change initial state?
    // could there be two that overlap ?

    // does PLLA matter? these print the lock status by doing i2c reads
    // si5351a_reset_PLLA(true);
    si5351a_reset_PLLB(true);
    vfo_turn_on_completed = true;
    V1_printf("vfo_turn_on END clk_num %u" EOL, clk_num);
}
//****************************************************
// do this on the tcxo 26Mhz, everything else shouldn't need correction
uint32_t doCorrection(uint32_t freq) {
    uint32_t freq_corrected = freq;
    if (atoi(_correction) != 0) {
    // this will be a floor divide
    // https://user-web.icecube.wisc.edu/~dglo/c_class/constants.html
        freq_corrected = freq + (atoi(_correction) * freq / 1000000000UL);
    }
    V1_printf("doCorrection (should be tcxo freq?) freq %lu freq_corrected %lu" EOL,
        freq, freq_corrected);
    return freq_corrected;
}
//****************************************************
void vfo_turn_off(void) {
    V1_println(F("vfo_turn_off START"));
    // V1_println(F("NEVER TURNING VFO OFF (DEBU)"));
    // return;
    // already off successfully?
    if (vfo_is_off()) return;
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
    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_NULL);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_NULL);
    vfo_turn_off_completed = true;
    V1_println(F("vfo_turn_off END"));
}

//**********************************
// calculate the actual tx frequency for a symbol,
// given the base xmit freq for the u4b channel config'ed
// THIS MUST NOT BE USED FOR SI5351A work..just for printing!
double calcSymbolFreq(uint32_t hf_freq, uint8_t symbol, bool print) {
    if (symbol > 3) {
        V1_printf("ERROR: calcSymbolFreq symbol %u is not 0 to 3 ..using 0"
            EOL, symbol);
        symbol = 0;
    }
    // the frequency shift is 12000 / 8192 (approx 1.46hz)
    double symbol_freq_x128 =
        (hf_freq << PLL_CALC_SHIFT) +
        ((symbol * (12000L << PLL_CALC_SHIFT) + 4096L) / 8192L);
    double calcPrecisionDivisor = pow(2, PLL_CALC_SHIFT);
    double symbolFreq = (double) symbol_freq_x128 / calcPrecisionDivisor;
    double symbolOffset = symbolFreq - hf_freq;

    // 1.46..Hz per symbol
    double expectedSymbolOffset = ((double) symbol * 12000.0) / 8192.0;
    if (print) {
        V1_printf("For hf_freq %lu symbol %u symbolFreq is %.4f",
            hf_freq, symbol, symbolFreq);
        V1_printf(" symbolOffset %.4f expectedSymbolOffset %.4f" EOL,
            symbolOffset, expectedSymbolOffset);
    }
    return symbolFreq;
}

//**********************************
uint32_t calcSymbolFreq_x128(uint32_t hf_freq, uint8_t symbol) {
    // not expensive to always recalc the symbol freq
    // don't want printing though (too slow)
    uint32_t freq_x128_with_symbol = (
        hf_freq << PLL_CALC_SHIFT) +
        ((symbol * (12000L << PLL_CALC_SHIFT) + 4096L) / 8192L);
    return freq_x128_with_symbol;
}

//**********************************
void startSymbolFreq(uint32_t hf_freq, uint8_t symbol, bool only_pll_num) {
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
        V1_printf("symbol_0_freq %.4f" EOL, symbol_0_freq);
        V1_printf("symbol_1_freq %.4f" EOL, symbol_1_freq);
        V1_printf("symbol_2_freq %.4f" EOL, symbol_2_freq);
        V1_printf("symbol_3_freq %.4f" EOL, symbol_3_freq);
        V1_print(F(EOL));
        oneShotDebugPrinted = true;
    }
    // not expensive to always recalc the symbol freq
    uint32_t freq_x128_with_symbol = calcSymbolFreq_x128(hf_freq, symbol);
    // FIX! does this change the state of the clock output enable?
    // no..
    // changes both clk0 and clk1
    vfo_set_freq_x128(WSPR_TX_CLK_0_NUM, freq_x128_with_symbol, only_pll_num);

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

//**********************************
void si5351a_calc_optimize(double *sumShiftError, double *sumAbsoluteError,
    uint32_t *pll_num, bool print) {
    V1_print(F("si5351a_calc_optimize() START" EOL));
    // just to see what we get, calculate the si5351 stuff for
    // all the 0.25 Hz variations for possible tx in a band.
    // all assuming u4b channel 0 freq bin.
    // stuff that's returned by vfo_calc_div_mult_num()
    double actual;
    double actual_pll_freq;
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num_here;
    uint32_t pll_denom;
    uint32_t r_divisor;

    // stuff that's input to vfo_calc_div_mult_num()
    uint32_t freq_x128;
    // should already be set for band, channel? (XMIT_FREQUENCY)
    uint32_t xmit_freq = XMIT_FREQUENCY;
    if (print) {
        V0_printf("band %s channel %s xmit_freq %lu" EOL, _Band, _U4B_chan, xmit_freq);
    }

    // compute the actual shifts too, which are the more important thing
    // as opposed to actual freq (since tcxo causes fixed error too 
    // Assume no drift thru the tx.
    // this is floats, because it's for printing only (accuracy)
    double symbol0desired = calcSymbolFreq(xmit_freq, 0, false);  // no print
    double symbol1desired = calcSymbolFreq(xmit_freq, 1, false);  // no print
    double symbol2desired = calcSymbolFreq(xmit_freq, 2, false);  // no print
    double symbol3desired = calcSymbolFreq(xmit_freq, 3, false);  // no print
    // will give the freq you should see on wsjt-tx if hf_freq is the xmit_freq 
    // for a channel. symbol can be 0 to 3. 
    // Can subtract 20 hz to get the low end of the bin
    // (assume freq calibration errors of that much, then symbol the 200hz passband?

    // in calcSymbolFreq(), could compare these offsets from the symbol0desired to expected?
    // (offset 1.46412884334 Hz)
    if (print) {
        V1_print(F(EOL));
        // + 0 Hz
        // +1*(12000/8196) Hz [1.464 Hz]
        // +2*(12000/8196) Hz [2.928 Hz]
        // +3*(12000/8196) Hz [4.392 Hz]
        V1_printf("band %s channel %s desired symbol 0 freq %.4f" EOL,
            _Band, _U4B_chan, symbol0desired);
        V1_printf("band %s channel %s desired symbol 1 freq %.4f" EOL,
            _Band, _U4B_chan, symbol1desired);
        V1_printf("band %s channel %s desired symbol 2 freq %.4f" EOL,
            _Band, _U4B_chan, symbol2desired);
        V1_printf("band %s channel %s desired symbol 3 freq %.4f" EOL,
            _Band, _U4B_chan, symbol3desired);
        V1_print(F(EOL));
    }

    // check what pll_num gets calced in the freq_x128 (shifted) domain
    // and also, the fp respresentation (actual) of the actual frequency after /128
    // of the *128 'shifted domain' integer representation
    // actual returned is now a double
    double symbol0actual;
    double symbol1actual;
    double symbol2actual;
    double symbol3actual;
    for (uint8_t symbol = 0; symbol <= 3; symbol++) {
        freq_x128 = calcSymbolFreq_x128(xmit_freq, symbol);
        // This will use the current PLL_DENOM_OPTIMIZE now in its calcs?
        vfo_calc_div_mult_num(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num_here, &pll_denom, &r_divisor, freq_x128);
        if (print) {
            V1_printf("channel %s symbol %u pll_num %lu pll_denom %lu actual %.4f" EOL,
                _U4B_chan, symbol,  pll_num_here, pll_denom, actual);
        }
        switch (symbol) {
            case 0: symbol0actual = actual; break;
            case 1: symbol1actual = actual; break;
            case 2: symbol2actual = actual; break;
            case 3: symbol3actual = actual; break;
        }
    }

    if (print) {
        V1_print(F(EOL));
        V1_print(F("Showing shifts in symbol frequencies, rather than absolute error" EOL));
        V1_printf("channel %s symbol 0 actual %.4f" EOL,
            _U4B_chan, symbol0actual);
        V1_printf("channel %s symbol 1 actual %.4f shift0to1 %.4f" EOL,
            _U4B_chan, symbol1actual, symbol1actual - symbol0actual);
        V1_printf("channel %s symbol 2 actual %.4f shift0to2 %.4f" EOL,
            _U4B_chan, symbol2actual, symbol2actual - symbol0actual);
        V1_printf("channel %s symbol 3 actual %.4f shift0to3 %.4f" EOL,
            _U4B_chan, symbol3actual, symbol3actual - symbol0actual);
    }

    // just one absolute error
    double sumAbsoluteError_here =
        // abs(symbol0actual - symbol0desired) +
        abs(symbol1actual - symbol1desired);
        // abs(symbol2actual - symbol2desired) +
        // abs(symbol3actual - symbol3desired);

    // just 0->1 0->2 0->3 incremental shift errors. compared to expected 12000/8192
    double expectedShift = 12000.0 / 8192.0;
    // assume it's a positive shift

    // just one shift
    double sumShiftError_here =
        abs((symbol1actual - symbol0actual) - expectedShift);
        // ((symbol2actual - symbol1actual) - expectedShift) +
        // ((symbol3actual - symbol2actual) - expectedShift);

    *sumShiftError = sumShiftError_here;
    *sumAbsoluteError = sumAbsoluteError_here;
    *pll_num = pll_num_here;

    if (print) {
        V1_print(F(EOL));
        V1_printf("sumAbsoluteError (just symbol 1): %.4f" EOL, sumAbsoluteError_here);
        V1_printf("sumShiftError (just symbol 1): %.4f" EOL, sumShiftError_here);
        V1_print(F(EOL));
        V1_print(F("si5351a_calc_optimize() END" EOL));
    }
}

//**********************************
void si5351a_calc_sweep(void) {
    V1_print(F("si5351a_calc_sweep() START" EOL));

    // global XMIT_FREQUENCY should already be set for band, channel?
    uint32_t xmit_freq = XMIT_FREQUENCY;
    V0_printf("band %s channel %s xmit_freq %lu" EOL, _Band, _U4B_chan, xmit_freq);
    double symbol0desired = calcSymbolFreq(xmit_freq, 0, true);  // print

    V1_printf("Now: sweep calc 5351a programming starting at %.4f" EOL, symbol0desired);
    V1_print(F(EOL "partial sweep at 0.25hz increment" EOL));
    uint32_t pll_num_last = 0;
    // integer start freq.
    // should be no fractional part in the double? (since it's the base symbol 0 freq?)
    // start at middle of the bin, u4b channel 0, symbol 0
    uint32_t freq = (uint32_t) symbol0desired;
    uint32_t freq_x128;

    double actual;
    double actual_pll_freq;
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num;
    uint32_t pll_denom;
    uint32_t r_divisor;
    // sweep 200 * 0.25 hz = 50hz (1/4th the passband)
    // sweep 80 * 1 hz = 80hz
    for (int i = 0; i < 80; i++) {
        if (false) {
            // use this for 0.25 steps
            freq_x128 = (freq + i/4) << PLL_CALC_SHIFT;
            switch (i % 4) {
                case 0: break;
                // adds 0.25 (shifted)
                case 1: freq_x128 += ((1 << PLL_CALC_SHIFT) >> 2); break;
                // adds 0.50 (shifted)
                case 2: freq_x128 += ((2 << PLL_CALC_SHIFT) >> 2); break;
                // adds 0.75 (shifted)
                case 3: freq_x128 += ((3 << PLL_CALC_SHIFT) >> 2); break;
            }
        } else {
            // use for 1.0 steps
            freq_x128 = (freq + i) << PLL_CALC_SHIFT;
        }

        // note this will include any correction to SI5351_TCXO_FREQ (already done)
        // everything is 32 bit in and out of this, but 64-bit calcs inside it.
        vfo_calc_div_mult_num(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num, &pll_denom, &r_divisor,
            freq_x128);

        // pow() returns double
        // not used (float version of the desired freq which was given in the *128 domain
        // double freq_float = (double)freq_x128 / pow(2, PLL_CALC_SHIFT);

        // hmm. don't bother printing the target freq_float values? enough to show the actual
        // changes we get during the sweep of target values?
        V1_printf(
            "actual_pll_freq %.4f "
            "ms_div %lu pll_mult %lu pll_num %lu pll_denom %lu r_divisor %lu actual %.4f" EOL,
            actual_pll_freq, ms_div, pll_mult, pll_num, pll_denom, r_divisor, actual);

        // no good if two pll_nums are the same (sequentially)
        // want unique pll_num changes for each 1 Hz change..
        if (pll_num == pll_num_last) {
            // V1_print(F("UNDESIREABLE: pll_num and pll_num_last same"));
        }
        pll_num_last = pll_num;
    }
    V1_print(F(EOL));
    V1_print(F("si5351a_calc_sweep() END" EOL));
}
//****************************************************************************
// does this work? no. Hans method on 144Mhz is to fix the numerator and 
// step the denominator causes non-uniform symbol shift
// alternate way to get steps of 1 on 144Mhz
// with 26Mhz
// c = 26e6 / 1.4648 in a + b/c equation
// 17749863. too large
// or  to require 3 steps:
// c = (3 * 26e6) / 1.4648 in a + b/c equation

// kbn: with the added choice of 1, 2 or 3 numerator steps 
// to get the desired single wspr transition
// per: https://groups.io/g/QRPLabs/topic/si5351a_issues_with_frequency/96467329
//
// divide the XTAL frequency by my output divider and then divide again 
// by my desired step,
// wpsr: 1.4648 and use that number for my value for c
// in the a + b/c equation.
// What this does is make each increment of b  in the equation result in
// the output frequency changing by the desired step,
// and then I have manipulated b directly to send the WSPR signals.

//****************************************************************************
// In your case if you divide 25mhz by 6 and then by 6.66666667
// you get the somewhat magic value (rounded) of 625000.

// 0.6080016 * 625000 is 380001 exactly.

// Downside of this method is that setting your base frequency will have error,
// but the steps up from that frequency will be accurate.
// (must be careful using this approach  as c needs to be below 1048575,
// but works for 6.6667 steps on 144mhz ).
//
// Looked at the Adafruit library. Interesting way to approach things.
// It does use float calculations for the divisions so
// I think it will have some error.
//
// My WSPR code using this method is here: https://github.com/roncarr880/QRP_LABS_WSPR
// The Si5351 routines would need changes to work on 144mhz
// with the fixed divider of 6.
// Project also used an R divider for the transmit. Would need to be removed.

//*********************************************************************************
// sweep the bands for the current PLL_FREQ_TARGET to get the mult/div 
// for the spreadsheet to compute optimum denom for the PLL_FREQ_TARGET
void si5351a_calc_sweep_band() {
    V1_print(F("si5351a_calc_sweep_band() START" EOL));
    double actual;
    double actual_pll_freq;
    uint32_t ms_div;
    uint32_t pll_mult;
    uint32_t pll_num_here;
    uint32_t pll_denom;
    uint32_t r_divisor;
    uint32_t freq_x128;

    char band[3];

    for (uint8_t i = 0; i <= 4 ; i++) {
        switch (i) {
            case 0: snprintf(band, sizeof(band), "10"); break;
            case 1: snprintf(band, sizeof(band), "12"); break;
            case 2: snprintf(band, sizeof(band), "15"); break;
            case 3: snprintf(band, sizeof(band), "17"); break;
            case 4: snprintf(band, sizeof(band), "20"); break;
        }
        // will pick the _lane we're using for the current u4b channel config
        uint8_t symbol = 0;
        char lane[2] = { 0 };  // '1', '2', '3', '4'
        lane[0] = '1';  // first freq bin

        set_PLL_DENOM_OPTIMIZE(band);
        uint32_t xmit_freq = init_rf_freq(band, lane);
        freq_x128 = calcSymbolFreq_x128(xmit_freq, symbol);
        // This will use the current PLL_DENOM_OPTIMIZE now in its calcs?
        vfo_calc_div_mult_num(&actual, &actual_pll_freq,
            &ms_div, &pll_mult, &pll_num_here, &pll_denom, &r_divisor, freq_x128);

        V1_print(F(EOL));
        V1_printf("sweep band %s xmit_freq %lu PLL_FREQ_TARGET %lu r_divisor %lu" EOL,
            band, xmit_freq, PLL_FREQ_TARGET, r_divisor);
        V1_printf("sweep band %s lane %s symbol %u", band, lane, symbol);
        V1_printf(" pll_mult %lu ms_div %lu actual_pll_freq %.4f" EOL,
            pll_mult, ms_div, actual_pll_freq);
        V1_printf("sweep band %s lane %s symbol %u", band, lane, symbol);
        V1_printf(" pll_num %lu pll_denom %lu actual %.4f" EOL,
            pll_num_here, pll_denom, actual);
        V1_print(F(EOL));
    }

    V1_print(F("si5351a_calc_sweep_band() END" EOL));
}

//*********************************************************************************
void set_PLL_DENOM_OPTIMIZE(char *band) {
    V1_println(F("set_PLL_DENOM_OPTIMIZE START"));
    // FIX! hack! we should have fixed values per band? do they vary by freq bin?
    uint32_t PLL_DENOM_MAX = 1048575;
    V1_print(F("WARN: leave PLL_DENOM_OPTIMIZE with optimal values so far" EOL));
    // this is the target PLL freq when making muliplier/divider initial 
    // calculations set in tracker.ino
    // from 900 history
    // goal seek result
    // case 17: PLL_DENOM_OPTIMIZE = 1048575; break; // can't do better?
    // goal seek result
    // case 20: PLL_DENOM_OPTIMIZE = 277333; break;
    // wspr_calc_direct_shift.xlsx result
    int b = atoi(band);
    switch (PLL_FREQ_TARGET) {
        case 90000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 554667; break;
                case 12: PLL_DENOM_OPTIMIZE = 986074; break;
                case 15: PLL_DENOM_OPTIMIZE = 845206; break;
                case 17: PLL_DENOM_OPTIMIZE = 709973; break;
                case 20: PLL_DENOM_OPTIMIZE = 832000; break;
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX;
            }
            break;
        case 700000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 739556; break;
                case 12: PLL_DENOM_OPTIMIZE = 633905; break;
                case 15: PLL_DENOM_OPTIMIZE = 1044078; break;
                case 17: PLL_DENOM_OPTIMIZE = 934175; break;
                case 20: PLL_DENOM_OPTIMIZE = 709973; break;
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX;
            }
            break;
        case 600000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 806788; break;
                case 12: PLL_DENOM_OPTIMIZE = 739556; break;
                case 15: PLL_DENOM_OPTIMIZE = 633905; break;
                case 17: PLL_DENOM_OPTIMIZE = 522039; break;
                case 20: PLL_DENOM_OPTIMIZE = 845206; break;
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX;
            }
            break;
        case 500000000:
            switch (b) {
                case 10: PLL_DENOM_OPTIMIZE = 986074; break;
                case 12: PLL_DENOM_OPTIMIZE = 887467; break;
                case 15: PLL_DENOM_OPTIMIZE = 739556; break;
                case 17: PLL_DENOM_OPTIMIZE = 633905; break;
                case 20: PLL_DENOM_OPTIMIZE = 986074; break;
                default: PLL_DENOM_OPTIMIZE = PLL_DENOM_MAX;
            }
            break;
        default:
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
