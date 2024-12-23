// Project: https://github.com/knormoyle/rp2040_si5351_wspr // Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

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

// FIX! we're using default 10pf load capacitance?
// Register 183. Crystal Internal Load Capacitance
// Bit
// Reset value = 11xx xxxx
// Type R/WR/W
// 7:6 XTAL_CL[1:0]
//
// Crystal Load Capacitance Selection.
// These 2 bits determine the internal load capacitance value for the crystal.
// See the Crystal Inputs section in the Si5351 data sheet.
// 00: Reserved. Do not select this option.
// 01: Internal CL = 6 pF.
// 10: Internal CL = 8 pF.
// 11: Internal CL = 10 pF (default).

// 5:0 Reserved
// Bits 5:0 should be written to 010010b.

//****************************************************
// Si5351A related functions
// Developed by Kazuhisa "Kazu" Terasaki AG6NS
// https://github.com/kaduhi/AFSK_to_FSK_VFO ..last update afsk_to_fsk_vfo.ino 6/30/21
// https://github.com/kaduhi/AFSK_to_FSK_VFO/tree/main
//
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
// for i2c0
#include "hardware/i2c.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

#include "si5351_functions.h"
#include "print_functions.h"
#include "led_functions.h"

// These are in arduino-pio core
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries
#include <SPI.h>
#include <Wire.h>

// from tracker.ino
extern uint32_t SI5351_TCXO_FREQ;  // 26 mhz with correction already done
extern const int SI5351A_I2C_ADDR;
extern const int VFO_I2C0_SCL_HZ;

extern uint32_t XMIT_FREQUENCY;
// decode of _verbose 0-9
extern bool VERBY[10];

extern char _tx_high[2];  // 0 is 2mA si5351. 1 is 8mA si5351
extern char _correction[7];  // parts per billion -3000 to 3000. default 0

extern const int Si5351Pwr;
// FIX! are these just used on the Wire.begin?
// FIX! should this be a extern const. Or: only used here?
#define VFO_I2C_INSTANCE i2c0
extern const int VFO_I2C0_SDA_PIN;
extern const int VFO_I2C0_SCL_PIN;

// when we set both?
extern const int WSPR_TX_CLK_NUM;
extern const int WSPR_TX_CLK_0_NUM;
extern const int WSPR_TX_CLK_1_NUM;

extern const int SI5351A_CLK_IDRV_8MA;
extern const int SI5351A_CLK_IDRV_6MA;
extern const int SI5351A_CLK_IDRV_4MA;
extern const int SI5351A_CLK_IDRV_2MA;

extern const int PLL_CALC_PRECISION;

static bool vfo_turn_on_completed = false;
static bool vfo_turn_off_completed = false;

static uint8_t si5351bx_clken = 0xff;
static bool s_is_on = false;

// updated with config _tx_high during vfo_turn_on()
// 0:2mA, 1:4mA, 2:6mA, 3:8mA
static uint8_t s_vfo_drive_strength[3] = { 0 };

// FIX! are these just initial values?
// what about the higher frequencies
// set MS0-2 for div_4 mode (min. division)
static const uint8_t s_ms_values[] = { 0, 1, 0x0C, 0, 0, 0, 0, 0 };

// set PLLA-B for div_16 mode (minimum even integer division)
static const uint8_t s_pll_values[] = { 0, 0, 0, 0x05, 0x00, 0, 0, 0 };

static uint8_t s_PLLB_regs_prev[8] = { 0 };
static uint32_t s_ms_div_prev = 0;

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

    // clear any old remembered state we have outside of the si5351
    // hmm. maybe not this though
    // si5351bx_clken = 0xff;
    s_ms_div_prev = 0;
    memset(s_PLLB_regs_prev, 0, 8);
    memset(s_vfo_drive_strength, 0, 3);

    // turn ON VFO VDD
    // pin 4 ?
    // do the init first
    gpio_init(Si5351Pwr);
    pinMode(Si5351Pwr, OUTPUT);

    // FIX! remove pull_up to save power. Don't need it?
    gpio_pull_up(Si5351Pwr);
    gpio_put(Si5351Pwr, 0);

    // init I2C0 for VFO
    i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
    gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
    gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);
    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
    V1_println(F("vfo_init END"));
}

//****************************************************
void vfo_set_power_on(bool turn_on) {
    // this works even if s_is_on is wrong relative to current on state
    // doesn't turn off/on first
    V1_printf("vfo_set_power_on START %u" EOL, turn_on);
    V0_flush();

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
    // si5351bx_clken = 0xff;
    s_ms_div_prev = 0;
    memset(s_PLLB_regs_prev, 0, 8);
    memset(s_vfo_drive_strength, 0, 3);

    V0_flush();
    V1_println(F("vfo_set_power_on After V0_flush()"));

    // always just turn it on!
    s_is_on = turn_on;

    // don't change the direction when it's on vs off. LightAPRS code did
    // we can just read the level
    // gpio_set_dir(Si5351Pwr, (turn_on ? GPIO_OUT : GPIO_IN));

    V1_printf("vfo_set_power_on END %u" EOL, s_is_on);
    V0_flush();
}

//****************************************************
bool reserved_reg(uint8_t reg) {
    // hung on read of d7 (215)
    // hung on read of d8 (216)
    bool bad = false;
    switch (reg) {
        case 4: {}
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
        case 0: {}  // has some status bits
        case 1: ; bad = true; break;  // has some status bits

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
//
// void writeRegister(uint8_t myRegister, uint8_t myValue) {
//     uint8_t error;
//     Wire.beginTransmission(_address);
//     Wire.write(myRegister);
//     Wire.write(myValue);
//     error = Wire.endTransmission();
//     if (error == 0) V1_print("I2C device found at address 0x");
//     if (error == 4) V1_print("Unknown error at address 0x");
// }
//
// byte readRegister(uint8_t myRegister) {
//     uint8_t error;
//     byte returnValue;
//
//     Wire.beginTransmission(_address);
//     Wire.write(myRegister);
//     error = Wire.endTransmission();
//     if (error == 0) V1_print("I2C device found at address 0x");
//     if (error == 4) V1_print("Unknown error at address 0x");
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

// just reads two byte
// FIX! don't need to read a stream of bytes? i2cWrReadn()?
int i2cWrRead(uint8_t reg, uint8_t *val) {  // read reg via i2c
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
        V1_print(F("i2cWrRead doing i2c_write_blockin then i2c_read_blocking" EOL));
        int res1, res2;
        // FIX! should these be _timeout_us instead?
        res1 = i2c_write_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, &reg, 1, true);
        res2 = i2c_read_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 1, false);
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
        V1_printf("i2cWrRead reserved reg %u", reg);
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

    // start and end are looked at below, if s_ms_div_prev = 0
    // so these are out of the for loops
    uint8_t start = 0;
    uint8_t end = 7;

    // is this just looking for a range of bits that changed?
    // the i2cWriten writes as burst below..maybe that's why?
    // we could keep a static copy of what was written , and only write
    // when values change.
    if (s_ms_div_prev != 0) {  // global. basically it implies s_PLLB_regs_prev has data
        for (; start < 8; start++) {
            if (PLLB_regs[start] != s_PLLB_regs_prev[start]) break;
        }
        // FIX! is this detecting a non-change?
        if (start == 8) return;

        for (; end > start; end--) {
            // is this so we just write the start to end that has changed?
            if (PLLB_regs[end] != s_PLLB_regs_prev[end]) break;
        }
    }

    uint8_t reg = SI5351A_PLLB_BASE + start;
    uint8_t len = end - start + 1;

    // FIX! does this really need to be &PLLB_regs[start]
    // to get the pointer, not the value? I guess?
    i2cWriten(reg, &PLLB_regs[start], len);

    // this can't be swap..so how could it have worked?
    // was it a pointer copy?
    // maybe PLLB_regs memory always got reallocated on the next call?
    // wouldn't if it was static?
    // old:
    // *((uint64_t *)s_PLLB_regs_prev) = *((uint64_t *)PLLB_regs);
    memcpy(s_PLLB_regs_prev, PLLB_regs, 8);

    // V1_printf("si5351a_setup_PLLB END mult %u num %lu denom %lu" EOL, mult, num, denom);
}

//****************************************************
// swapping pointers instead of memcpy:
// both should be static or global, so no mem allocaiton issue
// https://stackoverflow.com/questions/8403447/swapping-pointers-in-c-char-int
// best/obvious to just memcpy for just 8 bytes

//****************************************************
// divide-by-4. to add >1Hz accuracy for symbol freqs on 10M (easier on 20M)
// was getting +- 0.5Hz. This will give +- 0.125 hz ?
// maybe increase it to 3 for divide-by-8
// always have to verify that only pll_num changes in the range of symbol freqs
// (or safer: full 200Hz passband?)
const uint8_t R_DIVISOR_SHIFT = 2;
// div must be even number
void si5351a_setup_multisynth01(uint32_t div) {
    V1_printf("si5351a_setup_multisynth01 START div %lu" EOL, div);
    if ((div % 2) != 0) {
        V1_printf("ERROR: si5351a_setup_multisynth01 div %lu isn't even" EOL, div);
    }

    uint8_t s_regs[8] = { 0 };
    uint32_t p1 = 128 * div - 512;
    s_regs[0] = 0;
    s_regs[1] = 1;
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
        default: R_OUTPUT_DIVIDER_ENCODE = 0b000;  // divide-by-1
    }
    // bits [2:0] are R0 Output Divider
    // 000b: Divide by 1
    // 001b: Divide by 2
    // 010b: Divide by 4
    // 011b: Divide by 8
    // 100b: Divide by 16
    // ..
    // s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    // 11/22/24 force divide by 4 and propagate to all other calc
    // R_OUTPUT_DIVIDER_ENCODE is bits 6:4
    s_regs[2] = (R_OUTPUT_DIVIDER_ENCODE << 4) | ((uint8_t)(p1 >> 16) & 0x03);
    s_regs[3] = (uint8_t)(p1 >> 8);
    s_regs[4] = (uint8_t)p1;
    s_regs[5] = 0;
    s_regs[6] = 0;
    s_regs[7] = 0;

    i2cWriten(SI5351A_MULTISYNTH0_BASE, s_regs, 8);
    i2cWrite(SI5351A_CLK0_CONTROL,
        SI5351A_CLK0_MS0_INT |
        SI5351A_CLK0_MS0_SRC_PLLB |
        SI5351A_CLK0_SRC_MULTISYNTH_0 |
        s_vfo_drive_strength[0]);

    // this is used for antiphase (differential tx: clk0/clk1)
    i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);
    i2cWrite(SI5351A_CLK1_CONTROL,
        SI5351A_CLK1_MS1_INT |
        SI5351A_CLK1_MS1_SRC_PLLB |
        SI5351A_CLK1_SRC_MULTISYNTH_1 |
        SI5351A_CLK1_CLK1_INV |
        s_vfo_drive_strength[1]);

    // old #endif
    V1_printf("VFO_DRIVE_STRENGTH CLK0: %d" EOL, (int)s_vfo_drive_strength[0]);
    V1_printf("VFO_DRIVE_STRENGTH CLK1: %d" EOL, (int)s_vfo_drive_strength[1]);
    V1_printf("si5351a_setup_multisynth01 END div %lu" EOL, div);
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
    V1_println(F("si5351a_reset_PLLB START"));
    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
    // wait for the pll reset?
    sleep_ms(2000);
    V1_println(F("si5351a_reset_PLLB END"));
}

//****************************************************
// good for doing calc only, so see what changes with freq changes
void vfo_calc_div_mult_num(double *actual, double *actual_pll_freq,
    uint32_t *ms_div, uint32_t *pll_mult, uint32_t *pll_num, uint32_t *pll_denom,
    uint32_t *r_divisor, uint32_t freq_x128) {

    // interesting they had relatively low pll freq (620Mhz) for 15M case
    // https://rfzero.net/documentation/tools/si5351a-frequency-tool/
    // this is hans code from 2015:
    // https://qrp-labs.com/images/synth/demo6/si5351a.c

    // hans uses the max frequency?
    const int PLL_MAX_FREQ  = 900000000;
    const int PLL_MIN_FREQ  = 600000000;

    // divide by 2 result must be integer
    // uint32_t PLL_FREQ_TARGET;
    uint64_t PLL_FREQ_TARGET;
    if (false) {
        PLL_FREQ_TARGET  = ((PLL_MAX_FREQ + PLL_MIN_FREQ) / R_DIVISOR_SHIFT);
    } else {
        // two pll_nums per 0.5 hz on 14M
        // PLL_FREQ_TARGET  = 700000000;
        // this works with PLL_DENOM = 1000000 ..no matches on pll_Num
        // good symbol freq accuracy on 20M, but not great on 10M ..good enough I guess
        PLL_FREQ_TARGET  = 800000000;

        // maybe this will get better accuracy on 10M. Using the top end legal PLL freq.
        // getting over 900Mhz with this..better to back off?
        // PLL_FREQ_TARGET  = 900000000;
    }

    // http://www.wa5bdu.com/programming-the-si5351a-synthesizer/
    // http://www.wa5bdu.com/si5351a-quadrature-vfo/

    // Following the VCO is another divider stage that divides the VCO frequency
    // by a value of ‘d + e/f’
    // and can be used to take the frequency down into the low MHz range.
    // However the chip will provide an output with lower jitter if this value is an integer and
    // better still if it is an even integer. So we let e/f be zero and select a value for d
    // that’s an even number.

    uint64_t PLL_DENOM;
    if (true) {
        // hans uses 1048575 as max (which is 0xfffff) (2^20 - 1)
        // I guess no matter what, we will have fractional stuff with the 6 hz symbol adjustments
        // be interesting to see how close to desired freq, we get on the sdr?
        const uint64_t PLL_DENOM_MAX = 0x000fffff;
        PLL_DENOM = PLL_DENOM_MAX;
    } else {
        // this was sort of okay on 12/21/24. could it be better with PLL_DENOM_MAX above though?
        PLL_DENOM = 1000000;
    }

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

    // NEW: we hardwire in a divide-by-4 in the R0 and R1 output dividers
    // so this ms_div is 1/4th what it would be for a divide-by-1 R0 and R1
    // the << 2 in the divisor
    uint64_t ms_div_here =
        ( (PLL_FREQ_TARGET << PLL_CALC_PRECISION) / ((uint64_t)freq_x128 << R_DIVISOR_SHIFT) ) + 1;
    ms_div_here &= 0xfffffffe;   // make it even number

    // is 900 really the upper limiter or ?? we have 908 for 24Mhz
    // do we need lower pll freq?
    if (ms_div_here < 4 || ms_div_here > 900)
        V1_printf("ERROR: ms_div %" PRIu64 " is out of range 4 to 900" EOL, ms_div_here);

    // FIX! we should recalc this to deduce the final real pll_freq?
    // uint32_t pll_freq_here = ((uint64_t)freq_x128 * (uint64_t)ms_div_here) >> PLL_CALC_PRECISION;
    // NEW: *4 for the R0 and R1 output divider. don't lose bits beyond 32-bits
    uint64_t pll_freq_here = ((uint64_t)freq_x128 * ms_div_here) >> PLL_CALC_PRECISION;
    pll_freq_here = pll_freq_here << 2;
    // FIX! should we just apply correction to the crystal frequency? yes.
    // SI5351_TXCO_FREQ is calculated in tracker.ino set so correction calc
    // is just one once

    // I suppose there are integer roundoff issues in these operations?
    // uint32_t tcxo_freq = SI5351_TCXO_FREQ;  // 26 mhz?
    uint64_t tcxo_freq = (uint64_t) SI5351_TCXO_FREQ;  // 26 mhz?
    // remember: floor division (integer)
    uint64_t pll_mult_here = pll_freq_here / tcxo_freq;
    // mult has to be in the range 15 to 90
    if (pll_mult_here < 15 || pll_mult_here > 90) {
        V1_printf("ERROR: pll_mult %" PRIu64 " is out of range 15 to 90" EOL, pll_mult_here);
        V1_printf("pll_freq_here %" PRIu64 " tcxo_freq %" PRIu64 EOL, pll_mult_here, tcxo_freq);
    }

    // pll_num max 20 bits (0 to 1048575)?
    // In the Si5351A, the "PLL num" refers to a 20-bit register value
    // used to set the numerator of the fractional PLL multiplier,
    // allowing for fine-tuning of the internal PLL frequency within a specified range;
    // essentially, it provides the fractional part of the multiplication factor
    // with 20 bits of precision.
    // also look at https://github.com/etherkit/Si5351Arduino

    // new method.
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

    // uint32_t pll_remain = pll_freq_here - (pll_mult_here * tcxo_freq);
    uint64_t pll_remain = pll_freq_here - (pll_mult_here * tcxo_freq);

    // can see the benefit of PLL_DENOM / tcxo_freq being integer here?
    // the operation is done with 64 bits? i guess it matters given the size of the numbers
    // I guess there's a truncation rather than rounding when getting pll_num_here
    // uint32_t pll_num_here = (uint64_t)pll_remain * (uint64_t)PLL_DENOM / (uint64_t)tcxo_freq;

    // shift left 1 and then add 1 (0.5 scaled), to round, then shift right 1
    // otherwise it's just a floor divide here
    uint64_t pnh = ((pll_remain * PLL_DENOM) << 2) / tcxo_freq;
    uint64_t pll_num_here = (pnh + 1)  >> 2;
    if (pll_num_here > 1048575)
        V1_printf("ERROR: pll_num %" PRIu64 " is out of range 0 to 1048575" EOL, pll_num_here);

    // from https://rfzero.net/tutorials/si5351a/
    // When we're done, we can calc what the fout should be ?
    // uint32_t pll_freq_here = ((uint64_t)freq * (uint64_t)ms_div_here) >> PLL_CALC_PRECISION;
    // have time to print the 4 symbol freqs that will be used

    // Ah. this has more precision in it..can't just shift down to print it!
    // Doug has WSPR_TONE_SPACING_HUNDREDTHS_HZ = 146 (1.4648 Hz)
    // hmm. looking at the sweep of "actual" seems like they are 2 hz steps?
    // need to make that a real number
    double actual_pll_freq_here = (double)tcxo_freq * ((double)pll_mult_here + ((double)pll_num_here / (double)PLL_DENOM));

    // note we return a double here...only for printing
    double actual_here = actual_pll_freq_here / (double)(ms_div_here << R_DIVISOR_SHIFT);

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
        // note we only have one s_ms_div_prev copy state also
    }
    // we get pll_denom to know what was used in the calc
    // R_DIVISOR_SHIFT is hardwired constant  (/4 => shift 2)
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

    if (ms_div != s_ms_div_prev) {  // s_ms_div_prev is global
        // only reset pll if ms_div changed?
        if (only_pll_num) {
            V1_printf("ERROR: only_pll_num but ms_div %lu changed. s_ms_div_prev %lu" EOL,
                ms_div, s_ms_div_prev);
            V1_print(F("ERROR: will cause si5351a_reset_PLLB()" EOL));
        }
        // setting up multisynth0 and multisynth1
        si5351a_setup_multisynth01(ms_div);
        si5351a_reset_PLLB();
        // static global? for comparison next time
        s_ms_div_prev = ms_div;
    }
    // V1_printf("vfo_set_freq_x128 END clk_num %u freq %lu" EOL, clk_num, freq);
}

//****************************************************
// FIX! hans says we need pll reset whenever we turn clocks off/on
// to retain 180 degree phase relationship (CLK0/CLK1)
void vfo_turn_on_clk_out(uint8_t clk_num) {
    V1_printf("vfo_turn_on_clk_out START clk_num %u" EOL, clk_num);
    if (clk_num != 0)
        V1_printf("ERROR: vfo_turn_on_clk_out should only have clk_num 0 not %u"
            EOL, clk_num);

    // enable clock 0 and 1
    //  0 is enable
    uint8_t enable_bit = 1 << clk_num;
    // clk0 implies clk1 also
    if (clk_num == 0) enable_bit |= 1 << 1;

    si5351bx_clken &= ~enable_bit;
    i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
    // FIX! this should always have a pll reset after it?
    V1_println(F("vfo_turn_on_clk_out END"));
}

//****************************************************
// FIX! hans says we need pll reset whenever we turn clocks off/on
// to retain 180 degree phase relationship (CLK0/CLK1)
// Suppose we don't do this without a pll reset for some reason
// when clocks are turned back on
void vfo_turn_off_clk_out(uint8_t clk_num) {
    // are these enable bits inverted? Yes, 1 is disable
    V1_println(F("vfo_turn_off_clk_out START"));
    uint8_t enable_bit = 1 << clk_num;
    // clk0 implies clk1 also
    if (clk_num == 0) {
        enable_bit |= 1 << 1;
    }
    si5351bx_clken |= enable_bit;
    // if si5351a power is off we'll get ERROR: res -1 after i2cWrite 3
    i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
    V1_printf("vfo_turn_off_clk_out END clk_num %u" EOL, clk_num);
}

//****************************************************
void vfo_set_drive_strength(uint8_t clk_num, uint8_t strength) {
    V1_printf("vfo_set_drive_strength START clk_num %u" EOL, clk_num);

    // only called during the initial vfo_turn_on()
    s_vfo_drive_strength[clk_num] = strength;

    //**********************
    // reset the s_ms_div_prev to force vfo_set_freq_x128()
    // to call si5351a_setup_multisynth1() next time
    s_ms_div_prev = 0;
    // new 11/24/24 ..maybe clear all this old state too!
    memset(s_PLLB_regs_prev, 0, 8);
    // not these though?
    // si5351bx_clken = 0xff;
    // memset(s_vfo_drive_strength, 0, 4);

    // Triggering a pll reset requires some delay afterwards
    // so we don't want that to happen or be required (to retain 180 degree antiphase)
    // if we change enough, and don't pll reset,
    // we can lose the 180 degree phase relationship
    // between CLK0 and CLK1. see comments from Hans.
    // (not well documented when a pll reset is required).
    // Not for the small Hz shifts during symbol tx
    //**********************

    V1_printf("vfo_set_drive_strength END clk_num %u" EOL, clk_num);
}

//****************************************************
bool vfo_is_on(void) {
    // power on and completed successfully
    //*********************************
    // FIX! switching in/out direction on the gpio power pin doesn't make sense.

    // old code changed the dir of the gpio
    // static bool gpio_is_dir_out ( uint gpio )
    // Check if a specific GPIO direction is OUT.
    // gpio GPIO number
    // returns // true if the direction for the pin is OUT

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
    // when driving GPIOs –
    // instead functions like gpio_put() should be used to atomically update GPIOs.
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
void vfo_turn_on(uint8_t clk_num) {
    // FIX! what if clk_num is not zero?
    // turn on of 0 turns on 0 and 1 now
    clk_num = 0;
    V1_printf("vfo_turn_on START clk_num %u" EOL, clk_num);

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
    vfo_init();
    Watchdog.reset();
    // 2 secs enough?
    sleep_ms(2000);

    // what timer to use?
    // timer_busy_wait_ms
    // void timer_busy_wait_ms (timer_hw_t *timer, uint32_t delay_ms)
    // Busy wait wasting cycles for the given number of milliseconds
    // using the given timer instance.

    uint8_t reg;
    // Disable all CLK output drivers
    V1_println(F("vfo_turn_on trying to i2cWrite SI5351A_OUTPUT_ENABLE_CONTROL with 0xff"));
    V0_flush();

    // HACK ..don't iterate
    int tries = 0;
    // any error
    int res = i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff);
    V1_printf("vfo_turn_on first res %d of i2cWrite SI5351A_OUTPUT_ENABLE_CONTROL" EOL , res);
    V0_flush();
    while (res != 2) {
        if (tries > 5) {
            V1_println("Rebooting because couldn't init VFO_I2C_INSTANCE after 5 tries");
            V0_flush();
            Watchdog.enable(1000);  // milliseconds
            for (;;) {
                // FIX! put a bad status in the leds?
                updateStatusLED();
            }
        }
        Watchdog.reset();
        tries++;
        V1_println("VFO_I2C_INSTANCE trying re-init, after trying a i2cWrite and it failed");
        V0_flush();

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
        res = i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff);
        V0_flush();
    }

    V1_print(F("vfo_turn_on done trying to init the I2C0 pins in loop" EOL));

    // Moved this down here ..we don't know if we'll hang earlier
    // sets state to be used later
    if (_tx_high[0] == '0') {
        // this also clears the "prev" state, so we know we'll reload all state in the si5351
        // i.e. no optimization based on knowing what we had sent before!
        vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_2MA);
        vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_2MA);
    } else {
        // FIX! make sure this is default in config
        vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_8MA);
        vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_8MA);
    }

    // Powerdown CLK's
    for (reg = SI5351A_CLK0_CONTROL; reg <= SI5351A_CLK7_CONTROL; reg++) {
        i2cWrite(reg, 0xCC);
    }

    // set MS0 for div_4 mode (min. division)
    i2cWriten(42, (uint8_t *)s_ms_values, 8);
    // set MS1 for div_4 mode (min. division)
    i2cWriten(50, (uint8_t *)s_ms_values, 8);
    // set MS2 for div_4 mode (min.division)
    i2cWriten(58, (uint8_t *)s_ms_values, 8);

    // set PLLA for div_16 mode (minimum even integer division)
    i2cWriten(26, (uint8_t *)s_pll_values, 8);
    // set PLLB for div_16 mode (minimum even integer division)
    i2cWriten(34, (uint8_t *)s_pll_values, 8);

    i2cWrite(149, 0x00);  // Disable Spread Spectrum
    i2cWrite(177, 0xA0);  // Reset PLLA and PLLB
    // FIX! is this a reserved address?

    // in AN1234 register map, this is shown as CLKIN_FANOUT_EN and XO_FANOUT_EN
    // also MS_FANOUT_EN ? Rev  0.6 of spec (newer is 0.95?)
    // https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
    // also in AN619 "Manually Generating an Si5351 Register map" covers Si5351A/B/C
    // for Si5351 16QFN device only?
    // but what enables it?
    // I think it just doesn't exist in the 0.95 datasheet. let's leave it out!
    // i2cWrite(187, 0x00);  // Disable all fanout

    //***********************
    s_ms_div_prev = 0;
    // new 11/24/24 ..maybe clear all this old state
    memset(s_PLLB_regs_prev, 0, 8);
    si5351bx_clken = 0xff;
    memset(s_vfo_drive_strength, 0, 3);

    //***********************
    // debug only, on 20M
    // uint32_t freq = 14097100UL;
    uint32_t freq = XMIT_FREQUENCY;

    // this is aligned to integer. (symbol 0)
    V1_printf("initial freq for vfo_set_freq_x128() is %lu" EOL, freq);
    uint32_t freq_x128 = freq << PLL_CALC_PRECISION;
    vfo_set_freq_x128(clk_num, freq_x128, false);  // not only_pll_num

    // The final state is clk0/clk1 running but outputs not on
    si5351bx_clken = 0xff;
    vfo_turn_on_clk_out(clk_num);

    // FIX! hmm. we need a pll reset if we turn on the clk_out
    // where does that happen? Do one here (do we have to wait?) just in case?
    // it was done in vfo_set_freq_x128 if we didn't have initial state
    // or didn't change initial state?
    // new 11/28/24
    si5351a_reset_PLLB();

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
// given the base XMIT_FREQ for the u4b channel config'ed
// THIS MUST NOT BE USED FOR SI5351A work..just for printing!
double calcSymbolFreq(uint32_t hf_freq, uint8_t symbol) {
    if (symbol > 3) {
        V1_printf("ERROR: calcSymbolFreq symbol %u is not 0 to 3 ..using 0"
            EOL, symbol);
        symbol = 0;
    }

    // the frequency shift is 12000 / 8192 (approx 1.46hz)
    double symbol_freq_x128 =
        (hf_freq << PLL_CALC_PRECISION) +
        ((symbol * (12000L << PLL_CALC_PRECISION) + 4096L) / 8192L);

    double calcPrecisionDivisor = pow(2, PLL_CALC_PRECISION);
    double symbolFreq = (double) symbol_freq_x128 / calcPrecisionDivisor;

    double symbolOffset = symbolFreq - hf_freq;
    // 1.46..Hz per symbol
    double expectedSymbolOffset = ((double) symbol * 12000.0) / 8192.0;
    V1_printf("For hf_freq %lu symbol %u symbolFreq is %.4f",
        hf_freq, symbol, symbolFreq);
    V1_printf(" symbolOffset %.4f expectedSymbolOffset %.4f" EOL,
        symbolOffset, expectedSymbolOffset);

    return symbolFreq;
}

//**********************************
uint32_t calcSymbolFreq_x128(uint32_t hf_freq, uint8_t symbol) {
    // not expensive to always recalc the symbol freq
    // don't want printing though (too slow)
    uint32_t freq_x128_with_symbol = (
        hf_freq << PLL_CALC_PRECISION) +
        ((symbol * (12000L << PLL_CALC_PRECISION) + 4096L) / 8192L);
    return freq_x128_with_symbol;
}

//**********************************
void startSymbolFreq(uint32_t hf_freq, uint8_t symbol, bool only_pll_num) {
    // Calculate the frequency for a symbol
    // Note all the shifting so integer arithmetic is used everywhere,
    // and precision is not lost.
    static bool oneShotDebugPrinted = false;
    if (!oneShotDebugPrinted) {
        // if it's the first startSymbolFreq with only_pll_num false, we
        // have time to print the 4 symbol freqs that will be used
        // we can shift down, so they will be what the sdr sees
        // AH this has more precision in it..can't just shift down to print it!
        // Doug has WSPR_TONE_SPACING_HUNDREDTHS_HZ = 146 (1.4648 Hz)

        // don't do the >> PLL_CALC_PRECISION here, as that's integer roundff
        /// do float division by 16!
        double symbol_0_freq = calcSymbolFreq(hf_freq, 0);
        double symbol_1_freq = calcSymbolFreq(hf_freq, 1);
        double symbol_2_freq = calcSymbolFreq(hf_freq, 2);
        double symbol_3_freq = calcSymbolFreq(hf_freq, 3);
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
    vfo_set_freq_x128(WSPR_TX_CLK_NUM, freq_x128_with_symbol, only_pll_num);

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


// looking at the putty.log for u4b channel 0, on 10m
// seems like we got reasonable pll_num values and actual frequencies?
// but we should do the same sweep for the actual symbol frequencies?
// test calc'ing 5351a programming starting at 28126000.0000
// pll_freq 801591570 ms_div 456 pll_mult 30 pll_num 830443 pll_denom 1000000 freq 28126020 actual 28126020.00
// pll_freq 801591598 ms_div 456 pll_mult 30 pll_num 830444 pll_denom 1000000 freq 28126021 actual 28126020.98
// pll_freq 801591627 ms_div 456 pll_mult 30 pll_num 830446 pll_denom 1000000 freq 28126022 actual 28126022.00
// pll_freq 801591655 ms_div 456 pll_mult 30 pll_num 830447 pll_denom 1000000 freq 28126023 actual 28126022.98
// pll_freq 801591684 ms_div 456 pll_mult 30 pll_num 830448 pll_denom 1000000 freq 28126024 actual 28126024.00
// pll_freq 801591712 ms_div 456 pll_mult 30 pll_num 830449 pll_denom 1000000 freq 28126025 actual 28126024.98
// pll_freq 801591741 ms_div 456 pll_mult 30 pll_num 830450 pll_denom 1000000 freq 28126026 actual 28126026.00


//**************************************************
// comparing to traquito 12/21/24: https://groups.io/g/picoballoon/topic/110236980#msg18787
// thanks doug. I sorted it out as a print issue, not a config issue.
// Although I probably should adjust the math to be as close to perfect as you are
//
// details:
// We both operate in a shifted integer domain, when calculating what to init si5351a with.
// You work in a *100 domain so you maintain precision to the hundredths for symbol frequency
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
// Band 10 BAND_XMIT_FREQ  28126020
// For hf_freq 28126020 symbol 0 symbolFreq is 28126020.0000 symbolOffset 0.0000 expectedSymbolOffset 0.0000
// For hf_freq 28126020 symbol 1 symbolFreq is 28126021.4375 symbolOffset 1.4375 expectedSymbolOffset 1.4648
// For hf_freq 28126020 symbol 2 symbolFreq is 28126022.9375 symbolOffset 2.9375 expectedSymbolOffset 2.9297
// For hf_freq 28126020 symbol 3 symbolFreq is 28126024.3750 symbolOffset 4.3750 expectedSymbolOffset 4.3945
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

// poster notes difficulties at 144Mhz
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
// To do better would require calculating the PLL frequency to a fractional frequency.

// hans says:
// The Si5351A is capable of very high precision,
// you can achieve better than a milli-Hz at 144MHz and
// I suspect a few orders of magnitude higher precision than that.
// The PLL feedback and the Division down from the internal
// PLL VCO are both fractional, of the form a + b / c.
// Both b and c are 20-bit integers.
// Most code and libraries you see just fix c at 0xFFFFF and just vary b.
// This is what limits the resolution.
