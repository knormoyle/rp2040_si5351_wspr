// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// https://wellys.com/posts/rp2040_arduino_i2c/
// from https://github.com/lkoepsel/I2C/blob/main/Arduino/Pico/I2C_Scanner/I2C_Scanner.ino

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "i2c_functions.h"
#include "si5351_functions.h"
#include "print_functions.h"
#include <Adafruit_I2CDevice.h>  // https://github.com/adafruit/Adafruit_BusIO
#include <Wire.h>

// FIX! where should this go? also in si5351_functions.cpp
#define VFO_I2C_INSTANCE i2c0
extern const int SI5351A_I2C_ADDR;
extern const int VFO_I2C0_SDA_PIN;
extern const int VFO_I2C0_SCL_PIN;
extern const int VFO_I2C0_SCL_HZ;
extern const int BMP_I2C1_SDA_PIN;
extern const int BMP_I2C1_SCL_PIN;
extern const int BMP_I2C1_SCL_HZ;
extern const int Si5351Pwr;

// decode of verbose 0-9
extern bool VERBY[10];

// also ../i2c_test/i2c_test.ino for probing si5351 regs, with random read/write
// FIX! better scanner now from ../i2c_test/bus_scan/bus_scan.ino

// source: https://raw.githubusercontent.com/raspberrypi/pico-examples/refs/heads/master/i2c/bus_scan/bus_scan.c
// https://github.com/raspberrypi/pico-examples/blob/master/i2c/bus_scan/bus_scan.c

// Runs on AG6NS pcb with kbn or kbn2 components. (without or with BMP280)
// AG6NS does have 3.3k and 10k pullups on Si5351 and BMP i2c, respectively
// So don't use pullups from the RP2040

// RP2040 has i2c0 and i2c1
// If we use Wire/Wire1:
// have to use the pins labeled I2C0 for Wire
// have to use the pins labeleds I2C1 for Wire1
// we're not going to use the newer Wire() stuff
// going to just the older style

// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
// https://arduino-pico.readthedocs.io/en/latest/wire.html
// https://docs.arduino.cc/language-reference/en/functions/communication/wire/


// 100000 is probably the max we can do on either i2c bus?
extern const int PICO_I2C_CLK_HZ;

//*************************************
// https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2040
// https://github.com/raspberrypi/pico-examples/blob/master/i2c/bmp280_i2c/bmp280_i2c.c

// The Raspberry Pi Pico SDK
// https://github.com/raspberrypi/pico-sdk

// Reference for parameters and return values
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/include/hardware/i2c.h

// uint i2c_init(i2c_inst_t *i2c, uint baudrate);
// void i2c_deinit(i2c_inst_t *i2c);
// uint i2c_set_baudrate(i2c_inst_t *i2c, uint baudrate);
// void i2c_set_slave_mode(i2c_inst_t *i2c, bool slave, uint8_t addr);
// int i2c_write_blocking_until(i2c_inst_t *i2c, uint8_t addr,
//  const uint8_t *src, size_t len, bool nostop, absolute_time_t until);
// int i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr,
//  uint8_t *dst, size_t len, bool nostop, absolute_time_t until);
// int i2c_write_timeout_per_char_us(i2c_inst_t *i2c, uint8_t addr,
//  const uint8_t *src, size_t len, bool nostop, uint timeout_per_char_us);
// int i2c_read_timeout_per_char_us(i2c_inst_t *i2c, uint8_t addr,
//  uint8_t *dst, size_t len, bool nostop, uint timeout_per_char_us);
// int i2c_write_blocking(i2c_inst_t *i2c, uint8_t addr,
//  const uint8_t *src, size_t len, bool nostop);
// int i2c_read_blocking(i2c_inst_t *i2c, uint8_t addr,
//  uint8_t *dst, size_t len, bool nostop);

// PICO_ERROR_GENERIC used for errors

// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/include/hardware/i2c.h
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/include/hardware/i2c.h#L204
// does this for us:
// extern i2c_inst_t i2c0_inst;
// extern i2c_inst_t i2c1_inst;
// #define i2c0 (&i2c0_inst) ///< Identifier for I2C HW Block 0
// #define i2c1 (&i2c1_inst) ///< Identifier for I2C HW Block 1

//**************
// resolves at compile time only
// I2C_NUM(i2c)
// returns the I2c number for an I2C instance
// I2C_INSTANCE(i2c_num)
// returns the I2C instance with the given I2C number

// runtime resolutions
// Get hardware instance number,  from I2C instance
// uint i2c_get_index(i2c_inst_t *i2c)

// get pointer to structure containing i2c hardware registers, from the I2C instance.
// i2c_hw_t *i2c_get_hw(i2c_inst_t *i2c) {

// Get I2C instance from I2C hardware instance number
// i2c_inst_t *i2c_get_instance(uint num)

// The external pins of each controller are connected to GPIO pins as
// defined in the GPIO muxing table in the datasheet.
// The muxing options  give some IO flexibility,
// but each controller external pin should be connected to only one GPIO.

// The controller does NOT support High speed mode or Ultra-fast speed mode,
// the fastest operation being fast mode plus at up to 1000Kb/s.

//*******************************************************************************
// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//    0 1 2 3 4 5 6 7 8 9 A B C D E F
// 00 . . . . . . . . . . . . . . . .
// 10 . . @ . . . . . . . . . . . . .
// 20 . . . . . . . . . . . . . . . .
// 30 . . . . @ . . . . . . . . . . .
// 40 . . . . . . . . . . . . . . . .
// 50 . . . . . . . . . . . . . . . .
// 60 . . . . . . . . . . . . . . . .
// 70 . . . . . . . . . . . . . . . .
// E.g. if addresses 0x12 and 0x34 were acknowledged.


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
// why did it hang on isc1 on addr 7.

// AG6NS pullup resistors are different on i2c1 compared to i2c0
// 04 to 07 are HS-Mode Controller
// https://electronics.stackexchange.com/questions/680602/whats-the-actual-meaning-behind-the-i%C2%B2c-reserved-addresses
// https://www.ti.com/lit/an/sbaa565/sbaa565.pdf?ts=1731999247195

// HI   LO  R¯W
// 0000 000  0  general call ("broadcast address")
// 0000 000  1  START byte
// 0000 001     CBUS compatibility       *
// 0000 010     different bus format     *
// 0000 011     future purposes
// 0000 1       Hs-mode                  *
// 1111 1    1  device ID
// 1111 0       10bit address

bool reserved_addr(uint8_t addr) {
    uint8_t addr7f = addr & 0x7f;
    switch (addr7f) {
        case 0x00: {}
        case 0x01: {}
        case 0x02: {}
        case 0x03: {}
        case 0x04: {}
        case 0x05: {}
        case 0x06: {}
        case 0x07: {}
        case 0x78: {}
        case 0x79: return true;
        default: return false;
    }
}

//****************************************************
int scan_i2c(int i2c_number) {
    V1_printf(EOL "scan_i2c(%d) START" EOL, i2c_number);
    // power up the Si5351 if we're scanning that bus
    // FIX! what about the BMP..it's always on!
    if (i2c_number == 0) {
        V1_println(F("power on Si5351"));
        gpio_init(Si5351Pwr);
        pinMode(Si5351Pwr, OUTPUT_4MA);
        digitalWrite(Si5351Pwr, LOW);
    }

    // Get I2C instance from I2C hardware instance number
    // i2c_inst_t *i2c_get_instance(uint num)
    i2c_inst_t *i2c_instance_to_test = i2c_get_instance(i2c_number);

    //**********************************************
    // I suppose could set them both up at once, rather than conditional

    // FIX! is this okay? changes the init?
    V1_println(F("Deinit both i2c0 and i2c1"));
    // deinit both! Just to make sure we're only using the one we expect

    i2c_deinit(i2c0);
    i2c_deinit(i2c1);

    // only init the one we're testing
    V1_printf("Only init the isc we're testing:  %d with %d Hz rate" EOL,
        i2c_number, PICO_I2C_CLK_HZ);
    // uint i2c_init (i2c_inst_t * i2c, uint baudrate)
    i2c_init(i2c_instance_to_test, PICO_I2C_CLK_HZ);

    int sda_pin;
    int scl_pin;
    if (i2c_number == 0) {
        sda_pin = VFO_I2C0_SDA_PIN;
        scl_pin = VFO_I2C0_SCL_PIN;
    } else {
        sda_pin = BMP_I2C1_SDA_PIN;
        scl_pin = BMP_I2C1_SCL_PIN;
    }

    V1_printf("SDA is pin %d" EOL, sda_pin);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);

    V1_printf("SCL is pin %d" EOL, scl_pin);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    V1_println(F("no pullup or pulldowns by RP2040 on both SDA and SCL"));
    gpio_set_pulls(scl_pin, false, false);
    gpio_set_pulls(sda_pin, false, false);

    //**********************************************

    // from the orig. code.
    // Make the I2C pins available to picotool
    // bi_decl(bi_2pins_with_func(PICO_I2C_SDA_PIN, PICO_I2C_SCL_PIN, GPIO_FUNC_I2C));

    V1_print(F(EOL "I2C Bus Scan" EOL));
    V1_print(F("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F" EOL));

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) V1_printf("%02x ", addr);

        // Perform a 1-byte dummy blocking read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns -1.
        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr)) {
            // V1_printf("reserved addr %u", addr);
            ret = PICO_ERROR_GENERIC;
        } else {
            // why is it hanging on 7 on isc1 (BMP) bus?
            // switch to timeout
            // ret = i2c_read_blocking(i2c_instance_to_test, addr, &rxdata, 1, false);
            ret = i2c_read_timeout_us(i2c_instance_to_test, addr, &rxdata, 1, false, 5000);
        }

        // hmm was this conditional the reason my macros didn't work
        char char1[2] = { 0 };
        char1[0] = ret < 0 ? '.' : '@';
        // EOL
        if (addr % 16 == 15) {
            V1_printf("%s" EOL,  char1);
        // <space>
        } else {
            V1_printf("%s ",  char1);
        }
        V0_flush();
    }

    if (i2c_number == 0) {
        V1_println("power off Si5351");
        gpio_init(Si5351Pwr);
        pinMode(Si5351Pwr, OUTPUT_4MA);
        digitalWrite(Si5351Pwr, LOW);
        V0_flush();
    }

    //**********************************************
    V1_printf("scan_i2c(%d) END" EOL, i2c_number);
    return 0;
}

//****************************************************
void i2c_scan_both() {
    V1_println(F(EOL "Will scan i2c0 (si5351 should be only device)" EOL));
    scan_i2c(0);

    V1_println(F(EOL "Will scan i2c1 (bmp280 should be only device)" EOL));
    scan_i2c(1);

    // FIX! this is okay to reinit the is2?
    if (false) {
        V1_println(F("Deinit both i2c0 and i2c1"));
        i2c_deinit(i2c0);
        i2c_deinit(i2c1);
        V1_println(F("left sda/scl's as defined with pullups"));
        V1_println(F("power off Si5351"));
        gpio_init(Si5351Pwr);
        pinMode(Si5351Pwr, OUTPUT_4MA);
        digitalWrite(Si5351Pwr, LOW);
    }
}
//****************************************************
// this assumes Wire is created beforehand?
void i2c_scan_with_Wire(void) {
    V1_println(F("i2c_scan START"));
    uint8_t error, address;
    int nDevices;
    V1_println("Scanning...");
    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            V1_print(F("I2C device found at address 0x"));
            if (address < 16) {
                V1_print(F("0"));
            }
            V1_println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            V1_print(F("Unknown error at address 0x"));
            if (address < 16) {
                V1_print(F("0"));
            }
            V1_println(address, HEX);
        }
    }
    if (nDevices == 0) {
        V1_println("No I2C devices found" EOL);
    }
    else V1_println("done\n");
    V1_println(F("i2c_scan END"));
}

//************************************************
byte error, address;
unsigned int nDevices;
unsigned int wireN = 0;

// Pi Pico SDA/SCL Pins
const unsigned int SDA_00 = 0;
const unsigned int SCL_00 = 1;
const unsigned int SDA_10 = 2;
const unsigned int SCL_10 = 3;
const unsigned int SDA_01 = 4;
const unsigned int SCL_01 = 5;
const unsigned int SDA_11 = 6;
const unsigned int SCL_11 = 7;
const unsigned int SDA_02 = 8;
const unsigned int SCL_02 = 9;
const unsigned int SDA_12 = 10;
const unsigned int SCL_12 = 11;
const unsigned int SDA_03 = 12;
const unsigned int SCL_03 = 13;
const unsigned int SDA_13 = 14;
const unsigned int SCL_13 = 15;
const unsigned int SDA_04 = 16;
const unsigned int SCL_04 = 17;
const unsigned int SDA_14 = 18;
const unsigned int SCL_14 = 19;
const unsigned int SDA_05 = 20;
const unsigned int SCL_05 = 21;
const unsigned int SDA_15 = 26;
const unsigned int SCL_15 = 27;

void i2c_scanner_with_Wire_setup() {
    V1_println(F("i2c_scanner_setup START"));
    V1_println(F("Just Wire"));

    scan_Wire(SDA_00, SCL_00, Wire);
    scan_Wire(SDA_01, SCL_01, Wire);
    scan_Wire(SDA_02, SCL_02, Wire);
    scan_Wire(SDA_03, SCL_03, Wire);
    scan_Wire(SDA_04, SCL_04, Wire);
    scan_Wire(SDA_05, SCL_05, Wire);

    V1_println(F("Just Wire1"));
    scan_Wire(SDA_10, SCL_10, Wire1);
    scan_Wire(SDA_11, SCL_11, Wire1);
    scan_Wire(SDA_12, SCL_12, Wire1);
    scan_Wire(SDA_13, SCL_13, Wire1);
    scan_Wire(SDA_14, SCL_14, Wire1);
    scan_Wire(SDA_15, SCL_15, Wire1);

    V1_println("All Scans Successful!\n");
    V1_println(F("i2c_scanner_setup END"));
}

// scan the Wire interfaces for devices
void scan_Wire(unsigned int SDA, unsigned int SCL, TwoWire &Wire) {
    V1_printf("scan_WIRE %u %u START" EOL, SDA, SCL);
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    Wire.begin();

    V1_print(F("Wire"));
    V1_print(wireN % 2);
    V1_print(F(" SDA: "));
    V1_print(SDA);
    V1_print(F(" SCL: "));
    V1_println(SCL);

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            V1_print(F("I2C device found at address 0x"));
            if (address < 16) V1_print(F("0"));
            V1_println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            V1_print(F("Unknown error at address 0x"));
            if (address < 16) {
                V1_print(F("0"));
            }
            V1_println(address, HEX);
        }
    }
    if (nDevices == 0) {
        V1_print(F("No I2C devices found" EOL));
    } else {
        V1_print(F("Scan Complete" EOL));
    }
    Wire.end();
    wireN++;
    V1_printf("scan_WIRE %u %u END" EOL, SDA, SCL);
}

//****************************************************
// https://deepbluembedded.com/arduino-i2c-tutorial-examples/
int I2cWriteTest(uint8_t reg, uint8_t val) {  // write reg via i2c
    V1_printf("I2cWriteTest START reg %02x val %02x" EOL, reg, val);
    // FIX! shouldn't this be local ? or does it setup data for I2cWriteTestn
    // moved here to be local, and not static (shared) anymore
    // only need length 2!
    uint8_t i2c_buf[2];
    i2c_buf[0] = reg;
    i2c_buf[1] = val;

    int res;
    if (reserved_reg(reg)) {
        // don't want to hang on a reserved reg. so don't send
        V1_printf("reserved reg %u", reg);
        // make this a unique error to recognize my reserved reg detection
        res = 127;
    } else {
        // good examples at:
        // https://github.com/raspberrypi/pico-examples/blob/master/i2c/mpu6050_i2c/mpu6050_i2c.c
        // Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged,
        // no device present.
        V1_printf("I2cWriteTest doing i2c_write_blocking() reg %02x val %02x" EOL, reg, val);
        res = i2c_write_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false);
        V1_print(F(EOL));
    }

    if (res == 127) {
        // my decode for reserved
    } else if (res == PICO_ERROR_GENERIC) {
        V1_printf("ERROR: I2cWriteTest() got bad res %d reg %02x val %02x" EOL,
            res, reg, val);
    } else if (res == 2) {
        V1_printf("GOOD: I2cWriteTest() got good res %d reg %02x val %02x" EOL,
            res, reg, val);
    } else {
        V1_printf("UNEXPECTED: i2cWRite() got unexpected res %d reg %02x val %02x" EOL,
            res, reg, val);
    }
    V1_print(F(EOL));


    V1_printf("I2cWriteTest END reg %02x val %02x" EOL, reg, val);
    return res;
}

//****************************************************
// data returned to val after read of reg from i2c
// hardwired to the si5351 i2c bus/instance and to si5351 addr
// so just reg to specify
// checks for reserved addresses on si5351 and forces error (doesn't send them)
int i2cWrReadTest(uint8_t reg, uint8_t *val) {
    // V1_printf("i2cWrReadTest START reg %02x val %02x" EOL, reg, *val);
    // don't get confused by the old values in *val
    V1_printf("i2cWrReadTest START reg %02x" EOL, reg);

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
    // res = i2c_read_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false, 1000);

    // have to to write then read to get read data? Can keep master control of bus after the first

    // example getting 6 bytes
    // val = 0x43;
    // i2c_write_blocking(i2c_default, addr, &val, 1, true);
    // False - finished with bus
    // i2c_read_blocking(i2c_default, addr, buffer, 6, false);
    // attaching a bmp280 with i2c
    // https://github.com/raspberrypi/pico-examples/tree/master/i2c/bmp280_i2c

    // grab this code
    // https://github.com/raspberrypi/pico-examples/blob/master/i2c/bmp280_i2c/bmp280_i2c.c

    // raspberry pi bus scan
    // https://github.com/raspberrypi/pico-examples/blob/master/i2c/bus_scan/bus_scan.c

    // 3-axis digital accelerometer
    // https://github.com/raspberrypi/pico-examples/tree/master/i2c/mma8451_i2c

    // here is example that is probably similar to what si5351 spec says for reads
    // see si5351 spec page 15
    // https://github.com/raspberrypi/pico-examples/blob/master/i2c/bmp280_i2c/bmp280_i2c.c
    // i2c_write_blocking(i2c_default, ADDR, &reg, 1, true);  // true to keep master control of bus
    // i2c_read_blocking(i2c_default, ADDR, buf, 6, false);  // false - finished with bus

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


    int res, res1, res2;
    uint8_t i2c_buf[2] = { 0 };  // only really need 1 byte
    i2c_buf[0] = 0x73;  // just a value I don't expect to see
    if (reserved_reg(reg)) {
        // don't want to hang on a reserved reg. so don't send
        V1_printf("reserved reg %u", reg);
        // make this a unique error to recognize my reserved reg detection
        res = 127;
    } else {
        // Keep these blocking. should never hit reserved reg (checked above).. easier debug to hang
        // Watchdog timer will save us if i2c hangs in flight? have to reboot or retry anyhow.
        // There should be no data on this first write to set the register address. so just 1 byte.
        // This is just handling one byte reads. can do more. another function.
        // They want a ptr for the "buffer"..hence &reg
        // works with false, (full stop) also
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
        V1_printf("ERROR: i2cWrReadTest() got bad res %d reg %02x val %02x" EOL, res, reg, *val);
    } else if (res == 1) {
        V1_printf("GOOD: i2cWrReadTest() got good res %d reg %02x val %02x" EOL, res, reg, *val);
    } else {
        V1_printf("UNEXPECTED: i2cWrReadTest() got unexpected res %d reg %02x val %02x" EOL,
            res, reg, *val);
    }
    V1_print(F(EOL));

    // FIX! no expected data compare.
    // I just eyeball the sequential write/read in the serial monitor
    // without knowing the reg, it may be okay that some bits differ between write and read
    // especially since I'm writing random

    V1_printf("i2cWrReadTest END reg %02x val %02x" EOL, reg, *val);
    return res;
}
