//*******************************************
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdio.h"
#include "hardware/gpio.h"

#include <stdlib.h>
// #include <string.h>

#include <SPI.h>
#include <Wire.h>

#include <SPI.h>
#include <Wire.h>

#include "hardware/i2c.h"

// knowledge from Hans about si5351
// All the QRP Labs firmware (and example source code) use the minimum jitter even integer divider mode.
// When you first set up the Si5351A registers the first time, you need to do a PLL reset otherwise nothing works. You do NOT need to do a PLL reset for subsequent PLL feedback divider changes! I have not been able to find any size of frequency change where a PLL reset is required.

// What the datasheet does NOT tell you, is that if you wish to use the phase offset feature to maintain an accurate and constant phase offset between two of the Si5351A outputs on the same frequency - then you MUST do a PLL reset under two circumstances:
//
// 1) Every time you change the MultiSynth divider (whether integer or fractional) - note, the MutliSynth divider! Not the PLL feedback divider! Which is kind of opposite to the documentation... but trust me, this is the way the chip actually works
//
// 2) Every time you switch the outputs on/off using the Clock Control registers e.g. Register 16. If you are about phase relationships, INCLUDING if you have simply set a 180-degree phase relationship to another clock output using the clock invert bit in the clock control register, even then, you have to do a PLL Reset.
//
// If you don't do the PLL reset under these two circumstances then the phase offset will be random. Frequency will be fine though - this part of the PLL reset discussion only applies to when you wish to have a precise phase offset between outputs. Examples which I use are when you want push-pull outputs (180-degree phase difference), some people use this for driving a LF PA; or when you want 90-degree phase offset for switching a Quadrature Sampling Detector type mixer which requires a quadrature LO.
//
// I have not seen any of this anywhere in SiLabs documentation, I reached the above conclusions after hours and hours of experimenting...
//
// It is very interesting to me that the few circumstances in which a PLL reset is actually required, apparently are related to the use of the MutliSynth Divider stage of the chip - not the PLL! It almost makes me wonder if this PLL reset register is totally misnamed and totally misunderstood - should it really be called the "MultiSynth Reset Register"?

// example code
// https://qrp-labs.com/synth
// https://qrp-labs.com/synth/si5351ademo.html#demo1
// https://qrp-labs.com/synth/oe1cgs.html
// this uses the wire library. only writes?
// https://qrp-labs.com/images/synth/demo4/oscillator.ino

// https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
// good tutorial on the part
// https://rfzero.net/tutorials/si5351a/

// https://groups.io/g/BITX20/topic/si5351a_facts_and_myths/5430607?p=Created,,,20,1,0,0&jump=1

//*********************************************
// https://stackoverflow.com/questions/45530486/how-to-write-a-macro-with-optional-and-variadic-arguments
// https://stackoverflow.com/questions/26053959/what-does-va-args-in-a-macro-mean
// full details of issues at:
// https://en.wikipedia.org/wiki/Variadic_macro_in_the_C_preprocessor
// C99 has the trailing comma expansion issue
// c++20 works with the following:

// this doesn't seem to work

#ifndef FLY_WITH_NO_USBSERIAL
#define FLY_WITH_NO_USBSERIAL 0
#endif

// have to handle a varying number of args
#if FLY_WITH_NO_USBSERIAL == 1
#define xPrintf(...)
// just one arg allowed
#define xPrint(x)
#define xPrintLn(x)
#else
// prepend the function name?
#define xPrintf(cformat, ...) Serial.printf(cformat __VA_OPT__ (,) __VA_ARGS__)
// just one arg allowed
#define xPrint(x) Serial.print(x)
#define xPrintln(x) Serial.println(x)
#endif

//*********************************************

// hmm. why is \n not sufficient?
#define EOL "\r\n"
// ascii 13
#define CR "\r"
// ascii 10
#define LF "\n"
// #define EOL "\n"

#define kHz 1000U

#define Si5351Pwr 4
#define VFO_I2C_INSTANCE i2c0
#define VFO_VDD_ON_N_PIN 4
#define VFO_I2C0_SDA_PIN 12
#define VFO_I2C0_SCL_PIN 13
#define SI5351A_I2C_ADDR 0x60
// 50000 not good?
#define VFO_I2C0_SCL_HZ 100000
#define SI5351A_OUTPUT_ENABLE_CONTROL 3

void setup() {
    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
    // https://arduino-pico.readthedocs.io/en/latest/wire.html
    // RP2040 has i2c0 and i2c1
    // have to use the pins labeled I2C0 for Wire
    // have to use the pins labeleds I2C1 for Wire1

    // https://docs.arduino.cc/language-reference/en/functions/communication/wire/
    // Wire.setSDA(VFO_I2C0_SDA_PIN);
    // Wire.setSCL(VFO_I2C0_SCL_PIN);
    // Wire.begin()

    gpio_init(VFO_I2C0_SCL_PIN);
    gpio_init(VFO_I2C0_SDA_PIN);

    gpio_init(Si5351Pwr);
    pinMode(Si5351Pwr, OUTPUT_8MA);
    digitalWrite(Si5351Pwr, LOW);
    while (!Serial) {;}

    // GPIO function selectors
    // Each GPIO can have one function selected at a time.
    // Likewise, each peripheral input (e.g. UART0 RX)
    // should only be selected on one GPIO at a time.
    // If the same peripheral input is connected to multiple GPIOs,
    // the peripheral sees the logical OR of these GPIO inputs.

    // Please refer to the datasheet for more information on GPIO function selection.

    // https://arduino-pico.readthedocs.io/en/latest/digital.html
    // default is 4mA
    // pinMode(VFO_I2C0_SCL_PIN, INPUT_PULLUP);
    // pinMode(VFO_I2C0_SDA_PIN, INPUT_PULLUP);

    // gpio_pull_up(Si5351Pwr);
    // gpio_put(Si5351Pwr, 0);
    // gpio_set_dir(Si5351Pwr, true); // true for out, false for in

    // void gpio_set_pulls(uint gpio, bool up, bool down )
    // Select up and down pulls on specific GPIO.
    // gpio GPIO number
    // up If true set a pull up on the GPIO
    // down If true set a pull down on the GPIO
    // On the RP2040, setting both pulls enables a "bus keep" function,
    // i.e. a weak pull to whatever is current high/low state of GPIO.
    // gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
    // gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

    // static void gpio_pull_up(uint gpio)
    // Set specified GPIO to be pulled up.
    // Parameters
    // gpio GPIO number

    // i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
    Serial.println("i2c_deinit() start");
    i2c_deinit(VFO_I2C_INSTANCE);
    Serial.println("i2c_deinit() complete");
    busy_wait_ms(1000);

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
    Serial.println("i2c_init() start");
    i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
    Serial.println("i2c_init() complete");
    busy_wait_ms(1000);

    gpio_pull_up(VFO_I2C0_SDA_PIN);
    gpio_pull_up(VFO_I2C0_SCL_PIN);
    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
    i2c_set_slave_mode(VFO_I2C_INSTANCE, false, 0);
    busy_wait_ms(1000);

    // power the vfo off/on
    digitalWrite(Si5351Pwr, HIGH);
    busy_wait_ms(1000);
    digitalWrite(Si5351Pwr, LOW);
    busy_wait_ms(2000);

    // uint i2c_init ( i2c_inst_t *  i2c,
    // uint  baudrate
    // )
    // Initialise the I2C HW block.
    // Put the I2C hardware into a known state, and enable it.
    // Must be called before other functions.
    // By default, the I2C is configured to operate as a master.

    // The I2C bus frequency is set as close as possible to requested,
    // and the actual rate set is returned

    // Parameters
    // i2c	Either i2c0 or i2c1
    // baudrate	Baudrate in Hz (e.g. 100kHz is 100000)

}

// https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_i2c

//****************************************************
bool reserved_reg(uint8_t reg) {
    // hung on read of d7 (215)
    // hung on read of d8 (216)
    bool bad = false;
    switch (reg) {
        case 4: ;
        case 5: ;
        case 6: ;
        case 7: ;
        case 8: ;
        case 10: ;
        case 11: ;
        case 12: ;
        case 13: ;
        case 14: ;
        case 173: ;
        case 174: ;
        case 175: ;
        case 176: ;
        case 178: ;
        case 179: ;
        case 180: ;
        case 181: ;
        case 182: bad = true; break;
        // possible range is already constrained to 255 by uint8_t size
        default: if (reg >= 184) bad = true;
    }
    switch (reg) {
        case 0: ; // has some status bits
        case 1: ; bad = true; break; // has some status bits
        // FIX! we should check these rules for bad multisynth?
        // multisynth3 thru multisynth7 ??
        // do invalid numbers write ? what happens?
        // This 8-bit number is the Multisynth6 divide ratio. Multisynth6 divide ratio 
        // can only be even integers greater than or equal to 6. All other divide values are invalid.
        // Si5351B and C are 8-outputs. Si5351a we use is only 3. (there is 8 output version) that's why.
        default: if (reg >= 66 && reg <= 91) bad = true;
    }

    // also say the reg that don't return exact pattern just written are "reserved"
    // terms of this test can't just do write than read and compare data.
    return bad;

}

//****************************************************
int i2cWrite(uint8_t reg, uint8_t val) {  // write reg via i2c
    Serial.printf("i2cWrite START reg %02x val %02x" EOL, reg, val);
    // FIX! shouldn't this be local ? or does it setup data for i2cWriten
    // moved here to be local, and not static (shared) anymore
    // only need length 2!
    uint8_t i2c_buf[2];
    i2c_buf[0] = reg;
    i2c_buf[1] = val;

    int res;
    if (reserved_reg(reg)) {
        // don't want to hang on a reserved reg. so don't send
        Serial.printf("reserved reg %u", reg);

        // make this a unique error to recognize my reserved reg detection
        res = 127;
    }
    else {
        // good examples at:
        // https://github.com/raspberrypi/pico-examples/blob/master/i2c/mpu6050_i2c/mpu6050_i2c.c
        // Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged, no device present.
        Serial.printf("i2cWrite doing i2c_write_blocking() reg %02x val %02x" EOL, reg, val);
        res = i2c_write_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false);
        Serial.print(EOL);
    }

    if (res == 127) ; // my decode for reserved
    else if (res==PICO_ERROR_GENERIC)  Serial.printf("ERROR: i2cWrite() got bad res %d reg %02x val %02x" EOL, res, reg, val);
    else if (res==2) Serial.printf("GOOD: i2cWrite() got good res %d reg %02x val %02x" EOL, res, reg, val);
    else Serial.printf("UNEXPECTED: i2cWRite() got unexpected res %d reg %02x val %02x" EOL, res, reg, val);
    Serial.print(EOL);

    Serial.printf("i2cWrite END reg %02x val %02x" EOL, reg, val);
    return res;
}

//****************************************************
// data returned to val after read of reg from i2c
// hardwired to the si5351 i2c bus/instance and to si5351 addr
// so just reg to specify
// checks for reserved addresses on si5351 and forces error (doesn't send them)
int i2cRead(uint8_t reg, uint8_t *val) {
    // Serial.printf("i2cRead START reg %02x val %02x" EOL, reg, *val);
    // don't get confused by the old values in *val
    Serial.printf("i2cRead START reg %02x" EOL, reg);

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
    uint8_t i2c_buf[2] = { 0 }; // only really need 1 byte
    i2c_buf[0] = 0x73; // just a value I don't expect to see
    if (reserved_reg(reg)) {
        // don't want to hang on a reserved reg. so don't send
        Serial.printf("reserved reg %u", reg);
        // make this a unique error to recognize my reserved reg detection
        res = 127;
    }
    else {
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
            res = 1; // good one byte read! both parts
    }

    // cover all - errors above
    if (res == 127) ; // my decode for reserved
    else if (res == PICO_ERROR_GENERIC || res < 0)
        Serial.printf("ERROR: i2cRead() got bad res %d reg %02x val %02x" EOL, res, reg, *val);
    else if (res==1)
        Serial.printf("GOOD: i2cRead() got good res %d reg %02x val %02x" EOL, res, reg, *val);
    else
        Serial.printf("UNEXPECTED: i2cRead() got unexpected res %d reg %02x val %02x" EOL, res, reg, *val);
    Serial.print(EOL);

    // FIX! no expected data compare. I just eyeball the sequential write/read in the serial monitor
    // without knowing the reg, it may be okay that some bits differ between write and read
    // especially since I'm writing random

    Serial.printf("i2cRead END reg %02x val %02x" EOL, reg, *val);
    return res;
}
//****************************************
static uint8_t reg_addr = 0;
void loop() {
    while (true) {
        Serial.printf("while loop %d start" EOL, reg_addr);
        if (reg_addr > 183) { // 183 is the last valid reg in the datasheet
            Serial.print(EOL);
            Serial.print("start over at 0" EOL);
            Serial.print(EOL);
            reg_addr = 0;
            break;
        }
        int result;

        bool bad = false;
        Serial.print(EOL);
        Serial.print("i2cWrite() start" EOL);
        // result = i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff) ;
        int random_data = random(0,255);
        sleep_ms(200);
        result = i2cWrite(reg_addr, (uint8_t) random_data) ;

        Serial.print("i2cWrite() complete" EOL);
        if (result == 127) {  // my code
            Serial.printf("BAD: reserved reg_addr %d detected on i2cWrite" EOL, reg_addr);
            bad = true;
        } else if (result == 2) { // 2 bytes on the write!
            Serial.printf("GOOD: result %d after i2cWrite" EOL, result);
            bad = false;
        } else if (result == PICO_ERROR_GENERIC) {
            Serial.printf("ERROR: result %d after i2cWrite" EOL, result);
            bad = true;
        } else {
            Serial.printf("UNEXPECTED: result %d after i2cWrite" EOL, result);
            bad = true;
        }

        uint8_t val = 0xaa;
        if (!bad) {
            Serial.print(EOL);
            Serial.print("i2cRead() start" EOL);
            sleep_ms(200);
            result = i2cRead(reg_addr, &val) ;
            Serial.print("i2cRead() complete" EOL);

            if (result == 127) { // my code
                Serial.printf("BAD: reserved reg_addr %d detected on i2cRead" EOL, reg_addr);
                bad = true;
            } else if (result == 1) { // 1 byte!
                Serial.printf("GOOD: result %d after i2cRead" EOL, result);
                bad = false;
            } else if (result == PICO_ERROR_GENERIC) {
                Serial.printf("ERROR: result %d after i2cRead" EOL, result);
                bad = true;
            } else {
                Serial.printf("UNEXPECTED: result %d after i2cRead" EOL, result);
                bad = true;
            }
        }
        if (!bad && (result != 127) ) { 
            if (val != (uint8_t) random_data) {
                Serial.printf("BAD WR/RD COMPARE: reg_addr %u random_data 0x%02x  val 0x%02x" EOL, 
                    reg_addr, random_data, val);
                sleep_ms(7000);
            }
            else {
                Serial.printf("GOOD WR/RD COMPARE: reg_addr %u random_data 0x%02x  val 0x%02x" EOL, 
                    reg_addr, random_data, val);
            }
        }

            
        //******************************************
        if (!bad || result == 127 ) { // don't repeat reserved addresses
    
            Serial.print(EOL);
            reg_addr = reg_addr + 1;
            Serial.printf("incremented reg_addr %u" EOL, reg_addr);
            Serial.print(EOL);
            sleep_ms(2000);
            break;
        }

        // so we can see what went bad in the serial output
        sleep_ms(2000);

        //******************************************
        Serial.print(EOL);
        Serial.println("i2c_deinit() start" EOL);
        i2c_deinit(VFO_I2C_INSTANCE);
        Serial.println("i2c_deinit() complete" EOL);
        busy_wait_ms(1000);

        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
        Serial.println("i2c_init() start" EOL);
        i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
        Serial.println("i2c_init() complete" EOL);

        gpio_init(VFO_I2C0_SCL_PIN);
        gpio_init(VFO_I2C0_SDA_PIN);

        Serial.println("no pullup or pulldowns by RP2040 on both SDA and SCL.. external R on pcb");
        gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
        gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

        // gpio_pull_up(VFO_I2C0_SDA_PIN);
        // gpio_pull_up(VFO_I2C0_SCL_PIN);

        Serial.println("i2c_set_function() start" EOL);
        gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);

        // default init is master. don't need
        // i2c_set_slave_mode(VFO_I2C_INSTANCE, false, 0);
        Serial.println("i2c_set_function() complete" EOL);
        digitalWrite(Si5351Pwr, HIGH);

        // power the vfo off/on
        digitalWrite(Si5351Pwr, HIGH);
        busy_wait_ms(1000);
        digitalWrite(Si5351Pwr, LOW);
        busy_wait_ms(2000);

        Serial.print(EOL);
        Serial.printf("while loop end %d" EOL, reg_addr);

    }
}
// nice library here with timeouts and write+read methods
// https://github.com/rambo/I2C


// rfzero Wire* library for write and read
// I supose that's not too hard?

// https://rfzero.net/tutorials/si5351a/
//
// The I2C (Wire) read function.
//
// uint8_t ReadRegister(uint8_t regAddr)
// {
//     int data = 0xFF;                  // Set value often not seen
//
//     Wire.beginTransmission(0x60);     // The I2C address of the Si5351A
//     Wire.write((uint8_t)regAddr);
//     Wire.endTransmission();
//     Wire.requestFrom(0x60, 1);
//     if (Wire.available())
//         data = Wire.read();
//
//     return data;
// }
// The I2C (Wire) write function.
//
// void WriteRegister(uint8_t regAddr, uint8_t data)
// {
//     Wire.beginTransmission(0x60);     // The I2C address of the Si5351A
//     Wire.write((uint8_t) regAddr);
//     Wire.write((uint8_t) data);
//     Wire.endTransmission();
// }
// For the RFzero the Wire instance must be replaced by WireLocal.
