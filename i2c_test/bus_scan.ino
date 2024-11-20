// source: https://raw.githubusercontent.com/raspberrypi/pico-examples/refs/heads/master/i2c/bus_scan/bus_scan.c
// https://github.com/raspberrypi/pico-examples/blob/master/i2c/bus_scan/bus_scan.c
// Modified: Kevin Normoyle AD6Z 11/2024

// Runs standalone on AG6NS pcb with kbn or kbn2 components.
// The AG6NS pcb has two i2c. Neither have pullups, so the RP2040 does the pullup
// This might limite the i2c speed, because the RP2040 pullup is weak?

// we're not going to use the newer Wire() stuff
// going to just the older style

// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
// https://arduino-pico.readthedocs.io/en/latest/wire.html
// https://docs.arduino.cc/language-reference/en/functions/communication/wire/

// RP2040 has i2c0 and i2c1
// have to use the pins labeled I2C0 for Wire
// have to use the pins labeleds I2C1 for Wire1


// copy this into defines.h
#define BALLOONFLYING 1

#ifdef BALLONFLYING
#define xPrint(x);
#define xPrintf(x);
#define xPrintLn(x);
#else
#define xPrint(x); Serial.print(x);
#define xPrintf(x); Serial.printf(x);
#define xPrintLn(x); Serial.println(x);
#endif

// if BALLONFLYING and we get voltage<=4.9v, turn off
// turn on serial again and reboot into bootloader mode, to reload firmware
// you might want the non-flying firmware to be loaded
// print a message that we're rebooting because BALLONFLYING code and voltage > 4.9v
// hmm you wouldn't want to do that if solar voltage in the sky is > 4.9v
// so really want to detect the usb data pin voltages

// reducing arduino uno power
// https://www.defproc.co.uk/tutorial/how-to-reduce-arduino-uno-power-usage-by-95/

// how to detect usb power
// There's a signal from USB input that goes to GPIO24 via a potential divider (to create 3.3V logic levels).
// Don't plug in USB while solar panel is connected?
// you need a diode between your external power supply and VSYS, to prevent USB power trying to power the rest of your circuit.


// UPDATE: AG6NS does have 3.3k and 10k pullups on Si5351 and BMP i2c, respectively
// so don't use pullups from the RP2040

//************************************************************
// hmm. why is \n not sufficient?
// for printing
#define EOL "\r\n"

// driving LOW enables power. HIGH disables power
// we'll drive the default 4ma
#define Si5351Pwr 4

#define VFO_ISC isc0
#define VFO_SDA_PIN 12;
#define VFO_SCL_PIN 13;

#define BMP_ISC isc1
#define BMP280_SDA_PIN 2
#define BMP280_SCL_PIN 3

// For these:
// default=Usually provided via board header, group=hardware_i2c
// PICO_CONFIG: PICO_DEFAULT_I2C, Define the default I2C for a board, min=0, max=1,
// PICO_CONFIG: PICO_DEFAULT_I2C_SDA_PIN, Define the default I2C SDA pin, min=0, max=29
// this was replaced by use of constants so we could loop over both buses

// Are we affecting these defaults with this:
// we loop over both at runtime now, so this shouldn't matter?

// see this for use of the PICO_DEFAULT_*
// https://github.com/raspberrypi/pico-sdk/blob/master/src/boards/include/boards/adafruit_qtpy_rp2040.h
// thse are already defined by someone
// #define PICO_DEFAULT_I2C_SDA_PIN ..
// #define PICO_DEFAULT_I2C_SCL_PIN ..
// #define PICO_DEFAULT_I2C ..

// 100000 is probably the max we can do on either i2c bus?
#define PICO_I2C_CLK_HZ = 100000

//*************************************
// rp2040. may be useful reference
// https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2040

// examaple
// https://github.com/raspberrypi/pico-examples/blob/master/i2c/bmp280_i2c/bmp280_i2c.c

// The Raspberry Pi Pico SDK
// https://github.com/raspberrypi/pico-sdk

// Reference for parameters and return values
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/include/hardware/i2c.h

// uint i2c_init(i2c_inst_t *i2c, uint baudrate);
// void i2c_deinit(i2c_inst_t *i2c);
// uint i2c_set_baudrate(i2c_inst_t *i2c, uint baudrate);
// void i2c_set_slave_mode(i2c_inst_t *i2c, bool slave, uint8_t addr);
// int i2c_write_blocking_until(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop, absolute_time_t until);
// int i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, absolute_time_t until);
// int i2c_write_timeout_per_char_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop, uint timeout_per_char_us);
// int i2c_read_timeout_per_char_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint timeout_per_char_us);
// int i2c_write_blocking(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop);
// int i2c_read_blocking(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop);

// PICO_ERROR_GENERIC used for errors

// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/include/hardware/i2c.h
// they do this..do I already get it here somehow..?
// #include "hardware/structs/i2c.h"

// do we need this for any reason? or is it in the hardware/structs/i2c.h ?
// typedef struct i2c_inst i2c_inst_t;

// PICO_CONFIG: PICO_DEFAULT_I2C_SCL_PIN, Define the default I2C SCL pin, min=0, max=29

// the two i2c instances
// we don't need to do this, but you see how it goes:
// per
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/include/hardware/i2c.h#L204

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
//**************

// runtime resolutions
// Get hardware instance number,  from I2C instance
// uint i2c_get_index(i2c_inst_t *i2c)

// get pointer to structure containing i2c hardware registers, from the I2C instance.
// i2c_hw_t *i2c_get_hw(i2c_inst_t *i2c) {

// Get I2C instance from I2C hardware instance number
// i2c_inst_t *i2c_get_instance(uint num)

// The external pins of each controller are connected to GPIO pins as defined in the GPIO muxing table in the datasheet.
// The muxing options  give some IO flexibility, but each controller external pin should be connected to only one GPIO.

// Note that the controller does NOT support High speed mode or Ultra-fast speed mode,
// the fastest operation being fast mode plus aat up to 1000Kb/s.


//*******************************************************************************
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
// is this only needed for picotool?
// #include "pico/binary_info.h"

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int scan_i2c(int i2c_number) {
    Serial.printf(EOL "scan_i2c(%d) START" EOL, i2c_number);
    // power up the Si5351 if we're scanning that bus
    // FIX! what about the BMP..it's always on!
    if (i2c_number == 0) {
        Serial.println("power on Si5351");
        gpio_init(Si5351Pwr);
        pinMode(Si5351Pwr, OUTPUT_4MA);
        digitalWrite(Si5351Pwr, LOW);
    }

    // if we're running without USB_CONNECTED, it must be loaded/running with solar power
    // ideally with BALLON_FLYING firmware? BALLOON_FLYING firmware can be connected
    // to usb for power, but won't have any usb serial input/output capabilities?
    // (does power savings)
    // have to bootsel/power cycle, to reload non BALOON_FLYING software.
    while (!USB_CONNECTED) { ; }


    // FIX! doesn't arduino-pico core do this? so not necessary
    // stdio_init_all();
    // but we do have to wait until it's there
    while (!Serial) { ; }

    // wait 1 sec to be sure Si5351 has powered up
    sleep_ms(1000);


    // Get I2C instance from I2C hardware instance number
    // i2c_inst_t *i2c_get_instance(uint num)
    i2c_inst_t i2c_instance_to_test = i2c_get_instance(i2c_number);


    //**********************************************
    // I suppose could set them both up at once, rather than conditional

    // deinit both! Just to make sure we're only using the one we expect
    Serial.println("Deinit both isc0 and isc1");
    i2c_deinit(isc0);
    i2c_deinit(isc1);

    // only init the one we're testing
    Serial.printf("Only init the isc we're testing:  %d with %d Hz rate" EOL, i2c_number, PICO_I2C_CLK_HZ);
    // uint i2c_init (i2c_inst_t * i2c, uint baudrate)
    i2c_init(i2c_instance_to_test, PICO_I2C_CLK_HZ);

    const int SDA_pin;
    const int SCL_pin;
    if (i2c_number_to_test==0) {
        sda_pin = VFO_SDA_PIN;
        scl_pin = VFO_SCL_PIN;
    }
    else {
        sda_pin = VFO_SDA_PIN;
        scl_pin = VFO_SCL_PIN;
    }

    Serial.printf("SDA is pin %d" EOL, sda_pin);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    Serial.printf("SCL is pin %d" EOL, scl_pin);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    Serial.println("no pullup or pulldowns by RP2040 on both SDA and SCL");
    gpio_set_pulls(scl_pin, false, false);
    gpio_set_pulls(sda_pin, false, false);

    //**********************************************

    // from the orig. code.
    // Make the I2C pins available to picotool
    // bi_decl(bi_2pins_with_func(PICO_I2C_SDA_PIN, PICO_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf(EOL "I2C Bus Scan" EOL);
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F" EOL);

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy blocking read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(PICO_I2C, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }

    if (i2c_number == 0) {
        Serial.println("power off Si5351");
        gpio_init(Si5351Pwr);
        pinMode(Si5351Pwr, OUTPUT_4MA);
        digitalWrite(Si5351Pwr, LOW);
    }

    //**********************************************
    printf("scan_i2c(%d) END" EOL, i2c_number);
    return 0;
}

//**********************************************
loop() {
    Serial.println(F(EOL "Will scan isc0 (si5351 should be only device)" EOL);
    scan_i2c(0);

    Serial.println(F(EOL "Will scan isc1 (bmp280 should be only device)" EOL);
    scan_i2c(1);

    Serial.println("Deinit both isc0 and isc1");
    i2c_deinit(isc0);
    i2c_deinit(isc1);
    Serial.println("left sda/scl's as defined with pullups");
    Serial.println("power off Si5351");
    gpio_init(Si5351Pwr);
    pinMode(Si5351Pwr, OUTPUT_4MA);
    digitalWrite(Si5351Pwr, LOW);

}
//********************************************************************
// Things to do:
// It would be nice to turn off the usb pll when we know we're flying, to save power.
// would have to turn it off, right before we cut off the usb connector?
// Could we have a pin to detect, to decide whether usb should be powered up?
// can we detect usb voltage? (like if the voltage is below 4.5v, we must not be on usb)
// and can turn it off?


// we'd want to have a compile that gets rid of all Serial references
// i.e. load up a new firmware with no print

#define DEBUG 1

#if DEBUG==1
#define outputDebug(x); Serial.print(x);
#define outputDebugLine(x); Serial.println(x);
#else
#define outputDebug(x);
#define outputDebugLine(x);
#endif


// we could turn off usb pll if we're running with DEBUG disabled and the voltage is lower than 4.5v?
// or, if we have no debug print, disable usb serial input too
// don't look at voltage for either?

// A simple way is to monitor voltage coming from usb.
// But if you want to specifically detect data connection and ignore cases when power-only cable is plugged,
// that won't work

// https://www.reddit.com/r/arduino/comments/pwbq70/detect_usb_connection/

// what about USB_DP and USB_DM pins
// https://github.com/raspberrypi/pico-examples/tree/master/usb

// void setup() {
//   Serial.begin(115200); // Initialize serial communication
// }
//
// void loop() {
//   if (USB_CONNECTED) {
//     Serial.println("USB Connected!");
//   } else {
//     Serial.println("USB Disconnected!");
//   }
//   delay(1000); // Delay for readability
// }

// #include <usb_dev.h>
// void loop() {
//   Serial.println("USB Status: ");
//   while (1) {
//     if (USB_Status() == 1) {
//       Serial.println("USB Connected");
//     }
//     else {
//       Serial.println("You can't read this");
//     }
//     delay(500);
// }
// }
//
// byte USB_Status() {
//   // this only works in the Teensy core
//   // what about the arduino-pico core
//   if (usb_configuration != 0)
//     return 1;
//     else {
//     return 0;
//   }
// }

// When the USB is removed then the Serial call will return false.
// Just like this bit of code is waiting for one to be established before continuing :-
//  while (!Serial)
//      delay(10);

// usb_configuration doesn't seem to return to zero after connection, so a cycle or break is needed,
// but other than that, it works!



// Lowering core voltage:
// hmm. what is vreg_disable_voltage_limit() and vreg_set_voltage()
// sleep and dormant modes are not supported by arduino-pico core (earlephilhower core)

// Another easy way to reduce consumption, what I found out today, is to reduce the core voltage.
// I'm using the Pico at 50 MHz and can go as low as 0.90V and it saves me about 2mA compared to stock
// 50 mhz freezes with 2 cores

// Works for me, even down to about 20 MHz. Although, at this point the USB connection drops out,
// but the Pico still works otherwise.
// With 50 MHz, even the serial USB connections works like normal for me.

// Reported: sleep consumption even lower, from 8ma down to 3ma.
// Most improvement by halting the USB PLL during sleep and restarting it on wakeup.
// This drives the other main clock besides clock_sys.
// So far, this seems reliable, and it saved about 4ma.

// to sleep:
//   Serial.end();
//   pll_deinit(pll_usb);

// to wake:
//   pll_init(pll_usb, 1, 1440000000, 6, 5); // return USB pll to 48mhz
//   // High-level Adafruit TinyUSB init code, does many things to get the USB controller back online
//   tusb_init();
//   Serial.begin(115200);
//
// I'm pretty sure I'm not allowed to drop the sys_clk speed any lower than 18mhz with my 12mhz xtal,
// but I did turn the RPI's internal voltage regulator from 1.1v down to 0.9v during sleep,
// which saved maybe another 0.5ma. (This is done after slowing the system clock.)
//
// to sleep:
// vreg_set_voltage(VREG_VOLTAGE_0_90 ); // 0_85 crashes for me. YMMV.

// to wake:
// vreg_set_voltage(VREG_VOLTAGE_1_10);

// Putting my flash chip into power-down mode seems to save a tiny scrap of power;
// datasheet says 0.1-0.5ma .
// YMMV. Consult your flash chip datasheet and the sdk doc for flash_do_cmd()
// With this, current consumption during sleep is down to 0.003A on my ammeter.
//
// I still haven't tried powering down SRAM. I'm not sure how to power down
// only the SRAM that I'm not using.
// Also I'm not sure how to estimate the potential power savings.
// But I'd love to hear about anybody else's experiments with that!
