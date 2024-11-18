// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

//*******************************************
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
// do we need these two for  getchar_timeout_us() (return type is int
// #include "pico/stdlib.h"
// so we can use stdio_usb
#include "pico/stdio.h"

// what about this?
// #include "class/cdc/cdc_device.h"

#include <stdlib.h>
#include <string.h>
#include <avr/dtostrf.h>

#include "defines.h"

// These are in arduino-pio core
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries
#include <SPI.h>
#include <Wire.h>

// A complete copy of the Raspberry Pi Pico SDK is included with the arduino-pico core,
// and all functions in the core are available inside the standard link libraries.
// When you call SDK functions, the core and libraries are not aware of any changes 
// to the Pico you perform. This may break the functionality of certain libraries in doing so.

// Be wary of multicore and use of libraries. arduino-pico is not thread-safe.

// The Arduino-Pico core implements a software-based Serial-over-USB port
// using the USB ACM-CDC model to support a wide variety of operating systems.
// Serial is the USB serial port, and while Serial.begin() does allow specifying a baud rate,
// this rate is ignored since it is USB-based.

// Also be aware that this USB Serial port is responsible forresetting the RP2040 
// during the upload process,
// following the Arduino standard of 1200bps = reset to bootloader).

// The RP2040 provides two hardware-based UARTS with configurable pin selection.
// Serial1 is UART0, and Serial2 is UART1.

// The RP2040 chip has 2 cores that can run independently of each other,
// sharing peripherals and memory with each other.
// Arduino code will normally execute only on core 0,
// with the 2nd core sitting idle in a low power state.
// By adding a setup1() and loop1() function to your sketch you can make use of the second core.
// Anything called from within the setup1() or loop1() routines will execute on the second core.
// https://arduino-pico.readthedocs.io/en/latest/multicore.html

// Using the 2nd core to handle keyboard interrupts and entry to config coe
// core0 will stop balloon processing at an appropriate boundary which
// may be longer than optimal for quick keyboard response.
// changes to the config state may have atomicity issues, but the ballon code will
// always be rebooted when done, as if to start from scratch after config changes

// #include <ctype.h>
// #include <defines.h>
// #include "pico/stdlib.h"
// #include "hardware/clocks.h"
// #include "hardware/gpio.h"
// #include "hardware/adc.h"
// #include "hardware/clocks.h"



//**************************
// flash config
#include "hardware/flash.h"

//**************************
#include <TinyGPS++.h>  // https://github.com/mikalhart/TinyGPSPlus
// gets the head 1.1-beta? not released version
// wget https://github.com/mikalhart/TinyGPSPlus/archive/refs/heads/master.zip

/* v1.0.a
TinyGPSPlus is a new Arduino library for parsing NMEA data streams provided by GPS modules.

1.1-beta update: Several pull requests incorporated

Added Fix Quality and Fix Mode
Slight change to earth radius
Support all satellite groups

Provides compact and easy-to-use methods for extracting position,
date, time, altitude, speed, and course from consumer GPS devices.

TinyGPSPlus’s api is considerably simpler to use than TinyGPS
Can extract arbitrary data from any of the myriad NMEA sentences out there.
*/
//**************************

// got latest LightAPRS_Geofence from https://github.com/kaduhi/LightAPRS-W-2.0
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker/libraries/LightAPRS_Geofence
// libraries/LightAPRS_Geofence:
// wget https://raw.githubusercontent.com/kaduhi/LightAPRS-W-2.0/refs/heads/port_to_ag6ns_rp2040_picoballoon_tracker/libraries/LightAPRS_Geofence/GEOFENCE.cpp
// wget https://raw.githubusercontent.com/kaduhi/LightAPRS-W-2.0/refs/heads/port_to_ag6ns_rp2040_picoballoon_tracker/libraries/LightAPRS_Geofence/GEOFENCE.h
// Modified version of https://github.com/TomasTT7/TT7F-Float-Tracker/blob/master/Software/ARM_GEOFENCE.c
// was used in https://github.com/TomasTT7/TT7F-Float-Tracker
// #include <GEOFENCE.h>

// this is included in gps_functions.cpp? shouldn't be needed here or maybe need for Watchdog.* ?
// in libraries: wget https://github.com/adafruit/Adafruit_SleepyDog/archive/refs/heads/master.zip
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// needed for BMP085.h Uses Adafruit_I2CDevice.h and .cpp
// added Adafruit_BusIO to our repo 11_7_24 (libraries)
// in libraries: wget https://github.com/adafruit/Adafruit_BusIO/archive/refs/heads/master.zip
#include <Adafruit_I2CDevice.h>  // https://github.com/adafruit/Adafruit_BusIO

// board uses the BMP280? (doesn't have temp)
// in libraries: wget https://github.com/adafruit/Adafruit_BMP280_Library/archive/refs/heads/master.zip
// can I put the BMP085 on there?
#include <Adafruit_BMP280.h>  // https://github.com/adafruit/Adafruit_BMP280_Library
// this needs the Adafruit_Sensor.h also
// in libraries: wget https://github.com/adafruit/Adafruit_Sensor/archive/refs/heads/master.zip
// https://github.com/adafruit/Adafruit_sensor
// they have accelerometer, gyroscope, magnetometer, humidity sensors, light?

// don't need BMP085 ?
// Requires the https://github.com/adafruit/Adafruit_BusIO library for I2C abstraction
// #include <Adafruit_BMP085.h> // https://github.com/adafruit/Adafruit-BMP085-Library

// wget https://github.com/adafruit/Adafruit-BMP085-Library/archive/refs/heads/master.zip
#include <JTEncode.h>  // https://github.com/etherkit/JTEncode (JT65/JT9/JT4/FT8/WSPR/FSQ Encoder Library)
// in libraries: wget https://github.com/etherkit/JTEncode/archive/refs/heads/master.zip

// setTime() use moved to gps_functions.cpp
// libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip

// do we use minute() here?
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time

#include <MemoryFree.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/
#include "hardware/pwm.h"

// lots of things used from Arduino-Pico core
// https://arduino-pico.readthedocs.io/en/latest/rp2040.html

// const is typed, #define macros are not.
// const is scoped by C block, #define applies to a file (compilation unit)
// const is most useful with parameter passing.
// If you see const used on a prototype with pointers, it is safe to pass your array or struct
// because the function will not alter it.
// No const and it can.


// The RP2040 provides two hardware-based UARTS with configurable pin selection.
// Serial1 is UART0, and Serial2 is UART1.
// FIX! should we get a bigger buffer to avoid missing ATGM336 NMEA sentences?
// The size of the receive FIFO can be adjusted from the default 32 bytes with
// setFIFOSize call prior to calling begin()
//     Serial1.setFIFOSize(128);
//     Serial1.begin(baud);
//
// The FIFO is normally handled via an interrupt.
// For applications where an IRQ driven serial port is not appropriate,
// use setPollingMode(true) before calling begin()
//     Serial1.setPollingMode(true);

// extern so it links okay?
extern const int Si5351Pwr = 4;
extern const int BattPin = A3;

//******************************* CONFIG **********************************
// FIX! are these used now? remnants of APRS messaging
char comment[] = "tracker 1.0";

// extern so it links okay if we move stuff
extern const int WSPR_TONE_SPACING = 146;  // ~1.46 Hz
extern const int WSPR_DELAY = 683;         // Delay value for WSPR

char telemetry_buff[100] = { 0 };  // telemetry buffer
uint16_t Tx_0_cnt = 0;  // increase +1 after every callsign tx
uint16_t Tx_1_cnt = 0;  // increase +1 after every telemetry tx
uint16_t Tx_2_cnt = 0;  // increase +1 after every telen1 tx
uint16_t Tx_3_cnt = 0;  // increase +1 after every telen2 tx

// Global variables
// FIX! the tx_buffer doesn't need so much reserved space
// The maximum number of binary channel symbols in a WSPR message is 162.
// This is calculated by adding the constraint length (K) of 32 to the
// total number of bits in a standard message (50), and then multiplying by 2.

// the library agrees
// JTEncode.h:#define WSPR_SYMBOL_COUNT 162

// background
// https://hackaday.io/project/166875-careless-wspr/log/167301-encoding-wsprs
// http://www.g4jnt.com/Coding/WSPR_Coding_Process.pdf

// useful python code for playing around with wspr encoding
// https://github.com/robertostling/wspr-tools


// below we always loop thru the entire tx_buffer?
// so we always loop thru 162 symbols, but the last ones might not matter.
uint8_t symbol_count;  // is this less than 256? probaby the real max?

uint8_t tx_buffer[255];  // is this bigger than WSPR_SYMBOL_COUNT?
uint16_t tone_delay, tone_spacing;
// FIX! why is this volatile?
volatile bool proceed = false;

//******************************  GPS SETTINGS   *********************************
int GpsInvalidCnt = 0;

// gps_functions.cpp refers to this
TinyGPSPlus gps;

#include "debug_functions.h"
#include "config_functions.h"
#include "tele_functions.h"
#include "mh_functions.h"
#include "adc_functions.h"

//*********************************
// in AdaFruit_I2CDevice.h
// extern so it links okay
extern const int BMP280_I2C1_SDA_PIN = 2;
extern const int BMP280_I2C1_SCL_PIN = 3;
#include "bmp_functions.h"

Adafruit_BMP280 bmp;

JTEncode jtencode;

//*********************************
// all extern consts can be externed by a function
extern const int STATUS_LED_PIN = 25;
extern const int LED_STATUS_NO_GPS = 1;
extern const int LED_STATUS_GPS_TIME = 2;
extern const int LED_STATUS_GPS_FIX = 3;
extern const int LED_STATUS_TX_WSPR = 4;
extern const int LED_STATUS_TX_TELEMETRY = 5;
extern const int LED_STATUS_TX_TELEN1 = 6;
extern const int LED_STATUS_TX_TELEN2 = 7;
extern const int LED_STATUS_REBOOT_NO_SERIAL = 8;
extern const int LED_STATUS_USER_CONFIG = 9;

#include "led_functions.h"
#include "keyboard_functions.h"

//*********************************
// some stuff on using namespace
// https://forum.arduino.cc/t/using-a-constant-defined-in-the-header-file/380178

// flash background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

//*********************************
// all extern consts can be externed by a function
// so it can be used in gps_functions.cpp
// extern is needed or the linker doesn't find it. 
// see https://forum.arduino.cc/t/linker-problems-with-extern-const-struct/647136/2
extern const int GpsPwr = 16; // output ..this cuts VCC, leaves VBAT
// define is not used..GpsPwr is used.
// const int GPS_VCC_ON_N_PIN=16;

extern const int GPS_NRESET_PIN = 5;
extern const int GPS_ON_PIN = 6;

// FIX! where is this used (calibration maybe)
extern const int GPS_1PPS_PIN = 17;   // input

extern const int GPS_UART1_TX_PIN = 8;
extern const int GPS_UART1_RX_PIN = 9;

// talks to gps. can't really make the hardware uart fifo size bigger
extern const int SERIAL2_FIFO_SIZE = 32;
// earlephilhower says the hw serial units use the hardware rx fifo
// so only 32?

// legal choices? 9600 (default ATGM33651) 19200 38400
// is in receive fifo 12-bit wide default 32 deep ??
// https://arduino-pico.readthedocs.io/en/latest/serial.html

// hmm sometimes have to restart Arduino IDE when it gets stuck in
// bad state after changing baud rates
// is my full cold reset not working to gps and vbat is remembering the baud?
// so I can't talk to him without a full power off (vcc and vbat?)
// AG6NS board can't turn off VBAT under progam control

// Update: we use the gps resetn pin now, for full cold reset to revert
// baud to 9600 ALWAYS at init (then set desired baud rate
// created GpsWarmReset() and GpsColdReset()
// GpsWarmReset is the normal "just power" on/off management that keeps vbat on

// works
// see gps_functions.cpp and problems with resetting to 9600 from higher bauds
// too dangerous to use anything higher than 9600..could get stuck
// with out vcc plus vbat power cycle.. I can't control vbat when on usb power
// extern const int SERIAL2_BAUD_RATE = 9600;
// can't seem to restrict burst data
// with the GLONASS sats
// There are 821 chars in a burst, handling takes 892 milliseconds per burst
// try increasing the baud rate to see if duration takes less 
// works going from 9600 to 19200
extern const int SERIAL2_BAUD_RATE = 19200;
// it now does the burst in half that time, around 450 milliseconds
// so this is good..11/18/24

// FIX! recheck this
// does it work  going back from 19200 to 9600 without unplugging usb power?
// now it does! after asserting NRESET after power on, (during also)

// can't seem to recover to 9600 after trying 38400? full cold reset not working?
// need to unplug usb to get back to 9600
// doesn't work now?
// extern const int SERIAL2_BAUD_RATE = 38400;

// this is too fast. I can't keep up with incoming data
// GPS burst: duration is 165 millisecs for 646 chars. 3954 baud effective
// extern const int SERIAL2_BAUD_RATE = 57600;

// don't use. rx buffer overruns
// extern const int SERIAL2_BAUD_RATE = 115200;

// stuff moved to functions from this .ino (not libraries)
#include "gps_functions.h"

//*********************************
// all extern consts can be externed by a function
// when we set both?
extern const int WSPR_TX_CLK_1_NUM = 1;
// this is the other differential clock for wspr? (was aprs)
extern const int WSPR_TX_CLK_0_NUM = 0;
extern const int WSPR_TX_CLK_NUM = 0;

extern const int SI5351A_CLK_IDRV_8MA = (3 << 0);
extern const int SI5351A_CLK_IDRV_6MA = (2 << 0);
extern const int SI5351A_CLK_IDRV_4MA = (1 << 0);
extern const int SI5351A_CLK_IDRV_2MA = (0 << 0);

extern const int PLL_CALCULATION_PRECISION = 4;

extern const int VFO_VDD_ON_N_PIN = 4;
extern const int VFO_I2C0_SDA_PIN = 12;
extern const int VFO_I2C0_SCL_PIN = 13;

// FIX! should these be in tracker.ino (for consistency?)
extern const int SI5351A_I2C_ADDR = 0x60;
extern const int VFO_I2C0_SCL_HZ = (1000 * 1000);

#include "si5351_functions.h"

//*********************************
#include "u4b_functions.h"
#include "tele_functions.h"

// telemetry_buff all can be extern'ed by a function
// init to 0 is just in case. Should always be set to something valid before use
// empty string is not valid (not sure what will happen if used while empty..I suppose it can print ok)
// always positive. clamp to 0 I guess
char t_course[4] = { 0 };      // 3 bytes + null term (like all here
// always positive? 0-250 knots. clamp to 0 I guess
char t_speed[4] = { 0 };       // 3 bytes
// 60000 meters. plus 1 in case negative?
char t_altitude[7] = { 0 };    // 6 bytes
// 24 * 30 per hour = 720 per day if every two minutes
// reboot once per day? (starts at 0)
char t_tx_count_0[4] = { 0 };  // 3 bytes
char t_temp[7] = { 0 };        // 6 bytes
char t_pressure[8] = { 0 };    // 7 bytes
char t_temp_ext[8] = { 0 };    // 7 bytes
char t_humidity[8] = { 0 };    // 7 bytes
char t_voltage[6] = { 0 };     // 5 bytes
char t_sat_count[3] = { 0 };   // 2 bytes
// lat/lon precision: How much to store
// https://stackoverflow.com/questions/1947481/how-many-significant-digits-should-i-store-in-my-database-for-a-gps-coordinate
// 6 decimal places represent accuracy for ~ 10 cm
// 7 decimal places for ~ 1 cm
// The use of 6 digits should be enough. +/- is 1 more. decimal is one more. 0-180 is 3 more.
// so 7 + 5 = 12 bytes should enough, with 1 more rounding digit?
char t_lat[13] = { 0 };        // 12 bytes
char t_lon[13] = { 0 };        // 12 bytes

char t_callsign[7] = { 0 };
char t_grid6[7] = { 0 };       // 6 bytes
char t_power[3] = { 0 };       // 2 bytes

int t_snap_cnt = 0;

//***********************************************************
// config strings: all can be extern'ed by a function
// see config_functions.cpp
// these get set via terminal, and then from NVRAM on boot
// init with all null
char _callsign[7] = { 0 };
char _suffix[2] = { 0 };
char _verbose[2] = { 0 };
char _TELEN_config[5] = { 0 };
char _clock_speed[4] = { 0 };
char _U4B_chan[4] = { 0 };
char _Band[3] = { 0 };     // string with 10, 12, 15, 17, 20 legal. null at end
char _tx_high[2] = { 0 };  // 0 is 2mA si5351. 1 is 8mA si5351
char _devmode[2] = { 0 };
char _correction[7] = { 0 };
char _go_when_rdy[2] = { 0 };

//*****************************
// decoded stuff from config strings: all can be extern'ed by a function
// decodes from _Band _U4B_chan
// 0 should never happen for XMIT_FREQUENCY
uint32_t XMIT_FREQUENCY = 0;
char _id13[3] = { 0 };
char _start_minute[2] = { 0 };
char _lane[2] = { 0 };

// decode of _clock_speed
uint32_t PLL_SYS_MHZ = 133;

// decode of _devmode
bool DEVMODE = false;
// decode of _verbose 0-9
bool VERBY[10] = { false };
//*****************************

// t_power is clamped to string versions of these. use 0 if illegal
// int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60}
// ad6z will differentiate from u4b (10) and traquito (13) by using 3 or 7 in normal callsign tx
// 3 for low power, 7 for high power

// char and strings:
// A string is a pointer to an array of chars,
// by convention is terminated with a NUL character ('\0').

// A char is a single byte representing an ASCII character.
// You create a string literal by compiling a series of characters
// between opening and closing double quotes:
// "this is a string literal"

// You create a char literal by compiling a single character expression -
// possibly with a backslash escape - between a pair of single quotes

//*********************************
absolute_time_t loop_us_start = 0;
absolute_time_t loop_us_end = 0;
absolute_time_t GpsStartTime = 0;

int64_t loop_us_elapsed;
int64_t loop_ms_elapsed;

uint64_t GpsFixMillis = 0;
uint64_t GpsStartMillis = 0;

uint64_t  loopCnt = 0;

//***********************************************************
// FIX! should this be non-zero? 
// Maybe all a don't care now with the voltage monitor that causes reset.
float     GpsMinVolt = 0.0;   // min Volts for GPS to wake up.
float     BattMin = 0.0;      // min Volts to wake up.
float     WsprBattMin = 0.0;  // min Volts for HF (WSPR) radio module to transmit (TX) ~10 mW
// GPS is always on if the voltage exceeds this value to protect solar caps from overcharge
float     HighVolt = 9.9;


//***********************************************************
// To allocate a separate 8K stack for core 1, 
// resulting in 8K stacks being available for both cores, 
bool core1_separate_stack = true;

//***********************************************************
// https://arduino-pico.readthedocs.io/en/latest/multicore.html
// are the Serial.print* functions not threadsafe?  
// Generally Serial is not thread-safe.
// simple Serial.print() might be thread safe
// but not Serial.println or especially not Serial.printf()

// everything is shared/accessible between two cores, but little is thread safe?

void setup() {

    // what things are thread safe?
    // https://forums.raspberrypi.com/viewtopic.php?t=370841
    // sleep_us (Depends on Children, Unsolved due to "sleep_until")

    // absolute_time_diff_us (Depends on Children, Looks SMP Safe/Thread Safe)
    // get_absolute_time (Depends on Children, Looks SMP Safe/Thread Safe)

    // to_us_since_boot (Looks SMP Safe/Thread Safe)
    // update_us_since_boot (Looks SMP Safe/Thread Safe)
    // time_reached (Depends on Children, Looks SMP Safe/Thread Safe)
    // busy_wait_until (Depends on Children, Looks SMP Safe/Thread Safe)
    // tight_loop_contents (NoOp, SMP Safe/Thread Safe)
    // make_timeout_time_us (Depends on Children, Looks SMP Safe/Thread Safe)
    // delayed_by_us (Depends on Children, Looks SMP Safe/Thread Safe)
    // __get_current_exception (Looks SMP Safe/Thread Safe)

    // loop on this and leave if once we get Serial.available()
    // otherwise just stay here! This will give us a test
    // of Serial.print() thread-safeness. We can remove the prints later to be careful.
    // wait 1 sec for core1 to create Serial

    // "Arduino not support direct call to stdio getchar_timeout_us() is C function... but you call from CPP"
    // https://forums.raspberrypi.com/viewtopic.php?t=331207
    // https://github.com/earlephilhower/arduino-pico/discussions/2224
    // PICO_ERROR_GENERIC PICO_ERROR_TIMEOUT ??
    // int c = getchar_timeout_us(0);
    // shouldn't have to use the tud_cdc_connected() tud_cdc_available() hacks with ide
    // https://code.stanford.edu/sb860219/ee185/-/blob/master/software/firmware/circuitpython-main/supervisor/shared/serial.c

    // FIX! this forces going to the user interface always on boot. don't want this
    // for balloon, although it will time out there eventually
    // FIX! in case user is frantically trying to get to the config menu to avoid setting clock speed or ??
    // if anything was found by incomingByte above, go to the config menu
    // (potentially a balloon weird case would timeout)

    sleep_ms(1000);
    Serial.print(F(EOL "SETUP() ..LOOKING FOR Serial.available()" EOL EOL));
    Watchdog.reset();
    bool found_any = drainSerialTo_CRorNL();
    // how to compare char: ..== 'R' is the same as == 82 (ascii value)
    if (found_any) {
        // Must do this branching BEFORE setting clock speed in case of bad clock speed setting!
        Serial.print(F(EOL "SETUP() ..LEAVING AFTER SEEING Serial.available()" EOL EOL));
        updateStatusLED();
        // sleep_ms(1000);
        user_interface();
        // won't return here, since all exits from user_interface reboot
    }
    Serial.print(F(EOL "SETUP() ..LEAVING AFTER NOT SEEING Serial.available()" EOL EOL));

}

//*********************************************************

#define MSG_LOOP1_CONFIG_ACQUIRE 1
#define MSG_LOOP1_CONFIG_RELEASE 2

// Don't use the low level pi pico interprocessor stuff:
// hmm. the raw pico interprocessor fifo/irq stuff
// https://github.com/raspberrypi/pico-examples/tree/master/multicore/multicore_fifo_irqs

// use arduino-pico core interprocessor stuff
// run all balloon stuff on core1
// core 0 just looks for keyboard interrupts. Can interrupt core1 to prevent
// non-thread-safe flash access and Serial.print (will it interrupt a Serial.print in the middle? I suppose)
// what will happen when it resumes a Serial.print on core1?
// I guess undefined. So can't resume core1 after interrupting it. Can only reboot which
// is the normal behavior afte messing with config state.

// So core0 can stop core1 at any time. Do stuff with config, then reboot.
// That should be safe.

//**************************************
// Here is our very basic way to not worry about thread-safe for keyboard usb serial -> user configuration changes

// rp2040.idleOtherCore()
// Sends a message to stop the other core
// (i.e. when called from core 0 it pauses core 1, and vice versa).
// Waits for the other core to acknowledge before returning.

// The other core will have its interrupts disabled and be busy-waiting in an RAM-based routine,
// so flash and other peripherals can be accessed.

// NOTE idle core 0 too long, and the USB port can become frozen.
// Because core 0 manages the USB and needs to service IRQs in a timely manner (which it can’t do when idled).
// So we never idle core 0 !!
//**************************************

// void rp2040.resumeOtherCore()

void loop() {
    bool core1_idled = false;
    // FIX! musing: should never get any usb/serial input while balloon is flying
    // Could disable this if we knew we're flying.
    // How? equivalent to cutting off USB connector.
    // Shouldn't be necessary?
    // if garbage serial arrived, and we went to config mode, eventually we'd timeout/reboot
    absolute_time_t current_time_us  = 0;
    absolute_time_t last_current_time_us  = 0;
    int64_t loop_us_elapsed;
    int64_t loop_ms_elapsed;
    while (true) {
        // don't use watchog reset..not thread safe?
        last_current_time_us = current_time_us;
        current_time_us = get_absolute_time();
        loop_us_elapsed = absolute_time_diff_us(last_current_time_us, current_time_us);
        loop_ms_elapsed = loop_us_elapsed / 1000ULL;

        if (core1_idled) {
            Serial.print(F(EOL "loop() LOOPING QUICKLY WITH core1_idled()" EOL EOL));
            sleep_ms(1000);
        } else {
            // Serial.print(F(EOL "loop() LOOPING QUICKLY WITH !core1_idled()" EOL EOL));
            // with a 1 sec sleep..most of the time we're sleeping this core?
            // we couldn't sleep that long if we were updating leds? or ??
            sleep_ms(1000);
        }

        // no one should send me stuff
        while (rp2040.fifo.available()) {
            // int32_t fifo_TOS;
            uint32_t rp2040_fifo_TOS; 
            uint32_t *TOS_ptr; 
            TOS_ptr = &rp2040_fifo_TOS;
            // send a reference to a string?
            // https://forum.arduino.cc/t/how-to-transfer-strings-between-two-cores-on-pico/1310533/3

            // don't block if the fifo is empty. (that would be a bug case)
            // https://www.geeksforgeeks.org/passing-pointers-to-functions-in-c/
            // https://www.tutorialspoint.com/cprogramming/c_passing_pointers_to_functions.htm

            // pass the pointer so I get fifo_TOS modified
            // https://arduino-pico.readthedocs.io/en/latest/multicore.html
            // https://stackoverflow.com/questions/3168275/printf-format-specifiers-for-uint32-t-and-size-t
            bool msgFound = rp2040.fifo.pop_nb(TOS_ptr);
            // %d not okay with int32_t?
            Serial.printf(EOL "loop() DOING COOL STUFF: rp2040_fifo_TOS %" PRIu32 " msgFound %u" EOL EOL, 
                rp2040_fifo_TOS, msgFound);
            
            Serial.print(F(EOL "loop() WE SHOULDN'T BE SEEING THIS LOOPING HERE!" EOL EOL));
            // don't want to hang here
            break;
            sleep_ms(1000);
        }

        // This core can handle modifying config state, not the other core
        // so the other core just should be timely in stopping normal balloon work.

        int charsAvailable = (int) Serial.available();
        if (charsAvailable) {
            // bool rp2040.fifo.push_nb(uint32_t)
            // Pushes a value to the other core.
            // If the FIFO is full, returns false immediately and doesn’t block.
            // If the push is successful, returns true.
            // notify the other core to stop what it's doing (at a boundary where it checks)
            // should have < 2 sec response time for that, except if it's going thru
            // the lengthy boot sequence which only checks when done with setup?

            // 1 means we're taking over access to flash and config data. core 0 should idle
            // until it gets a 0 which means we're done
            // use the blocking calls

            // Not really using this fifo for acquire/release. Testing for future possibilities.
            // make this non-blocking in case fifo fills
            rp2040.fifo.push_nb(MSG_LOOP1_CONFIG_ACQUIRE);

            // this waits for the other core to acknowledge before completing.
            // what if the other core is already idle?
            // Have core1_idled state? don't send again if we know it's idle

            if (!core1_idled) rp2040.idleOtherCore();
            core1_idled = true;

            Serial.print(F(EOL "Core 0 TOOK OVER AFTER SUCCESSFULLY IDLING Core 1" EOL EOL));
            Serial.print(F(EOL "Core 0 IS CURRENTLY DOING NOTHING" EOL EOL));

            // FIX! what if did all the updateStatusLED from this core?
            // the other core could update the count as a writer and we would just be a reader
            // we could strip all the updateStatusLED() out of the other core
            // I guess we don't have a watch dog timer just for this core
            // but that's okay..worst case we lose keyboard response and if we do led here
            // we could lose led update if we hang here.
            // so when we loop here, we also do updateStatusLED()
            // have to remove all of it from the other core.
            // NO! we'd have to wake up more than once a sec. Leave it on the other core
            // which is active anyhow

            // moved here from loop1
            if (Serial.available()) {
                Serial.println(F("tracker.ino: (A) Going to user_interface() from loop()"));
                setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
                updateStatusLED();
                // sleep_ms(1000);
                user_interface();
                // won't return here, since all exits from user_interface reboot
            }

            // so will never resume the other core if we idled it?
            // rp2040.resumeOtherCore();

            // bool rp2040.fifo.pop_nb(uint32_t *dest)
            // Reads a value from this core’s FIFO and places it in dest.
            // Will return true if successful, or false if the pop would block.
            // but since he will be also using Serial.print* that should be thread safe?

            // make this non-blocking in case fifo fills
            // we should be able to overflow this and it's no big deal?
            rp2040.fifo.push_nb(MSG_LOOP1_CONFIG_RELEASE);
        }
    }
}

//***********************************************************
void setup1() {
    Watchdog.enable(30000);
    Watchdog.reset();
    // this is a don't care because USB
    // Serial.begin(115200);

    // from SleepyDog library
    // any use for Watchdog.sleep()  instead of sleep_ms() ?
    // To enter low power sleep mode call Watchdog.sleep() like below
    // and the watchdog will allow low power sleep for as long as possible.
    // The actual amount of time spent in sleep will be returned in // milliseconds).
    // digitalWrite(LED_BUILTIN, LOW); // Show we're asleep
    // int sleepMS = Watchdog.sleep();

    // temp hack to force DEVMODE and verbose 9
    forceHACK();

    //**********************
    // this is the usb serial. the baud rate doesn't really change usb data rates
    // Wait up to 5 seconds for serial to be opened, to allow catching
    // startup messages on native USB boards (that do not reset when serial is opened).

    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

    Watchdog.reset();
    uint64_t serial_millis = millis();
    // wait 10 secs looking for Serial
    while (((millis() - serial_millis) < 10000) && !Serial) {
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
    }
    Serial.println(F("setup1() START"));

    //**********************
    // Apparently I don't need to init the usb serial port?
    // Can't use printf for unknown reason. But Serial.printf() etc is fine?
    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__pico__stdio__usb.html

    // PICO_STDIO_USB_CONNECT_WAIT_TIME_MS ??
    // stdio_usb_init() not needed/supported
    // stdio_init_all() not needed/supported
    // if (!stdio_usb_init()) {
    // if (!stdio_init_all()) Serial.println("ERROR: stdio_init_all() failed)");

    //**********************
    adc_INIT();

    //**********************
    vfo_init();
    vfo_turn_off();
    vfo_turn_on(WSPR_TX_CLK_NUM);

    //**********************
    // necessary for Serial2 to work properly
    // we have only one i2c? what about the BMP280 ?
    // probably don't even need this. the core may have already done it?
    Wire.begin();

    GpsINIT(); // also turns on and checks for output
    GpsOFF();

    GpsFixMillis = 0;
    GpsStartMillis = millis();

    // just full cold gps reset
    // this means we get a full cold result on the aruduino IDE with usb power
    // otherwise usb power means vbat is always on. so a hot reset!
    GpsON(true);

    setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
    updateStatusLED();

    // FIX! assume this is the state it was in before config menu?
    // not always right. but loop will self-correct?

    if (!Serial) {
        setStatusLEDBlinkCount(LED_STATUS_REBOOT_NO_SERIAL);
        // we're going to have to reboot..even balloon needs Serial created?
        // if serial data output buf is full, we just overflow it (on balloon)
        // DEVMODE and verbose used to limit output?
        // reboot
        Watchdog.enable(1000);  // milliseconds
        for (;;) {
            // FIX! put a bad status in the leds
            updateStatusLED();
        }
    }

    //**********************
    Watchdog.reset();
    vfo_init();

    // sets minute/lane/id from chan number.
    // FIX! is it redundant at this point?..remove?
    process_chan_num();


    //***************
    // get all the _* config state set and fix any bad values (to defaults)
    int result = read_FLASH();
    // if anything got fixed to defaults, no read again

    Watchdog.reset();
    if (result == -1) {
        Serial.println(F("WARN: read_FLASH got result -1 first time, redo. ..errors were fixed to default"));
        result = read_FLASH();
    }
    Watchdog.reset();
    if (result == -1) {
        Serial.println(F("ERROR: read_FLASH got result -1 a second time, ignore"));
    }

    //***************
    const uint32_t clkhz =  atoi(_clock_speed) * 1000000L;
    if (!set_sys_clock_khz(clkhz / kHz, false)) {
        Serial.print(RED);
        Serial.printf(" RP2040 can't change clock to %luMhz. Using 133 instead" EOL, PLL_SYS_MHZ);
        Serial.print(NORMAL);
        snprintf(_clock_speed, sizeof(_clock_speed), "133");
        write_FLASH();
        // FIX! should we have this in parallel to _clock_speed? have to maintain it
        // should we call it _clock_speed_int ? or just always do atoi(_clock_speed)
        PLL_SYS_MHZ = 133;
    }

    // This should work now
    // FIX! don't do it for now
    // InitPicoClock(PLL_SYS_MHZ);

    Watchdog.reset();
    bmp_init();
    // i2c_scan();

    // Adafruit_BMP805 bmp;
    if (!bmp.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor"));
    }
    else {
        // Default settings from datasheet.. should we do forced sample
        // like weather station recommendations (rather than free running)
        bmp.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::SAMPLING_X16,
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_500);
    }

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    Watchdog.reset();
    Serial.println(F("setup1() END"));
    sleep_ms(1000);
}

//*************************************************************************
uint64_t GpsTimeToLastFix = 0;    // milliseconds

// these are used as globals
int TELEN1_val1;
int TELEN1_val2;
int TELEN2_val1;
int TELEN2_val2;

int tx_cnt_0;
int tx_cnt_1;
int tx_cnt_2;
int tx_cnt_3;

//*************************************************************************
// GPS NMEA bursts: The thinking behind how we deal with it:
// LightAPRS only looked for GPS NMEA data when it needed a fix. Not all the time.

// TinyGPS++ is not running as a separate task. Only does work when we call it.
// we call it with every new char, and to get data it creates from a history 
// of NMEA sentences in those chars.

// NMEA sentence come in bursts at 1Hz ..
// And the pi pico only has 32 byte receive bufffer on the UART that talks to ATGM336H-51

// Maybe not aligned to a second, but the burst is less than one full second of data.
// and burst intervals are at 1 sec. Data is not spread out over the full second.

// Order of each NMEA sentence is not random either. Order stays the same for each burst. 
// Just a interesting note

// we can handle each char at about 300 usec avg. 
// Obviously a char processing time is more when TinyGPS++ sees a char is a "end of NMEA sentence" 
// (32 deep rx fifo can absorb some chars if we're delayed. 
// Always want the rx fifo to be almost empty, to allow that little bit buffering if there's any backup.

// If characters arrived at max 9600 baud, thats 1 char per .1 ms or 100 us.

// So we're too slow to handle that.
// but the effective baud rate out of ATGM336H-51 is maybe 900 chars/sec (at 9660 baud Serial2)

// so that's around 1.1ms allowed time per char. 

// Plenty..probably even if more processingsometimes by TinyGPS++. 
// We can even allow faster baud rate for increased effective chars/sec

// we don't have to locally buffer chars to allow for backpressure from TinyGPS++
// (we could create a local fifo, to effectively absorb more than the 32 deep uart rx fifo)

// If we created a secondary RX buffer to hold 600 plus chars (the entire burst of multiple
// NMEA sentences, per second, for the default enabled sentences (US and Baidu satellites)

// Then if we could absorb/empty it at least every second, we'd never lose anything.

// Less processing/power if we only absorb GPS data when we need it. 
// Allows us to sleep when  we don't need a fix update.

// Key that TinyGPS++ absorbs our char send at bounded delays (per NMEA sentence). 
// end of NMEA sentence has longer delay. Apparently the CR LF is needed as a 'boundary' post checksum?
// FIX! do we need both CR and LF or is one enough? Shouldn't matter.

//*********************************************************************************
// Now about sleepSeconds() for next beacon (HF or VHF).
// not sure why this was 50 secs
// is there any benefit to this trying to be aligned to second boundaries
// this shouldn't lead to a old gps.location.age since we do gps while in sleepSeconds()
// with this wait

// how to deal with what a "fix" is, and .age
// https://arduiniana.org/libraries/tinygps/
// The NMEA sentences must report valid data. 
// If the $GPRMC sentence reports a validity of “V” (void) instead of “A” (active), 
// or if the $GPGGA sentence reports fix type “0” (no fix) then those sentences are discarded.

// hmm. we just always qualify it with valid
// TinyGPS::GPS_INVALID_AGE is the value when you never got a valid fix.
// if (fix_age == TinyGPS::GPS_INVALID_AGE)
//   Serial.println("No fix detected");
// else if (fix_age > 5000)
//   Serial.println("Warning: possible stale data!");
// else
//   Serial.println("Data is current.");


uint16_t  BEACON_WAIT = 61; // secs
// seconds sleep if super capacitors/batteries are below BattMin
uint16_t  BATT_WAIT = 1;  // secs     

// GPS_LOCATION_AGE_MAX should a bit greater than GPS_WAIT_FOR_NMEA_BURST_MAX
// we could live with data that is more 'stale' but theoretically it should be no older than this?
// how stale can it get? we might be waiting for a new hot fix? and the old one is in there and usable
// maybe make an old fix good for up to 5 secs? (if we did a quick gps power off/on we won't get new
// one for maybe 5 secs. but maybe that says we don't want anything older than 1-2 secs?
// but who knows how tinyGPS++ creates the .age ??
// maybe it needs to be hot fix max time? (allow 5 secs?)
// worse would be if it only updates .age to 0 when the new fix changes any of location/altitude etc?
// #define GPS_LOCATION_AGE_MAX 8000
// 60 secs
// could be this old if we do the BEACON_WAIT with gps off? no gps data to wake us out of sleepSeconds()
// #define GPS_LOCATION_AGE_MAX 61000
// seeing some 68727 with 61000 (our beacon wait is 61 secs)
// increase to 70000
// at 180mph we could move 3 horizontal miles in 1 minute? hmm. what about a descending balloon?
// fixes could be 1 minute old? that's kind of like a cold fix time
#define GPS_LOCATION_AGE_MAX 70000

//
// smallest seen
// fix_age 1211
// biggest seen
// fix_age 60322


// FIX! since we break out of the sleepSeconds when gps data starts (Serial2.available()) ..
// we could make this bigger? needs to be at least 1 sec (a little more) since it
// wants to grab a full burst, and we don't know where we are in the repeating
// burst behavior when we start (idle or in the middle of a burst?)
#define GPS_WAIT_FOR_NMEA_BURST_MAX 1100

//*************************************************************************
void loop1() {
    // getting 25 to 35 sec loop times from the BATT_WAIT/BEACON_WAIT ?? (baud 19200)
    // some loops were 775 millis
    // now getting 50 sec loops from the BEACON_WAIT

    // FIX! should change baud back to 9600 (lower power?) only gettin 1800 baud during 300-400ms
    // when looking for data for slight >1 sec time period (all broadcasts are at 1 sec intervals)
    // they all go in a burst together? so all within one sec, and actually a tighter burst.
    loopCnt++;
    if (VERBY[0]) Serial.printf(EOL "loop1() loopCnt %" PRIu64 EOL, loopCnt);

    // temp hack to force DEVMODE and verbose 9
    // shouldn't need to do this on every loop. done in config (and in setup)
    // forceHACK();

    // moved to loop()
    if (false) {
        if (Serial.available()) {
            Serial.println(F("tracker.ino: (A) Going to user_interface() from loop1()"));
            setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
            updateStatusLED();
            // sleep_ms(1000);
            user_interface();
            // won't return here, since all exits from user_interface reboot
        }
    }

    Watchdog.reset();
    Serial.println(F("loop1() START"));

    // copied from loop_us_end while in the loop (at bottom)
    if (loop_us_start == 0) loop_us_start = get_absolute_time();
    updateStatusLED();
    // always make sure tx is off?
    // does nothing if already off
    vfo_turn_off();

    //******************
    // Gps may already be on
    // this loads the voltage
    if (!GpsIsOn()) {
        GpsFixMillis = 0;
        GpsTimeToLastFix = 0;
        GpsStartMillis = millis();
    }
    // just full cold gps reset
    // this means we get a full cold result on the aruduino IDE with usb power
    // otherwise usb power means vbat is always on. so a hot reset!

    // we do a full cold reset in setup1() now (part of the GpsInit()?)
    // if (loopCnt == 1) GpsON(true);
    // else GpsON(false);
    GpsON(false);

    //******************
    // no need to have GpsFirstFix
    // maybe nice to report it?
    // if ( !(GpsFirstFix && (readVoltage() > BattMin)) || (!GpsFirstFix && (readVoltage() > GpsMinVolt)) )) {
    //     sleepSeconds(BATT_WAIT);

    float solar_voltage;
    solar_voltage = readVoltage();
    if ( solar_voltage <= BattMin || solar_voltage <= GpsMinVolt ) {
        sleepSeconds(BATT_WAIT);
    } else {
        Serial.println(F("loop1() solar_voltage good"));
        //*********************
        // FIX! this can set time and unload NMEA sentences?
        // unload for 2 secs to make sure we get 1 sec broadcasts?
        // actually just need a little over 1 sec.
        // Also: what about corruption if buffer overrun?)
        // does CRC check cover that? so okay if we have overrun?
        updateGpsDataAndTime(GPS_WAIT_FOR_NMEA_BURST_MAX);
        gpsDebug();

        // looks like we're getting age < 1200 ..so we're a little slower than 1000 age (was)
        // change to 1400
        // it takes 700 millis to get the burst of NMEA sentences?

        // seeing this when we weren't snapTelemetry'ing
        // gps.time.isValid():1
        // gps.location.age():1191
        // gps.location.isUpdated():0

        // now seeing this with 1300 max. changed to 3000
        // gps.location.age():2345


        // this just handles led for time/fix and gps reboot check/execution
        // "%lu" or "%" PRIu32 " to use with printf?
        uint32_t fix_age = gps.location.age();
        bool fix_valid = gps.location.isValid();
        // isUpdated() indicates whether the object’s value has been updated (not necessarily changed) 
        // since the last time you queried it
        bool fix_updated = gps.location.isUpdated();
        uint32_t fix_sat_cnt = gps.satellites.value();

        if ( fix_valid && (fix_age < GPS_LOCATION_AGE_MAX) ) {
            GpsInvalidCnt = 0;
            setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
        } else {
            // FIX! at what rate is this incremented? ..once per loop iteration (time varies)
            GpsInvalidCnt++;
            Serial.printf("loop1() GpsInvalidCnt++ %d" EOL, GpsInvalidCnt);

            // why doesn't this year check get included in determining valid gps fix?
            // if gps time is valid, we constantly (each NMEA burst grab) 
            // update RP2040 time from gps time in gps_functions.cpp updateGpsDataAndTime() 
            // so don't here. Only update LED state here, though
            if (gps.date.year() >= 2024 && gps.date.year() <= 2034)
                // FIX! where do we grab the time
                setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
            else
                setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

            // FIX! this is loop iterations? could be 60 * 30 secs per loop (30 minutes)
            if (GpsInvalidCnt > 60 ) {
                Serial.println(F("ERROR: loop1() GpsInvalidCnt > 60 ..gps full cold reset"));
                // FIX! have to send cold gps reset, in case ephemeris is corrupted? since vbat is always there
                // otherwise this is a warm reset?
                GpsOFF();
                // note that GpsOFF() has public access to TinyGPS++ now and clears these 3 which is sufficient
                // for "reset TinyGPS++ stuff"
                // gps.date.valid = false;
                // gps.date.updated = false;
                // gps.date.date = 0;

                Watchdog.reset();
                sleep_ms(1000);
                // also do gps cold reset.
                GpsON(true);
                GpsInvalidCnt = 0;
                setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
                // https://forum.arduino.cc/t/possible-to-continue-the-main-loop/95541/5
                return;
            }
        }
        updateStatusLED();

        //*********************
        // some detail on TinyGPS. precision?
        // https://sites.google.com/site/wayneholder/self-driving-rc-car/getting-the-most-from-gps

        // FIX! why use isUpdated()  Allows ignoring gps.location.age() compare
        // we read location in many places, so I don't think the isUpdated() info is useful?
        // removing it from the age compare
        //    (fix_age < GPS_LOCATION_AGE_MAX || fix_updated) ||
        // fix_age will be 4294967295 if not valid
        if (VERBY[0]) {
            Serial.printf("fix_valid %u" EOL, fix_valid);
            Serial.printf("fix_age %lu" EOL, fix_age);
            Serial.printf("fix_sat_cnt %lu" EOL, fix_sat_cnt);
            Serial.printf("fix_updated %u" EOL, fix_updated);
        }
        if (!fix_valid || (fix_age >= GPS_LOCATION_AGE_MAX) ) {
            if (VERBY[0]) 
                Serial.println(F("loop1() WARN: GPS fix issue ..stail or not valid"));

            // these are the waits that give us the long loop times
            // Looping with sleep
            // Serial2.Activity() is the thing that gets it to wake up early
            // no change of LED here. done above in time set/reboot check
            sleepSeconds(BEACON_WAIT);

        } else if (fix_sat_cnt <= 3) { // implied also 'not the first if clause' .. i.e good fix
            // FIX! should we have separate led count for 2d fix and 3d fix?
            if (VERBY[0]) 
                Serial.println(F("loop1() WARN: GPS fix issues ..not enough sats ..2d only"));

            // these are the waits that give us 25-30 sec loop times?
            sleepSeconds(BEACON_WAIT);
            // FIX! how much should we wait here?

        } else {
            if (VERBY[0]) Serial.println(F("loop1() Good recent 3d fix"));
            // snapForTelemetry (all the t_* state) right before we do all the WSPRing
            // we can update the telemetry buffer any minute we're not tx'ing

            // we know we have a good 3d fix at this point
            // we don't check the valid bits again? they could have changed
            // at any time (async) ..so can't really be an atomic grab anyhow?
            // keep the snap close to the valid checks above
            snapForTelemetry();
            if (VERBY[0]) Serial.println(F("loop1() Gps fix .. good 3d"));
            GpsInvalidCnt = 0;

            // GpsStartTime is reset every time we turn the gps on
            // cleared every time we turn it off (don't care)
            // Should this is also cleared when we turn gps off? no?
            // floor divide to get milliseconds
            /// FIX! is this the same as GpsFixMillis ?

            // GpsStartTime is set by gps_functions.cpp
            if (GpsTimeToLastFix == 0) {
                // FIX! odd case. Did the GPS get turned off, but TinyGPS++
                // still says it has valid fix?
                // until I figure out why, set GpsTimeToLastFix to 0 for this case
                if (GpsStartTime == 0) {
                    GpsTimeToLastFix = 0;
                } else {
                    GpsTimeToLastFix = (
                        absolute_time_diff_us(GpsStartTime, get_absolute_time()) ) / 1000ULL;
                }
            }

            // FIX! just need one or the other of these
            // Just print this the first time we have a good fix
            if (GpsFixMillis == 0) {
                // FIX! odd case. Did the GPS get turned off, but TinyGPS++
                // still says it has valid fix?
                // until I figure out why, set GpsStartMillies would be 0 for this case
                if (GpsStartMillis == 0) {
                    GpsFixMillis = 0;
                } else {
                    GpsFixMillis = millis() - GpsStartMillis;
                }
            }
            if (VERBY[0]) Serial.printf("loop1() first Gps Fix, after off->on! "
                    "GpsFixMillis %" PRIu64 " GpsTimeToLastFix %" PRIu64 EOL,
                    GpsFixMillis, GpsTimeToLastFix);

            // sets all the t_* strings above
            // voltage is captured when we write the buff? So it's before GPS is turned off?
            // should we look at the live voltage instead?
            if (VERBY[0]) Serial.printf(
                "loop1() After snapTelemetry() timeStatus():%u minute():%u second():%u" EOL,
                timeStatus(), minute(), second());

            // freeMem();

            // FIX! it should depend on the channel starting minute - 1 (modulo 10)
            // preparations for HF starts one minute before TX time
            // at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57.

            // FIX! why not look at seconds here?
            // it will stall until lined up on secs below
            // align to somewhere in the minute before the callsign starting minute

            // so we can start the vfo 30 seconds before needed
            // if we're in the minute before sending..just live with gps fix
            // because it might take a minute to have another one?
            Watchdog.reset(); 
            // make sure readVoltage always returns postive # (> 0)
            // readVoltage can return 0
            if (readVoltage() >= WsprBattMin) {
                if (alignMinute(-1)) {
                    if (second() > 30) {
                        // to late..don't try to send
                        Serial.println(F("WARN: past needed 30 sec setup in the minute before WSPR should start"));
                        // minute() second() come from Time.h as ints
                        Serial.printf("WARN: because minute() %d second() %d alignMinute(-1) %u" EOL, 
                            minute(), second(), alignMinute(-1));
                    } else {
                        while (second() < 30) {
                            delay(10); // 10 millis
                            // we could end up waiting for 30 secs so update LED
                            updateStatusLED();
                        }
                        // will call this with less than or equal to 30 secs to go
                        alignAndDoAllSequentialTx();
                    }
                } else {
                    // we fall thru and can get another gps fix or just try again.
                    // hopefully the BeaconWait or BattWait doesn't kick in?
                }
            }
        }
    }

    loop_us_end = get_absolute_time();
    loop_us_elapsed = absolute_time_diff_us(loop_us_start, loop_us_end);
    // floor divide to get milliseconds
    loop_ms_elapsed = loop_us_elapsed / 1000ULL;

    if (VERBY[0]) {
        // maybe show GpsInvalidCnt also? how old are they
        // FIX! StampPrintf doesn't seem to be printing correctly with this format string?
        // oh I had the wrong formats for the variables. Serial.printf exposed that. StampPrintf didn't.
        // use Serial.printf()
        Serial.printf(
            "t_tx_count_0: %s "
            "t_callsign: %s "
            "t_temp: %s "
            "t_voltage: %s "
            "t_altitude: %s "
            "t_grid6: %s "
            "t_power: %s "
            "t_sat_count: %s "
            "GpsTimeToLastFix %" PRIu64 " "
            "GpsInvalidCnt %d" EOL,
            t_tx_count_0, t_callsign, t_temp, t_voltage, t_altitude, t_grid6, t_power, t_sat_count,
            GpsTimeToLastFix, GpsInvalidCnt);
    }

    if (VERBY[5]) {
        Serial.printf(
            "main/20: _Band %s "
            "loop_ms_elapsed: %" PRIu64 " millisecs "
            "loop_us_start: %llu microsecs "
            "loop_us_end: %llu microsecs",
            _Band, loop_ms_elapsed, loop_us_start, loop_us_end);
    }
    updateStatusLED();

    // next start is this end
    loop_us_start = loop_us_end;
    // whenever we have spin loops we need to updateStatusLED()
    if (VERBY[0]) Serial.println(F("loop1() END"));
}


//*******************************************************
void alignAndDoAllSequentialTx (void) {
    // if we called this to early, just return so we don't wait 10 minuts here
    // at most wait up to a minute if called the minute before start time)
    if (!alignMinute(-1)) {
        if (VERBY[0]) Serial.println(F("alignAndDoAllSequentialTX END early out: align way off!"));
        return;
    }

    if (VERBY[0]) Serial.println(F("alignAndDoAllSequentialTX START"));
    Watchdog.reset(); 
    while (second() < 30)  {
        delay(5); // 5 millis
        // we could end up waiting for 30 secs
        updateStatusLED();
    }

    if (VERBY[0]) Serial.println(F("alignAndDoAllSequentialTX START"));
    // don't want gps power and tx power together
    GpsOFF();

    // start the vfo 30 seconds before needed
    vfo_turn_on(WSPR_TX_CLK_NUM);
    setStatusLEDBlinkCount(LED_STATUS_TX_WSPR);

    // GPS will stay off for all
    char hf_callsign[7];
    snprintf(hf_callsign, sizeof(hf_callsign), "%s", t_callsign);
    // same declared size, so could just strncpy
    // strncpy(hf_callsign, t_callsign, 6);

    double lat_double = atof(t_lat);
    double lon_double = atof(t_lon);

    // get_mh_6 users the first arg as pointer to a char array for the return data
    char hf_grid6[7] = { 0 };
    char hf_grid4[5] = "AA00";
    get_mh_6(hf_grid6, lat_double, lon_double);
    // not same declared size, so use snprintf)
    // just the first 4 chars
    snprintf(hf_grid4, sizeof(hf_grid4), "%s", hf_grid6);

    char hf_power[3];
    snprintf(hf_power, sizeof(hf_power), "%s", t_power);
    // same declared size, so can just strncpy
    // strncpy(hf_power, t_power, 2);

    // FIX! make the drive strength conditional on the config
    // we could even make the differential dependent on the config

    // this is done in the vfo_turn_on. shouldn't need to do again
    // vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_8MA);
    // vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_8MA);
    // turns on both tx clk. The first time, has pll setup already?

    Watchdog.reset(); 
    // will sync up to the right minute and second == 0
    syncAndSendWspr(0, hf_callsign, hf_grid4, hf_power, false);
    if (VERBY[0]) {
        tx_cnt_0 += 1;
        // we have 10 secs or so at the end of WSPR to get this off?
        if (VERBY[0]) {
            StampPrintf("WSPR callsign Tx sent. minutes %d secs %d",
                minute(), second());
            DoLogPrint();  // we might have to delay this?
        }
    }
    // we don't loop around again caring about gps fix, because we've saved
    // telemetry (and sensor data in there) right before the callsign tx
    setStatusLEDBlinkCount(LED_STATUS_TX_TELEMETRY);

    // input: uses t_callsign t_grid6 t_power
    // output: modifies globals: hf_callsign, hf_grid4, hf_power
    u4b_encode_std();
    syncAndSendWspr(1, hf_callsign, hf_grid4, hf_power, false);
    if (VERBY[0]) {
        tx_cnt_1 += 1;
        // we have 10 secs or so at the end of WSPR to get this off?
        if (VERBY[0]) {
            StampPrintf("WSPR telemetry Tx sent. minutes %d secs %d",
                minute(), second());
            DoLogPrint();  // we might have to delay this?
        }
    }
    // have to send this if telen1 or telen2 is enabled
    if ( (_TELEN_config[0] != '-' || _TELEN_config[1] != '-') ||
         (_TELEN_config[2] != '-' || _TELEN_config[3] != '-') ) {
        setStatusLEDBlinkCount(LED_STATUS_TX_TELEN1);

        // void u4b_encode_telen(uint32_t telen_val1, uint32_t telen_val2, for_telen2) {
        // output: modifies globals: hf_callsign, hf_grid4, hf_power
        // unint32_t globals? could be just int? don't use full range
        // TELEN1_val1
        // TELEN1_val2
        u4b_encode_telen(TELEN1_val1, TELEN1_val2, false);
        syncAndSendWspr(2, hf_callsign, hf_grid4, hf_power, false);
        if (VERBY[0]) {
            tx_cnt_2 += 1;
            // we have 10 secs or so at the end of WSPR to get this off?
            if (VERBY[0]) {
                StampPrintf("WSPR telen1 Tx sent. minutes %d secs %d",
                    minute(), second());
                DoLogPrint();  // we might have to delay this?
            }
        }
    }
    // have to send this if telen2 is enabled
    if ( (_TELEN_config[2] != '-' || _TELEN_config[3] != '-') ) {
        setStatusLEDBlinkCount(LED_STATUS_TX_TELEN2);
        // unint32_t globals? could be just int? don't use full range
        // TELEN1_val1
        // TELEN2_val1
        // TELEN2_val2
        u4b_encode_telen(TELEN1_val2, TELEN2_val2, true);
        syncAndSendWspr(3, hf_callsign, hf_grid4, hf_power, true);
        if (VERBY[0]) {
            tx_cnt_3 += 1;
            // we have 10 secs or so at the end of WSPR to get this off?
            if (VERBY[0]) {
                StampPrintf("WSPR telen2 Tx sent. minutes %d secs %d",
                    minute(), second());
                DoLogPrint();  // we might have to delay this?
            }
        }
    }

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    GpsON(false);  // no gps cold reset

    if (VERBY[0]) Serial.println(F("alignAndDoAllSequentialTX END"));
}

//*******************************************************
extern const int WSPR_PWM_SLICE_NUM=4;

void PWM4_Handler(void) {
    if (VERBY[0]) Serial.println(F("PWM4_Handler() START"));
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    static int cnt = 0;
    if (++cnt >= 500) {
        cnt = 0;
        proceed = true;
    }
    if (VERBY[0]) Serial.println(F("PWM4_Handler() END"));
}

void zeroTimerSetPeriodMs(float ms) {
    if (VERBY[0]) Serial.println(F("zeroTimerSetPeriodMs() START"));
    static pwm_config wspr_pwm_config = pwm_get_default_config();

    pwm_config_set_clkdiv_int(&wspr_pwm_config, 250);  // 2uS
    pwm_config_set_wrap(&wspr_pwm_config, ((uint16_t)ms - 1));
    pwm_init(WSPR_PWM_SLICE_NUM, &wspr_pwm_config, false);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, PWM4_Handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
    if (VERBY[0]) Serial.println(F("zeroTimerSetPeriodMs() END"));
}

//***********************************************************
void sleepSeconds(int secs) {
    // this doesn't have an early out (although the updateGpsDataAndTime() can
    // so the delay will be >= secs
    if (VERBY[0]) Serial.println(F("sleepSeconds() START"));
    Serial.flush();
    uint64_t start_millis = millis();
    uint64_t current_millis = start_millis;
    uint64_t duration_millis = (uint64_t) secs * 1000;
    float solar_voltage;
    do {
        Watchdog.reset();
        // can power off gps depending on voltage
        // normally keep gps on, tracking after first fix. we are moving!
        // uint32_t usec = time_us_32();
        GpsON(false);

        solar_voltage = readVoltage();
        if (solar_voltage < BattMin) GpsOFF();
        if (solar_voltage < GpsMinVolt) GpsOFF();
        // FIX! should we unload/use GPS data during this?
        // gps could be on or off, so no?
        // whenever we have spin loops we need to updateStatusLED()

        updateStatusLED();
        if (GpsIsOn()) {
            // does this have a updateStatusLED() ??
            // long enough to be sure to catch all NMEA during the broadcast interval of 1 sec
            // 1050: was this causing rx buffer overrun (21 to 32)
            updateGpsDataAndTime(1500);  // milliseconds
        } else {
            sleep_ms(1500);
        }
        current_millis = millis();
    } while ((current_millis - start_millis) < duration_millis);

    Watchdog.reset();
    // Gps gets left off it the voltage was low at any point
    if (VERBY[0]) Serial.println(F("sleepSeconds() END"));
}

//***********************************************************
// -1 is returned if anything illegal
// FIX! should check what caller does if -1
bool alignMinute(int offset) {
    bool aligned = false;
    // need good time
    if (timeStatus() != timeSet) return false;

    // offset can be -1, 0, 1, 2, 3, 4, 5, 6 (3 messages)
    // if not one of those, set the start minute to -1?
    // caller should detect that and not wait?
    if (offset < -1 || offset > 6) {
        offset = 0;
    }

    // this should have been only set to be char strs 0, 2, 4, 6, or 8
    int align_minute = atoi(_start_minute);
    switch (align_minute) {
        case 0: {;}
        case 2: {;}
        case 4: {;}
        case 6: {;}
        case 8: align_minute = (align_minute + offset) % 10; break;
        default: align_minute = 0;
    }

    // FIX! update to look at u4b channel config
    aligned = (minute() % 10) == align_minute;
    if (_go_when_rdy[0] == '1') {
        // any odd minute
        if (offset == -1) aligned = (minute() % 2) == 1;
        // any even minute
        else aligned = (minute() % 2) == 0;
        // telemetry buffer is also updated whenever if _go_when_ready
    }
    return aligned;
}

//********************************************
// expected this is called at least 10 secs before starting minute
// if not in the minute before starting minute,
// it will wait until the right starting minute (depends on txNum)
// txNum can be 0, 1, 2, 3
void sendWspr(int txNum, char *hf_callsign, char *hf_grid4, char *hf_power, bool vfoOffWhenDone) {
    if (VERBY[0]) Serial.println(F("sendWSPR() START"));
    Watchdog.reset();
    if (txNum < 0 || txNum > 3) {
        if (VERBY[0]) Serial.printf("bad txNum %d ..ignoring" EOL, txNum);
        return;
    }

    // FIX! use the u4b channel freq
    uint32_t hf_freq = XMIT_FREQUENCY;
    if (atoi(_correction) != 0) {
        // this will be a floor divide
        // https://user-web.icecube.wisc.edu/~dglo/c_class/constants.html
        hf_freq = hf_freq + (atoi(_correction) * hf_freq / 1000000000UL);
    }

    // if (VERBY[0]) Serial.printf("WSPR desired freq: %lu used hf_freq %u with correction %s" EOL,
    //    XMIT_FREQUENCY, hf_freq, _correction);

    symbol_count = WSPR_SYMBOL_COUNT;  // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;


    // need uint8_t for power
    uint8_t hf_power_val = (uint8_t)atoi(hf_power);
    set_tx_buffer(hf_callsign, hf_grid4, hf_power_val, tx_buffer);

    zeroTimerSetPeriodMs(tone_delay);

    uint8_t i;
    for (i = 0; i < symbol_count; i++) {
        uint32_t freq_x16 =
            (hf_freq << PLL_CALCULATION_PRECISION) +
            (tx_buffer[i] * (12000L << PLL_CALCULATION_PRECISION) + 4096) / 8192L;
        // Serial.printf(__func__ " vfo_set_freq_x16(%u)" EOL, (freq_x16 >> PLL_CALCULATION_PRECISION));
        vfo_set_freq_x16(WSPR_TX_CLK_NUM, freq_x16);
        // PWM handler sets proceed?
        proceed = false;
        while (!proceed) {
            // whenever we have spin loops we need to updateStatusLED()
            // the latency to decide if we had to do anything, and do it if we do
            // must be pretty small, so that it's okay that it introduces variance here
            // we could disable this if not DEVMODE. means lights don't blink while transmitting?
            // max time in this loop is less than watchdog interval?
            // Leave it out for now, just in case it affects symbols
            // maybe monitor elapsed time for updateStatusLED() ??
            // updateStatusLED();
        }
        Watchdog.reset();
    }
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    irq_set_enabled(PWM_IRQ_WRAP, false);
    irq_remove_handler(PWM_IRQ_WRAP, PWM4_Handler);

    // FIX! leave on if we're going to do more telemetry?
    if (vfoOffWhenDone) {
        vfo_turn_off();
    }
    Watchdog.reset();
    if (VERBY[0]) Serial.println(F("sendWSPR() END"));
    // Serial.println(F(__func__ " END"));
}

void syncAndSendWspr(int txNum, char *hf_callsign, char *hf_grid4, char *hf_power, bool vfoOffWhenDone) {
    if (VERBY[0]) Serial.println(F("syncAndSendWSPR() START"));
    if (txNum < 0 || txNum > 3) txNum = 0;
    // txNum is between 0 and 3..means 0,2,4,6 offsets, given the u4b channel configured
    // we turned the vfo on in the minute before 0, separately

    if (VERBY[0]) {
        Serial.printf("will start WSPR txNum %d when aligned zero secs, currently %d secs\n",
            txNum, second());
        Serial.printf("WSPR txNum %d Preparing", txNum);
        Serial.printf("hf_grid4: %s", hf_grid4);
    }
    // this should be fine even if we wait a long time
    Watchdog.reset();
    int i = 2 * txNum;  // 0, 2, 4, 6
    while (!alignMinute(i) || (second() != 0)) {
        // FIX! delay 1 sec? change to pico busy_wait_us()?
        sleep_ms(20);
        // delay(1);
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
    }

    sendWspr(txNum, hf_callsign,  hf_grid4, hf_power, vfoOffWhenDone);
    if (VERBY[0]) Serial.println(F("syncAndSendWSPR() END"));
}

//**********************************
void set_tx_buffer(char *hf_callsign, char *hf_loc, uint8_t hf_power, uint8_t *tx_buffer) {
    Serial.println(F("set_tx_buffer() START"));
    // Clear out the transmit buffer
    memset(tx_buffer, 0, 255);  // is this bigger than WSPR_SYMBOL_COUNT ?
    // Set the proper frequency and timer CTC
    // legalPower = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]

    // get all the message date outside this?
    // FIX! does this strip null terms? what about strlen(hf_all) < 6 ?
    /*
     * wspr_encode(const char * call, const char * loc, const uint8_t dbm, uint8_t * symbols)
     *
     * Takes a callsign, grid locator, and power level and returns a WSPR symbol
     * table for a Type 1, 2, or 3 message.
     *
     * call - Callsign (12 characters maximum.. we guarantee 6 max).
     * loc - Maidenhead grid locator (6 characters maximum).
     * dbm - Output power in dBm.
     * symbols - Array of channel symbols to transmit returned by the method.
     *  Ensure that you pass a uint8_t array of at least size WSPR_SYMBOL_COUNT to the method.
     *
    */
    // hf_power needs to be passed as uint8_t
    jtencode.wspr_encode(hf_callsign, hf_loc, hf_power, tx_buffer);
    // maybe useful python for testing wspr encoding
    // https://github.com/robertostling/wspr-tools/blob/master/README.md
    Serial.println(F("set_tx_buffer() END"));
}

void freeMem() {
    Serial.println(F("freeMem() START"));
    if (!VERBY[0]) return;
    // NIce to use F() for strings that are constant
    // compiled string stays in flash. does not get copied to SRAM during the C++ initialization
    // string it has the PROGMEM property and runs from flash.
    Serial.print(F("Free RAM: "));
    Serial.print(freeMemory(), DEC);
    Serial.println(F(" byte"));
    Serial.println(F("freeMem() END"));
}

int InitPicoClock(int PLL_SYS_MHZ) {
    Serial.println(F("InitPicoClock START"));
    const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;

    // frequencies like 205 mhz will PANIC, System clock of 205000 kHz cannot be exactly achieved
    // should detect the failure and change the nvram, otherwise we're stuck even on reboot
    if (!set_sys_clock_khz(clkhz / kHz, false)) {
      // won't work
      Serial.printf("Can not set clock to %dMhz. 'pico 'Cannot be achieved''" EOL, PLL_SYS_MHZ);
      return -1;
    }

    Serial.printf("Attempt to set rp2040 clock to %dMhz (legal)" EOL, PLL_SYS_MHZ);
    // 2nd arg is "required"
    set_sys_clock_khz(clkhz / kHz, true);
    clock_configure(clk_peri, 0,
      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
      PLL_SYS_MHZ * MHZ,
      PLL_SYS_MHZ * MHZ);

    return 0;
    Serial.println(F("InitPicoClock END"));
}

//**********************
// Standard for strings in this *.ino and in *functions.cpp
// we'll just use c-style char arrays for strings
// other options:

// c++
// #include <string>
// string charString = Serial.parseString();
// Strings in C++ can be defined either
// using the std::string class
// or the C-style character arrays.

// c style string. stored in array of characters terminated by null: '\0'
// char e[] = "geeks";
// char el[] = {'g', 'f', 'g', '10'}'
// char* c = "geeksforgeeks"

// c++ style string (string class in std library). don't bother with String
// string str = ("gfg");
// string str = "gfg";
// string str; str = "gfg";

//**********************
// should I work on a local branch and commit to repo less often?
// https://stackoverflow.com/questions/13276909/how-to-do-a-local-only-commit-in-git


// interesting old benchmark testing wiggling gpio
// should run it on the pi pico and see
// https://github.com/hzeller/rpi-gpio-dma-demo#direct-output-loop-to-gpio


