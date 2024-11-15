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

// https://arduino-pico.readthedocs.io/en/latest/serial.html
// Arduino-Pico core implements a software-based Serial-over-USB port using the USB ACM-CDC model
// Serial is the USB serial port, and while Serial.begin() does allow specifying a baud rate,
// this rate is ignored since it is USB-based.
// Be aware that this USB Serial port is responsible for
// resetting the RP2040 during the upload process,
// following the Arduino standard of 1200bps = reset to bootloader).

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
volatile bool proceed = false;

//******************************  GPS SETTINGS   *********************************
int16_t GpsInvalidCnt = 0;

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

//*********************************
// some stuff on using namespace
// https://forum.arduino.cc/t/using-a-constant-defined-in-the-header-file/380178

// flash background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

//*********************************
// so it can be used in gps_functions.cpp
// extern is needed or the linker doesn't find it. see https://forum.arduino.cc/t/linker-problems-with-extern-const-struct/647136/2
extern const int GpsPwr = 16; // output ..this cuts VCC, leaves VBAT
// define is not used..GpsPwr is used. 
// const int GPS_VCC_ON_N_PIN=16;     


// FIX! where are these used
extern const int GPS_NRESET_PIN = 5;  // output ..active low..should be 1?
extern const int GPS_ON_PIN = 6;      // output ..this puts in sleep mode? should be 1?

// FIX! where is this used (calibration maybe)
extern const int GPS_1PPS_PIN = 17;   // input

extern const int GPS_UART1_TX_PIN = 8;
extern const int GPS_UART1_RX_PIN = 9;

// talks to gps
extern const int SERIAL2_FIFO_SIZE = 4096;
// choices: 9600 (default ATGM33651) 19200 38400
extern const int SERIAL2_BAUD_RATE = 38400;

// stuff moved to functions from this .ino (not libraries)
#include "gps_functions.h"

//*********************************
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
// 0 should never happen (init_rf_freq will always init from saved nvram/live state)
uint32_t XMIT_FREQUENCY = 0;
uint32_t PLL_SYS_MHZ = 133;

//*********************************
#include "u4b_functions.h"
#include "tele_functions.h"

// telemetry_buff
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

//***********************************************************
// for config_functions.cpp
// these get set via terminal, and then from NVRAM on boot
// init with all null
char _callsign[7] = { 0 };
char _id13[3] = { 0 };
char _start_minute[2] = { 0 };
char _lane[2] = { 0 };
char _suffix[2] = { 0 };
char _verbosity[2] = { 0 };
char _TELEN_config[5] = { 0 };
char _clock_speed[4] = { 0 };
char _U4B_chan[4] = { 0 };
char _Band[3] = { 0 };     // string with 10, 12, 15, 17, 20 legal. null at end
char _tx_high[2] = { 0 };  // 0 is 2mA si5351. 1 is 8mA si5351
char _devmode[2] = { 0 };
char _correction[7] = { 0 };
char _go_when_rdy[2] = { 0 };

// FIX! always true now for debug
bool DEVMODE = true;  // set when _devmode is set

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

bool drainSerialTo_CRorNL (void) {
    // Support hitting enter frantically to get to config menu right away on boot
    int i;
    char incomingByte = { 0 };
    bool found_CRorNL = false;
    bool found_any = false;
    for (i = 0; i < 3; i++) {
        Watchdog.reset();
        if (!Serial.available()) {
            Serial.println(F("Good: no Serial.available() ..sleep and reverify"));
            updateStatusLED();
            sleep_ms(200);
        }
        else {
            found_any = true;
            incomingByte = Serial.read(); 
            Serial.println(incomingByte);
            // FIX! 13 is ascii CR \r.
            // FIX! 10 is ascii LF \n.
            // we don't drain past CR/LF. so if you hit enter, the stuff after that stays as input
            if (incomingByte == 13) {
                Serial.println(F("Uh-oh. Found Serial incomingByte == 13 (CR)..will not drain the rest"));
                // what happens if there is \r\n...I guess it will go to the setup menu with the \n
                found_CRorNL = true;
                break;
            }
            if (incomingByte == 10) {
                Serial.println(F("Uh-oh. Found Serial incomingByte == 10 (CR)..will not drain the rest"));
                found_CRorNL = true;
                break;
            }
        }
    }
    return found_CRorNL | found_any;
}

//***********************************************************
void setup() {
    //**********************
    Watchdog.enable(30000);
    Watchdog.reset();
    // this is the usb serial. the baud rate doesn't really change usb data rates
    Serial.begin(115200);
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
    Serial.println(F("setup() START"));

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
    adc_init();

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
    GpsON(false); // no full cold gps reset

    //**********************
    setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
    updateStatusLED();
    // FIX! assume this is the state it was in before config menu? 
    // not always right. but loop will self-correct?

    if (!Serial) {  
        setStatusLEDBlinkCount(LED_STATUS_REBOOT_NO_SERIAL);
        // we're going to have to reboot..even balloon needs Serial created?
        // if serial data output buf is full, we just overflow it (on balloon)
        // DEVMODE and verbosity used to limit output?
        // reboot
        Watchdog.enable(1000);  // milliseconds
        for (;;) { 
            // FIX! put a bad status in the leds
            updateStatusLED();
        }
    }

    //**********************

    Watchdog.reset();

    Serial.println(F("Starting with println"));

    vfo_init();

    // sets minute/lane/id from chan number.
    // FIX! is it redundant at this point?..remove?
    process_chan_num();

    // "Arduino not support direct call to stdio getchar_timeout_us() is C function... but you call from CPP"
    // https://forums.raspberrypi.com/viewtopic.php?t=331207
    // https://github.com/earlephilhower/arduino-pico/discussions/2224
    // PICO_ERROR_GENERIC PICO_ERROR_TIMEOUT ??
    // int c = getchar_timeout_us(0);
    // shouldn't have to use the tud_cdc_connected() tud_cdc_available() hacks with ide
    // https://code.stanford.edu/sb860219/ee185/-/blob/master/software/firmware/circuitpython-main/supervisor/shared/serial.c

    Watchdog.reset();

    // FIX! this forces going to the user interface always on boot. don't want this 
    // for balloon, although it will time out there eventually
    // FIX! in case user is frantically trying to get to the config menu to avoid setting clock speed or ??
    // if anything was found by incomingByte above, go to the config menu 
    // (potentially a balloon weird case would timeout)

    bool found_any = drainSerialTo_CRorNL();
    // how to compare char: ..== 'R' is the same as == 82 (ascii value)
    if (found_any) {
        // Old: getchar_timeout_us(0) returns a -2 (as of sdk 2) if no keypress.
        // Must do this branching BEFORE setting clock speed in case of bad clock speed setting!
        Serial.println(F("tracker.ino: Going to user_interface() from setup()"));
        updateStatusLED();
        sleep_ms(1000);
        user_interface();
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

    bmp_init();
    i2c_scan();

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
    Serial.println(F("setup() END"));
    sleep_ms(5000);
}


//*************************************************************************
uint16_t  BeaconWait = 50;    // seconds sleep for next beacon (HF or VHF). Optimized value, do not change this if possible.
uint16_t  BattWait = 1;       // seconds sleep if super capacitors/batteries are below BattMin

// FIX! should this be non-zero?
float     GpsMinVolt = 0.0;   // min Volts for GPS to wake up.
float     BattMin = 0.0;      // min Volts to wake up.
float     WsprBattMin = 0.0;  // min Volts for HF (WSPR) radio module to transmit (TX) ~10 mW
float     HighVolt = 9.9;     // GPS is always on if the voltage exceeds this value to protect solar caps from overcharge

uint64_t GpsTimeToLastFix;    // milliseconds

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
void loop() {
    bool found_any = drainSerialTo_CRorNL();
    if (found_any) {
        Serial.println(F("tracker.ino: Going to user_interface() from loop()"));
        updateStatusLED();
        sleep_ms(1000);
        user_interface();
    }

    Watchdog.reset();
    Serial.println(F("loop() START"));
    // sleep_ms(5000);
    // copied from loop_us_end while in the loop (at bottom)

    if (loop_us_start == 0) loop_us_start = get_absolute_time();
    updateStatusLED();
    // always make sure tx is off?
    // does nothing if already off
    vfo_turn_off();
    // Gps may already be on
    // this loads the voltage
    GpsON(false);  // doesn't send cold reset

    // no need to have GpsFirstFix
    // if ( !(GpsFirstFix && (readVoltage() > BattMin)) || (!GpsFirstFix && (readVoltage() > GpsMinVolt)) )) {
    //     sleepSeconds(BattWait);

    float solar_voltage;
    solar_voltage = readVoltage();
    if ( solar_voltage <= BattMin || solar_voltage <= GpsMinVolt ) {
        sleepSeconds(BattWait);
    } else {
        Serial.println(F("loop() solar_voltage good"));
        //*********************
        // FIX! what does this do? can set time and unload NMEA sentences?
        // unload for 2 secs to make sure we get 1 sec broadcasts?
        // (also: what about corruption if buffer overrun?)
        // does CRC check cover that? so okay if we have overrun?)
        updateGpsDataAndTime(2000);
        gpsDebug();

        if (gps.location.isValid() && gps.location.age() < 1000) {
            GpsInvalidCnt = 0;
            setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
        } else {
            Serial.println(F("loop() GpsInvalidCnt++"));
            // FIX! at what rate is this incremented? ..once per loop iteration (time varies)
            GpsInvalidCnt++;
            // FIX! instead of looking for not 2000, look for valid years
            // why doesn't this get included in valid gps fix?
            if (gps.date.year() >= 2024 && gps.date.year() <= 2034)
                setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
            else
                setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

            // FIX! how fast this this get big?
            if (GpsInvalidCnt > 1000) {
                Serial.println(F("loop() GpsInvalidCnt > 1000 ..gps full cold reset"));
                // FIX! have to send cold gps reset, in case ephemeris is corrupted? since vbat is always there
                // otherwise this is a warm reset?
                GpsOFF();
                Watchdog.reset();
                sleep_ms(1000);
                GpsON(true);  // also do gps cold reset
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

        // FIX! why does isUpdated() get us past here?
        if (!gps.location.isValid() || (gps.location.age() >=1000 && !gps.location.isUpdated())) {
            sleepSeconds(BeaconWait);
        } else {
            if (!gps.satellites.isValid() || gps.satellites.value() <= 3) {
                if (DEVMODE) Serial.println(F("loop() GPS not enough satelites"));
                sleepSeconds(BeaconWait);
                // FIX! how much should we wait here?
            } else {
                if (DEVMODE) Serial.println(F("loop() GPS 3d fix good?"));
                // GpsStartTime is reset every time we turn the gps on
                // cleared every time we turn it off (don't care)
                // Should this is also cleared when we turn gps off? no?
                // floor divide to get milliseconds
                GpsTimeToLastFix = (
                    absolute_time_diff_us(GpsStartTime, get_absolute_time()) ) / 1000ULL;

                GpsInvalidCnt = 0;
                // don't snapTelemetry buffer if there is an WSPR TX window soon (any if config)
                // since we'll grab info from that buffer in sendWSPR ??
                // maybe make it less than 50

                // we can update the telemetry buffer any minute we're not tx'ing

                // FIX! this should check if gps is valid before updating?
                snapTelemetry();
                // sets all the t_* strings above
                // voltage is captured when we write the buff? So it's before GPS is turned off?
                // should we look at the live voltage instead?
                if (DEVMODE)
                    StampPrintf("loop() After snapTelemetry() timeStatus():%u minute():%u\n",
                        timeStatus(), minute());
                // make sure the buffer of prints is empty to avoid overflow
                DoLogPrint();

                // freeMem();

                // FIX! it should depend on the channel starting minute - 1 (modulo 10)
                // preparations for HF starts one minute before TX time
                // at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57.

                // FIX! why not look at seconds here?
                // it will stall until lined up on secs below
                // align to somewhere in the minute before the callsign starting minute

                // so we can start the vfo 30 seconds before needed
                if ( (readVoltage() > WsprBattMin) && alignMinute(-1) && second() > 30 ) {
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

                    // get_mh_6 returns a pointer
                    char *hf_grid6;
                    char hf_grid4[5] = "AA00";
                    hf_grid6 = get_mh_6(lat_double, lon_double);
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

                    syncAndSendWspr(0, hf_callsign, hf_grid4, hf_power, false);
                    if (DEVMODE) {
                        tx_cnt_0 += 1;
                        // we have 10 secs or so at the end of WSPR to get this off?
                        if (DEVMODE) {
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
                    if (DEVMODE) {
                        tx_cnt_1 += 1;
                        // we have 10 secs or so at the end of WSPR to get this off?
                        if (DEVMODE) {
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
                        if (DEVMODE) {
                            tx_cnt_2 += 1;
                            // we have 10 secs or so at the end of WSPR to get this off?
                            if (DEVMODE) {
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
                        if (DEVMODE) {
                            tx_cnt_3 += 1;
                            // we have 10 secs or so at the end of WSPR to get this off?
                            if (DEVMODE) {
                                StampPrintf("WSPR telen2 Tx sent. minutes %d secs %d",
                                    minute(), second());
                                DoLogPrint();  // we might have to delay this?
                            }
                        }
                    }

                    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
                    GpsON(false);  // no gps cold reset
                }
            }
        }
    }

    loop_us_end = get_absolute_time();
    loop_us_elapsed = absolute_time_diff_us(loop_us_start, loop_us_end);
    // floor divide to get milliseconds
    loop_ms_elapsed = loop_us_elapsed / 1000ULL;

    if (DEVMODE) {
        if (_verbosity[0] >= '1') {
            // maybe show GpsInvalidCnt also? how old are they
            StampPrintf(
                "t_tx_count_0: %d "
                "t_temp: %0.2f "
                "t_altitude: %0.0f "
                "t_grid6: %s "
                "t_sat_count: %d "
                "GpsTimeToLastFix %lu\n",
                t_tx_count_0, t_temp, t_voltage, t_altitude, t_grid6, t_sat_count, GpsTimeToLastFix);
        }
        if (_verbosity[0] >= '5') {
            StampPrintf(
                "main/20: _Band %s "
                "loop_ms_elapsed: %d millisecs "
                "loop_us_start: %llu microsecs "
                "loop_us_end: %llu microsecs",
                _Band, loop_ms_elapsed, loop_us_start, loop_us_end);
        }
    }
    DoLogPrint();
    updateStatusLED();

    // next start is this end
    loop_us_start = loop_us_end;
    // whenever we have spin loops we need to updateStatusLED()
    Serial.println(F("loop() END"));
}

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


void sleepSeconds(int sec) {
    Serial.println(F("sleepSeconds() START"));
    Serial.flush();
    int s = sec / 2;  // roughly 2 secs per loop?
    for (int i = 0; i < s; i++) {
        Watchdog.reset();
        // can power off gps depending on voltage
        // normally keep gps on, tracking after first fix. we are moving!
        // uint32_t usec = time_us_32();
        GpsON(false);
        float solar_voltage;

        solar_voltage = readVoltage();
        if (solar_voltage < BattMin) GpsOFF();
        if (solar_voltage < GpsMinVolt) GpsOFF();
        // FIX! should we unload/use GPS data during this?
        // gps could be on or off, so no?
        // whenever we have spin loops we need to updateStatusLED()

        updateStatusLED();
        if (GpsIsOn()) {
            // does this have a updateStatusLED() ??
            updateGpsDataAndTime(2000);  // milliseconds
            Serial.flush();
        } else {
            sleep_ms(2000);
        }
    }
    Watchdog.reset();
    // Gps gets left off it the voltage was low at any point
    Serial.println(F("sleepSeconds() END"));
}

//*******************************************************
extern const int WSPR_PWM_SLICE_NUM=4;

void PWM4_Handler(void) {
    Serial.println(F("PWM4_Handler() START"));
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    static int cnt = 0;
    if (++cnt >= 500) {
        cnt = 0;
        proceed = true;
    }
    Serial.println(F("PWM4_Handler() END"));
}

void zeroTimerSetPeriodMs(float ms) {
    Serial.println(F("zeroTimerSetPeriodMs() START"));
    static pwm_config wspr_pwm_config = pwm_get_default_config();

    pwm_config_set_clkdiv_int(&wspr_pwm_config, 250);  // 2uS
    pwm_config_set_wrap(&wspr_pwm_config, ((uint16_t)ms - 1));
    pwm_init(WSPR_PWM_SLICE_NUM, &wspr_pwm_config, false);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, PWM4_Handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
    Serial.println(F("zeroTimerSetPeriodMs() END"));
}

//********************************************
// expected this is called at least 10 secs before starting minute
// if not in the minute before starting minute,
// it will wait until the right starting minute (depends on txNum)
// txNum can be 0, 1, 2, 3
void sendWspr(int txNum, char *hf_callsign, char *hf_grid4, char *hf_power, bool vfoOffWhenDone) {
    Serial.println(F("sendWSPR() START"));
    Watchdog.reset();
    if (txNum < 0 || txNum > 3) {
        if (DEVMODE) Serial.printf("bad txNum %d ..ignoring" EOL, txNum);
        return;
    }

    // FIX! use the u4b channel freq
    uint32_t hf_freq = XMIT_FREQUENCY;
    if (atoi(_correction) != 0) {
        // this will be a floor divide
        // https://user-web.icecube.wisc.edu/~dglo/c_class/constants.html
        hf_freq = hf_freq + (atoi(_correction) * hf_freq / 1000000000UL);
    }

    // if (DEVMODE) Serial.printf("WSPR desired freq: %lu used hf_freq %u with correction %s" EOL,
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
            updateStatusLED();
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
    Serial.println(F("sendWSPR() END"));
    // Serial.println(F(__func__ " END"));
}

void syncAndSendWspr(int txNum, char *hf_callsign, char *hf_grid4, char *hf_power, bool vfoOffWhenDone) {
    Serial.println(F("syncAndSendWSPR() START"));
    // txNum is between 0 and 3..means 0,2,4,6 offsets, given the u4b channel configured
    // we turned the vfo on in the minute before 0, separately

    if (DEVMODE) {
        Serial.printf("will start WSPR txNum %d when aligned zero secs, currently %d secs\n",
            txNum, second());
        Serial.printf("WSPR txNum %d Preparing", txNum);
        Serial.printf("hf_grid4: %s", hf_grid4);
    }
    // this should be fine even if we wait a long time
    int i = 2 * txNum;  // 0, 2, 4, 6
    while (!alignMinute(i) || (second() != 0)) {
        Watchdog.reset();
        // FIX! delay 1 sec? change to pico busy_wait_us()?
        sleep_ms(1000);
        // delay(1);
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
    }

    sendWspr(txNum, hf_callsign,  hf_grid4, hf_power, vfoOffWhenDone);
    Serial.println(F("syncAndSendWSPR() END"));
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
    if (DEVMODE) return;

    // Using F() for strings
    // what happens in a Harvard architecture uC is that the compiled string stays in flash
    // and does not get copied to SRAM during the C++ initialization that happens before
    // your sketch receives run control.
    // Since the string is not moved to SRAM, it has the PROGMEM property and runs from flash.

    // FIX! do we want this? slower?
    // compile your program: it says
    // how much program memory (stored in flash) and how much dynamic ram you are using.
    // rp2040: 264 KB ram (256 KB?), 2MB flash
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

