// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

//*******************************************
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>

// These are in arduino-pio core
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries
#include <SPI.h>
#include <Wire.h>

//**************************
// flash config
#include "hardware/flash.h"

//**************************
#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
// gets the head 1.1-beta? not released version
// wget https://github.com/mikalhart/TinyGPSPlus/archive/refs/heads/master.zip

/* v1.0.a
TinyGPSPlus is a new Arduino library for parsing NMEA data streams provided by GPS modules.

1.1-beta update: Several pull requests incorporated

Added Fix Quality and Fix Mode
Slight change to earth radius
Support all satellite groups

Provides compact and easy-to-use methods for extracting position, date, time, altitude, speed, and course from consumer GPS devices.

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
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog

// needed for BMP085.h Uses Adafruit_I2CDevice.h and .cpp
// added Adafruit_BusIO to our repo 11_7_24 (libraries)
// in libraries: wget https://github.com/adafruit/Adafruit_BusIO/archive/refs/heads/master.zip
#include <Adafruit_I2CDevice.h> // https://github.com/adafruit/Adafruit_BusIO

// board uses the BMP280? (doesn't have temp)
// in libraries: wget https://github.com/adafruit/Adafruit_BMP280_Library/archive/refs/heads/master.zip
// can I put the BMP085 on there?
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_Library
// this needs the Adafruit_Sensor.h also
// in libraries: wget https://github.com/adafruit/Adafruit_Sensor/archive/refs/heads/master.zip
// https://github.com/adafruit/Adafruit_sensor
// they have accelerometer, gyroscope, magnetometer, humidity sensors, light?

// don't need BMP085 ?
// Requires the https://github.com/adafruit/Adafruit_BusIO library for I2C abstraction
// #include <Adafruit_BMP085.h> // https://github.com/adafruit/Adafruit-BMP085-Library

// wget https://github.com/adafruit/Adafruit-BMP085-Library/archive/refs/heads/master.zip
#include <JTEncode.h> // https://github.com/etherkit/JTEncode (JT65/JT9/JT4/FT8/WSPR/FSQ Encoder Library)
// in libraries: wget https://github.com/etherkit/JTEncode/archive/refs/heads/master.zip

// setTime() use moved to gps_functions.cpp
// libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip

// do we use minute() here?
#include <TimeLib.h> //https://github.com/PaulStoffregen/Time

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
// Be aware that this USB Serial port is responsible for resetting the RP2040 during the upload process,
// following the Arduino standard of 1200bps = reset to bootloader).

// The RP2040 provides two hardware-based UARTS with configurable pin selection.
// Serial1 is UART0, and Serial2 is UART1.
// FIX! should we get a bigger buffer to avoid missing ATGM336 NMEA sentences?
// The size of the receive FIFO can be adjusted from the default 32 bytes with setFIFOSize call prior to calling begin()
//     Serial1.setFIFOSize(128);
//     Serial1.begin(baud);
//
// The FIFO is normally handled via an interrupt.
// For applications where an IRQ driven serial port is not appropriate, use setPollingMode(true) before calling begin()
//     Serial1.setPollingMode(true);

const int Si5351Pwr = 4;
const int BattPin = A3;

//******************************* CONFIG **********************************
// FIX! are these used now? remnants of APRS messaging
char comment[] = "tracker 1.0";

const int WSPR_TONE_SPACING=146;  // ~1.46 Hz
const int WSPR_DELAY=683;         // Delay value for WSPR

char telemetry_buff[100] = { 0 |;  // telemetry buffer
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
uint8_t symbol_count; // is this less than 256? probaby the real max?

uint8_t tx_buffer[255]; // is this bigger than WSPR_SYMBOL_COUNT?
uint16_t tone_delay, tone_spacing;
volatile bool proceed = false;

//******************************  GPS SETTINGS   *********************************
int16_t GpsInvalidCnt=0;
int16_t TelemetryBuffInvalidCnt=0;

// gps_functions.cpp refers to this
TinyGPSPlus gps;

#include "debug_functions.h"

//*********************************
// in AdaFruit_I2CDevice.h
const int BMP280_I2C1_SDA_PIN=2;
const int BMP280_I2C1_SCL_PIN=3;
#include "bmp_functions.h"

Adafruit_BMP280 bmp;
// Adafruit_BMP805 bmp;

JTEncode jtencode;

//*********************************
extern const int STATUS_LED_PIN=25;

extern const int LED_STATUS_NO_GPS=1;
extern const int LED_STATUS_GPS_TIME=2;
extern const int LED_STATUS_GPS_FIX=3;
extern const int LED_STATUS_TX_WSPR=4;
extern const int LED_STATUS_TX_TELEMETRY=5;
extern const int LED_STATUS_TX_TELEN1=6;
extern const int LED_STATUS_TX_TELEN2=7;

#include "led_functions.h"

//*********************************
// some stuff on using namespace
// https://forum.arduino.cc/t/using-a-constant-defined-in-the-header-file/380178

// flash background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

//*********************************
// so it can be used in gps_functions.cpp
// extern is needed or the linker doesn't find it. see https://forum.arduino.cc/t/linker-problems-with-extern-const-struct/647136/2
extern const int GpsPwr=16;
// not used..GpsPwr is used
// const int GPS_VCC_ON_N_PIN=16;     // output ..this cuts VCC, leaves VBAT

// FIX! where are these used
extern const int GPS_NRESET_PIN=5; // output ..active low..should be 1?
extern const int GPS_ON_PIN=6;     // output ..this puts in sleep mode? should be 1?

// FIX! where is this used (calibration maybe)
extern const int GPS_1PPS_PIN=17;         // input

extern const int GPS_UART1_TX_PIN=8;
extern const int GPS_UART1_RX_PIN=9;

// stuff moved to functions from this .ino (not libraries)
#include "gps_functions.h"

//*********************************
// when we set both?
extern const int WSPR_TX_CLK_1_NUM=1;
// this is the other differential clock for wspr? (was aprs)
extern const int WSPR_TX_CLK_0_NUM=0;
extern const int WSPR_TX_CLK_NUM=0;

//*********************************
extern const int SI5351A_CLK_IDRV_8MA=(3 << 0);
extern const int SI5351A_CLK_IDRV_6MA=(2 << 0);
extern const int SI5351A_CLK_IDRV_4MA=(1 << 0);
extern const int SI5351A_CLK_IDRV_2MA=(0 << 0);

extern const int PLL_CALCULATION_PRECISION=4;

extern const int VFO_VDD_ON_N_PIN=4;
extern const int VFO_I2C0_SDA_PIN=12;
extern const int VFO_I2C0_SCL_PIN=13;

#include "si5351_functions.h"
// 0 should never happen (init_rf_freq will always init from saved nvram/live state)
uint32_t XMIT_FREQUENCY=0;

//*********************************
#include "u4b_functions.h"
#include "tele_functions.h"

// telemetry_buff
// init to 0 is just in case. Should always be set to something valid before use
// empty string is not valid (not sure what will happen if used while empty..I suppose it can print ok)
// always positive. clamp to 0 I guess
char t_course[4] = { 0 };     // 3 bytes + null term (like all here
// always positive? 0-250 knots. clamp to 0 I guess
char t_speed[4] = { 0 };      // 3 bytes
// 60000 meters. plus 1 in case negative?
char t_altitude[7] = { 0 };   // 6 bytes
// 24 * 30 per hour = 720 per day if every two minutes
// reboot once per day? (starts at 0)
char t_tx_count_0[4] = { 0 }; // 3 bytes
char t_temp[7] = { 0 };       // 6 bytes
char t_pressure[8] = { 0 };   // 7 bytes
char t_voltage[6] = { 0 };    // 5 bytes
char t_sat_count[3] = { 0 };  // 2 bytes
// lat/lon precision: How much to store
// https://stackoverflow.com/questions/1947481/how-many-significant-digits-should-i-store-in-my-database-for-a-gps-coordinate
// 6 decimal places represent accuracy for ~ 10 cm
// 7 decimal places for ~ 1 cm
// The use of 6 digits should be enough. +/- is 1 more. decimal is one more. 0-180 is 3 more.
// so 7 + 5 = 12 bytes should enough, with 1 more rounding digit?
char t_lat[13] = { 0 };       // 12 bytes
char t_lon[13] = { 0 };       // 12 bytes
char t_grid6[7] = { 0 };      // 6 bytes
char t_power[3] = { 0 };      // 2 bytes
// t_power is clamped to string versions of these. use 0 if illegal
// int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60}
// ad6z will differentiate from u4b (10) and traquito (13) by using 3 or 7 in normal callsign tx 
// 3 for low power, 7 for high power



// In C, there are two text related data types: char and strings. 
// A string is a pointer to an array of chars, that by convention is terminated with a NUL character ('\0'). 
// A char is a single byte representing an ASCII character.
// You create a string literal by compiling a series of characters contained between opening and closing double quotes: 
// "this is a string literal"
// You create a char literal by compiling a single character expression - possibly with a backslash escape - 
// between a pair of single quotes

//*********************************
absolute_time_t loop_us_start = 0;
absolute_time_t loop_us_end = 0;
absolute_time_t gps_us_start = 0

int64_t loop_us_elapsed;
int64_t loop_ms_elapsed;

//***********************************************************
void setup() {
    Watchdog.enable(30000);
    Watchdog.reset();
    // While the energy rises slowly with the solar panel,
    // using the analog reference low solves the analog measurement errors.

    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    // FIX! why was this commented out?
    pinMode(Si5351Pwr, OUTPUT);
    pinMode(GpsPwr, OUTPUT);
    // FIX! why was this commented out?
    pinMode(BattPin, INPUT);
    analogReadResolution(12);

    GpsOFF();
    vfo_turn_off()

    GpsINIT();
    bmp_init();

    // FIX! why is this commented out?
    // pinMode(BattPin, INPUT);
    analogReadResolution(12);

    //**********************
    // this is the usb serial. the baud rate doesn't really change usb data rates
    Serial.begin(115200);
    // Wait up to 5 seconds for serial to be opened, to allow catching
    // startup messages on native USB boards (that do not reset when serial is opened).

    // FIX! should I do this?
    while (!Serial) // Serial is via USB; wait for enumeration
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
    }

    if (Serial.read() > 0) { // read and discard data
        Serial.println("Serial.read() detected input");
    }

    // should we only look at input when IDE is not connected
    // Waits for usb serial input?
    // while (Serial.available() == 0 {
    //   ;
    // }
    // to parse the data in the serial buffer
    // Serial.parseInt();
    // Serial.parseFloat();

    Watchdog.reset();
    unsigned long start = millis();
    while (millis() - start < 5000 && !Serial) {
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
    }

    Watchdog.reset();

    Serial.println(F("Starting"));

    Wire.begin(); // somehow this is necessary for Serial2 to work properly
    vfo_init();

    Serial.print(F("WSPR (HF) CallSign: "));
    Serial.println(hf_call);
    Serial.println(F(""));

    process_chan_num(); //sets minute/lane/id from chan number. usually redundant at this point, but can't hurt

    if (getchar_timeout_us(0)>0) {
    // looks for input on USB serial port only.
    // Note: getchar_timeout_us(0) returns a -2 (as of sdk 2) if no keypress.
    // Must do this check BEFORE setting Clock Speed in Case you bricked it
        DCO._pGPStime->user_setup_menu_active=1;
        user_interface();
    }
    //***************
    if (!set_sys_clock_khz(clkhz / kHz, false)) {
        printf("%s\n RP2040 can't change clock to %dMhz. Using 133 instead\n%s",
            RED, PLL_SYS_MHZ, NORMAL);
        strcpy(_clock_speed, "133");
        write_FLASH();
        PLL_SYS_MHZ = 133;
    }

    // This should work now
    InitPicoClock(PLL_SYS_MHZ);

}

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
char _Band[3] = { 0 }; // string with 10, 12, 15, 17, 20 legal. null at end
char _tx_high[2] = { 0 }; // 0 is 2mA si5351. 1 is 8mA si5351
char _devmode[2] = { 0 };
char _correction[6] = { 0 };
char _go_when_rdy[2] = { 0 };

bool DEVMODE = false; // set when _devmode is set

//*************************************************************************
uint16_t  BeaconWait = 50;  // seconds sleep for next beacon (HF or VHF). Optimized value, do not change this if possible.
uint16_t  BattWait = 1;     // seconds sleep if super capacitors/batteries are below BattMin

// FIX! should this be non-zero?
float     GpsMinVolt = 0.0; // min Volts for GPS to wake up.
float     BattMin = 0.0;    // min Volts to wake up.
float     WsprBattMin = 0.0;// min Volts for HF (WSPR) radio module to transmit (TX) ~10 mW
float     HighVolt = 9.9;   // GPS is always on if the voltage exceeds this value to protect solar caps from overcharge

uint64_t GpsTimeToLastFix; // milliseconds
void loop() {
    // copied from loop_us_end while in the loop (at bottom)
    if (loop_us_start == 0) loop_us_start = get_absolute_time();
    Watchdog.reset();
    updateStatusLED();
    TelemetryBuffInvalidCnt++;

    if ( !(GpsFirstFix && (readVoltage() > BattMin)) || (!GpsFirstFix && (readVoltage() > GpsMinVolt)) )) {
        sleepSeconds(BattWait);
    } else {
        //*********************
        // FIX! what does this do? Just get time? Is unload of sentences interrupt-driven?
        // unload for 2 secs to make sure we get 1 sec broadcasts? (also: what about corruption if buffer overrun?)
        // does CRC check cover that? so okay if we have overrun?
        updateGpsDataAndTime(2000);
        gpsDebug();

        if (gps.location.isValid() && gps.location.age() < 1000) {
            GpsInvalidCnt=0;
            setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
        } else {
            // FIX! at what rate is this incremented? ..once per loop iteration (time varies)
            GpsInvalidCnt++;
            // FIX! instead of looking for not 2000, look for valid years
            // why doesn't this get included in valid gps fix?
            if (gps.date.year() >= 2024 && gps.date.year <= 2034) setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
            else setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

            if(GpsInvalidCnt > GpsResetTime){
                // FIX! have to send cold gps reset, in case ephemeris is corrupted? since vbat is always there
                // otherwise this is a warm reset?
                GpsOFF();
                Watchdog.reset();
                sleep_ms(1000);
                GpsON(true); // also do gps cold reset
                GpsInvalidCnt=0;
                setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
            }
        }
        updateStatusLED();

        //*********************
        // some detail on TinyGPS. precision?
        // https://sites.google.com/site/wayneholder/self-driving-rc-car/getting-the-most-from-gps

        // FIX! why does isUpdated() get us past here?
        if (! (gps.location.isValid() && (gps.location.age() < 1000 || gps.location.isUpdated())) ) {
            sleepSeconds(BeaconWait);
        }
        } else {
            if (! (gps.satellites.isValid() && gps.satellites.value() > 3)) {
                if (DEVMODE) Serial.println(F("Not enough satelites"));
                // FIX! how much should we wait here?
            }
            else {
                GpsFirstFix = true;
                // gps_us_start is reset every time we turn the gps on
                // cleared every time we turn it off (don't care)
                // Should this is also cleared when we turn gps off? no?
                // floor divide to get milliseconds
                GpsTimeToLastFix = ( absolute_time_diff_us(gps_us_start, get_absolute_time()) ) / 1000ULL;
                GpsInvalidCnt=0;

                // don't snapTelemetry buffer if there is an WSPR TX window soon (any if config)
                // since we'll grab info from that buffer in sendWSPR ??
                // maybe make it less than 50

            
                // we can update the telemetry buffer any minute we're not tx'ing
                //**************************************

                // FIX! this should check if gps is valid before updating?
                snapTelemetry();
                // sets all the t_* strings above
                // voltage is captured when we write the buff? So it's before GPS is turned off?
                // should we look at the live voltage instead?
                TelemetryBuffInvalidCnt = 0;
                if (DEVMODE) StampPrintf("After snapTelemetry() timeStatus():%u minute():%u\n", timeStatus(), minute());
                // make sure the buffer of prints is empty to avoid overflow
                DoLogPrint();

                // freeMem();

                // FIX! it should depend on the channel starting minute - 1 (modulo 10)
                // preparations for HF starts one minute before TX time
                // at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57.

                // FIX! why not look at seconds here? Oh, it will stall until lined up on secs below
                // align to somewhere in the minute before the callsign starting minute
                if ( (readVoltage() > WsprBattMin) && alignMinute(-1) ) {
                    GpsOFF();
                    // GPS will stay off for all
                    char hf_grid6[7] = { 0 };
                    hf_callsign = _callsign;
                    hf_grid6 = get_mh_6((double) t_lat, (double) t_lon);
                    hf_grid4 = hf_loc[0:3]
                    hf_grid4[4] = 0;
                    

                    char hf_power[3]  = "15";

                    setStatusLEDBlinkCount(LED_STATUS_TX_WSPR);
                    // syncAndSendWspr(char[7] hf_callsign, char[5] hf_grid4, char hf_power,  char[2]) {
                    syncAndSendWspr(hf_callsign, hf_grid4, hf_power)
                    if (DEVMODE) {
                        tx_0_cnt += 1;
                        // we have 10 secs or so at the end of WSPR to get this off?
                        if (DEVMODE) {
                            StampPrintf("WSPR callsign Tx sent. minutes %d secs %d", minutes(), seconds());
                            DoLogPrint(); // we might have to delay this?
                        }
                    }
                    // we don't loop around again caring about gps fix, because we've saved
                    // telemetry (and sensor data in there) right before the callsign tx
                    setStatusLEDBlinkCount(LED_STATUS_TX_TELEMETRY);
                    syncAndSendWspr(1)
                    if (DEVMODE) {
                        tx_1_cnt += 1;
                        // we have 10 secs or so at the end of WSPR to get this off?
                        if (DEVMODE) {
                            StampPrintf("WSPR telemetry Tx sent. minutes %d secs %d", minutes(), seconds());
                            DoLogPrint(); // we might have to delay this?
                        }
                    }
                    // have to send this if telen1 or telen2 is enabled
                    if ( (_TELEN_config[0]!='-' || _TELEN_config[1]!='-') ||
                         (_TELEN_config[2]!='-' || _TELEN_config[3]!='-') ) {
                        setStatusLEDBlinkCount(LED_STATUS_TX_TELEN1);
                        syncAndSendWspr(2)
                        if (DEVMODE) {
                            tx_2_cnt += 1;
                            // we have 10 secs or so at the end of WSPR to get this off?
                            if (DEVMODE) {
                                StampPrintf("WSPR telen1 Tx sent. minutes %d secs %d", minutes(), seconds());
                                DoLogPrint(); // we might have to delay this?
                            }
                        }
                    }
                    // have to send this if telen2 is enabled
                    if ( (_TELEN_config[2]!='-' || _TELEN_config[3]!='-') ) {
                        setStatusLEDBlinkCount(LED_STATUS_TX_TELEN2);
                        syncAndSendWspr(2)
                        if (DEVMODE) {
                            tx_3_cnt += 1;
                            // we have 10 secs or so at the end of WSPR to get this off?
                            if (DEVMODE) {
                                StampPrintf("WSPR telen2 Tx sent. minutes %d secs %d", minutes(), seconds());
                                DoLogPrint(); // we might have to delay this?
                            }
                        }
                    }

                    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
                    GpsON();
                }
            }
        }
    }


    loop_us_end = get_absolute_time();
    loop_us_elapsed = absolute_time_diff_us(loop_us_start, loop_us_end);
    // floor divide to get milliseconds
    loop_ms_elapsed = loop_us_elapsed / 1000ULL;

    if (DEVMODE) {
        if (verbosity>=1) {
            // FIX! should these be the data from the telemetry buffer?
            // maybe show GpsInvalidCnt also? how old are they
            // maybe a TelemetryBuffInvalidCnt..cleared when we load it?
            StampPrintf(t_temp: %0.2f  t_altitude: %0.0f  t_grid6: %s t_sat_count: %d GpsTimeToLastFix %d\n",
                t_temp, t_voltage, t_altitude, t_grid6, t_sat_count, GpsTimeToLastFix)
        } 
        if (verbosity>=5) {
            StampPrintf("main/20: _Band %s loop_ms_elapsed: %d millisecs loop_us_start: %llu microsecs loop_us_end: %llu microsecs",
                _Band, loop_ms_elapsed, loop_us_start, loop_us_end);
        }
    }
    DoLogPrint();
    updateStatusLED();

    // next start is this end
    loop_us_start = loop_us_end;
    // whenever we have spin loops we need to updateStatusLED()
}

// -1 is returned if anything illegal
// FIX! should check what caller does if -1
void alignMinute (int offset int ) {
    // need good time
    if ( (timeStatus() != timeSet) return false;

    // offset can be -1, 0, 1, 2, 3, 4, 5, 6 (3 messages)
    // if not one of those, set the start minute to -1?
    // caller should detect that and not wait?
    if (offset < -1 || offset > 6) {
        offset = 0
    }

    // this should have been only set to be char strs 0, 2, 4, 6, or 8
    int align_minute = atoi(_start_minute)
    switch (align_minute) {
        case 0: ;
        case 2: ;
        case 4: ;
        case 6: ;
        case 8: align_minute = (align_minute + offset) % 10 ;
        default: align_minute = 0;
    }

    // FIX! update to look at u4b channel config
    bool aligned = False;
    aligned = (minutes() % 10)) == align_minute;
    if (_go_when_rdy[0] == '1') {
        // any odd minute
        if (offset == -1) aligned = (minutes() % 2)) == 1
        // any even minute
        else aligned = (minutes() % 2)) == 0
        // telemetry buffer is also updated whenever if _go_when_ready

    }


}

void syncAndSendWspr(char[7] hf_callsign, char[5] hf_grid4, char hf_power, char[2]) {

    if (DEVMODE) {
        Serial.println("will start WSPR messageType %d when aligned zero secs, currently %d secs\n",
            messageType, seconds());
        Serial.println(F("WSPR messageType %d Preparing", messageType));
        Serial.print(F("Grid Locator: "));
        Serial.println(hf_loc);
    }

    // this should be fine even if we wait a long time
    while (! second() != 0) {
        Watchdog.reset();
        // FIX! delay 1 sec? change to pico busy_wait_us()?
        sleep_ms(1000)
        // delay(1);
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
    }

    sendWspr(hf_callsign, hf_grid4, hf_power);
    sendWSPR(messageType);
    //HFSent=true;
}

bool GpsIsOn = false;

void sleepSeconds(int sec) {
    // always make sure tx is off?
    vfo_turn_off();
    Serial.flush();
    for (int i = 0; i < sec; i++) {
        // power off gps depending on voltage, after first fix
        if (GpsFirstFix & (readVoltage() < HighVolt)) GpsOFF();
        else if (!GpsFirstFix & (readVoltage() < BattMin) GpsOFF();

        Watchdog.reset();
        uint32_t usec = time_us_32();
        while ((time_us_32() - usec) < 1000000) {
            // whenever we have spin loops we need to updateStatusLED()
            updateStatusLED();
            // FIX! should we unload/use GPS data during this?
            // gps could be on or off, so no?
            if (isGpsOn()) {
                updateGpsData(1);
                Serial.flush();
            else {
                sleep_us(1000000);
            }
        }
    }
    Watchdog.reset();
}

//*******************************************************
static pwm_config wspr_pwm_config;
void PWM4_Handler(void) {
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    static int cnt = 0;
    if (++cnt >= 500) {
        cnt = 0;
        proceed = true;
    }
}

void zeroTimerSetPeriodMs(float ms){
    wspr_pwm_config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&wspr_pwm_config, 250); // 2uS
    pwm_config_set_wrap(&wspr_pwm_config, ((uint16_t)ms - 1));
    pwm_init(WSPR_PWM_SLICE_NUM, &wspr_pwm_config, false);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, PWM4_Handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
}

//********************************************
// expected this is called at least 10 secs before starting minute
// if not in the minute before starting minute,
// it will wait until the right starting minute (depends on messageType)
// messageType can be 0, 1, 2, 3
void sendWSPR(int messageType, bool vfoOffWhenDone) {
    Watchdog.reset();
    if (messageType < 0 || messageType > 3) {
        if (DEVMODE) printf("bad messageType %d ..ignoring", messageType);
        return;
    }

    // FIX! make the drive strength conditional on the config
    // we could even make the differential dependent on the config

    // this is done in the vfo_turn_on. shouldn't need to do again
    // vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_8MA);
    // vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_8MA);
    // turns on both tx clk. The first time, has pll setup already?
    vfo_turn_on(WSPR_TX_CLK_NUM);

    // FIX! use the u4b channel freq
    unsigned long hf_freq;
    hf_freq = XMIT_FREQUENCY;
    if (atoi(_correction) != 0) {
        // this will be a floor divide
        hf_freq = hf_freq + (atoi(_correction) * hf_freq / 1000000000UL)
    }

    // if (DEVMODE) printf("WSPR desired freq: %lu used hf_freq %lu with correction %s\n",
    //    XMIT_FREQUENCY, hf_freq, _correction);

    symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;

    char[7] hf_callsign;
    char[5] hf_grid4;
    uint8_t hf_power;
    if (message_type == 3) {
        // include the null term?
        hf_callsign =
        hf_grid4 = ??
        hf_power = ??
    else if (message_type == 2) {
        // include the null term?
        hf_callsign = ??
        hf_grid4 = ??
        hf_power = ??
    else if (message_type == 1) {
        // include the null term?
        hf_callsign = ??
        hf_grid4 = ??
        hf_power = ??
    else { // default to normal callsign
        // include the null term?
        hf_callsign = _callsign;
        hf_grid4 = ??
        hf_grid4 = 17; // ad6z tracker ?
    }

    set_tx_buffer(hf_callsign, hf_grid4, hf_power, tx_buffer);

    zeroTimerSetPeriodMs(tone_delay);

    uint8_t i;
    for(i = 0; i < symbol_count; i++)
    {
        uint32_t freq_x16 =
            (hf_freq << PLL_CALCULATION_PRECISION) +
            (tx_buffer[i] * (12000L << PLL_CALCULATION_PRECISION) + 4096) / 8192L;
        // printf("%s vfo_set_freq_x16(%u)\n", __func__, (freq_x16 >> PLL_CALCULATION_PRECISION));
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

    // FIX! leave on if we're going to do telemetry?
    if (vfoOffWhenDone) {
        vfo_turn_off();
    }
    Watchdog.reset();
}

//**********************
void set_tx_buffer(const hf_callsign const char[7], const char[5] hf_loc, uint8_t hf_power, uint8_t * tx_buffer) {
    // Clear out the transmit buffer
    memset(tx_buffer, 0, 255); // is this bigger than WSPR_SYMBOL_COUNT ?
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
    jtencode.wspr_encode(hf_call, hf_loc, hf_power, tx_buffer);
    // maybe useful python for testing wspr encoding
    // https://github.com/robertostling/wspr-tools/blob/master/README.md

}

void freeMem() {
    if (DEVMODE) return;

    // Using F() for strings
    // what happens in a Harvard architecture uC is that the compiled string stays in flash and does not get copied to SRAM during the C++ initialization that happens before your sketch receives run control.
    // Since the string is not moved to SRAM, it has the PROGMEM property and runs from flash.
    // FIX! do we want this? slower?
    // When you compile your program it says how much program memory (stored in flash) you are using and
    // how much dynamic ram you are using.
    // rp2040: 264 KB ram (256 KB?), 2MB flash
    Serial.print(F("Free RAM: ")); Serial.print(freeMemory(), DEC); Serial.println(F(" byte"));
}

// were any of these needed for InitPicoClock?
// #include <stdio.h>
// #include <string.h>
// #include <ctype.h>
// #include <defines.h>
// #include "pico/stdlib.h"
// #include "hardware/clocks.h"
// #include "hardware/gpio.h"
// #include "hardware/adc.h"

int InitPicoClock(int PLL_SYS_MHZ) {
    const uint32_t clkhz = PLL_SYS_MHZ * 1000000L;

    // frequencies like 205 mhz will PANIC, System clock of 205000 kHz cannot be exactly achieved
    // should detect the failure and change the nvram, otherwise we're stuck even on reboot
    if (!set_sys_clock_khz(clkhz / kHz, false)) {
      // won't work
      printf("\nCan not set clock to %dMhz. 'pico 'Cannot be achieved''\n", PLL_SYS_MHZ);
      return -1;
    }

    printf("\n Attempt to set rp2040 clock to %dMhz (legal)\n", PLL_SYS_MHZ);
    // 2nd arg is "required"
    set_sys_clock_khz(clkhz / kHz, true);
    clock_configure(clk_peri, 0,
      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
      PLL_SYS_MHZ * MHZ,
      PLL_SYS_MHZ * MHZ);

    return 0;
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
