// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// tracker firmware:
// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

// Incorporates work by: Roman Piksaykin R2BDY. Thank you
// https://github.com/RPiks/pico-WSPR-tx

// Open source c/c++/arduino ide with arduino-pico core allows customization if you have software skills

// No auto-calibration of Si5351 Tx frequency (yet?).
// Manual config for passing 'correction" (parts per billion) to Si5351, in case Tx Frequency is outside of U4B channel bin.
// Traquito website can fingerprint callsign/telemetry frequency even if out of bin.
// LU7AA website not so well (although 'wide' helps).
// SDR can be used to report actual TX freq and correction applied to adjust.

// Any correction should be fixed: the same for all bands for a particular tracker.

// No auto-calibration of RP2040 clock frequency
// Manual config for setting rp2040 clock frequency, 115 to 250 Mhz. 115Mhz default
// (could calibrate/adjust with GPS PPS and then calibrate the Si5351 Tx frequency (signals go to RP2040 input)

//*******************************************
// Arduino IDE created by many
// Adafruit libraries created by many
// arduino-pico core https://github.com/earlephilhower/arduino-pico
// JTEncode library: https://github.com/etherkit/JTEncode
// https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/blob/main/pcb/tracker/v0.4_kbn/corrected_placement_jlcpcb.png// TinyGPSPlus library: //https://github.com/mikalhart/TinyGPSPlus
// Time library: //https://github.com/PaulStoffregen/Time
// Si5351 programming: based on work by: Kazuhisa “Kazu” Terasaki AG6NS
// U4B telemetry protocol defined by Hans Summers G0UPL
// WSPR protocol defined by Joe Taylor K1JT

// Thanks to all authors and contributors
// Thanks to the entire WSPR RX ecosystem of spotters and database maintainers

// Thanks to the tracker websites and authors/maintainers
// http://lu7aa.org/wsprx.asp Pedro Converso LU7AA
// https://traquito.github.io/channelmap/ Doug Malnati KD2KDD
// https://amateur.sondehub.org

// Thanks for knowledge/support from everyone at https://groups.io/g/picoballoon

//*******************************************

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>

// These are in arduino-pio core
// see https://github.com/earlephilhower/arduino-pico/tree/master/libraries
#include <SPI.h>
#include <Wire.h>
//**************************
// nvram? from kc3lbr
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

TinyGPSPlus’s api is considerably simpler to use than TinyGPS,
and the new library can extract arbitrary data from any of the myriad NMEA sentences out there, even proprietary ones.
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

// old
// Requires the https://github.com/adafruit/Adafruit_BusIO library for I2C abstraction
// #include <Adafruit_BMP085.h> // https://github.com/adafruit/Adafruit-BMP085-Library

// wget https://github.com/adafruit/Adafruit-BMP085-Library/archive/refs/heads/master.zip
#include <JTEncode.h> // https://github.com/etherkit/JTEncode (JT65/JT9/JT4/FT8/WSPR/FSQ Encoder Library)
// in libraries: wget https://github.com/etherkit/JTEncode/archive/refs/heads/master.zip

// not used here any more.
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
// RP2040 Helper Class.
// https://arduino-pico.readthedocs.io/en/latest/rp2040.html

// const is typed, #define macros are not.
// const is scoped by C block, #define applies to a file (compilation unit)
// const is most useful with parameter passing.
// If you see const used on a prototype with pointers, you know it is safe to pass your array or struct because the function will not alter it. No const and it can.
// example: strcpy()
// Apply "const-ness" to function prototypes at the outset.

/*
https://arduino-pico.readthedocs.io/en/latest/serial.html

Arduino-Pico core implements a software-based Serial-over-USB port using the USB ACM-CDC model
Serial is the USB serial port, and while Serial.begin() does allow specifying a baud rate,
this rate is ignored since it is USB-based.
Be aware that this USB Serial port is responsible for resetting the RP2040 during the upload process,
following the Arduino standard of 1200bps = reset to bootloader).
*/

/*
The RP2040 provides two hardware-based UARTS with configurable pin selection.
Serial1 is UART0, and Serial2 is UART1.
FIX! should we get a bigger buffer to avoid missing ATGM336 NMEA sentences?
The size of the receive FIFO may also be adjusted from the default 32 bytes by using the setFIFOSize call prior to calling begin()
    Serial1.setFIFOSize(128);
    Serial1.begin(baud);

The FIFO is normally handled via an interrupt, which reduced CPU load and makes it less likely to lose characters.
For applications where an IRQ driven serial port is not appropriate, use setPollingMode(true) before calling begin()
    Serial1.setPollingMode(true);
    Serial1.begin(300)
*/

const int Si5351Pwr=4;
const int BattPin=A3;

// FIX! does nothing?
#define Si5351ON
#define Si5351OFF   vfo_turn_off()

//******************************* CONFIG **********************************
// FIX! are these used now? remnants of APRS messaging
// Sized for APRS?
char    comment[46] = "tracker 1.0";
char    StatusMessage[50] = "tracker 1.0";

//*************************************************************************
uint16_t  BeaconWait=50;  // seconds sleep for next beacon (HF or VHF). Optimized value, do not change this if possible.

uint16_t  BattWait=1;     // seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel)
float     BattMin=0.0;    // min Volts to wake up.
float     GpsMinVolt=0.0; // min Volts for GPS to wake up. (important if power source is solar panel)
float     WsprBattMin=0.0;// min Volts for HF (WSPR) radio module to transmit (TX) ~10 mW
float     HighVolt=9.9;   // GPS is always on if the voltage exceeds this value to protect solar caps from overcharge

//******************************  HF (WSPR) CONFIG *************************************
char hf_call[7] = "NOCALL";// DO NOT FORGET TO CHANGE YOUR CALLSIGN

// const unsigned long WSPR_DEFAULT_FREQ=10140200UL; //30m band
const unsigned long WSPR_DEFAULT_FREQ=14097100UL; //20m band
// const unsigned long WSPR_DEFAULT_FREQ=18106100UL; //17M band
// const unsigned long WSPR_DEFAULT_FREQ=21096100UL; //15m band
// const unsigned long WSPR_DEFAULT_FREQ=24926100UL; //12M band
// const unsigned long WSPR_DEFAULT_FREQ=28126100UL; //10m band
// for all bands -> http://wsprnet.org/drupal/node/7352

// Supported modes, default HF mode is WSPR
enum mode {MODE_WSPR};
enum mode cur_mode = MODE_WSPR; //default HF mode

// #define DEVMODE // Development mode. Uncomment to enable for debugging.
boolean DEVMODE = true;

//******************************  APRS SETTINGS (old)****************************
boolean  aliveStatus = true; //for tx status message on first wake-up just once.
static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1; //increase +1 after every APRS transmission

//******************************  HF SETTINGS   *********************************
const int WSPR_TONE_SPACING=146; // ~1.46 Hz
const int WSPR_DELAY=683;        // Delay value for WSPR

// int HF_CORRECTION=-13000  // Change this for your ref osc
int HF_CORRECTION=0;

// Global variables
unsigned long hf_freq;
char hf_message[13] = "NOCALL AA00";//for WSPR, updated by hf_call and GPS location
char hf_loc[] = "AA00";             //for WSPR, updated by GPS location. You don't have to change this.
uint8_t dbm = 10;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
volatile bool proceed = false;

//******************************  GPS SETTINGS   *********************************
int16_t   GpsResetTime=1800; // timeout for reset if GPS is not fixed

// FIX! removed all geofence, not used for wspr

boolean GpsFirstFix = false; //do not change this
boolean ublox_high_alt_mode_enabled = false; //do not change this

// how many loop iterations where since it's been grabbed to the buffer?
int16_t GpsInvalidTime=0;
int16_t TelemetryInvalidTime=0;
int16_t SensorIvalidTime=0;

// gps_functions.cpp refers to this
TinyGPSPlus gps;

#include "debug_functions.h"

//*********************************
// in AdaFruit_I2CDevice.h
// theWire The I2C bus to use, defaults to &Wire
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

//*********************************
#include "u4b_functions.h"
// 0 should never happen (init_rf_freq will always init from saved nvram/live state)
uint32_t XMIT_FREQUENCY=0;

//*********************************
absolute_time_t loop_us_start = 0;
absolute_time_t loop_us_end = 0;
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
    // FIX! why is this commented out?
    // pinMode(Si5351Pwr, OUTPUT);
    pinMode(GpsPwr, OUTPUT);
    // FIX! why is this commented out?
    // pinMode(BattPin, INPUT);
    analogReadResolution(12);

    GpsOFF();
    Si5351OFF;
    GpsINIT();
    bmp_init();

    //**********************
    Serial.begin(115200);
    // Wait up to 5 seconds for serial to be opened, to allow catching
    // startup messages on native USB boards (that do not reset when serial is opened).

    // FIX! should I do this?
    while (!Serial) { ; } // Serial is via USB; wait for enumeration

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
    while (millis() - start < 5000 && !Serial){;}
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

/*
Verbosity:
0: none
1: temp/volts every second, message if no gps
2: GPS status every second
3: messages when a Tx started
4: x-tended messages when a Tx started
5: dump context every 20 secs
6: show PPB every second
7: Display GxRMC and GxGGA messages
8: display ALL NMEA sentences from GPS module
9: same as 8
*/

void loop() {
    // copied from loop_us_end while in the loop (at bottom)
    if (loop_us_start == 0) loop_us_start = get_absolute_time();

    Watchdog.reset();
    updateStatusLED();

    TelemetryInvalidTime++;
    SensorInvalidTime++;

    if ( !(GpsFirstFix && (readBatt() > BattMin)) || (!GpsFirstFix && (readBatt() > GpsMinVolt)) )) {
        sleepSeconds(BattWait);
    } else {
        // FIX! what does this do? Just get time?
        updateGpsData(1000);
        gpsDebug();

        if (gps.location.isValid() && gps.location.age() < 1000) {
            GpsInvalidTime=0;
            setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
        } else {
            // FIX! at what rate is this incremented? what looks at it?
            GpsInvalidTime++;
            // FIX! instead of looking for not 2000, look for valid years
            // why doesn't this get included in valid gps fix?
            // if (gps.date.year() != 2000) setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
            if (gps.date.year() > 2000 && gps.date.year < 2030) setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
            else setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

            if(GpsInvalidTime > GpsResetTime){
                GpsOFF();
                Watchdog.reset();
                delay(1000);
                GpsON();
                // FIX! wait some for maybe gps hot fix to come on. 10 secs?
                delay(5000);
                GpsInvalidTime=0;
                if (gps.date.year() > 2000 && gps.date.year < 2030) setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
                else setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
            }
        }

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
                GpsInvalidTime=0;

                // FIX! it should depend on the channel starting minute - 1 (modulo 10)
                // preparations for HF starts one minute before TX time 
                // at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57.
                if (DEVMODE) printf("timeStatus():%u minute():%u\n", timeStatus(), minute());

                // FIX! why not look at seconds here? Oh, it will stall until lined up on secs below
                bool doSerialFlush = false;
                if (
                    (readBatt() > WsprBattMin) && 
                    (timeStatus() == timeSet) && u4bStartMinute()
                    ) {

                    // could we be too close to the start of sending? for this off transition? 
                    GpsOFF();
                    // FIX! change the message sent depending on where we are relative to start minute?
                    // FIX! add parameter for 1, 2, 3, 4 consecutive U4B Tx
                    // GPS will stay off for all
                    syncAndSendWspr(1)
                    // syncAndSendWspr(2)
                    // FIX! when do we grab/buffer the extra sensor data? grab it during updateTelemetry?
                    // maybe have updateTelenSensors() ?

                    // syncAndSendWspr(3)
                    // syncAndSendWspr(4)
                    if (DEVMODE) Serial.println(F("WSPR callsign Tx sent"));
                    // FIX! should we sneak in the 2nd Tx here? 

                    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
                    GpsON();
                    // FIX! is it okay to flush after Tx but before the telemetry Tx ?
                    doSerialFlush = true;

                }

                // don't updateTelemetry buffer if there is an HF (WSPR) TX window soon
                // since we'll grab info from that buffer in sendWSPR ??
                // maybe make it less than 50
                
                // FIX! shouldn't change it during the whole plus 2 minute either since telemetry is going to use it 
                // only blocks for 120 secs + 10 secs (out of 10 minute cycle)
                if ( (timeStatus() == timeSet) && 
                    (!u4bStartMinuteP2 || (u4bStartMinute && (second < 50))) ) {
                    // FIX! this should check if gps is valid before updating
                    updateTelemetry();
                    TelemetryInvalidTime = 0;
                    // freeMem();
                }
                if ( (timeStatus() == timeSet) && 
                    (!(u4bStartMinuteP4 | u4bStartMinuteP6) || (u4bStartMinuteP2 && (second < 50))) ) {
                    updateTelenSensors();
                    SensorInvalidTime = 0;
                }
                // FIX! could this take more than 10 secs?
                if (DEVMODE && doSerialFlush) Serial.flush();
            }
        }
    }

    if (verbosity>=1) {
        // FIX! should these be the data from the telemetry buffer?
        // maybe show GpsInvalidTime also? how old are they
        // maybe a TelemetryInvalidTime..cleared when we load it?
        StampPrintf("Temp: %.1f  Volts: %0.2f  Altitude: %0.0f  Satellite count: %d grid6: %s\n",
            tempU,volts,_altitude, sat_count, grid6);
    }

    DoLogPrint();
    //***************
    // FIX ?? put in a conditional delay that depends on clock frequency
    // faster cpu clock will want more delay? (won't affect the PIO block doing RF)
    // time the loop

    // static uint64_t to_us_since_boot    ( absolute_time_t t    )
    // convert an absolute_time_t into a number of microseconds since boot.
    //****************

    loop_us_end = get_absolute_time();
    loop_us_elapsed = absolute_time_diff_us(loop_us_start, loop_us_end);
    // floor divide to get milliseconds
    loop_ms_elapsed = loop_us_elapsed / 1000ULL;

    if (verbosity>=5) {
        if(0==(tick % 20)) { // every ~20 * 0.5 = 10 secs
        StampPrintf("main/20: _Band %s loop_ms_elapsed: %d millisecs loop_us_start: %llu microsecs loop_us_end: %llu microsecs", 
            _Band, loop_ms_elapsed, loop_us_start, loop_us_end);
        }
    }

    // next start is this end
    loop_us_start = loop_us_end;
    // will always 0 or greater? (unless bug with time)
    /*
    if ((loop_ms_elapsed < 500) && (loop_ms_elapsed > 0)) {
        sleep_ms(500 - loop_ms_elapsed);
    }
    */
}

void u4bStartMinute {
    // FIX! update to look at u4b channel config
    return ((minute() % 10 == 3) || (minute() % 10 == 7)) 
}

// telemetry
void u4bStartMinuteP2 { // plus 2 minutes (modulo 10)
    // FIX! update to look at u4b channel config
    return ((minute() % 10 == 3) || (minute() % 10 == 7)) 
}
// telen1
void u4bStartMinuteP4 { // plus 2 minutes (modulo 10)
    // FIX! update to look at u4b channel config
    return ((minute() % 10 == 3) || (minute() % 10 == 7)) 
}
// telen2
void u4bStartMinuteP6 { // plus 2 minutes (modulo 10)
    // FIX! update to look at u4b channel config
    return ((minute() % 10 == 3) || (minute() % 10 == 7)) 
}

void syncAndSendWspr() {
    printf("will start WSPR when aligned to the minute\n");
    // FIX! it should use the captured telemetry, not live gps
    GridLocator(hf_loc, gps.location.lat(), gps.location.lng());
    sprintf(hf_message, "%s %s", hf_call, hf_loc);

    if (DEVMODE) {
        Serial.println(F("Digital HF Mode Preparing"));
        Serial.print(F("Grid Locator: "));
        Serial.println(hf_loc);
    }

    // HF transmission starts at minute 4, 8, 14, 18, 24, 28, 34, 38, 44, 48, 54 or 58
    // FIX! start on the starting minute of the U4B channel
    while (((minute() % 10 != 4) || (minute() % 10 != 8)) && second() != 0) {
        Watchdog.reset();
        delay(1);
        updateStatusLED();
    }

    if (DEVMODE) Serial.println(F("Digital HF Mode Sending..."));

    setStatusLEDBlinkCount(LED_STATUS_TX_WSPR);
    sendWSPR();
    //HFSent=true;
}

void sleepSeconds(int sec) {
    Si5351OFF;
    Serial.flush();
    for (int i = 0; i < sec; i++) {
        // sleep gps after first fix
        if (GpsFirstFix & (readBatt() < HighVolt)) GpsOFF();
        else if (!GpsFirstFix & (readBatt() < BattMin) GpsOFF();
    }
}

Watchdog.reset();

uint32_t usec = time_us_32();
while ((time_us_32() - usec) < 1000000) {
    updateStatusLED();
}

}
Watchdog.reset();
}


void updatePosition(int high_precision, char *dao) {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  RawDegrees rawDeg = gps.location.rawLat();
  uint32_t min_nnnnn;
  // not used
  // char lat_dao = 0;
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100)*5)) ) {
    // round up. Avoid overflow (59.999999 should never become 60.0 or more)
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100)*5;
  }
  sprintf(latStr, "%02u%02u.%02u%c", (unsigned int ) (rawDeg.deg % 100), (unsigned int ) ((min_nnnnn / 100000) % 100), (unsigned int ) ((min_nnnnn / 1000) % 100), rawDeg.negative ? 'S' : 'N');
  if (dao)
    dao[0] = (char) ((min_nnnnn % 1000) / 11) + 33;

  // FIX! what is this
  // APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  rawDeg = gps.location.rawLng();
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100)*5)) ) {
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100)*5;
  }
  sprintf(lonStr, "%03u%02u.%02u%c", (unsigned int ) (rawDeg.deg % 1000), (unsigned int ) ((min_nnnnn / 100000) % 100), (unsigned int ) ((min_nnnnn / 1000) % 100), rawDeg.negative ? 'W' : 'E');
  if (dao) {
    dao[1] = (char) ((min_nnnnn % 1000) / 11) + 33;
    dao[2] = 0;
  }

  // FIX! what is this?
  // APRS_setLon(lonStr);
  // APRS_setTimeStamp(gps.time.hour(), gps.time.minute(),gps.time.second());
}


void updateTelemetry() {
    // FIX! why does isUpdated() get us past here?
    if (! (gps.location.isValid() && (gps.location.age() < 1000 || gps.location.isUpdated())) ) {
        return
    }
    if (! (gps.satellites.isValid() && gps.satellites.value() > 3)) {
        return
    }
    sprintf(telemetry_buff, "%03d", gps.course.isValid() ? (int)gps.course.deg() : 0);
    telemetry_buff[3] = '/';

    sprintf(telemetry_buff + 4, "%03d", gps.speed.isValid() ? (int)gps.speed.knots() : 0);
    telemetry_buff[7] = '/';

    telemetry_buff[8] = 'A';
    telemetry_buff[9] = '=';
    //sprintf(telemetry_buff + 10, "%06lu", (long)gps.altitude.feet());

    //fixing negative altitude values causing display bug on aprs.fi
    float tempAltitude = gps.altitude.feet();

    if (tempAltitude>0){
    //for positive values
    sprintf(telemetry_buff + 10, "%06lu", (long)tempAltitude);
    } else{
    //for negative values
    sprintf(telemetry_buff + 10, "%06d", (int)tempAltitude);
    }

    telemetry_buff[16] = ' ';
    sprintf(telemetry_buff + 17, "%03d", TxCount);
    telemetry_buff[20] = 'T';
    telemetry_buff[21] = 'x';
    telemetry_buff[22] = 'C';

    // FIX! don't need anymore?
    // Si5351ON; //little hack to prevent a BMP180 related issue (does nothing?)
    delay(1);

    telemetry_buff[23] = ' '; float tempC = bmp_read_temperature();
    // telemetry_buff[23] = ' '; float tempC = 0.f;
    dtostrf(tempC, 6, 2, telemetry_buff + 24);

    telemetry_buff[30] = 'C';
    telemetry_buff[31] = ' '; float pressure = bmp_read_pressure() / 100.0; //Pa to hPa
    // telemetry_buff[31] = ' '; float pressure = 0.f; //Pa to hPa
    dtostrf(pressure, 7, 2, telemetry_buff + 32);

    // FIX! don't need anymore?
    // Si5351OFF;

    telemetry_buff[39] = 'h';
    telemetry_buff[40] = 'P';
    telemetry_buff[41] = 'a';
    telemetry_buff[42] = ' ';

    dtostrf(readBatt(), 5, 2, telemetry_buff + 43);
    telemetry_buff[48] = 'V';
    telemetry_buff[49] = ' ';

    sprintf(telemetry_buff + 50, "%02d", gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
    telemetry_buff[52] = 'S';
    telemetry_buff[53] = ' ';
    sprintf(telemetry_buff + 54, "%s", comment);

    // FIX! why?
    // remove temperature and pressure info
    // memmove(&telemetry_buff[24], &telemetry_buff[43], (sizeof(telemetry_buff) - 43));

    if (DEVMODE) Serial.println(telemetry_buff);

}

//****************************************************
void sendLocation() {
    if (DEVMODE) Serial.println(F("Location sending with comment");
    // FIX! how do we end the telemetry_buff
    int GEOFENCE_APRS_frequency=0;

    // vfo_set_drive_strength(APRS_TX_CLK_NUM, SI5351A_CLK_IDRV_8MA);
    // vfo_turn_on(APRS_TX_CLK_NUM);
    // vfo_set_freq_x16(APRS_TX_CLK_NUM, (GEOFENCE_APRS_frequency << PLL_CALCULATION_PRECISION));
    delay(500);
    // FIX! how do we end the telemetry_buff
    // APRS_sendLoc(telemetry_buff);
    delay(10);
    vfo_turn_off();
    Serial.print(F("APRS Location sent (Freq: "));
    Serial.print(GEOFENCE_APRS_frequency);
    Serial.print(F(") - "));
    Serial.println(TxCount);
    TxCount++;
}

void sendStatus() {
    // FIX! how do we end the telemetry_buff
    int GEOFENCE_APRS_frequency=0;

    // vfo_set_drive_strength(APRS_TX_CLK_NUM, SI5351A_CLK_IDRV_8MA);
    // vfo_turn_on(APRS_TX_CLK_NUM);
    // vfo_set_freq_x16(APRS_TX_CLK_NUM, (GEOFENCE_APRS_frequency << PLL_CALCULATION_PRECISION));
    delay(500);
    // APRS_sendStatus(StatusMessage);
    delay(10);
    vfo_turn_off();
    Serial.print(F("Status sent (Freq: "));
    Serial.print(GEOFENCE_APRS_frequency);
    Serial.print(F(") - "));
    Serial.println(TxCount);
    TxCount++;
}

float readBatt() {
  int adc_val = 0;
  adc_val = analogRead(BattPin);
  adc_val += analogRead(BattPin);
  adc_val += analogRead(BattPin);
  // The Raspberry Pi Pico's analog to digital converter (ADC) can measure voltages between 0 and 3.3 volts.
  // The ADC uses a 3.3V reference voltage,
  // and a read operation returns a number between 0 and 4095.
  // The ADC's resolution is 3.3/4096, or roughly 0.8 millivolts.
  // is the precision set to 4096? (12 not 16 bits resolution)
  // 4096/3.3 = 1241
  // 1241 / 3 = 413.66

  // FIX! this doesn't seem right. should I just multiply by the conversion factor
  // he's got some special 1/3 voltage divider for VBUS to BATT_V
  // you leave it open than the ADC converter voltage reference is the 3.3V .
  // In reality it is the voltage of the pin 3V3 - ( ~150uA * 200) which is roughly a 30mv drop. (0.8mv * 30 = 24 steps)

  // this must be a calibrated linear equation? only need to calibrate between 2.8v and 5v?
  float solar_voltage = ((float)adc_val / 3.0f - 27.0f) / 412.0f;
  // there is a 200 ohm resistor between 3V3 and ADC_AVDD
  // we did 3 reads above ..averaging? so don't need the 3x because of onboard voltage divider
  // pico-WSPRer does this (no use of ADC_AVDD) ?
  // const float conversionFactor = 3.3f / (1 << 12);
  // float solar_voltage = 3 * (float)adc_read() * conversionFactor;

  // if (solar_voltage < 0.0f) solar_voltage = 0.0f;
  // if (solar_voltage > 9.9f) solar_voltage = 9.9f;
  return solar_voltage;
}

#define WSPR_PWM_SLICE_NUM  4
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
void sendWSPR() {
    Watchdog.reset();
    vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_8MA);
    vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_8MA);
    vfo_turn_on(WSPR_TX_CLK_NUM);

    // do we have to turn on the differential clock?
    // they share a pll ? so the differential enable ifdef should handle that?
    // vfo_turn_on(WSPR_TX_CLK_1_NUM);
    uint8_t i;

    // FIX! use the u4b channel freq
    hf_freq = WSPR_DEFAULT_FREQ - 100 + (rand() % 200);
    if (DEVMODE) printf("WSPR freq: %lu\n", hf_freq);

    symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;
    set_tx_buffer();

    zeroTimerSetPeriodMs(tone_delay);

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
            updateStatusLED();
        }
        Watchdog.reset();
    }
    pwm_set_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, false);
    pwm_clear_irq(WSPR_PWM_SLICE_NUM);
    irq_set_enabled(PWM_IRQ_WRAP, false);
    irq_remove_handler(PWM_IRQ_WRAP, PWM4_Handler);
    vfo_turn_off();
    Watchdog.reset();
}

//********************************************
void set_tx_buffer() {
    // Clear out the transmit buffer
    memset(tx_buffer, 0, 255);
    // Set the proper frequency and timer CTC
    jtencode.wspr_encode(hf_call, hf_loc, dbm, tx_buffer);
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
      printf("\nCan not set system clock to %dMhz. 'pico 'Cannot be achieved''\n", PLL_SYS_MHZ);
      return -1;
    }

    printf("\n ATTEMPT TO SET SYSTEM KLOCK TO %dMhz (legal)\n", PLL_SYS_MHZ);
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
// options:

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
