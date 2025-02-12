// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// REMEMBER: no references to Serial.* or usb in BALLOON_MODE!

// which gps chip is used?
// ATGM336N if false
extern bool USE_SIM65M;
// change it if we have the 5sec fix/broadcast on USE_SIM65M
extern int GPS_WAIT_FOR_NMEA_BURST_MAX;

// Don't reconfig if not necessary
bool WARM_RESET_REDO_CONFIG = false;
bool SIM65M_BROADCAST_5SECS = false;

//*******************************************
// Reference docs (for SIM28 but should apply)
// can download all from https://simcom.ee/documents/?dir=SIM28
// SIM28M
// https://simcom.ee/documents/?dir=SIM28M
// SIM28ML
// https://simcom.ee/documents/?dir=SIM28ML
// https://forum.arduino.cc/t/configuration-of-chinese-atgm336-gnss/1265640

//*******************************************
// ATGM336H uses AT6558 silicon ??
// AT6558 BDS/GNSS Full Constellation SOC Chip Data Sheet Version 1.14
// AT6558-5N-3X is GPS + BDS
// QFN package 40 pin 5x5x0.8mm
// https://www.icofchina.com/d/file/xiazai/2016-12-05/b1be6f481cdf9d773b963ab30a2d11d8.pdf

// says VDD_POR going low causes internal reset (nRESET)
// nRST pin going low causes internal reset (nRESET)
// nRST can be low while power is transition on, or it can be asserted/deasserted afer power on
// tcxo_xref needs to be running

//*******************************************
// Printing too much
// Many programmers run into trouble because they try to print too much debug info.
// Serial.print() will "block" until output characters can be stored in a buffer.
// While blocked at V1_print, GPS is probably still sending data.
// The input buffer on a rp2040 is only 32 bytes
// After 32 bytes have been received stored, all other data is dropped!

// It is crucial to call gps.available( gps_port ) or Serial2.read() frequently,
// and never to call a blocking function that takes more than
// (1/baud) = 104 usecs @ 9600 baud.. but: effective baud rate from gps chip is less than peak
// though?  Don't depend on depth 32 to absorb delay?

// I decided to use polling mode, not be interrupt driven on Serial2 data.
// more deterministic behavior? dunno.

//**************************************************
// SIM65M
// Getting AG3352 (core gps chip from Airoha company) in version info from SIM65M
// https://www.airoha.com/products/p/zy4r082hgNywp1bg
// AG3352 series
// The new generation of single-frequency GNSS,
// in addition to the continuation of the 3352 features,
// adds the B1C and L1C frequency bands of Beidou-3 and GPS,
// making the positioning accuracy close to the dual-frequency standard,
//**************************************************

#include <Arduino.h>
// for isprint()
#include <ctype.h>
#include "gps_functions.h"
#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"
#include "time_functions.h"
// enums for voltage at:
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h
#include "hardware/vreg.h"

// for disabling pll
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__pll.html
#include "hardware/pll.h"

// for setting drive strength/slew rate of gpio
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
// ugh, do we need to include this for tusb_init()
#include "hardware/gpio.h"

// for re-init of the tinyUSB
#include "tusb.h"

// in libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// for setTime()
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// gps+bds+glonass
// #define DEFAULT_CONSTELLATIONS_CHOICE 7
// gps+bds
// #define DEFAULT_CONSTELLATIONS_CHOICE 3
// gps
#define DEFAULT_CONSTELLATIONS_CHOICE 1

// object for TinyGPSPlus state
extern TinyGPSPlus gps;

extern const int GpsPwr;
extern const int GPS_NRESET_PIN;  // connected!
extern const int GPS_ON_PIN;

// input..not used..calibration?
extern const int GPS_1PPS_PIN;

extern const int GPS_UART1_RX_PIN;
extern const int GPS_UART1_TX_PIN;

extern const int SERIAL2_FIFO_SIZE;
extern const int ATGM336H_BAUD_RATE;
extern const int SIM65M_BAUD_RATE;

// for tracking gps fix time. we only power gps on/off..we don't send it gps reset commands
extern absolute_time_t GpsStartTime;  // usecs

// decode of verbose 0-9
extern bool VERBY[10];

extern uint32_t GpsInvalidAllCnt;
extern bool GpsInvalidAll;
// to avoid servicing keyboard while in aggressive power transition
extern bool IGNORE_KEYBOARD_CHARS;

// FIX! gonna need an include for this? maybe note
// # include <TimeLib.h>

// we make RP2040 to 18mhz during the long gps cold reset fix time
// then restore to this
extern uint32_t PLL_SYS_MHZ;  // decode of _clock_speed
extern bool BALLOON_MODE;

//************************************************
// false and true work here
bool PWM_GPS_POWER_ON_MODE = false;

bool ALLOW_UPDATE_GPS_FLASH_MODE = false;
// causing intermittent fails if true?
bool ALLOW_LOWER_CORE_VOLTAGE_MODE = false;

// why isn't this true 12/15/24..true now. works (18Mhz)
// occassionally having problems...Not needed?
// bool ALLOW_USB_DISABLE_MODE = true;
bool ALLOW_USB_DISABLE_MODE = false;
// true not working with Serial2?
bool ALLOW_KAZU_12MHZ_MODE = false;
// is true getting intermittent fails on usbserial (after restore)
bool ALLOW_TEMP_12MHZ_MODE = false;

//************************************************
static bool GpsIsOn_state = false;
bool GpsIsOn(void) {
    return GpsIsOn_state;
}

//************************************************
// Is the maximum length of any in or out packet = 255 bytes?
#define NMEA_BUFFER_SIZE 8 * 255
static char nmeaBuffer[NMEA_BUFFER_SIZE] = { 0 };

//************************************************
// subfunction just to have consistent incomingChar/charsAvailable management
// in updateGpsDataAndTime() and nmeaBufferFastPoll()
char incomingChar;
int charsAvailable;
void getChar() {
    // setup for next loop iteration
    charsAvailable = Serial2.available();
    if (charsAvailable) incomingChar = Serial2.read();
    else incomingChar = '0';
}
//************************************************
// tries to get all data from gps without losing any, for a blocking period of time
// loops as fast as possible into a ram buffer
void nmeaBufferFastPoll(uint64_t duration_millis, bool printIfFull) {
    V1_println(F(EOL "nmeaBufferFastPoll START"));
    uint64_t start_millis = millis();
    // nmeaBuffer should be empty the first time we use this?
    // should be no harm (delay) in checking here?
    nmeaBufferPrintAndClear();
    bool spaceChar, nullChar, printable;
    while (millis() - start_millis < duration_millis) {
        // set globals: incomingChar, charsAvailable
        getChar();
        while (charsAvailable) {
            // do we get any null chars?
            // are CR LF unprintable?
            spaceChar = false;
            nullChar = false;
            printable = isprint(incomingChar);
            // good to eliminate garbage to save buffer room
            // we'll add appropriate EOLs when printing buffer
            // FIX! will there be enough garbage visible when baud rate is wrong
            // that we'll still see bad baud rate issues?
            switch (incomingChar) {
                case '\0': nullChar = true; break;
                case ' ':  spaceChar = true; break;
                default: { ; }
            }
            if (!spaceChar && !nullChar && printable) {
                nmeaBufferAndPrint(incomingChar, printIfFull);
            }
            getChar();
        }
        busy_wait_ms(1);  // just wait 1 milli?
    }
    nmeaBufferPrintAndClear();
    V1_println(F("nmeaBufferFastPoll END"));
}

//***************************************************
// Outputs the content of the nmea buffer to stdio (UART and/or USB)
void nmeaBufferPrintAndClear(void) {
    if (!VERBY[1]) return;

    if (nmeaBuffer[0] != 0) {
        // don't add an EOL to the print since we can accumulate multiple to look good?
        // Might have been missing a EOL. Add one
        V1_println(nmeaBuffer);
        nmeaBuffer[0] = 0;  // Clear the buffer
    }
    // whenever something might have taken a long time like printing the big buffer
    updateStatusLED();
    Watchdog.reset();
}

// add one char at a time
void nmeaBufferAndPrint(const char charToAdd, bool printIfFull) {
    if (!VERBY[1]) return;

    // we might add a EOL before a '$' that begins a sentence. so check for +2
    // EOL might be /r /n or /r/n (two chars). so check for 3.
    // possible 2 in front. 0 null term at end
    if ((strlen(nmeaBuffer) + 3) >= NMEA_BUFFER_SIZE) {
        // make NMEA_BUFFER_SIZE bigger or
        // can just do more nmeaBufferPrint() if we run into a problem realtime
        // we shouldn't have to add EOL to the sentences. they come with CR LF ?
        V1_printf(
            "WARNING: with NMEA_BUFFER_SIZE %d strlen(nmeaBuffer) %d "
            "there is no room for char %c <newline>",
            NMEA_BUFFER_SIZE, strlen(nmeaBuffer), charToAdd);
        V1_println(F("..flushing by emptying first (no print)"));
        // we can't afford to print it before flushing..we'll drop NMEA incoming in the UART
        if (printIfFull) nmeaBufferPrintAndClear();
        else nmeaBuffer[0] = 0;
    }

    int n = strlen(nmeaBuffer);
    if (charToAdd == '$') {
        // put a EOL in first (at the start of every NMEA sentence)
        // https://subethasoftware.com/2024/08/26/in-c-you-can-sizeof-a-string-constant/
        // sizeof EOL will have the null term, so keeping nmeaBuffer always good
        // doing the extra + 1 will put the 0 null term in, also
        strncpy(nmeaBuffer + n, EOL, strlen(EOL) + 1);
        n += strlen(EOL);
    }
    nmeaBuffer[n] = charToAdd;
    nmeaBuffer[n + 1] = 0;
}

// ************************************************
void gpsSleepForMillis(int n, bool enableEarlyOut) {
    // FIX! should we do this here or where?
    Watchdog.reset();
    if (n < 0 || n > 120000) {
        // V1_printf("ERROR: gpsSleepForMillis() n %d too big (120000 max)" EOL, n);
        // n = 1000;
        // UPDATE: this is used while USB is disabled,
        // but BALLOON_MODE/VERBY don't protect us ..just don't print here
    }
    int milliDiv = n / 10;
    // sleep approx. n millisecs
    for (int i = 0; i < milliDiv ; i++) {
        if (enableEarlyOut) {
            if (Serial2.available()) break;
        }
        // https://docs.arduino.cc/language-reference/en/functions/time/delay/
        // check for update every 10 milliseconds
        if ((milliDiv % 10) == 0) {
            // no prints in this
            updateStatusLED();
            Watchdog.reset();
        }
        // faster recovery with delay?
        delay(10);
    }
}

//************************************************
int checkGpsBaudRate(int desiredBaud) {
    int usedBaud = desiredBaud;
    switch (desiredBaud) {
        case 4800: break;
        case 9600: break;
        case 19200: break;
        case 39400: break;
        case 57600: break;
        case 115200: break;
        default: usedBaud = 9600;
    }
    return usedBaud;
}

//************************************************
bool getInitialGpsOutput(void) {
    V1_println(F("getInitialGpsOutput START"));
    // there can be a lot of bogus chars after warm/cold reset, like over 200
    // if we can get effective 900 chars/sec, probably want 5x that
    V1_println(F("Look for some Serial2 bytes for 5 secs or 5000 chars or 2 sentences"));
    char incomingChar = { 0 };
    uint32_t incomingCharCnt = 0;
    uint32_t incomingSentenceCnt = 0;
    // we drain during the GpsINIT now, oh. we should leave gps on so we get chars
    // 5 secs?
    uint64_t start_millis = millis();
    uint64_t duration_millis = 0;
    while (duration_millis < 5000) {
        Watchdog.reset();
        if (!Serial2.available()) {
            ;
        } else {
            while (Serial2.available()) {
                incomingChar = Serial2.read();
                // buffer it up like we do normally below, so we can see sentences
                // skip any non-printable, as we won't be able to dos2unix the putty.log if in there
                bool printable = isprint(incomingChar);
                if (printable) {
                    nmeaBufferAndPrint(incomingChar, true);  // print if full
                    incomingCharCnt += 1;
                    if (incomingChar == '$') incomingSentenceCnt += 1;
                }
            }
            if (incomingCharCnt >= 5000) break;
            if (incomingSentenceCnt >= 2) break;
        }
        gpsSleepForMillis(1000, true);  // return early if Serial2.available()
        duration_millis = millis() - start_millis;
    }
    nmeaBufferPrintAndClear();
    updateStatusLED();
    Watchdog.reset();
    V1_println(F("getInitialGpsOutput END"));

    return (incomingSentenceCnt >= 2);
}

//************************************************

void setGpsBalloonMode(void) {
    V1_println(F("setGpsBalloonMode START"));
    //************************
    // Interesting! what kind of extra debuglog output?
    // SIM65M
    // 2.3.54 Packet Type:087 PAIR_COMMON_GET_DEBUGLOG_OUTPUT
    // Query setting of debug log output.
    // $PAIR087*CS<CR><LF>
    // 2. $PAIR087,<Status>*<checksum>
    // DataField:
    // 0: Disable
    // 1: Enable with full debuglog output
    // 2: Enable with lite debuglog output
    // Example:
    // $PAIR087,1*28*35 ==> Enable Debuglog output

    //************************
    // SIM65M
    // wow! just noticed the default mode is fitness mode
    // FIX! should change it to general purpose
    // Packet Type:080 PAIR_COMMON_SET_NAVIGATION_MODE
    // Set navigation mode
    // $PAIR080,<CmdType>*<checksum>
    // '0' Normal mode: For general purpose
    // '1' [Default Value] Fitness mode: For running and walking activities
    //     so that the low-speed (< 5 m/s) movement will have more of an effect
    //     on the position calculation.
    // '2' Reserved Could this be Aviation mode?
    // '3' Reserved Could this be Balloon mode?
    // quectel_l70.pdf has 3 = balloon mode (2015)
    // '4' Stationary mode: For stationary applications where a zero dynamic assumed.
    // '5' Reserved
    // is this really drone mode
    // Drone mode: used for drone applications with equivalent dynamics range
    // and vertical application on different flight phase (Ex. Hovering, cruising etc)
    // '6' Reserved
    // '7' Swimming mode: For swimming purpose so that it smooths the trajectory and
    //     improves the accuracy of distance calculation.
    //************************

    if (USE_SIM65M) {
        if (true) {
            Serial2.print("$PAIR080,3*2D" CR LF);  // Reserved Balloon mode?
            V1_print(F("Balloon mode: sent $PAIR080,3*2D" CR LF));  // Reserved Balloon mode?
        } else if (false) {
            Serial2.print("$PAIR080,5*2B" CR LF);  // Reserved Drone mode?
            V1_print(F("Drone mode: sent $PAIR080,5*2B" CR LF));  // Reserved Drone mode?
        } else {
            Serial2.print("$PAIR080,0*2E" CR LF);  // Normal mode: general purpose
            V1_print(F("Normal mode: sent $PAIR080,0*2E" CR LF));
        }
        Serial2.flush();
        sleep_ms(1000);
    }
    // FIX! should we not worry about setting balloon mode (3) for ATGM336?
    // doesn't seem like ATGM336 has a balloon mode. no PCAS10 cmd in
    // the CASIC specifiction pdf

    // Serial2.print("$PSIMNAV,W,3*3A\r\n");
    // normal mode
    // Serial2.print("$PSIMNAV,W,0*39\r\n");

    // wb8elk said:
    // $PCAS11,5*18 is the Airborne command but not sure if the 336H accepts
    // have to wait for the sentence to get out, and also complete
    // gpsSleepForMillis(1000, false);
    V1_println(F("setGpsBalloonMode END"));
}

//************************************************
void setGnssOn_SIM65M(void) {
    // Packet Type:002 PAIR_GNSS_SUBSYS_POWER_ON
    // Power on the GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // Please send this command before using any location service.
    // $PAIR002*38

    // Packet Type:003 PAIR_GNSS_SUBSYS_POWER_OFF
    // Power off GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // CM4 also can receive commands after sending this command
    // (Include the AT command / the race Command / the part of PAIR
    // command which is not dependent on DSP.)
    // $PAIR003*39

    // Packet Type:020 PAIR_GET_VERSION
    // Query the firmware release information

    // Packet Type:021 PAIR_GET_SETTING_INFO
    // Query the customer related setting,
    // such as the firmware release information, DCB values, HW interface,
    // ULP enable and NVRAM auto saving.

    V1_println(F("setGnsOn_SIM65M START"));
    // PAIR_GET_VERSION
    Serial2.print("$PAIR020*38" CR LF);
    // Serial2.flush();
    // can see that nmeaBufferFastPoll() does get us the responses
    // we don't validate or wait for responses (yet??)
    // we can see version numbers here, et
    // interesting: AG3352Q_V2.5.0.AG3352_20230420

    // I suppose there is some flow control, i.e. I shouldn't send
    // to many overlapping requests. I guess it depends on the service
    // that is absorbing and responding to requests..the ACK/NACK responses
    // aid sw flow control

    // $GNGGA,213449.096,,,,,0,0,,,M,,M
    // $PAIR001,000,4*3F
    // $PAIR001,020,0*39
    // $PAIR020,AG3352Q_V2.5.0.AG3352_20230420,S,N,9ec1cc8,2210141406,2ba,3,,,5bebcf5b,2210141404,72555ce,2210141406,,*17
    nmeaBufferFastPoll(2000, true);  // duration_millis, printIfFull

    //*****************
    if (false) {
        // PAIR_GET_SETTING
        Serial2.print("$PAIR021*39" CR LF);
        // Serial2.flush();
        nmeaBufferFastPoll(2000, true);  // duration_millis, printIfFull
    }

    //*****************
    // this worked but does it already default to on after power on or ?? Seems to
    // FIX! we could change power on config to off,
    // to have softer power-on peak current?
    // Could change default power on baud rate config also
    // (when writing to flash)

    // PAIR_GNSS_SUBSYS_POWER_ON
    // in case we changed the default config to powered off
    Serial2.print("$PAIR002*38" CR LF);
    Serial2.flush();
    nmeaBufferFastPoll(2000, true);  // duration_millis, printIfFull

    sleep_ms(2000);
    V1_println(F(EOL "setGnsOn_SIM65M END"));
}

//************************************************
void setGnssOff_SIM65M(void) {
    V1_println(F("setGnsOff_SIM65M START"));
    // PAIR_GNSS_SUBSYS_POWER_OFF
    Serial2.print("$PAIR003*38" CR LF);
    Serial2.flush();
    sleep_ms(2000);
    V1_println(F("setGnsOff_SIM65M END"));
}

//************************************************
// always GGA GLL GSA GSV RMC
// nver ZDA TXT
void setGpsBroadcast(void) {
    // FIX! we'll have to figure this out for SIM65M
    V1_print(F("setGpsBroadcast START" EOL));
    updateStatusLED();
    Watchdog.reset();
    // room for a 60 char sentence with CR and LF also
    char nmeaSentence[62] = { 0 };

    if (USE_SIM65M) {
        //****************
        // Packet Type:050 PAIR_COMMON_SET_FIX_RATE
        // Set Position Fix Interval.
        // If set less than 1000 ms, ASCII NMEA will automatically increase
        // the update interval in order to decrease IO throughput.
        // It will return false if the operating voltage setting is not correct.

        // For SIM65M module, <Fix_Interval> parameter only support 1000 ms.
        // $PAIR050,<Fix_Interval>*<checksum>
        // Fix_Intervalmsec--Position fix interval in milliseconds (ms).
        // [Range: 100 ~ 1000]
        // [Example]
        // $PAIR050,1000*12

        //****************
        // Packet Type:062 PAIR_COMMON_SET_NMEA_OUTPUT_RATE
        // Set the NMEA sentence output interval of corresponding NMEA type
        // $PAIR062,<Type>,<Output_Rate>*<checksum>
        // -1 Reset all sentence to default value
        // 0 NMEA_SEN_GGA, // GGA interval - GPS Fix Data
        // 1 NMEA_SEN_GLL, // GLL interval - Geographic Position - Latitude longitude
        // 2 NMEA_SEN_GSA, // GSA interval - GNSS DOPS and Active Satellites
        // 3 NMEA_SEN_GSV, // GSV interval - GNSS Satellites in View
        // 4 NMEA_SEN_RMC, // RMC interval - Recommended Minimum Specific GNSS Sentence
        // 5 NMEA_SEN_VTG, // VTG interval - Course Over Ground and Ground Speed
        // 6 NMEA_SEN_ZDA, // ZDA interval - Time & Date

        // Output interval: <what is default? should we only output 1 per 5 position fixes?
        // is the position fix rate 1 per sec to 10 per sec?
        // 0 - Disabled or not supported sentence
        // 1 - Output once every one position fix
        // 2 - Output once every two position fixes
        // 3 - Output once every three position fixes
        // 4 - Output once every four position fixes
        // 5 - Output once every five position fixes

        // FIX! do we not get enough info in a single sec if we change to 5 here?
        // enable this, because we're disabling broadcast in default config now
        // for SIM65M. Assumes the default fix rate is 1000ms (1 per sec?)
        if (SIM65M_BROADCAST_5SECS) {
            strncpy(nmeaSentence, "$PAIR062,0,5*3B" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,1,5*3A" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,2,5*39" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,3,5*38" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,4,5*3F" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,5,5*3E" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,6,5*3D" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            GPS_WAIT_FOR_NMEA_BURST_MAX = 5500;

            // none of this works?
            // PAIR_COMMON_SET_FIX_RATE $PAIR050,time*<checksum>
            // SIM65M says: 100ms to 1000ms
            // want to straddle the 5 sec broadcast
            // GPS_WAIT_FOR_NMEA_BURST_MAX = 5200;
            // interesting: fix rate for Quectel L76, says this:
            // If the set frequency exceeds 1 Hz, 
            // only RMC, GGA and GNS massages will be output at the set frequency, 
            // whereas VTG, GLL, ZDA, GRS and GST messages will not be output, 
            // and GSA and GSV messages will be output at 1Hz
            // this is the default
            // strncpy(nmeaSentence, "$PAIR050,1000*12" CR LF, 62);

            // 1000ms to 10000ms at 1 sec boundaries ? who said that?
            // ULP mode only support 1Hz.
            // doesn't work?
            // strncpy(nmeaSentence, "$PAIR050,5000*16" CR LF, 62);
            // Serial2.print(nmeaSentence);

            busy_wait_us(2000);

        } else {
            // set all to interval of 1
            strncpy(nmeaSentence, "$PAIR062,0,1*3F" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,1,1*3E" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,2,1*3D" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,3,1*3C" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,4,1*3B" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,5,1*3A" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,6,1*39" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(2000);
            // assume the default 1sec position fix rate is there
        }

    } else {
        //*************************************************
        // ATGM336H

        // ZDA
        // this time info is in other sentences also?
        // $–ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx
        // hhmmss.ss = UTC
        // xx = Day, 01 to 31
        // xx = Month, 01 to 12
        // xxxx = Year // xx = Local zone description, 00 to +/- 13 hours
        // xx = Local zone minutes description (same sign as hours)

        //*************************************************
        // from the latest CASIC_ProtocolSpecification_english.pdf
        // $PCAS03 string nGGA value Message ID, sentence header

        // no 0 or 1 fields?

        // 2  GGA output frequency,
        // statement output frequency is based on positioning update rate
        // n (0~9) means output once every n positioning times, 0 means no output
        // If this statement is left blank, the original configuration will be retained.

        // 3  nGLL GLL output frequency,  same as nGGA
        // 4  nGSA GSA output frequency,  same as nGGA
        // 5  nGSV SV output frequency,   same as nGGA
        // 6  nRMC RMC output frequency,  same as nGGA
        // 7  nVTG VTG output frequency,  same as nGGA
        // 8  nZDA ZDA output frequency,  same as nGGA
        // 9  nANT ANT output frequency,  same as nGGA (this is the antenna open TXT ?)
        // 10 nDHV DHV output frequency,  same as nGGA
        // 11 nLPS LPS output frequency,  same as nGGA
        // 12 res1 reserve
        // 13 res2 reserve
        // 14 nUTC UTC output frequency,  same as nGGA
        // 15 nGST GST output frequency,  same as nGST
        // 16 res3 reserve
        // 17 res4 reserve
        // 18 res5 reserve
        // 19 nTIM TIM (PCAS60) output frequency, same as nGGA

        // 20 CSvalue Hexadecimal value checksum,
        //    XOR result of all characters between $ and * (excluding $ and *)
        // 21 <CR><LF> charactersCarriage return and line feed

        // hmm this didn't work? still got zda and ANT txt.
        // this was a forum posting. wrong apparently
        // strncpy(nmeaSentence, "$PCAS03,1,1,1,1,1,1,0,0*02" CR LF, 21);

        // spec has more/new detail. see below
        // FIX! was I still getting GNZDA and GPTXT ANTENNAOPEN with this?
        strncpy(nmeaSentence, "$PCAS03,1,1,1,1,1,1,0,0,0,0,,,1,1,,,,1*33" CR LF, 62);

        // example in pdf
        // first field after 3, is field ..no it's field 2
        // reserved fields are empty here
        // all data
        // $PCAS03,1,1,1,1,1,1,1,1,0,0,,,1,1,,,,1*33

        // using this:
        // no ANT or ZDA (example already disabled DHV and LPS)
        // why is 19 TIM PCAS60 needed? That's another receiver time in "subsequent versions"
        // $PCAS03,1,1,1,1,1,0,0,1,0,0,,,1,1,,,,1*33
        Serial2.print(nmeaSentence);
        Serial2.flush();
        busy_wait_us(2000);
    }

    V1_printf("setGpsBroadcast sent %s" EOL, nmeaSentence);
    V1_print(F("setGpsBroadcast END" EOL));
}
//************************************************
void disableGpsBroadcast(void) {
    // FIX! we'll have to figure this out for SIM65M
    V1_print(F("disableGpsBroadcast START" EOL));
    updateStatusLED();
    Watchdog.reset();
    char nmeaSentence[64] = { 0 };

    if (USE_SIM65M) {
        if (true) {
            // have to disable each NMEA sentence type individually?
            strncpy(nmeaSentence, "$PAIR062,0,0*3E" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,1,0*3F" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,2,0*3C" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,3,0*3D" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,4,0*3A" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,5,0*3B" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(500);
            strncpy(nmeaSentence, "$PAIR062,6,0*38" CR LF, 62);
            Serial2.print(nmeaSentence);
            busy_wait_us(2000);
        }
    } else {
        // checksum from https://www.meme.au/nmea-checksum.html
        strncpy(nmeaSentence, "$PCAS03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*02" CR LF, 64);
        Serial2.print(nmeaSentence);
        Serial2.flush();
        busy_wait_us(2000);
    }
    // delay(1000);
    V1_printf("disableGpsBroadcast sent %s" EOL, nmeaSentence);
    V1_print(F("disableGpsBroadcast END" EOL));
}

//***************************************
// my GPTXT on 11/18/24.
// $GPTXT,01,01,02,HW=ATGM336H,0004090746370*1E
// $GPTXT,01,01,02,IC=AT6558-5N-31-0C510800,EF16CKJ-F2-008017*5A
// $GPTXT,01,01,02,SW=URANUS5,V5.3.0.0*1D
// $GPTXT,01,01,02,TB=2020-04-28,13:43:10*40
// $GPTXT,01,01,02,MO=GBR*25
// $GPTXT,01,01,02,BS=SOC_BootLoader,V6.2.0.2*34
// $GPTXT,01,01,02,FI=00856014*71
// $GPTXT,01,01,01,ANTENNAOPEN*25

//***************************************
// examples of GPTXT messages from spec. we are disabling them now in setGpsBroadcast
// this is not from oiurs
// $GPTXT,01,01,02,MA=CASIC*27
// Indicates the manufacturer's name (CASIC)
// $GPTXT,01,01,02,IC=ATGB03+ATGR201*71
// Indicates the model of the chip or chipset
// (baseband chip model ATGB03, RF chip model ATGR201)
// $GPTXT,01,01,02,SW=URANUS2,V2.2.1.0*1D
// Indicates the software name and version number
// (software name URANUS2, version number V2.2.1.0)
// $GPTXT,01,01,02,TB=2013-06-20,13:02:49*43
// Indicates the code compilation time (June 20, 2013, 13:02:49)
// $GPTXT,01,01,02,MO=GB*77
// Indicates the working mode of the receiver at this startup
// (GB indicates the dual-mode mode of GPS+BDS)
// $GPTXT,01,01,02,CI=00000000*7A
// Indicates the customer number (the customer number is 00000000)

//************************************************
// re: GSV nmea sentences from SIM65M
// Depending on the number of satellites tracked,
// multiple messages of GSV data may be required.
// In some software versions, maximum number of satellites reported as visible
// is limited to 12, even though more may be visible
// Hmm: is this fully compatible with max number expected by TinyGPS++ parsing?

//************************************************
// re: SIM65M comments on the *RMC nmea sentences
// A valid status is derived from all the parameters set in the software.
// This includes the minimum number of satellites required,
// any DOP mask setting,
// presence of DGPS corrections, etc.
// If the default or current software setting requires that a factor is met,
// then if that factor is not met, the solution will be marked as invalid
// Does not support magnetic declination.
// All “course over ground” data are geodetic WGS84 directions relative to true North

//************************************************
// re: SIM65M 'PAIR' sentences:
// PAIR command is an AIROHA proprietary GNSS data transferring protocol.
// This protocol is used to configure the GNSS module’s parameters,
// to set/get aiding information,
// and to receive notifications from the GNSS module.
// To process data conveniently, the PAIR commands matches with the NMEA sentence format.

// Quectel LC76G spec says:j
// also mentions some weird things in LC86G ?

// GNSS search modes supported by LC26G (AB), LC26G-T (AA), LC76G series and LC86G (LA, PA):
// why does it say QZSS is always enabled by default?
// GPS only
// GPS + QZSS
// GPS + GLONASS
// GPS + GLONASS + QZSS
// GPS + Galileo
// GPS + Galileo + QZSS
// GPS + BDS
// GPS + BDS + QZSS
// GPS + GLONASS + Galileo + BDS
// GPS + GLONASS + Galileo + BDS + QZSS

void setGpsConstellations(int desiredConstellations) {
    // FIX! we'll have to figure this out for SIM65M
    V1_printf("setConstellations START %d" EOL, desiredConstellations);
    updateStatusLED();
    Watchdog.reset();
    // FIX! should we ignore desiredConstellations and force 3 (BDS + GPS
    int usedConstellations = desiredConstellations;
    char nmeaSentence[62] = { 0 };
    if (USE_SIM65M) {
        switch (usedConstellations) {
            // are these not right, and Galileo is in default?
            // Gallileo is field 3. the last two fields are QZSS and NavIC
            case 0: ; // FIX! should I make 0 disable everything?
            case 1:  // GPS
                strncpy(nmeaSentence, "$PAIR066,1,0,0,0,0,0,*3B" CR LF, 62); break;
            case 2:  // BDS
                strncpy(nmeaSentence, "$PAIR066,0,0,0,1,0,0,*3B" CR LF, 62); break;
            case 3:  // GPS+BDS
                strncpy(nmeaSentence, "$PAIR066,1,0,0,1,0,0,*3A" CR LF, 62); break;
            case 4:  // GLONASS
                strncpy(nmeaSentence, "$PAIR066,0,1,0,0,0,0,*3B" CR LF, 62); break;
            case 5:  // GPS+GLONASS
                strncpy(nmeaSentence, "$PAIR066,1,1,0,0,0,0,*3A" CR LF, 62); break;
            case 6:  // BDS+GLONASS
                strncpy(nmeaSentence, "$PAIR066,0,1,0,1,0,0,*3A" CR LF, 62); break;
            case 7:  // GPS+BDS+GLONASS
                strncpy(nmeaSentence, "$PAIR066,1,1,0,1,0,0,*3B" CR LF, 62); break;
            case 8: ; // FIX! should I make 8 enable everything?
            default:  // GPS+BDS
                usedConstellations = 3;
                strncpy(nmeaSentence, "$PAIR066,1,0,0,1,0,0,*3A" CR LF, 62); break;
        }
    } else {
        switch (usedConstellations) {
            // case 0 isn't defined in the CASIC_ProtocolSpecification.pdf?
            case 1: strncpy(nmeaSentence, "$PCAS04,1*18" CR LF, 62); break;  // GPS
            case 2: strncpy(nmeaSentence, "$PCAS04,2*1B" CR LF, 62); break;  // BDS
            case 3: strncpy(nmeaSentence, "$PCAS04,3*1A" CR LF, 62); break;  // GPS+BDS
            case 4: strncpy(nmeaSentence, "$PCAS04,4*1D" CR LF, 62); break;  // GLONASS
            case 5: strncpy(nmeaSentence, "$PCAS04,5*1C" CR LF, 62); break;  // GPS+GLONASS
            case 6: strncpy(nmeaSentence, "$PCAS04,6*AF" CR LF, 62); break;  // BDS+GLONASS
            case 7: strncpy(nmeaSentence, "$PCAS04,7*1E" CR LF, 62); break;  // GPS+BDS+GLONASS
            default:
                usedConstellations = 3;
                strncpy(nmeaSentence, "$PCAS04,3*1D" CR LF, 62);  // GPS+BDS
        }
    }
    // SIM65M
    // huh. this is like causing another GPS reset? (will be warm reset?)
    // Packet Type:066
    // PAIR_COMMON_SET_GNSS_SEARCH_MODE
    // Configure the receiver to start searching for satellites.
    // The setting is available when the NVRAM data is valid.
    // The device restarts when it receives this command.
    // Abbreviation: (GPS: "G", GLONASS: "R", Galileo: "E", BeiDou: "B", NavIC, "I")

    // GPS_Enabled
    // "0", disable (DO NOT search GPS satellites).
    // "1", search GPS satellites
    // GLONASS_Enabled
    // "0", disable (DO NOT search GLONASS satellites).
    // "1", search GLONASS satellites.
    // Galileo_Enabled
    // "0", disable (DO NOT search Galileo satellites).
    // "1", search Galileo satellites
    // BeiDou_Enabled
    // "0", disable (DO NOT search BeiDou satellites).
    // "1", search BeiDou satellites
    // QZSS_Enabled
    // "0", disable (DO NOT search QZSS satellites).
    // "1", search QZSS satellites
    // NavIC_Enabled
    // "0", disable (DO NOT search Nav
    // "1", search NavIC satellites

    // For SIM65M:
    // L1 single frequency, supports 5 modes G/ GR/ GE/ GB/ GREB as follows:
    // PAIR066,1,0,0,0,0,0 GPS only
    // PAIR066,1,1,0,0,0,0 GPS+GLONASS
    // PAIR066,1,0,1,0,0,0 GPS+GALILEO
    // PAIR066,1,0,0,1,0,0 GPS+ BEIDOU
    // PAIR066,1,1,1,1,0,0 GPS+GLONASS+GALILEO+BEIDOU
    // PAIR066,1,1,0,1,0,0 GPS+GLONASS+BEIDOU
    // QZSS is always switchable.


    Serial2.print(nmeaSentence);
    Serial2.flush();
    delay(2000);

    V1_printf("setGpsConstellations for usedConstellations %d, sent %s" EOL,
        desiredConstellations, nmeaSentence);
    V1_printf("setGpsConstellations END %d" EOL, desiredConstellations);
}

//************************************************
void setupSIM65M(int desiredBaud) {
    // Currently nothing?

    // Packet Type:002 PAIR_GNSS_SUBSYS_POWER_ON
    // Power on the GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // Please send this command before using any location service.
    // $PAIR002*38

    // Packet Type:003 PAIR_GNSS_SUBSYS_POWER_OFF
    // Power off GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // CM4 also can receive commands after sending this command
    // (Include the AT command / the race Command / the part of PAIR
    // command which is not dependent on DSP.)
    // The location service is not available after this command is executed.
    // The system can still receive configuration PAIR commands.
    // The application is running if necessary.
    // CM4 will go to sleep if the application is not working at this time.
    // The system can be awoken by the GNSS_DATA_IN_EINT pin after going to sleep.
    // $PAIR003*39

    // Packet Type:004 PAIR_GNSS_SUBSYS_HOT_START
    // Hot Start. Use the available data in the NVRAM
    // $PAIR004*3E

    // Packet Type:005 PAIR_GNSS_SUBSYS_WARM_START
    // Warm Start. Not using Ephemeris data at the start
    // $PAIR005*3F

    // Packet Type:006 PAIR_GNSS_SUBSYS_COLD_START
    // Cold Start. Not using the Position, Almanac and Ephemeris data at the start
    // $PAIR006*3C

    // Packet Type:007 PAIR_GNSS_SUBSYS_FULL_COLD_START
    // Full Cold Start
    // In addition to Cold start,
    // this command clears the system/user configurations at the start
    // It resets the GNSS module to the factory default
    // $PAIR007*3D

    // Packet Type:022 PAIR_READY_TO_READ
    // Host system wake up notification.
    // There is no need to use this command,
    // if the host does not enter sleep or HW not set
    // the configuration of GNSS_NOTIFY_HOST_WAKEUP_PIN.

    // Application (gnss_demo project) will pull high
    // GNSS_NOTIFY_HOST_WAKEUP_PIN > 10ms when data is
    // ready to send to wake up host application.
    // Please send this command as ACK to SIM65M after wakeup done.
    // GPIO 24 is default to wakeup
    // $PAIR022*3A

    // Packet Type:023 PAIR_SYSTEM_REBOOT
    // Reboot GNSS whole chip,
    // including the GNSS submodule and other all CM4 modules.
    // $PAIR023*3B

    // 2.3.32 Packet Type:062
    // PAIR_COMMON_SET_NMEA_OUTPUT_RATE
    // Set the NMEA sentence output interval of corresponding NMEA type
    // see SIM65M Series_NMEA Message_User Guide_V1.00.pdf for details

    // Packet Type:864 PAIR_IO_SET_BAUDRATE
    // Set port baud rate configuration
    // Must reboot the device after changing the port baud rate.
    // The change will valid after reboot

    // Port_Type----HW Port Type:
    // 0: UART [ER1 support]
    // Port_Index----HW Port Index:
    // 0: UART0
    // 1: UART1
    // 2: UART2
    // Baudrate----the baud rate need config:
    // Support 115200, 230400, 460800, 921600, 3000000
    // $PAIR864,0,0,115200*1B

    // default baud rate is 115200 maybe?

    // Packet Type:866 PAIR_IO_SET_FLOW_CONTROL
    // Port_Type----HW Port Type.
    // 0: UART
    // Port_Index----HW Port Index
    // UART - 0: UART0, 1: UART1, 2: UART2
    // Flow_control
    // 0, disable flow control. 1, enable SW flow control. 2, enable HW flow control
    // $PAIR866,0,2,1*2D ==> Set UART2 SW Flow Control ON

    // Packet Type:860 PAIR_IO_OPEN_PORT
    // Open a GNSS data port
    // DataField:
    // $PAIR860,<Port_Type>,<Port_Index>,<Data_Type>,<Baudrate>,<Flow_control>*CS<CR><LF>
    // NameUnitDefaultDescription
    // Port_Type----HW Port Type:
    // 0: UART [ER1 support]
    // 1: I2C
    // [ER2 support]
    // 2: SPI
    // [ER2 support]
    // 3: USB
    // [ER1 support]
    // 4: SD-Card [ER3 support]
    // Port_Index----HW Port Index:
    // UART - 0: UART0, 1: UART1, 2: UART2
    // USB - 0: USB Virtual Port 0, 1: USB Virtual Port 1
    // Others - 0: Only one port
    // Data_Type----A bitmap to config data type:
    // GNSS_IO_FLAG_OUT_NMEA 0x01
    // GNSS_IO_FLAG_OUT_LOG 0x02
    // GNSS_IO_FLAG_OUT_CMD_RSP 0x04
    // GNSS_IO_FLAG_OUT_DATA_RSP 0x08
    // GNSS_IO_FLAG_OUT_RTCM 0x10
    // GNSS_IO_FLAG_IN_CMD 0x20
    // GNSS_IO_FLAG_IN_DATA 0x40
    // GNSS_IO_FLAG_IN_RTCM 0x80
    // Baudrate----the baud rate must be configured. This parameter is only
    // valid for UART. Please use 0 for other port type:
    // Support 110, 300, 1200, 2400, 4800, 9600, 19200, 38400,
    // 57600, 115200, 230400, 460800, 921600, 3000000
    // Flow_control----0, disable flow control. 1, enable SW flow control. 2,
    // enable HW flow control. This parameter is only valid for
    // UART. Please use 0 for other port type

    // $PAIR860,0,0,37,9600,0*23\r\n ==> Open UART0 to NMEA output without flow control.
    // Baudrate is 9600.
    // $PAIR860,0,1,37,9600,0*22\r\n ==> Open UART1 to NMEA output without flow control.
    // Baudrate is 9600.
    // $PAIR860,0,2,37,9600,0*21\r\n ==> Open UART2 to NMEA output without flow control.
    // Baudrate is 9600.

    // $PAIR860,0,0,37,115200,0*2B\r\n ==> Open UART0 to NMEA output without flow control.
    // Baudrate is 115200.
    // $PAIR860,0,1,37,115200,0*2A\r\n ==> Open UART1 to NMEA output without flow control.
    // Baudrate is 115200.
    // $PAIR860,0,2,37,115200,0*29\r\n ==> Open UART2 to NMEA output without flow control.
    // Baudrate is 115200.

    // Packet Type:862 PAIR_IO_SET_DATA_TYPE
    // Set GNSS port data type configuration
    // hopefully default for uart0/1/2 is NMEA output
}

//************************************************
void setGpsBaud(int desiredBaud) {
    // Assumes we can talk to gps already at some existing agreed
    // on Serial2/gps chip setup (setup by int/warm reset/full cold reset)
    V1_printf("setGpsBaud START %d" EOL, desiredBaud);
    updateStatusLED();
    Watchdog.reset();
    // after power on, start off talking at 9600 baud. when we change it

    // have to send CR and LF and correct checksum
    // CR and LF are in print_functions.h. they are not part of the checksum, nor is the $
    // Example: $PMTK251,38400*27<CR><LF>
    // just pre-calculate the checksums here and hardwire them in the static sentences used.
    // https://www.meme.au/nmea-checksum.html
    // should just get legal ones here
    int usedBaud = checkGpsBaudRate(desiredBaud);
    char nmeaBaudSentence[64] = { 0 };
    // BAUD
    if (USE_SIM65M) {
        switch (usedBaud) {
            // $PAIR860,0,0,37,9600,0*23 means:
            // Open UART0 to NMEA output without flow control.  Baudrate is 9600.
            case 9600:
                // alternate baudrate only but says min is 115200?
                // yes! 9600 works after boot with 115200!. no buffer overflow
                if (true) strncpy(nmeaBaudSentence, "$PAIR864,0,0,9600*13" CR LF, 64);
                else strncpy(nmeaBaudSentence, "$PAIR860,0,0,37,9600,0*23" CR LF, 64);
                break;
            // $PAIR860,0,0,37,19200,0*16
            case 19200:
                if (true) strncpy(nmeaBaudSentence, "$PAIR864,0,0,19200*26" CR LF, 64);
                else strncpy(nmeaBaudSentence, "$PAIR860,0,0,37,19200,0*16" CR LF, 64);
                break;
            // $PAIR860,0,0,37,38400,0*13
            case 38400:
                if (true) strncpy(nmeaBaudSentence, "$PAIR864,0,0,38400*23" CR LF, 64);
                else strncpy(nmeaBaudSentence, "$PAIR860,0,0,37,38400,0*13" CR LF, 64);
                break;
            // $PAIR860,0,0,37,57600,0*18
            case 57600:
                if (true) strncpy(nmeaBaudSentence, "$PAIR864,0,0,57600*28" CR LF, 64);
                else strncpy(nmeaBaudSentence, "$PAIR860,0,0,37,57600,0*18" CR LF, 64);
                break;
            // $PAIR860,0,0,37,115200,0*2B\r\n ==> Open UART0 to NMEA output without flow control.
            // Baudrate is 115200.
            case 115200:
                if (true) strncpy(nmeaBaudSentence, "$PAIR864,0,0,115200*1B" CR LF, 64);
                else strncpy(nmeaBaudSentence, "$PAIR860,0,0,37,115200,0*2B" CR LF, 64);
                break;
            default:
                usedBaud = 9600;
                if (true) strncpy(nmeaBaudSentence, "$PAIR864,0,0,9600*13" CR LF, 64);
                else strncpy(nmeaBaudSentence, "$PAIR860,0,0,37,9600,0*23" CR LF, 64);
                break;
        }
    } else {
        switch (usedBaud) {
            case 4800:   strncpy(nmeaBaudSentence, "$PCAS01,0*1C" CR LF, 21); break;

            // didn't work . now it worked?
            // seems to be okay if chip is in 9600 state
            case 9600:   strncpy(nmeaBaudSentence, "$PCAS01,1*1D" CR LF, 21); break;
            // worked.. hmm broken? had to restart arduino to get it to work
            // case 9600:   strncpy(nmeaBaudSentence, "$PMTK251,9600*17" CR LF, 21); break;

            // worked
            case 19200:  strncpy(nmeaBaudSentence, "$PCAS01,2*1E" CR LF, 21); break;
            // didn't work
            // case 19200:  strncpy(nmeaBaudSentence, "$PMTK251,19200*22" CR LF, 21); break;

            // worked
            case 38400:  strncpy(nmeaBaudSentence, "$PCAS01,3*1F" CR LF, 21); break;
            // didn't work
            // case 38400:  strncpy(nmeaBaudSentence, "$PMTK251,38400*27" CR LF, 21); break;

            // didn't work
            case 57600:  strncpy(nmeaBaudSentence, "$PCAS01,4*18" CR LF, 21); break;
            // ?
            // case 57600:  strncpy(nmeaBaudSentence, "$PMTK251,57600*2C" CR LF, 21); break;
            // worked (prints). but lots of buffer overrun ERRORs overflowing Rx buffer
            case 115200: strncpy(nmeaBaudSentence, "$PCAS01,5*19" CR LF, 21); break;
            // ?
            // case 115200: strncpy(nmeaBaudSentence, "$PMTK251,115200*1F" CR LF, 21); break;
            default:
                usedBaud = 9600;
                // strncpy(nmeaBaudSentence, "$PMTK251,9600*17" CR LF, 21);
                strncpy(nmeaBaudSentence, "$PCAS01,1*1D" CR LF, 21);
        }
    }

    // https://forum.arduino.cc/t/solved-proper-way-to-change-baud-rate-after-initial-setup/419860/5
    // get rid of anything still in the cpu output buffer
    Serial2.flush();
    delay(1000);
    Serial2.print(nmeaBaudSentence);
    Serial2.flush();

    V1_printf("setGpsBaud for usedBaud %d, sent %s" EOL, usedBaud, nmeaBaudSentence);
    // have to wait for the sentence to get out and complete at the GPS
    delay(3000);

    // Note:
    // Serial2.end() Disables serial communication,
    // allowing the RX and TX pins to be used for general input and output.
    // To re-enable serial communication, call Serial2.begin().

    // FIX! we could try RP2040 using different bauds and see what baud rate it's at,
    // then send the command to change baud rate.
    // But really, how come we can't cold reset it to 9600?
    // when it's in a not-9600 state? it's like vbat keeps the old baud rate on reset
    // and/or power cycle?? makes it dangerous to use anything other than 9600 baud.
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);
    Serial2.begin(usedBaud);
    V1_printf("setGpsBaud did Serial2.begin(%d)" EOL, usedBaud);
    // then have to change Serial2.begin() to agree
    gpsSleepForMillis(1000, false);
    V1_printf("setGpsBaud END %d" EOL, usedBaud);
}

//************************************************
void GpsINIT(void) {
    V1_println(F(EOL "GpsINIT START"));
    updateStatusLED();
    Watchdog.reset();

    //****************
    // The gps power pin is floating? likely GPS is off but unknown
    // Should turn gps off before doing init if you know it's been
    // initted already?

    //****************
    // equations and schematic for gate resistor on the mosfet
    // https://electronics.stackexchange.com/questions/666204/gate-resistors-on-the-mosfet

    // HEY! can't I control the source impedance of RP2040 GPIO outputs?
    // What if change the GPIO slew rate controls?
    // I could use a 2MA output drive and a slow slew rate
    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
    // i.e. (GpsPwr is my mosfet control gpio for the gps power control)
    // A03401A is a p-channel mosfet
    // https://www.aosmd.com/sites/default/files/res/datasheets/AO3401A.pdf

    // Doug's inrush current limiting design. affecting gate resistance with the mosfet
    // https://docs.google.com/document/d/1b1TdheBbXtl7U7HTO9mjVkThD9QHASqJirI2kSnv4LE/edit?tab=t.0#heading=h.lcp1i04bgm4q

    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_gpio/include/hardware/gpio.h
    // On the RP2040, setting both pulls enables a "bus keep" function,
    //  i.e. a weak pull to whatever is current high/low state of GPIO.
    // void gpio_set_pulls(uint gpio, bool up, bool down);

    // static inline void gpio_pull_down(uint gpio) {
    //     gpio_set_pulls(gpio, false, true);
    // }


    // static inline void gpio_disable_pulls(uint gpio) {
    //     gpio_set_pulls(gpio, false, false);
    // }

    // hmm.. the weak pullup will change the slew rate down also?
    gpio_init(GpsPwr);  // defaults to 8mA drive strength
    pinMode(GpsPwr, OUTPUT);
    // this will be undone by the next thing that uses gpio_set_pulls()
    gpio_pull_up(GpsPwr);
    gpio_put(GpsPwr, HIGH);  // deassert
    // if you set them to input after this, they will be high impedance!
    gpio_set_slew_rate(GpsPwr, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(GpsPwr, GPIO_DRIVE_STRENGTH_2MA);

    //****************
    gpio_init(GPS_NRESET_PIN);
    pinMode(GPS_NRESET_PIN, OUTPUT);
    gpio_pull_up(GPS_NRESET_PIN);
    // gpio_put(GPS_NRESET_PIN, HIGH); // deassert

    // FIX! should toggle this for low power operation?
    // instead of powering GpsPwr on/off (even if VBAT gives hot fix)
    gpio_init(GPS_ON_PIN);
    pinMode(GPS_ON_PIN, OUTPUT);
    gpio_pull_down(GPS_ON_PIN);
    // gpio_put(GPS_ON_PIN, LOW); // deassert

    //****************
    // Updated: Do a full reset since vbat may have kept old settings
    // don't know if that includes baud rate..maybe?
    digitalWrite(GpsPwr, HIGH);
    V1_printf("set GpsPwr %d HIGH (power off)" EOL, GpsPwr);
    digitalWrite(GPS_NRESET_PIN, HIGH);
    V1_printf("set GPS_NRESET_PIN %d HIGH" EOL, GPS_NRESET_PIN);
    digitalWrite(GPS_ON_PIN, LOW);
    V1_printf("set GPS_ON_PIN %d LOW" EOL, GPS_ON_PIN);

    //****************
    V1_printf("GPS_UART1_RX_PIN %d" EOL, GPS_UART1_RX_PIN);
    V1_printf("GPS_UART1_TX_PIN %d" EOL, GPS_UART1_TX_PIN);
    V1_printf("GpsPwr %d" EOL, GpsPwr);
    V1_printf("GPS_NRESET_PIN %d" EOL, GPS_NRESET_PIN);
    V1_printf("GPS_ON_PIN %d" EOL, GPS_ON_PIN);

    // FIX! is it okay that RX is powered on while gps chip is powered off?
    Serial2.setRX(GPS_UART1_RX_PIN);
    Serial2.setTX(GPS_UART1_TX_PIN);

    //****************
    Serial2.setPollingMode(true);
    // tried making bigger...seems like 32 is the reality though?
    // Serial2.setFIFOSize(SERIAL2_FIFO_SIZE);
    Serial2.flush();
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);

    if (USE_SIM65M) {
        // default uart baud rate for SIM65
        Serial2.begin(115200);
    } else {
        // first talk at 9600..but GpsFullColdReset() will do..so a bit redundant
        Serial2.begin(9600);
    }
    gpsSleepForMillis(2000, false);

    // full cold reset, set baud to target baud rate, and setGpsBalloonMode done
    // FIX! hmm will sim65 reset to 9600?
    // will it stay at the new baud rate thru warm reset and cold reset
    // like ATGM366N (weird) or will it default to 115200 again.
    GpsFullColdReset();
    // gps is powered up now

    //****************
    // drain the rx buffer. GPS is off, but shouldn't keep
    while (Serial2.available()) Serial2.read();
    // sleep 3 secs
    gpsSleepForMillis(3000, false);
    V1_println(F("GpsINIT END" EOL));
}

//************************************************
void pwmGpsPwrOn() {
    // soft power-on for GpsPwr (assert low, controls mosfet)
    // note that vbat doesn't have mosfet control, so it will be high right away
    // with availability of power
    // NO: doesn't work because externally there's a 10k ohm pullup on the pcb
    bool WEAK_PULLDOWN_FOR_ASSERT = false;
    if (WEAK_PULLDOWN_FOR_ASSERT) {
        // assumed current gpio GpsPwr state (driving the mosfet for gps chip power)
        // output, driven with 1, with pullup (by Init) (active deassert)
        pinMode(GpsPwr, INPUT);
        // pulldown is 50 to 80 kohms on the rp2040 (so is pullup)
        gpio_pull_down(GpsPwr);  // this also disables the pullup
        sleep_ms(5000);  // sleep 5 seconds.
        // make sure it ends with GpsPwr on! Should have RC ramped to low by now.
        pinMode(GpsPwr, OUTPUT);
        gpio_put(GpsPwr, LOW);  // assert with active driver now!
    } else {
        uint64_t on_usecs = 1;
        uint64_t off_usecs = 200;
        uint64_t duty_cycle;

        // 10 iterations taking approx 200 usec/iteration? 2 secs total?
        while (off_usecs > 0) {
            Watchdog.reset();
            digitalWrite(GpsPwr, LOW);  // assert to mosfet
            sleep_us(on_usecs);  // lower power light sleep
            digitalWrite(GpsPwr, HIGH);  // deassert to mosfet
            sleep_us(off_usecs);  // lower power light sleep

            // shifts to get some accuracy on the division.
            // the delta (1000000/10000) should give a % ? (*100)
            duty_cycle = (on_usecs * 1000000UL)  / ((on_usecs + off_usecs) * 10000UL);
            // print duty_cycle at every %10 boundary
            // USB pll should be off when we're turing on the gps during cold reset? no print
            if (false && ((duty_cycle % 100) == 0)) {
                V1_printf("pwmGpsPwrOn() duty_cycle (pct) %" PRIu64 EOL, duty_cycle);
            }
            on_usecs += 20;
            off_usecs -= 20;
        }
        // make sure it ends with GpsPwr on!
        digitalWrite(GpsPwr, LOW);
    }
}

//************************************************
bool GpsFullColdReset(void) {
    // BUG: can't seem to reset the baud rate to 9600 when
    // the GPS chip has a non-working baud rate?

    // a full cold reset reverts to 9600 baud
    // as does standby modes? (don't use)
    V1_println(F(EOL "GpsFullColdReset START"));
    uint64_t start_millis = millis();
    Watchdog.reset();

    GpsIsOn_state = false;
    GpsStartTime = get_absolute_time();  // usecs
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    // turn it off first. may be off or on currently
    V1_println(F("Turn off the serial2..float at gps? no UART stuff at poweron?"));
    V1_flush();
    Serial2.end();

    // experiment idea: can we float the rx/tx pins of Serial2? maybe switch it
    // to other pins temporarily?
    // https://github.com/earlephilhower/arduino-pico/discussions/199

    // assert reset during power off
    // IDEA! since we KNOW the power demand will be high for 1 minute after poweron
    // just go into light sleep to reduce rp2040 power demand for 1 minute
    // i.e. guarantee that cold reset, takes 1 minute?

    // FIX! what if we power on with GPS_ON_PIN LOW and GPS_NRESET_PIN HIGH
    V1_println(F("Doing Gps COLD POWER_ON (GPS_ON_PIN off with first power)"));

    // NOTE: do I have to keep inputs like GPS_ON_PIN low
    // until after power is on? What about reset?
    // to avoid latchup of LNA? see
    // https://www.eevblog.com/forum/rf-microwave/gps-lna-overheating-on-custom-pcb/
    digitalWrite(GPS_ON_PIN, LOW);  // deassert
    digitalWrite(GPS_NRESET_PIN, LOW);  // assert
    digitalWrite(GpsPwr, HIGH);  // deassert
    Serial2.end();
    // full 2 secs off?
    gpsSleepForMillis(2000, false);

    //******************
    // Cold Start. doesn't clear any system/user configs
    // does this PMTK command work or not work
    // Serial2.print("$PMTK103*30\r\n");
    // or according to CASIC_ProtocolSpecification_english.pdf

    // does this force the reset right away? no power transition?
    // some say it's just a boot mode configuration?
    // i.e. send before power cycle?

    // factory start. clear all data, reset rcvr.
    // Serial2.print("$PCAS10,3*1F\r\n");
    // cold start. no init info, clear all data except config
    // Serial2.print("$PCAS10,2*1E\r\n");
    // warm start. no init info, all data valid. clear ephemris
    // Serial2.print("$PCAS10,1*1D\r\n");
    // hot start. no init info, all data valid.
    // Serial2.print("$PCAS10,0*1C\r\n");

    // Full Cold Start. any system/user configs (back to factory status)
    // FIX! should we wait for ack or no?
    // have to toggle power off/on to get this effect?

    // always do this just in case the GpsIsOn() got wrong?
    // but we're relying on the Serial2.begin/end to be correct?
    // might as well commit to being right!

    // FIX! hmm. does driving the uart rx/tx to gps while gps is powering up
    // change it's behavior. What if we left them floating until after powerup?
    // seems like the gps backs up on the serial data?

    // we still have usb pll on, and default clock frequency at this point?
    if (PWM_GPS_POWER_ON_MODE) {
        // this is probably at least 2 secs. let's measure
        uint64_t start_millis2 = millis();
        pwmGpsPwrOn();
        uint64_t duration_millis2 = millis() - start_millis2;
        V1_printf("Used pwmGpsPwrOn() and took %" PRIu64 " millisecs" EOL, duration_millis2);
        // soft power-on for GpsPwr (assert low, controls mosfet)
        // note that vbat doesn't have mosfet control, so it will be high right away
        // with availability of power
    } else {
        digitalWrite(GpsPwr, LOW);  // assert to mosfet
        gpsSleepForMillis(500, false);
    }

    // deassert NRESET after power on (okay in both normal and experimental case)
    // new 12/7/24 disable Serial2 while powering on!
    // should we float the rx/tx also somehow?
    gpsSleepForMillis(500, false);
    digitalWrite(GPS_NRESET_PIN, HIGH);  // deassert
    gpsSleepForMillis(1000, false);
    Watchdog.reset();

    // IDEA! since we KNOW the power demand will be high for 1 minute after poweron
    // just go into light sleep to reduce rp2040 power demand for 1 minute
    // i.e. guarantee that cold reset, takes 1 minute?
    // hmm. we're stalling things now. maybe only sleep for 15 secs
    // FIX! this apparently makes the Serial2 dysfunctional
    // so the gps chip can't send output
    // it backs up on the initial TXT broadcast (revisions) and then hits a power peak
    // right after we fix the clock back to normal (50Mhz min tried)
    // so: is that worth it? dunno.

    // UPDATE: is the broadcast right after power on the issue
    // other power saving: disable usb pll (and restore)

    // https://github.com/earlephilhower/arduino-pico/discussions/1544
    // we had to make sure we reset the watchdog, now, in gpsSleepForMillis
    // we already wakeup periodically to update led, so fine

    //******************
    // so we can undo the temporary lower clock setting
    uint32_t PLL_SYS_MHZ_restore = PLL_SYS_MHZ;

    // the global IGNORE_KEYBOARD_CHARS is used to guarantee no interrupting of core1
    // while we've messed with clocks during the gps agressive power on control
    // it should always be re-enabled after 15 secs.
    // Worst case to recover: unplug power and plug in again

    Watchdog.reset();
    // don't bother in balloon mode
    if (!BALLOON_MODE && !ALLOW_USB_DISABLE_MODE) measureMyFreqs();

    V1_print(F("GPS power demand high during cold reset..try to minimize rp2040 power" EOL));
    //**************************************
    if (ALLOW_LOWER_CORE_VOLTAGE_MODE) {
        V1_print(F("Also lowering core voltage to 0.95v" EOL));
    }
    if (ALLOW_USB_DISABLE_MODE) {
        V1_print(F("No keyboard interrupts will work because will disable USB PLL too" EOL));
    }
    if (ALLOW_KAZU_12MHZ_MODE) {
        V1_print(F("Will stay in 12Mhz using xosc, after things are restored" EOL));
    }
    //**************************************
    if (ALLOW_TEMP_12MHZ_MODE) {
        V1_printf("Switch pll_sys PLL_SYS_MHZ %lu to xosc 12Mhz then sleep" EOL, PLL_SYS_MHZ);
    } else {
        V1_printf("Switch pll_sys PLL_SYS_MHZ %lu to pll 18Mhz then sleep" EOL, PLL_SYS_MHZ);
    }
    V1_flush();

    // hmm core0 has to know to drain garbage chars if we assert this? then deassert?
    IGNORE_KEYBOARD_CHARS = true;
    // DRASTIC measures, do before sleep!
    // save current sys freq
    // FIX! does the flush above not wait long enough?
    // Wait another second before shutting down serial
    // sleep may be problematic in this transition?
    // some more thoughts about low power rp2040 and clocks
    // was wondering what measureMyFreqs() sees differing ring osc and rtc freqs
    // https://forums.raspberrypi.com/viewtopic.php?t=342156

    busy_wait_ms(500);
    // remember not to touch Serial if in BALLOON_MODE!!
    if (!BALLOON_MODE) {
        Serial.flush();
        Serial.end();
        busy_wait_ms(500);
    }

    // includes deinit of the usb pll now?
    kazuClocksSlow();

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__pll.html
    // There are two PLLs in RP2040. They are:
    // pll_sys - Used to generate up to a 133MHz (actually more) system clock
    // pll_usb - Used to generate a 48MHz USB reference clock


    // Release/uninitialise specified PLL.This will turn off the power to the specified PLL.
    // pll_deinit(pll_usb) does not check if the PLL is in use before powering it off.
    // use care.

    // examples: https://sourcevu.sysprogs.com/rp2040/picosdk/symbols/pll_deinit
    // sidenote: pi pico sdk has set_sys_clock_48mhz() function
    // Initialise the system clock to 48MHz Set the system clock to 48MHz,
    // and set the peripheral clock to match.
    // example:
    // https://sourcevu.sysprogs.com/rp2040/examples/clocks/hello_48MHz/files/hello_48MHz.c#tok293
    // 18 is the slowest legal I can go for the sys pll

    // enums for voltage at:
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h

    // FIX! we never restore from this core voltage. assuming we stay at 18 Mhz
    if (ALLOW_LOWER_CORE_VOLTAGE_MODE && PLL_SYS_MHZ == 18) {
        vreg_set_voltage(VREG_VOLTAGE_0_95);  // 0_85 crashes for him. 0.90 worked for him
    }

    // finally turn on the gps here! (if we didn't already above (experimental mode)
    digitalWrite(GPS_ON_PIN, HIGH);  // assert

    // Not worth doing if USB is disabled? (no print)
    // but if we can't disable deinit USB (see above), we can?
    if (false && !BALLOON_MODE && !ALLOW_USB_DISABLE_MODE) measureMyFreqs();

    // FIX! still getting intermittent cases where we don't come back (running 60Mhz)
    // this should have no printing either?
    gpsSleepForMillis(5000, false);  // 5 secs

    //******************
    // DRASTIC measures, undo after sleep!
    Watchdog.reset();
    busy_wait_ms(500);
    // FIX! this restores/keeps sys clk to 12mhz and sys pll off
    // the problem is _clock_speed doesn't have 12Mhz?
    // and we need PLL_SYS_MHZ correct for PWM div/wrap calcs
    // can we just change PLL_SYS_MHZ here?
    // NOTE: doesn't include the usb pll?
    kazuClocksRestore(PLL_SYS_MHZ_restore);

    // V1_print(F("Restored core voltage back to 1.1v" EOL));
    V1_flush();
    // don't bother if ?
    if (!BALLOON_MODE && !ALLOW_USB_DISABLE_MODE) measureMyFreqs();
    IGNORE_KEYBOARD_CHARS = false;

    //******************
    Watchdog.reset();

    // hmm. we get a power surge here then? Is it because the Serial2 data
    // was backed up in the gps chip and busy waiting?
    // Drain it? (usually it's the TXT stuff (versions)
    // at power on. Don't care.

    // we should be able to start talking to it
    // gps shold come up at 9600 so look with our uart at 9600?

    // FIX! if we're stuck at 4800, okay..this won't matter
    // initially talking to it at what baud rate?

    // But then we'll be good when we transition to the target rate also
    int BAUD_RATE;
    if (USE_SIM65M) BAUD_RATE = SIM65M_BAUD_RATE;
    else BAUD_RATE = ATGM336H_BAUD_RATE;
    int desiredBaud = checkGpsBaudRate(BAUD_RATE);

    // FIX! does SIM65M sometimes come up in 115200 and
    // sometimes in the last BAUD_RATE set?
    // do both?
    if (USE_SIM65M) {
        // it either comes up in desiredBaud from some memory, or comes up in 115200?
        Serial2.begin(115200);
        busy_wait_ms(500);
        // since Serial2 was reset by setGpsBaud()..
        // could try it again. might aid recovery
        // then up the speed to desired (both gps chip and then Serial2
        setGpsBaud(desiredBaud);
    } else {
        // it either comes up in desiredBaud from some memory, or comes up in 9600?
        Serial2.begin(9600);
        // then up the speed to desired (both gps chip and then Serial2

        // since we're not changing from default 9600 for ATGM336..don't do!
        // setGpsBaud(desiredBaud);
    }

    gpsSleepForMillis(2000, false);  // 1 sec
    // this is all done earlier in the experimental mode
    // FIX! we don't need to toggle power to get the effect?
    if (USE_SIM65M) setGpsBalloonMode();

    // from the CASIC_ProtocolSpecification_english.pdf page 24
    // Could be dangerous,
    // Since it's writing a baud rate to the power off/on reset config state?
    // could change it from 9600 and we'd lose track of what it is?
    // As long as we stick with 9600 we should be safe
    // CAS00. Description Save the current configuration information to FLASH.
    // Even if the receiver is completely powered off,
    // the information in FLASH will not be lost.
    // Format $PCAS00*CS<CR><LF>
    // Example $PCAS00*01<CR><LF>

    // will this help us to boot in a better config so
    // we don't get the power demand peaks we see (on subsequent boots)
    // maybe we shouldn't do this all the time? just once.
    // Does the FLASH have a max # of writes issue? (100k or ??)
    // we only do gps cold reset at start of day.
    // Don't do it in BALLOON_MODE. that should fix the issue
    if (ALLOW_UPDATE_GPS_FLASH_MODE && !BALLOON_MODE) {
        if (USE_SIM65M) {
            V1_print(F("SIM65M: Write GPS config"));
            V1_print(F(" (no broadcast, GNSS service disabled)" EOL));
            V1_print(F("SIM65M: (NO) still default constellations? (4)"));
            V1_print(F(" GPS/BDS/GLONASS/GALILEO" EOL));
            V1_print(F("SIM65M: to GPS Flash (for use in next GPS cold reset?)" EOL));
        } else {
            V1_print(F("ATGM336H: Write GPS current config"));
            V1_print(F(" (no broadcast, just GPS constellations" EOL));
            V1_print(F("ATGM336H: to GPS Flash (for use in next GPS cold reset?)" EOL));
        }
        // this will init to just GPS for the right then restore as below
        writeGpsConfigNoBroadcastToFlash();
        // restores to desired constellations and broadcast
    }

    // all constellations GPS/BaiDou/Glonass
    // setGpsConstellations(7);
    // FIX! try just gps to see effect on power on current
    setGpsConstellations(DEFAULT_CONSTELLATIONS_CHOICE);
    // no ZDA/ANT TXT (NMEA sentences) after this:
    setGpsBroadcast();

    // I guess it doesn't power on with location service on
    if (USE_SIM65M) setGnssOn_SIM65M();

    bool sentencesFound = getInitialGpsOutput();
    // flush out any old state in TinyGPSplus, so we don't get a valid fix that's got
    // a big fix_age
    invalidateTinyGpsState();

    GpsStartTime = get_absolute_time();  // usecs

    if (sentencesFound) GpsIsOn_state = true;
    uint64_t duration_millis = millis() - start_millis;
    V1_print(F("GpsFullColdReset END"));
    V1_printf(" sentencesFound %u", sentencesFound);
    V1_printf(" duration_millis %" PRIu64 EOL, duration_millis);
    return sentencesFound;
}

//************************************************
bool GpsWarmReset(void) {
    // FIX! SIM65M spec says when the power supply is off, settings
    // are reset to factory config and receiver performs a cold start
    // on next power up
    // I suppose we should just switch to idle mode instead of powering
    // gps chip off?
    V1_println(F("GpsWarmReset START"));
    uint64_t start_millis = millis();
    GpsIsOn_state = false;
    GpsStartTime = get_absolute_time();  // usecs
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();
    // warm reset doesn't change baud rate from prior config?

    // turn it off first. may be off or on currently
    // turn off the serial
    V1_flush();

    // don't assert reset during power off
    // FIX! what if we power on with GPS_ON_PIN LOW and GPS_NRESET_PIN HIGH
    V1_println(F("Doing Gps WARM POWER_ON (GPS_ON_PIN off with power off-on)"));
    // NOTE: should we start with NRESET_PIN low also until powered (latchup?)?
    // NOTE: do we need to start low until powered to avoid latchup of LNA?

    // reorganized to match GpsFullColdReset()
    digitalWrite(GPS_ON_PIN, LOW);
    digitalWrite(GPS_NRESET_PIN, HIGH);
    digitalWrite(GpsPwr, HIGH);
    Serial2.end();
    // 2 secs off?
    gpsSleepForMillis(2000, false);  // no early out

    // match the pwm that's done for cold reset
    if (PWM_GPS_POWER_ON_MODE) {
        // this is probably at least 2 secs. let's measure
        uint64_t start_millis2 = millis();
        pwmGpsPwrOn();
        uint64_t duration_millis2 = millis() - start_millis2;
        V1_printf("Used pwmGpsPwrOn() and took %" PRIu64 " millisecs" EOL, duration_millis2);
        // soft power-on for GpsPwr (assert low, controls mosfet)
        // note that vbat doesn't have mosfet control, so it will be high right away
        // with availability of power
    } else {
        digitalWrite(GpsPwr, LOW);   // assert to mosfet
        gpsSleepForMillis(500, false);
    }

    gpsSleepForMillis(2000, false);  // no early out
    // now assert the on/off pin
    digitalWrite(GPS_ON_PIN, HIGH);
    gpsSleepForMillis(2000, false);  // no early out

    //****************************
    // not used?
    int BAUD_RATE;
    if (USE_SIM65M) BAUD_RATE = SIM65M_BAUD_RATE;
    else BAUD_RATE = ATGM336H_BAUD_RATE;
    int desiredBaud = checkGpsBaudRate(BAUD_RATE);

    //****************************
    // should come up in the last programmed baud rate (from cold reset)
    if (USE_SIM65M) {
        // Hmm. did we have failures where it didn't come up in the right baud rate?
        // it either comes up in desiredBaud from some memory (9600?), or comes up in 115200?
        Serial2.begin(115200);
        gpsSleepForMillis(500, false);  // no early out
        setGpsBaud(desiredBaud);
        Serial2.begin(9600);
        gpsSleepForMillis(500, false);  // no early out
        setGpsBaud(desiredBaud);
        
    } else {
        // it either comes up in desiredBaud from some memory, or comes up in 9600?
        // we never change ATGM336 baud rate now.
        Serial2.begin(9600);
    }
    gpsSleepForMillis(500, false);  // no early out

    if (WARM_RESET_REDO_CONFIG) {
        //****************************
        // all constellations
        setGpsConstellations(DEFAULT_CONSTELLATIONS_CHOICE);
        // set desired broadcast
        // we don't need no ZDA/TXT
        setGpsBroadcast();

        // always reconfig this. there were known issues with ublox losing this?
        // also: hard to realize if we lost it, unless we read the mode?
        if (USE_SIM65M) setGpsBalloonMode();
    }

    if (USE_SIM65M) {
        // always read it to make sure it's right thru warm reset
        V1_println(F("Read the navigation mode: $PAIR081*33"));
        // Packet Type:081 PAIR_COMMON_GET_NAVIGATION_MODE
        Serial2.print("$PAIR081*33" CR LF);
        nmeaBufferFastPoll(2000, true);  // duration_millis, printIfFull
        // we could change the default config to power up with GNSS off?
        // so always do this?
        setGnssOn_SIM65M();
    }

    bool sentencesFound = getInitialGpsOutput();
    // flush out any old state in TinyGPSplus, so we don't get a valid fix that's got
    // a big fix_age
    invalidateTinyGpsState();
    GpsIsOn_state = true;
    GpsStartTime = get_absolute_time();  // usecs

    if (sentencesFound) GpsIsOn_state = true;
    uint64_t duration_millis = millis() - start_millis;
    V1_print(F("GpsWarmReset END"));
    V1_printf(" sentencesFound %u", sentencesFound);
    V1_printf(" duration_millis %" PRIu64 EOL, duration_millis);
    return sentencesFound;
}

//************************************************
void writeGpsConfigNoBroadcastToFlash() {
    V1_println(F("writeGpsConfigNoBroadcastToFlash() START"));
    // FIX! we'll have to figure this out for SIM65M
    // turn off the GNSS_SUBSYS so it's not on at gps cold reset?

    // Packet Type:002 PAIR_GNSS_SUBSYS_POWER_ON
    // Power on the GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // Please send this command before using any location service.
    // $PAIR002*38

    // Packet Type:003 PAIR_GNSS_SUBSYS_POWER_OFF
    // Power off GNSS system. Include DSP/RF/Clock and other GNSS modules.
    // CM4 also can receive commands after sending this command
    // (Include the AT command / the race Command / the part of PAIR
    // command which is not dependent on DSP.)
    // The location service is not available after this command is executed.
    // The system can still receive configuration PAIR commands.
    // $PAIR003*39

    // risk: do we ever power on and not do this full cold reset
    // that sets up broadcast?
    // the warm gps reset shouldn't get new state from config?

    // HMM! should we change it to no broadcast, in the FLASH,
    // so cold reset power on might try to do no broadcast
    disableGpsBroadcast();
    // FIX! just gps. what about 0. would that save power at gps power on?
    // FIX! this doesn't change SIM65M default constellations (yet)
    setGpsConstellations(1);

    // this will cause a gps cold reset config with GNSS service off (SIM65M only)
    if (USE_SIM65M) setGnssOff_SIM65M();

    char nmeaSentence[64] = { 0 };
    if (USE_SIM65M) {
        // Packet Type:513 PAIR_NVRAM_SAVE_SETTING
        // Save the current configuration from RTC RAM to flash.
        // $PAIR513*3D
        // In multi-Hz, this command can only be set when
        // the GNSS system is powered off,
        // while 1Hz does not have this limitation.
        // <what does that mean? we do have GNSS off, so I guess okay?>

        // in case we mess up the gps flash
        // Packet Type:514 PAIR_NVRAM_RESTORE_DEFAULT_SETTING
        // Clear the current configuration and restore the default settings.
        // This function does not support run time restore when GNSS is power on.
        // Please send PAIR_GNSS_SUBSYS_POWER_OFF to power off GNSS
        // before use this command
        // $PAIR514*3A
        strncpy(nmeaSentence, "$PAIR513*3D" CR LF, 64);
    } else {
        strncpy(nmeaSentence, "$PCAS00*01" CR LF, 64);
    }
    V1_printf("%s" EOL, nmeaSentence);
    Serial2.print(nmeaSentence);
    Serial2.flush();
    sleep_ms(1000);

    // we could change the default config to power up with GNSS off?
    if (USE_SIM65M) setGnssOn_SIM65M();
    // set desired constellations
    // FIX! this doesn't change SIM65M default constellations (yet)
    setGpsConstellations(DEFAULT_CONSTELLATIONS_CHOICE);
    // set desired broadcast.
    setGpsBroadcast();

    V1_println(F("writeGpsConfigNoBroadcastToFlash END"));
}

//************************************************
void GpsON(bool GpsColdReset) {
    // no print if no cold reset request.
    // So I can grep on GpsColdReset as a special case only
    if (!GpsColdReset) {
        V1_printf(EOL "GpsON START GpsIsOn_state %u" EOL, GpsIsOn_state);
    } else {
        V1_printf(EOL "GpsON START GpsIsOn_state %u GpsColdReset %u" EOL,
            GpsIsOn_state, GpsColdReset);
    }

    // could be off or on already
    // Assume GpsINIT was already done (pins etc)
    Watchdog.reset();

    bool sentencesFound = false;
    uint32_t tryCnt = 0;

    if (!GpsColdReset && GpsIsOn()) {
        // fake this to avoid doing a gps warm reset if successfully on?
        V1_println(F("do nothing because GpsIsOn()"));
        sentencesFound = true;
    }

    // don't care what the initial state is, for cold reset
    while (!sentencesFound) {
        tryCnt += 1;
        if (tryCnt >= 5) {
            if (GpsColdReset) {
                V1_println(F("ERROR: tryCnt 5 on GpsFullColdReset.. not retrying any more"));
                break;
            } else {
                V1_println(F("ERROR: tryCnt 5 on GpsWarmReset.. switch to trying GpsColdReset"));
                GpsColdReset = true;
                tryCnt = 0;
            }
        }
        if (GpsColdReset) {
            sentencesFound = GpsFullColdReset();
        } else {
            sentencesFound = GpsWarmReset();
        }
    }

    if (!GpsColdReset) {
        V1_printf("GpsON END GpsIsOn_state %u" EOL EOL, GpsIsOn_state);
    } else {
        V1_printf("GpsON END GpsIsOn_state %u GpsColdReset %u" EOL EOL,
            GpsIsOn_state, GpsColdReset);
    }

    V1_flush();
}

//************************************************
/*
This used to be in the LightAPRS version of TinyGPSPlus-0.95
instead updated TinyGPSPlus (latest) in libraries to make them public,
not private
< #if defined(ARDUINO_ARCH_RP2040)
< void TinyGPSDate::clear()
< {
<    valid = updated = false;
<    date = 0;
< }b
< #endif
*/

//************************************************
void invalidateTinyGpsState(void) {
    V1_println(F("invalidateTinyGpsState() START"));
    // gps.date.clear(); // kazu had done this method
    // are these declared private?
    // FIX! how can we clear these? Do we change the library to make them public?

    // from TinyGPS++.h in libraries..modify it and move to public?
    // (in struct TinyGPSDate what about location etc? they have valid and updated

    // private:
    // bool valid, updated;
    // uint32_t date, newDate;
    // uint32_t lastCommitTime;
    // void commit();
    // void setDate(const char *term);
    // TinyGPS++.h:   bool valid, updated;

    // TinyGPS also has lastCommitTime = millis()
    // we don't change that?

    // these three are the initial value
    // this should work without changing the TinyGPS++ library
    // did this not work?
    if (false) {
        // should get us ignoring least 2 GPS broadcasts? Two cycles through loop1() ?
        GpsInvalidAllCnt = 2;
        GpsInvalidAll = true;
    }

    //***********************************
    // how do we clear a fix from TinyGPS++ ?
    // do we wait until we turn GpsON() on, beforre clearing GPSInvalidAll
    // we can decrement the count only if gps is on? (in setup1())
    // new public methods created in TinyGPS++.h
    gps.location.flush();
    gps.location.fixQualityFlush();
    gps.location.fixModeFlush();
    gps.date.flush();
    gps.time.flush();
    V1_println(F("invalidateTinyGpsState END"));
}

//************************************************
void GpsOFF() {
    V1_printf("GpsOFF START GpsIsOn_state %u" EOL, GpsIsOn_state);
    digitalWrite(GpsPwr, HIGH);
    // Serial2.end() Disables serial communication,
    // To re-enable serial communication, call Serial2.begin().
    // FIX! do we really need or want to turn off Serial2?
    // Remember to Serial2.begin() when we turn it back on
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);
    // unlike i2c to vfo, we don't tear down the Serial2 definition...just .end()
    // so we can just .begin() again later
    // have to flush everything. Can't keep enqueued time. not worth saving altitude/lat/lon..we have to 
    // wait for time to get set again?
    invalidateTinyGpsState();

    GpsIsOn_state = false;
    GpsStartTime = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    V1_flush();
    V1_printf("GpsOFF END GpsIsOn_state %u" EOL, GpsIsOn_state);
}


//************************************************
// FIX! why was this static void before?
uint64_t updateGpsDataAndTime(int ms) {
    // to make sure we get some update, even if fix_age is larger than 1 sec.
    V1_println(F("updateGpsDataAndTime START"));
    Watchdog.reset();

    // ms has to be positive?
    // grab data for no more than ms milliseconds
    // stop if no data for 50 milliseconds
    // all the durations below won't start counting until we get the first char
    // (sets start_millis())
    uint64_t start_millis = 0;
    uint64_t last_serial2_millis = 0;
    uint64_t timeSinceLastChar_millis = 0;
    uint64_t duration_millis = 0;
    uint64_t current_millis = millis();
    uint64_t entry_millis = current_millis;

    V1_printf(
        "updateGpsDataAndTime started looking for NMEA current_millis %" PRIu64 EOL,
        current_millis);

    // clear the StampPrintf buffer, in case it had anything.
    if (VERBY[1]) DoLogPrint();

    // FIX! we could leave here after we get N sentences?
    // we could keep track of how many sentences we get?
    // ideally we'd synchronize on the currently uknown start/end sentences?
    // Or we could exit when we get two of the same sentence? two of what?
    GpsON(false);

    // inc on '$'
    // static: keep the count since the first time we start doing this function
    // can visually compare these in the stdout...ideally should always be the same
    static int sentenceStartCnt = 0;
    // inc on '*' (comes before the checksum)
    // hmm. can we inc on getting complete *nn (checksum)
    static int sentenceEndCnt = 0;

    // unload each char to TinyGps++ object as it arrives and print it
    // (actually into nmeaBuffer)
    // so we can see NMEA sentences for a period of time.
    // assume 1 sec broadcast rate
    // https://arduino.stackexchange.com/questions/13452/tinygps-plus-library

    // Could keep the sum for all time, and track total busy time..
    // but I think better not to average..just track this particular call.
    int incomingCharCnt = 0;
    int charsToDrain = 0;
    bool nullChar = false;
    bool spaceChar = false;
    bool printable = true;

    // do at most charsAvailable. the initial state of the fifo
    // Can't use them because we may have discarded some chars after buffer full
    getChar();
    if (charsAvailable >= 31) {
        if (VERBY[1])
            StampPrintf("INFO: initially drained NMEA chars because rx full. uart rx %d" EOL,
                (int) charsAvailable);
        charsToDrain = charsAvailable;
        Watchdog.reset();
        // should be at most 31 to drain
        while (charsToDrain != 0) {
            getChar();
            charsToDrain -= 1;
        }
    }

    // don't start sending to TinyGPS until we get $|CR|LF so we know we're aligned
    // can't do in another loop, because of delays getting chars. Have
    // to have it in this main timeout loop
    bool aligned = false;

    // incomingChar will be '0' if charsAvailable is 0 at this point
    // incomingChar could have a valid char if charsAvailable was nonzero.
    // don't drop it. (the '$' start of sentence draining case above)
    current_millis = millis();
    // works if ms is 0
    while ((current_millis - entry_millis) < (uint64_t) ms) {
        while (charsAvailable > 0) {
            // we count all chars, even CR LF etc
            incomingCharCnt++;
            // start the duration timing when we get the first char
            if (start_millis == 0) start_millis = current_millis;
            // shouldn't happen any more?
            if (VERBY[1] && charsAvailable >= 31) {
                StampPrintf("ERROR: full. uart rx depth %d incomingCharCnt %d" EOL,
                    (int) charsAvailable, incomingCharCnt);
            }
            // do we get any null chars?
            // are CR LF unprintable?
            spaceChar = false;
            nullChar = false;
            printable = isprint(incomingChar);
            // aligned set to true only matters for the first one
            switch (incomingChar) {
                case '$':  aligned = true; sentenceStartCnt++; break;
                case '\r': aligned = true; break;
                case '\n': aligned = true; break;
                case '*':  sentenceEndCnt++; break;
                case '\0': nullChar = true; break;
                case ' ':  spaceChar = true; break;
                default: { ; }
            }
            // after aligning, send everything to TinyGPS++ ??
            // does it expect the CR LF between sentences?
            if (aligned) gps.encode(incomingChar);

            // always strip these here, and continue the loop
            if (spaceChar || nullChar || !printable) {
                getChar();
                current_millis = millis();
                continue;
            }

            // FIX! ignoring unprintables. Do we even get any? maybe in error?
            // either a number (0123456789),
            // an uppercase letter ABCDEFGHIJKLMNOPQRSTUVWXYZ
            // a lowercase letter  abcdefghijklmnopqrstuvwxyz
            // a punctuation character !"#$%&'()*+,-./:;<=>?@[\]^_`{|}~ ,
            // or <space>,
            // or any character classified as printable by the current C locale.

            // hmm are we getting any space chars?
            // FIX! we even send the CR and LF to the TinyGPS++ ??
            // is it necessary?

            // Note we disabled the GPTXT broadcast to reduce the NMEA load (for here)

            // make the nmeaBuffer big enough so that we never print while getting data?
            // and we never throw it away (lose data) ?? (for debug only though)
            // this should eliminate duplicate CR LF sequences and just put one in the stream

            // FIX! might be odd if a stop is spread over two different calls here?
            // always start printing again on inital call to this function
            // (see inital state)

            // Do we get any unprintable? ignore unprintable chars, just in case.
            if (VERBY[1]) {
                if (printable && !nullChar && !spaceChar) {
                    // I guess we get the \r \n in the buffer now
                    nmeaBufferAndPrint(incomingChar, false);
                }
            }
            current_millis = millis();
            last_serial2_millis = current_millis;
            // setup for loop iteration
            getChar();
        }

        //*******************
        // keep it as close as possible to the NMEA sentence arrival?
        // I suppose we'll see gps.time.updated every time?
        checkUpdateTimeFromGps();
        //*******************

        // did we wait more than ?? millis() since good data read?
        // we wait until we get at least one char or go past the ms total wait
        // break out when we don't the next char right away
        updateStatusLED();

        if (last_serial2_millis == 0) timeSinceLastChar_millis = 0;
        else timeSinceLastChar_millis = current_millis - last_serial2_millis;

        // FIX! should the two delays used be dependent on baud rate?
        if (timeSinceLastChar_millis >= 50) {
            // FIX! could the LED blinking have gotten delayed?
            // we don't check in the available loop above.
            // save the info in the StampPrintf buffer..don't print it yet
            // if we didn't get any chars, start_millis will be 0, use entry_millis instead
            break;
        }
        gpsSleepForMillis(50, true);  // stop the wait early if symbols arrive
        // setup for loop iteration
        getChar();
        current_millis = millis();
    }

    if (start_millis != 0)
        duration_millis = current_millis - start_millis;
    else
        duration_millis = 0;

    //*******************
    if (VERBY[1]) {
        // print/clear any accumulated NMEA sentence stuff
        nmeaBufferPrintAndClear();
        V1_print(F(EOL));
        DoLogPrint();
    }

    int diff = sentenceStartCnt - sentenceEndCnt;
    V1_print(F("updateGpsDataAndTime:"));
    V1_printf(" start_millis %" PRIu64 " current_millis %" PRIu64,
        start_millis, current_millis);
    V1_printf(" sentenceStartCnt %d sentenceEndCnt %d diff %d" EOL,
        sentenceStartCnt, sentenceEndCnt, diff);
    V1_flush();

    // This will be lower than a peak rate
    // It includes dead time at start, dead time at end...
    // With some constant rate in the middle? but sentences could be split..
    // fixed: entry_millis is the entrance to the function
    // star_millis is the first char. so duration_millis will
    // include the end stall detect (25 millis)
    // So it's an average over that period.
    float AvgCharRateSec;
    if (duration_millis == 0) AvgCharRateSec = 0;
    else AvgCharRateSec = 1000.0 * ((float)incomingCharCnt / (float)duration_millis);
    // can it get too big?
    if (AvgCharRateSec > 999999.9) AvgCharRateSec = 999999.9;
    V1_printf(
        "NMEA sentences: AvgCharRateSec %.f duration_millis %" PRIu64 " incomingCharCnt %d" EOL,
        AvgCharRateSec, duration_millis, incomingCharCnt);
    V1_flush();

    //******************************
    // checksum errors at TinyGPS?
    static uint32_t last_gcp;
    static uint32_t last_gswf;
    static uint32_t last_gfc;

    uint32_t gcp = gps.charsProcessed();
    uint32_t gswf = gps.sentencesWithFix();
    uint32_t gfc = gps.failedChecksum();

    V1_printf("TinyGPS       charsProcessed %10lu sentencesWithFix %5lu failedChecksum %5lu" EOL,
        gcp, gswf, gfc);
    // can visually compare to prior sentences received and printed and see if tinygps
    // is reporting check sum errors that shouldn't exist?
    V1_printf("TinyGPS delta charsProcessed %10lu sentencesWithFix %5lu failedChecksum %5lu" EOL,
        gcp - last_gcp, gswf - last_gswf, gfc - last_gfc);
    last_gcp = gcp;
    last_gswf = gswf;
    last_gfc = gfc;

    //******************************
    updateStatusLED();
    uint64_t total_millis = millis() - entry_millis;
    // will be interesting to compare total_millis to duration_millis
    V1_printf("updateGpsDataAndTime END total_millis %" PRIu64 EOL EOL, total_millis);
    return total_millis;
}

//************************************************
void checkUpdateTimeFromGps() {
    static bool gpsDateTimeWasUpdated = false;
    static uint64_t lastUpdate_millis = 0;

    // time since last update. Don't update more than once every 30 secs
    uint64_t elapsed_millis = millis() - lastUpdate_millis;
    if (gpsDateTimeWasUpdated && elapsed_millis < 30000) { 
        return;
    }

    uint16_t gps_year = gps.date.year();
    bool gps_year_valid = gps_year >= 2025 && gps_year <= 2034;
    uint32_t fix_age = gps.time.age();

    // so how about we only update when fix_age is < 100 millisecs??
    // (10 ms for possible code delays here
    if (!gps_year_valid || fix_age > 100 || 
            GpsInvalidAll || !gps.date.isValid() || !gps.time.isValid()) {
        return;
    }
            
    // FIX! don't be updating this every time
    // this will end up checking every time we get a burst?
    // uint8_t for gps data
    // the Time things are int
    // see example https://arduiniana.org/libraries/TinyGPS/
    // use now for setting rtc, so we have a better solar elevation
    // calc if the time is old
    // lat/lon will still be old. we don't use altitude for that calc?

    // these all should be stable/consistent as we're gathering them?
    uint8_t gps_month = gps.date.month();
    uint8_t gps_day = gps.date.day();
    uint8_t gps_hour = gps.time.hour();
    uint8_t gps_minute = gps.time.minute();
    uint8_t gps_second = gps.time.second();

    // to get a consistent snapshot of all
    // store the current time in time variable t
    time_t t = now();

    // all the Time things are int
    // https://stackoverflow.com/questions/6636793/what-are-the-general-rules-for-comparing-different-data-types-in-c
    uint16_t y = (uint16_t) year(t);
    uint8_t m = (uint8_t) month(t);
    uint8_t d = (uint8_t) day(t);
    uint8_t hh = (uint8_t) hour(t);
    uint8_t mm = (uint8_t) minute(t);
    uint8_t ss = (uint8_t) second(t);

    if (gpsDateTimeWasUpdated && 
        y == gps_year && m == gps_month && d == gps_day &&
        hh == gps_hour && mm == gps_minute && ss == gps_second) {
        return;
    }
    V1_print(F("WARN: checkUpdateTimeFromGps not set or drift?" EOL));

    uint8_t gps_hundredths = gps.time.centisecond();

    //******************************
    // void setTime(int hr,int min,int sec,int dy, int mnth, int yr){
    // year can be given as full four digit year or two digts (2010 or 10 for 2010);
    // it is converted to years since 1970
    // we don't compare day/month/year on time anywhere, except when looking
    // for bad time from TinyGPS state (tracker.ino)
    // FIX! should I work about setting day month year? why not
    // then we can get all that info from the rtc, so if the gps data is
    // stail for time, we get a better solar elevation calculation..accurate time?

    // JUST IN CASE: let's validate the ranges and not update if invalid!!
    // all uint8_t so don't have to check negatives
    bool gpsDateTimeBad = false;
    if (gps_hour > 23) gpsDateTimeBad = true;
    if (gps_minute > 59) gpsDateTimeBad = true;
    if (gps_second > 59) gpsDateTimeBad = true;
    // should we validate by the month? forget about that.
    // unlikely to glitch that way?
    if (gps_day > 31) gpsDateTimeBad = true;
    if (gps_month > 12) gpsDateTimeBad = true;
    if (gps_month < 1) gpsDateTimeBad = true;
    // should already have validated year range? but do it here too
    // will have to remember to update this in 10 years (and above too!)
    if (gps_year < 2025 && gps_year > 2034) gpsDateTimeBad = true;

    if (gps_hundredths > 99) {
        V1_printf("ERROR: TinyGPS gps_hundredths %u > 99" EOL,
            gps_hundredths);
        gpsDateTimeBad = true;
    }

    // what the heck, check the days in a month is write.
    // (subtract one from month)
    // if we have a valid month for this array!
    static const uint8_t monthDays[] =
        {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (!gpsDateTimeBad) {
        // we know will be 0 to 11, from above check of gps_month
        uint8_t validDays = monthDays[gps_month - 1];
        if (gps_day > validDays) {
            V1_print(F("ERROR: gps time is bad."));
            V1_printf(" day %u too big. max %u for the month" EOL,
                gps_day, validDays);
            gpsDateTimeBad = true;
        }
    }

    if (gpsDateTimeBad) {
        V1_print(F("ERROR: gps time is bad. Maybe no received gps time yet."));
        V1_printf(" gps_hour %u gps_minute %u gps_second %u",
            gps_hour, gps_minute, gps_second);
        V1_printf(" gps_day %u gps_month %u gps_year %u" EOL,
            gps_day, gps_month, gps_year);
    } else {
        // did we flush TinyGPS state out when we turn gps off?
        if (false) {
            V1_print(F(EOL));
            V1_print(F(EOL));
            bool validA = gps.satellites.isValid() && !GpsInvalidAll;
            bool validB = gps.hdop.isValid() && !GpsInvalidAll;
            bool validC = gps.location.isValid() && !GpsInvalidAll;
            bool validD = gps.altitude.isValid() && !GpsInvalidAll;
            bool validE = gps.course.isValid() && !GpsInvalidAll;
            bool validF = gps.speed.isValid() && !GpsInvalidAll;
            // FIX! don't have GpsInvalidAll in these
            bool validG = gps.date.isValid();
            bool validH = gps.time.isValid();

            V1_printf("gps valids: %u %u %u %u %u %u %u %u %u" EOL,
            !GpsInvalidAll, validA, validB, validC, validD, validE, validF, validG, validH);
        }
        setTime(gps_hour, gps_minute, gps_second, gps_day, gps_month, gps_year);
        // should be UTC time zone?


        if (USE_SIM65M) {
            // V1_print(F("Do a read of gps time to see if we get <1 sec precision" EOL));
            // this is only time to the second
            // Serial2.print("$PAIR001,591,0*36" CR LF);
            // GLL GGA RMC ZDA is time to thousandsths
        }

        V1_print(F("GOOD: rtc setTime() with"));
        V1_printf(" gps_hour %u gps_minute %u gps_second %u",
            gps_hour, gps_minute, gps_second);
        V1_printf(" gps_day %u gps_month %u gps_year %u" EOL,
            gps_day, gps_month, gps_year);

        V1_print(F("time (t) was:"));
        V1_printf(" hour %d minute %d second %d", hh, mm, ss);
        V1_printf(" day %d month %d year %d", d, m, y);
        V1_printf(" gpsDateTimeWasUpdated %u" EOL, gpsDateTimeWasUpdated);

        // did we cross a minute boundary for comparing the two
        int minuteDelta = 0;
        if (gps_minute != mm) {
            minuteDelta = ((int) mm) - ((int) gps_minute);
        }

        // check hour delta also? to cover hour transitions
        int hourDelta = 0;
        if (gps_hour != mm) {
            hourDelta = ((int) hh) - ((int) gps_hour);
        }

        // check day delta also? to cover day transitions
        // won't bother with month or year transitions
        int dayDelta = 0;
        if (gps_day != mm) {
            dayDelta = ((int) d) - ((int) gps_day);
        }
        int secondDelta = ((int) ss) - ((int) gps_second);
        secondDelta += (60 * minuteDelta) + (60 * 60 * hourDelta) + (60 * 60 * 24 * dayDelta);

        // add in the minuteDelta cover minute transitions
        // too much drift/error?
        // don't print the first time thru..since that doesn't matter
        if (gpsDateTimeWasUpdated) {
            V1_printf("system vs gps: total secondDelta %d" EOL, secondDelta);
            if (abs(secondDelta) > 1) {
                V1_printf("ERROR: unexpected abs(secondDelta) > 1:  secondDelta %d" EOL, 
                    secondDelta);
            }
        }

        V1_printf("gps fix_age was: %lu" EOL, fix_age);
        fix_age = gps.time.age();
        V1_printf("gps fix_age currently: %lu" EOL, fix_age);

        // bump a sec to account for delays from gps to the code above?
        // and maybe any floor effects (ignoring millisecs) that the Time library does?
        // adjustTime(1);

        // we could just look at hundredths and bump if > 50 ? (rounding?)
        // could bump time by 1 sec?
        // does Time library chop things down to sec by using millis()
        // does that introduce error also?

        // don't do. we should never get hundredths (although sometimes we do)
        // broadcast at 1 sec should have time always at 1 sec granularity?
        // don't think they round though.
        if (false and gps_hundredths > 50) {
            V1_printf("Adjusting time +1 sec because gps_hundredths %u" EOL,
                gps_hundredths);
            adjustTime(1);
        }
        gpsDateTimeWasUpdated = true;
        lastUpdate_millis = millis();
        V1_print(EOL);
    }
}

//************************************************
void gpsDebug() {
    if (!VERBY[1]) return;
    // am I getting problems with constant strings in ram??
    char debugMsg0[] = "Before any gpsDebug prints";
    realPrintFlush(debugMsg0, true);  // print

    V1_println(F("GpsDebug START"));
    V1_print(F(EOL));
    V1_print(F(EOL));
    bool validA = gps.satellites.isValid() && !GpsInvalidAll;
    bool validB = gps.hdop.isValid() && !GpsInvalidAll;
    bool validC = gps.location.isValid() && !GpsInvalidAll;
    bool validD = gps.altitude.isValid() && !GpsInvalidAll;
    bool validE = gps.course.isValid() && !GpsInvalidAll;
    bool validF = gps.speed.isValid() && !GpsInvalidAll;
    // FIX! don't have GpsInvalidAll in these
    bool validG = gps.date.isValid();
    bool validH = gps.time.isValid();
    V1_printf("gps valids: %u %u %u %u %u %u %u %u %u" EOL,
        !GpsInvalidAll, validA, validB, validC, validD, validE, validF, validG, validH);

    printStr("Sats", true, 5);
    printStr("HDOP", true, 5);
    printStr("Latitude", true, 12);
    printStr("Longitude", true, 12);
    printStr("FixAge", true, 7);
    // 2025-01-04 21:46:31
    printStr("Date", true, 11);
    printStr("Time", true, 9);
    printStr("DTAge", true, 6);
    printStr("Alt", true, 8);
    printStr("Course", true, 7);
    printStr("Degs.", true, 6);
    printStr("Speed", true, 6);
    printStr("ChrsRx", true, 10);
    printStr("SentsWfix", true, 10);
    printStr("failCksum", true, 10);
    V1_print(F(EOL));

    // am I getting problems with constant strings in ram??
    char debugMsg1[] = "Before printInt/Float/String gpsDebug prints";
    realPrintFlush(debugMsg1, false);  // no print

    // https://github.com/StuartsProjects/GPSTutorial
    if (VERBY[1]) {
        printInt(gps.satellites.value(), validA, 5);
        printInt(gps.hdop.value(), validB, 5);
        printFloat(gps.location.lat(), validC, 12, 6);
        printFloat(gps.location.lng(), validC, 12, 6);
        printInt(gps.location.age(), validC, 7);
        // printAge. date & time isValid() is in the function
        printGpsDateTime(gps.date, gps.time, true);
        printFloat(gps.altitude.meters(), validD, 8, 2);
        printFloat(gps.course.deg(), validE, 7, 2);
        printFloat(gps.speed.kmph(), validF, 6, 2);
        printStr(TinyGPSPlus::cardinal(gps.course.value()), validE, 6);
        // FIX! does this just wrap wround if it's more than 6 digits?
        printInt(gps.charsProcessed(), true, 10);
        // FIX! does this just wrap wround if it's more than 10 digits?
        printInt(gps.sentencesWithFix(), true, 10);
        printInt(gps.failedChecksum(), true, 10);
    }
    V1_print(F(EOL));
    V1_print(F(EOL));
    // am I getting problems with constant strings in ram??
    char debugMsg2[] = "After all gpsDebug prints";
    realPrintFlush(debugMsg2, true);  // print

    V1_println(F("GpsDebug END"));
}

//*****************
// Was wondering why the HDOP was so high.
// it seems the 90 really means 90 / 100 = .9 HDOP
// i.e. less than 1. so that's ideal. Yeah!

// https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)

// DOP
// < 1 Ideal Highest possible confidence
// 1–2 Excellent
// 2–5 Good Represents a level that marks the minimum appropriate
// 5–10 Moderate
// 10–20 Fair Represents a low confidence level.
// > 20 Poor At this level, measurements should be discarded.

// https://github.com/mikalhart/TinyGPSPlus/issues/8
// Was design decision to deliver the HDOP exactly as in the NMEA string.
// NMEA reports HDOP in hundredths.
// We could have made this return a floating-point with the correct value,
// but at the time it seemed better to not introduce floating-point values.
// Just convert to float and divide by 100 and you should be good.
// Mikal

//*****************
// Notes:
// Arduino IDE allows function definitions after the point they are used
// used without needing an explicit function prototype before hand.
// The Arduino build creates these prototypes but not always correctly,
// leading to errors which are not obvious.
// (Did I have case of a uint64_t return variable, getting chopped to 32-bits?)
// Example: if the function argument list contains user defined data types and
// the automatically created function prototype is placed before
// the declaration of that data type.

//*****************
// notes on bad power on of ATGM336
// https://www.eevblog.com/forum/rf-microwave/gps-lna-overheating-on-custom-pcb/

// Magic sequence to turn the LNA into a toaster is:
// 1) Have the 3V3 power off;
// 2) Enable the ON_OFF pin
//   (HIGH signal from an STM32 - powered by separate 3.0V LDO linear regulator);
// 3) Turn 3V3 power on.
// 4) Hot LNA!
// After that, the only thing that cools down the LNA is turning 3V3 off again.
// Disabling the ON_OFF pin does nothing.

// classic example of latch up caused by an input voltage exceeding a power rail?
// https://en.wikipedia.org/wiki/Latch-up

// https://forum.arduino.cc/t/gps-power-management-reset-loop/529253/5
// which had a good suggestion to simply increase
// the value of the gate resistor to slow the switching of the MOSFET.
// So, increased gate resistor to 47 kOhm,
// Turn on time increased to 150 μs
// and the Arduino didn't reset when the GPS was turned on!
// Increased the value of my gate resistor again to 100 kOhm,
// which further increased the turn on time to 250 μs and
// also seemed to fix the reset problem.
// With these higher value gate resistors,
// measured the source voltage dropping to only ~3.0 V
// when the GPS is turned on (~250 mV).


//************************************************
void kazuClocksSlow() {
    V1_println(F("kazuClocksSlow START" EOL));
    V1_flush();

    if (ALLOW_TEMP_12MHZ_MODE) {
        // We go down to 12 mhz for lowest power no matter what. We just don't
        // stay at 12 unless ALLOW_KAZU_12MHZ_MODE
        // maybe on restore, we have to go to 18 before we restore usb?
        // Change clk_sys to be 12MHz
        // the external crystal is 12mhz
        clock_configure(clk_sys,
            CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
            CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
            12 * MHZ,
            12 * MHZ);

        // now can turn off pll sys to save power
        pll_deinit(pll_sys);
    }

    // turn off pll usb to save power
    if (!BALLOON_MODE && ALLOW_USB_DISABLE_MODE) {
        // this will stop the ability to print
        // in BALLOON_MODE, this causes a reboot cause no usb
        pll_deinit(pll_usb);
    }

    // so we can visually ID we were here
    blockingLongBlinkLED(3);

    // don't do this. because we actually restore to a pll sys value
    // so we should not change these..assume go back just the same as it was
    if (ALLOW_KAZU_12MHZ_MODE) {
        // Change clk_peri to be 12MHz, and disable pll_sys and pll_usb
        // the external crystal is 12mhz

        // CLK peri is clocked from clk_sys so need to change clk_peri's freq
        // 12/15/24 hmm maybe clk_peri needs to be 48 Mhz ?
        // nope 48 doesn't help Serial2
        // maybe it needs the ..AUXSRC_VALUE_XOSC_CLKSRC ? seems legal here
        // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_clocks/include/hardware/clocks.h

        // this guy had similar issues?
        // https://github.com/raspberrypi/pico-sdk/issues/1037

        clock_configure(clk_peri,
            0,  // No GLMUX
            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
            12 * MHZ,
            12 * MHZ);
            // 48 * MHZ,
            // 8 * MHZ);  // should this be 8 per the link above?

        // was:
        // CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,

        // he reinits uart this way
        // Re init uart now that clk_peri has changed
        // stdio_init_all();
        // Fixes baudrate form unchanged clock. 48MHz / 8MHz = 6
        // uart_set_baudrate(uart0, 115200/6);

        // tried
        // CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,

        // CLK RTC = XOSC 12MHz / 256 = 46875Hz
        // FIX! this should be usb clk / 1024 ?? around
        // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_runtime_init/runtime_init_clocks.c

        clock_configure(clk_rtc,
            0,  // No GLMUX
            CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
            12 * MHZ,
            46875);

        // CLK ADC = XO (12MHZ) / 1 = 12MHz
        clock_configure(clk_adc,
            0,  // No GLMUX
            CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
            12 * MHZ,
            12 * MHZ);
    }

    // can't print without USB now
    // V1_println(F("kazuClocksSlow END" EOL));
}

//************************************************
void kazuClocksRestore(uint32_t PLL_SYS_MHZ_restore) {
    uint32_t freq_khz = PLL_SYS_MHZ_restore * 1000UL;
    // if kazuClocksRestore() is only used after kazuClocksSlow() why do we need it?
    // it's not changing anything anything except restoring usb if not balloon mode

    // was VERBY cleared while USB was off. no. so don't print here
    // V1_println(F("kazuClocksRestore START" EOL));
    // V1_flush();

    // clk_per, clk_rtc and clk_adc is the same as it was due to kazuClocksSlow()?
    // don't change. because we actually restore to a pll sys value
    // so we should not change these..assume go back just the same as it was

    // hmm we're not getting Serial2 when we use the Kazu 12 Mhz past here
    // just reinit the sys pll to PLL_SYS_MHZ?
    // PLL_SYS_MHZ = 12;
    // could reference for sdk api stuff for clocks
    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__clocks.html#gae78816cc6112538a12adcc604be4b344
    if (!ALLOW_KAZU_12MHZ_MODE) {
        busy_wait_ms(500);
        set_sys_clock_khz(freq_khz, true);
        PLL_SYS_MHZ = freq_khz / 1000UL;
    }
    busy_wait_ms(500);

    // FIX! we need usb at 48 mhz ?
    if (!BALLOON_MODE && ALLOW_USB_DISABLE_MODE) {
        // void pll_init(PLL pll, uint ref_div, uint vco_freq, uint post_div1, uint post_div2);
        // pll pll_sys or pll_usb
        // ref_div Input clock divider.
        // vco_freq Requested output from the VCO (voltage controlled oscillator)
        // post_div1 Post Divider 1 - range 1-7. Must be >= post_div2
        // post_div2 Post Divider 2 - range 1-7
        pll_init(pll_usb, 1, 1440000000, 6, 5);  // return USB pll to 48mhz
        busy_wait_ms(1500);
        // High-level Adafruit TinyUSB init code,
        // does many things to get USB back online
        tusb_init();
        Serial.begin(115200);
        busy_wait_ms(500);
        V1_print(F("Restored USB pll to 48Mhz, and did Serial.begin()" EOL));
    }

    V1_print(F("After long sleep,"));
    if (ALLOW_KAZU_12MHZ_MODE) {
        V1_printf(" left it at kazu 12Mhz? PLL_SYS_MHZ %lu" EOL, PLL_SYS_MHZ);
    } else {
        V1_printf(" Restored sys_clock_khz() and PLL_SYS_MHZ to %lu" EOL, PLL_SYS_MHZ);
    }


    // I guess printing should work now? (if not BALLOON_MODE)
    V1_println(F("kazuClocksRestore END" EOL));
    // Serial communication uses the same system clock as everything else.
    // Baud rate of the serial communication is derived from this main clock frequency.
    // What does it mean for Serial2 when we're running at 12Mhz?
}

//************************************************
// blurb on pll_usb -> clk_peri uart 48 Mhz (clk_peri) and i2c can be different
// https://github.com/raspberrypi/pico-sdk/issues/841
// rosc @ 1-12 MHz
//
// xosc @ 12 MHz
//     |
//     \-- clk_ref @ 12 MHz
//             |
//             \-- watchdog tick 1:12, @ 1 Mhz
//             |       |
//             |       \-- timer/alarm: get_absolute_time() in micro seconds
//             |
//             \-- pll_sys @ 125 MHz
//             |       |
//             |       \-- clk_sys @ 125 MHz
//             |
//             \-- pll_usb @ 48 MHz
//                   |
//                   \-- clk_peri @ 48 MHz, for UART but not I2C
//                   |       |
//                   |       \-- DMA pacing timers
//                   |
//                   \-- clk_usb
//                   |
//                   \-- clk_adc
//                   |
//                   \-- clk_rtc 1:1024 @ 46,875 Hz
//                             |
//                             \-- RTC 1:46875 @ 1Hz
