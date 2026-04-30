// FIX! SIM65M CB labelled parts won't let me change baud rate?
// SIM65M did?

// is this true:
// When the firmware supports GPS and GLONASS systems, the NMEA sentences output as below:
// If the receiver is fixed by GPS only, it will print GPRMC, GPVTG, GPGGA, GPGSA, GPGSV and GPGLL.
// If the receiver is fixed by GLONASS only, it will print GNRMC, GPVTG, GPGGA, GNGSA,
// GPGSV, GLGSV and GNGLL.

// If the receiver is fixed by GPS and GLONASS, it will print GNRMC, GPVTG, GPGGA, GNGSA,
// GPGSV, GLGSV and GNGLL.

// In the state of no satellite positioning, it will print initial state of NMEA, such as GPRMC, GPVTG,
// GPGGA, GPGSA, GPGSV and GPGLL. The time before satellite positioning after cold start,
// warm start or hot start is belong to this situation.

// When the firmware supports GPS and BeiDou systems, the NMEA sentences output as below:
// If the receiver is fixed by GPS only, it will print GPRMC, GPVTG, GPGGA, GPGSA, GPGSV and GPGLL.
// If the receiver is fixed by BeiDou only, it will print BDRMC, BDVTG, BDGGA, BDGSA, BDGSV and BDGLL.
// If the receiver is fixed by GPS and BeiDou, it will print GNRMC, GNVTG, GNGGA,GPGSA, BDGSA ,GPGSV, BDGSV and GNGLL.

// In the state of no satellite positioning, it will print initial state of NMEA, such as GNRMC, GNVTG, GNGGA and GNGLL. 
// The time before satellite positioning after cold start, warm start or hot start is belong to this situation.

// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// REMEMBER: no references to Serial.* or usb in BALLOON_MODE!
#include <Arduino.h>

// which gps chip is used?
// ATGM336N if false
extern bool USE_SIM65M;
// change it if we have the 5sec fix/broadcast on USE_SIM65M
extern uint32_t GPS_WAIT_FOR_NMEA_BURST_MAX;
extern uint32_t setTime_millis;  // last millis() when we setTime()
extern bool BALLOON_MODE;

// Don't reconfig if not necessary
// what if we lose config because vbat glitches?
// not worth the risk to avoid reconfig
bool HOT_RESET_REDO_CONFIG = true;
bool SIM65M_BROADCAST_5SECS = false;
bool ATGM336H_BROADCAST_5SECS = true;

#include "global_structs.h"
#include <stdlib.h>
extern ConfigStruct cc;
int CONSTELLATIONS_GROUP;

// clear these to zero if gps goes off (for PPS tracking)
extern int32_t PPS_rise_millis;
extern int32_t PPS_rise_micros;
extern int32_t PPS_rise_cnt;
extern bool PPS_rise_valid;

// gps+bds+glonass
// int CONSTELLATIONS_GROUP = 7;
// gps+bds
// int CONSTELLATIONS_GROUP = 3;
// gps
// int CONSTELLATIONS_GROUP = 1;

//*******************************************
// ATGM336H uses AT6558 silicon ??
// AT6558 BDS/GNSS Full Constellation SOC Chip Data Sheet Version 1.14
// AT6558-5N-3X is GPS + BDS
// QFN package 40 pin 5x5x0.8mm
// https://www.icofchina.com/d/file/xiazai/2016-12-05/b1be6f481cdf9d773b963ab30a2d11d8.pdf

// says VDD_POR going low causes internal reset (nRESET)
// nRST pin going low causes internal reset (nRESET)
// nRST can be low while power is transition on,
// or it can be asserted/deasserted afer power on
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
// (1/baud) = 104 usecs @ 9600 baud..
// but: effective baud rate from gps chip is less than peak
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

// for isprint()
#include <ctype.h>
#include "gps_functions.h"
#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"
#include "time_functions.h"
#include "pps_functions.h"
#include "slow_clock_functions.h"

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
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

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

//************************************************
// false and true work here
bool PWM_GPS_POWER_ON_MODE = true;
bool ALLOW_UPDATE_GPS_FLASH_MODE = false;
// 7/10/25
// bool ALLOW_UPDATE_GPS_FLASH_MODE = true;

// does this close putty if true?
extern bool ALLOW_USB_DISABLE_MODE;
extern bool ALLOW_KAZU_12MHZ_MODE;
extern bool ALLOW_TEMP_12MHZ_MODE;
// causing intermittent fails if true?
extern bool ALLOW_LOWER_CORE_VOLTAGE_MODE;

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
// =============================================================================
// nmeaBufferFastPoll
// -----------------------------------------------------------------------------
// Tries to get all data from the GPS without losing any over a blocking
// window of `duration_millis`. Loops as fast as possible, draining all
// available characters into a RAM buffer each pass.
//
// Filters out spaces and null characters and anything non-printable (CR/LF
// included -- those are unprintable). We add appropriate EOLs when printing
// the buffer, so dropping CR/LF from the captured stream is intentional and
// saves buffer room.
//
// FIX! will there be enough garbage visible when baud rate is wrong that
// we'll still see bad baud rate issues?
// =============================================================================

// -----------------------------------------------------------------------------
// True if `c` is a character we want to capture into the buffer.
// We drop spaces, NULs, and anything non-printable.
// -----------------------------------------------------------------------------
static bool shouldCaptureChar(char c) {
    if (c == '\0') return false;
    if (c == ' ')  return false;
    return isprint((unsigned char)c) != 0;
}

// -----------------------------------------------------------------------------
// Drain all characters currently available from the GPS UART into the buffer,
// filtering on the way. Uses the existing globals: incomingChar, charsAvailable.
// -----------------------------------------------------------------------------
static void drainAvailableCharsIntoBuffer(bool printIfFull) {
    // set globals: incomingChar, charsAvailable
    getChar();
    while (charsAvailable) {
        if (shouldCaptureChar(incomingChar)) {
            nmeaBufferAndPrint(incomingChar, printIfFull);
        }
        getChar();
    }
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void nmeaBufferFastPoll(uint32_t duration_millis, bool printIfFull) {
    V1_println(F(EOL "nmeaBufferFastPoll START"));

    // nmeaBuffer should be empty the first time we use this; should be no
    // harm (delay) in checking here.
    nmeaBufferPrintAndClear();

    uint32_t start_millis = millis();
    while (millis() - start_millis < duration_millis) {
        drainAvailableCharsIntoBuffer(printIfFull);
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

// =============================================================================
// gpsSleepForMillis
// -----------------------------------------------------------------------------
// Sleep approximately `n` milliseconds in 10ms increments. While sleeping:
//   - Kicks the watchdog and updates the status LED every 100ms
//     (every 10 ticks of 10ms each), regardless of total duration.
//   - Optionally exits early if Serial2 has data available (a GPS character
//     arrived).
//
// `n` must be in [0, 120000]. Out-of-range values are silently clamped/ignored
// because this function is called while USB is disabled, and BALLOON_MODE /
// VERBY don't protect us from accidental prints in that state -- so we just
// don't print at all here.
// =============================================================================

#define GPS_SLEEP_MAX_MILLIS         120000
#define GPS_SLEEP_TICK_MILLIS        10
#define GPS_SLEEP_TICKS_PER_SERVICE  10  // service LED/watchdog every 10 ticks (~100ms)

void gpsSleepForMillis(int n, bool enableEarlyOut) {
    // FIX! should we do this here or where?
    Watchdog.reset();

    if (n < 0 || n > GPS_SLEEP_MAX_MILLIS) {
        // V1_printf("ERROR: gpsSleepForMillis() n %d too big (120000 max)" EOL, n);
        // n = 1000;
        // UPDATE: this is used while USB is disabled, but BALLOON_MODE/VERBY
        // don't protect us ..just don't print here.
    }

    // Number of 10ms ticks we need.
    int tickCount = n / GPS_SLEEP_TICK_MILLIS;

    for (int i = 0; i < tickCount; i++) {
        // Early-out on incoming GPS data, if the caller asked for it.
        if (enableEarlyOut && Serial2.available()) break;

        // every 100ms kick watchdog and update the LED.
        // No prints here -- USB may be disabled.
        // https://docs.arduino.cc/language-reference/en/functions/time/delay/
        if ((i % GPS_SLEEP_TICKS_PER_SERVICE) == 0) {
            updateStatusLED();
            Watchdog.reset();
        }

        // faster recovery with delay?
        busy_wait_ms(GPS_SLEEP_TICK_MILLIS);
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

// =============================================================================
// getInitialGpsOutput
// -----------------------------------------------------------------------------
// After a hot or cold GPS reset, read whatever the chip emits for up to 5
// seconds, looking for proof-of-life. Returns true if we saw at least 2
// NMEA sentence starts ('$').
//
// There can be a lot of bogus chars after hot/cold reset (over 200). If we
// can get effective 900 chars/sec, we probably want 5x that as our cap --
// hence the 5000-char early-out.
//
// Three early-exit conditions:
//   1. Saw 2+ NMEA sentence starts ('$') -> success
//   2. Read 5000+ characters             -> bail (probably noise)
//   3. 5 seconds elapsed                 -> bail (timeout)
// =============================================================================

#define GPS_INITIAL_OUTPUT_TIMEOUT_MS  5000
#define GPS_INITIAL_OUTPUT_MAX_CHARS   5000
#define GPS_INITIAL_OUTPUT_MIN_NMEA    2

// -----------------------------------------------------------------------------
// Drain everything currently waiting on Serial2 into the NMEA buffer.
// Increments *charCount and *sentenceCount as we go. Skips non-printable
// characters so the captured log is clean (otherwise dos2unix on putty.log
// would choke on stray binary bytes).
//
// Returns true once we hit one of the early-exit thresholds, signalling
// the outer loop to break.
// -----------------------------------------------------------------------------
static bool drainSerial2AndCount(uint32_t *charCount, uint32_t *sentenceCount) {
    while (Serial2.available()) {
        char incomingChar = Serial2.read();
        // skip any non-printable, as we won't be able to dos2unix the
        // putty.log if those are in there
        if (!isprint((unsigned char)incomingChar)) continue;
        // buffer it up like we do normally below, so we can see sentences
        nmeaBufferAndPrint(incomingChar, true);  // print if full
        *charCount += 1;
        if (incomingChar == '$') *sentenceCount += 1;
    }
    if (*charCount >= GPS_INITIAL_OUTPUT_MAX_CHARS) return true;
    if (*sentenceCount >= GPS_INITIAL_OUTPUT_MIN_NMEA) return true;
    return false;
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
bool getInitialGpsOutput(void) {
    V1_println(F("getInitialGpsOutput START"));
    V1_println(F("Look for some Serial2 bytes for 5 secs or 5000 chars or 2 sentences"));
    uint32_t incomingCharCnt     = 0;
    uint32_t incomingSentenceCnt = 0;
    uint32_t start_millis = millis();
    while ((millis() - start_millis) < GPS_INITIAL_OUTPUT_TIMEOUT_MS) {
        Watchdog.reset();
        // Drain whatever's waiting. If we hit an early-exit threshold
        // (enough chars or enough sentences), break out.
        if (Serial2.available()) {
            if (drainSerial2AndCount(&incomingCharCnt, &incomingSentenceCnt)) break;
        }
        // Sleep up to 1 sec, returning early if Serial2 has data again.
        gpsSleepForMillis(1000, true);
    }
    nmeaBufferPrintAndClear();
    updateStatusLED();
    Watchdog.reset();
    V1_println(F("getInitialGpsOutput END"));
    return (incomingSentenceCnt >= GPS_INITIAL_OUTPUT_MIN_NMEA);
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
    // Should not worry about setting balloon mode (3) for ATGM336?
    // doesn't seem like ATGM336 has a balloon mode in the CASIC specifiction pdf

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
    // 2/16/25 faster
    nmeaBufferFastPoll(1000, true);  // duration_millis, printIfFull

    //*****************
    if (false) {
        // PAIR_GET_SETTING
        Serial2.print("$PAIR021*39" CR LF);
        // 2/16/25 faster
        nmeaBufferFastPoll(1000, true);  // duration_millis, printIfFull
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
    nmeaBufferFastPoll(2000, true);  // duration_millis, printIfFull
    // 2/16/25 faster
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
// always GGA GSA GSV RMC
// never ZDA TXT
// never VTG GLL?
// no GST (ATM336H
void setGpsBroadcast(void) {
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
        // Fix_Interval--Position fix interval in milliseconds (ms).
        // [Range: 100 ~ 1000]
        // [Example]
        // $PAIR050,1000*12

        //****************
        // Packet Type:062 PAIR_COMMON_SET_NMEA_OUTPUT_RATE
        // Set the NMEA sentence output interval of corresponding NMEA type
        // $PAIR062,<Type>,<Output_Rate>*<checksum>
        // -1 Reset all sentence to default value
        // 0 NMEA_SEN_GGA  // GGA interval - GPS Fix Data
        // 1 NMEA_SEN_GLL, // GLL interval - Geographic Position - Latitude longitude
        // 2 NMEA_SEN_GSA, // GSA interval - GNSS DOPS and Active Satellites
        // 3 NMEA_SEN_GSV, // GSV interval - GNSS Satellites in View
        // 4 NMEA_SEN_RMC, // RMC interval - Minimum Specific GNSS Sentence
        // 5 NMEA_SEN_VTG, // VTG interval - Course Over Ground and Ground Speed
        // 6 NMEA_SEN_ZDA, // ZDA interval - Time & Date

        // Output interval: default 1?
        // 1 per 5 position fixes?
        // is the position fix rate 1 per sec to 10 per sec?
        // see elsewhere. they might have bugs if fix rate is reduced
        // 0 - Disabled or not supported sentence
        // 1 - Output once every one position fix
        // 2 - Output once every two position fixes
        // 3 - Output once every three position fixes
        // 4 - Output once every four position fixes
        // 5 - Output once every five position fixes

        // enable this, because we're disabling broadcast in default config now
        // for SIM65M. Assumes the default fix rate is 1000ms (1 per sec?)

        // why are we getting repeated GNGSA? disable it (just used for custom hdop/vdop/pdop)
        // $GNGSA,A,3,28,32,31,02,10,01,,,,,,,1.56,0.69,1.40,1*02
        // $GNGSA,A,3,67,82,66,76,83,81,,,,,,,1.56,0.69,1.40,2*02
        // $GNGSA,A,3,29,33,,,,,,,,,,,1.56,0.69,1.40,3*00
        // $GNGSA,A,3,19,20,35,29,,,,,,,,,1.56,0.69,1.40,4*0B

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

        // is typical NMEA talker order this? GGA GLL GSA GSV RMC VTG ZDA)
        // wait.we're getting GSA?

        // all the same size?
        static const char *pair062_1sec[] = {
            "$PAIR062,0,1*3F" CR LF,  // GGA on
            "$PAIR062,1,0*3F" CR LF,  // GLL off (2/17/25)
            "$PAIR062,2,0*3C" CR LF,  // GSA off (2/24/25)
            "$PAIR062,3,1*3C" CR LF,  // GSV on
            "$PAIR062,4,1*3B" CR LF,  // RMC on
            "$PAIR062,5,0*3B" CR LF,  // VTG off (2/17/25)
            "$PAIR062,6,0*38" CR LF,  // ZDA off (keep burst <= 1 sec)
        };

        // all the same size?
        static const char *pair062_5sec[] = {
            "$PAIR062,0,5*3B" CR LF,  // GGA on
            "$PAIR062,1,0*3F" CR LF,  // GLL off (2/17/25)
            "$PAIR062,2,0*3C" CR LF,  // GSA off (2/24/25)
            "$PAIR062,3,5*38" CR LF,  // GSV on
            "$PAIR062,4,5*3F" CR LF,  // RMC on
            "$PAIR062,5,0*3B" CR LF,  // VTG off (2/17/25)
            "$PAIR062,6,0*38" CR LF,  // ZDA off (keep burst <= 1 sec)
        };

        // the fix rate can't be slower than 1 sec?
        // 1 Hz (1000 ms): $PCAS02,1000*2E<CRLF>
    
        // don't really need the strncpy
        // strncpy(nmeaSentence, pair062_1sec[i], 62);

        const char **cmds = SIM65M_BROADCAST_5SECS ? pair062_5sec : pair062_1sec;
        size_t n = SIM65M_BROADCAST_5SECS
            ? sizeof(pair062_5sec)/sizeof(*pair062_5sec)
            : sizeof(pair062_1sec)/sizeof(*pair062_1sec);

        for (size_t i = 0; i < n; i++) {
            Serial2.print(cmds[i]);
            busy_wait_us(500);
        }

        // 4/26/26 try enabling NAV-STATUS binary output
        // byte message[] = {0xBA,0xCE,0x04,0x00,0x06,0x01,0x01,0x01,0x01,0x00,0x05,0x01,0x07,0x01,0x0D,0x0A};
        // enables DOP messages?
        // BA CE 04 00 06 01 01 01 01 00 05 01 07 01 0D 0A
        // Serial2.write(message, sizeof(message));
        // busy_wait_us(500);

        Serial2.flush();
        busy_wait_us(2000);
        // was 1200 2/16/2025
        // don't make too big. best if eventually GGA is always first in burst
        GPS_WAIT_FOR_NMEA_BURST_MAX = SIM65M_BROADCAST_5SECS ? 5200 : 1200;

    } else {
        //*************************************************
        // ATGM336H
        // ZDA. does this exist for ATGM336H? disabled?
        // this time info is in other sentences also?
        // $–ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx
        // hhmmss.ss = UTC
        // xx = Day, 01 to 31
        // xx = Month, 01 to 12
        // xxxx = Year // xx = Local zone description, 00 to +/- 13 hours
        // xx = Local zone minutes description (same sign as hours)

        //*************************************************
        // from the latest CASIC_ProtocolSpecification_english.pdf
        // Field 1 is the PCAS03

        // 2  nGGA output frequency,
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
        // 21 <CR><LF> characters Carriage return and line feed

        // spec has more/new detail. see below
        // strncpy(nmeaSentence, "$PCAS03,1,1,1,1,1,1,0,0,0,0,,,1,1,,,,1*33" CR LF, 62);

        // 2/17/25 no VTG or GLL
        // 2/17/25 disable UTC GST, TIM (wasn't ever getting UTC, TIM?)
        // 2/24/25 no GSA
        // strncpy(nmeaSentence, "$PCAS03,1,0,0,1,1,0,0,0,0,0,,,0,0,,,,0*33" CR LF, 62);
        // 4/25/26 reenable GSA to see 2d vs 3d fix
        const char *cmd = ATGM336H_BROADCAST_5SECS
            ? "$PCAS03,5,0,5,5,5,0,0,0,0,0,,,0,0,,,,0*32" CR LF
            : "$PCAS03,1,0,1,1,1,0,0,0,0,0,,,0,0,,,,0*32" CR LF;
        strncpy(nmeaSentence, cmd, 62);
        Serial2.print(nmeaSentence);
        Serial2.flush();
        busy_wait_us(2000);
        GPS_WAIT_FOR_NMEA_BURST_MAX = ATGM336H_BROADCAST_5SECS ? 5200 : 1200;
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
            // have to disable each NMEA sentence type individually?
            static const char *pair062_all_off[] = {
                "$PAIR062,0,0*3E" CR LF,
                "$PAIR062,1,0*3F" CR LF,
                "$PAIR062,2,0*3C" CR LF,
                "$PAIR062,3,0*3D" CR LF,
                "$PAIR062,4,0*3A" CR LF,
                "$PAIR062,5,0*3B" CR LF,
                "$PAIR062,6,0*38" CR LF,
            };

            for (size_t i = 0; i < sizeof(pair062_all_off)/sizeof(*pair062_all_off); i++) {
                Serial2.print(pair062_all_off[i]);
                busy_wait_us(500);
            }
            busy_wait_us(1500);

    } else {
        // checksum from https://www.meme.au/nmea-checksum.html
        strncpy(nmeaSentence, "$PCAS03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*02" CR LF, 64);
        Serial2.print(nmeaSentence);
        Serial2.flush();
        busy_wait_us(2000);
    }
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
// In some software versions, maximum number of satellites reported as visible is limited to 12, 
// even though more may be visible
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
// =============================================================================
// setGpsConstellations
// -----------------------------------------------------------------------------
// Tells the GPS module which GNSS constellations to search for. The encoding
// is a small integer:
//
//   1 = GPS                 5 = GPS + GLONASS
//   2 = BDS                 6 = BDS + GLONASS
//   3 = GPS + BDS           7 = GPS + BDS + GLONASS
//   4 = GLONASS             default (anything else) = GPS + BDS (i.e. 3)
//
// Future ideas (kept from original):
//   case 0: ; // FIX! should I make 0 disable everything?
//   case 8: ; // FIX! should I make 8 enable everything?
//
// SIM65M reference:
//   Packet Type 066 -- PAIR_COMMON_SET_GNSS_SEARCH_MODE
//   Configure the receiver to start searching for satellites.
//   The setting is available when the NVRAM data is valid.
//   The device restarts when it receives this command.
//   Abbreviations: GPS "G", GLONASS "R", Galileo "E", BeiDou "B", NavIC "I".
//
//   Field meaning (1 = enable, 0 = disable):
//     PAIR066, GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC
//
//   L1 single-frequency supports 5 documented modes G / GR / GE / GB / GREB:
//     PAIR066,1,0,0,0,0,0   GPS only
//     PAIR066,1,1,0,0,0,0   GPS+GLONASS
//     PAIR066,1,0,1,0,0,0   GPS+GALILEO
//     PAIR066,1,0,0,1,0,0   GPS+BEIDOU
//     PAIR066,1,1,1,1,0,0   GPS+GLONASS+GALILEO+BEIDOU
//     PAIR066,1,1,0,1,0,0   GPS+GLONASS+BEIDOU
//   QZSS is always switchable.
//
// FIX! we'll have to figure this out for SIM65M
// FIX! should we ignore desiredConstellations and force 3 (BDS + GPS)?
// =============================================================================

// -----------------------------------------------------------------------------
// SIM65M sentence lookup. Returns a static string (not heap-allocated).
// Galileo is field 3. The last two fields are QZSS and NavIC.
// (Are some of these wrong, and is Galileo set in the default?)
// On unknown input, falls back to GPS+BDS and updates *usedConstellations.
// -----------------------------------------------------------------------------
static const char *sim65mConstellationSentence(int *usedConstellations) {
    switch (*usedConstellations) {
        case 1: return "$PAIR066,1,0,0,0,0,0*3B" CR LF;  // GPS
        case 2: return "$PAIR066,0,0,0,1,0,0*3B" CR LF;  // BDS
        case 3: return "$PAIR066,1,0,0,1,0,0*3A" CR LF;  // GPS+BDS
        case 4: return "$PAIR066,0,1,0,0,0,0*3B" CR LF;  // GLONASS
        case 5: return "$PAIR066,1,1,0,0,0,0*3A" CR LF;  // GPS+GLONASS
        case 6: return "$PAIR066,0,1,0,1,0,0*3A" CR LF;  // BDS+GLONASS
        case 7: return "$PAIR066,1,1,0,1,0,0*3B" CR LF;  // GPS+BDS+GLONASS
        default:
            *usedConstellations = 3;
            return "$PAIR066,1,0,0,1,0,0*3A" CR LF;      // GPS+BDS
    }
}

// -----------------------------------------------------------------------------
// PCAS (non-SIM65M) sentence lookup.
// Note: case 0 isn't defined in the CASIC_ProtocolSpecification.pdf.
// On unknown input, falls back to GPS+BDS and updates *usedConstellations.
// -----------------------------------------------------------------------------
static const char *pcasConstellationSentence(int *usedConstellations) {
    switch (*usedConstellations) {
        case 1: return "$PCAS04,1*18" CR LF;   // GPS
        case 2: return "$PCAS04,2*1B" CR LF;   // BDS
        case 3: return "$PCAS04,3*1A" CR LF;   // GPS+BDS
        case 4: return "$PCAS04,4*1D" CR LF;   // GLONASS
        case 5: return "$PCAS04,5*1C" CR LF;   // GPS+GLONASS
        case 6: return "$PCAS04,6*AF" CR LF;   // BDS+GLONASS
        case 7: return "$PCAS04,7*1E" CR LF;   // GPS+BDS+GLONASS
        default:
            *usedConstellations = 3;
            // Note: original default used checksum *1D rather than *1A here. (error)
            return "$PCAS04,3*1A" CR LF;       // GPS+BDS
    }
}

void setGpsConstellations(int desiredConstellations) {
    V1_printf("setConstellations START %d" EOL, desiredConstellations);
    updateStatusLED();
    Watchdog.reset();

    int usedConstellations = desiredConstellations;
    char nmeaSentence[62] = { 0 };

    // Pick the chip-specific sentence. The helper may reset
    // usedConstellations to 3 (GPS+BDS) on unsupported input.
    const char *src = USE_SIM65M
        ? sim65mConstellationSentence(&usedConstellations)
        : pcasConstellationSentence(&usedConstellations);
    strncpy(nmeaSentence, src, sizeof(nmeaSentence));

    // Send to the GPS.
    // Huh -- on SIM65M this PAIR066 command also causes a GPS reset
    // (presumably a hot reset).
    Serial2.print(nmeaSentence);
    Serial2.flush();
    busy_wait_ms(1000);

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

// =============================================================================
// setGpsBaud
// -----------------------------------------------------------------------------
// Tells the GPS module to switch to `desiredBaud`, then reconfigures Serial2
// on our side to match.
//
// Assumes we can already talk to the GPS at some agreed-on Serial2 baud
// (set up by init / hot reset / full cold reset). After power-on the GPS
// starts at 9600 baud.
//
// NMEA notes:
//   - Sentences must be terminated with CR + LF.
//   - CR/LF are defined in print_functions.h. They are not part of the
//     checksum, nor is the leading '$'.
//   - Example: $PMTK251,38400*27<CR><LF>
//   - Checksums are pre-calculated and hardwired into the static sentences
//     used here (see https://www.meme.au/nmea-checksum.html). After
//     checkGpsBaudRate() we should only ever be looking up legal bauds.
// =============================================================================

// Set to false to use the older $PAIR860 sentences instead of $PAIR864.
// PAIR864 is preferred -- strange that the spec doesn't list all baud rate
// values for PAIR860 but does for PAIR864. 
// PAIR860 and PAIR864 work for SIM65M 9600 baud 4/30/26
#define SIM65M_USE_PAIR864 false

// -----------------------------------------------------------------------------
// Pick the SIM65M baud-change NMEA sentence for a given baud.
// Falls back to 9600 for any unsupported baud (and updates *usedBaud).
// -----------------------------------------------------------------------------
static void buildSIM65MBaudSentence(int *usedBaud, char *out, size_t outSize) {
    // $PAIR860,0,0,37,9600,0*23 means:
    // Open UART0 to NMEA output without flow control.  Baudrate is 9600.
    // PAIR864 lists alternate baudrates only but says min is 115200?

    // Yes! 9600 works after boot with 115200! (no buffer overflow)
    // Did this stop working?
    // This worked if default 115200 originally..for SIM65M module. (not SIM65M-CB?)
    const char *s;
    switch (*usedBaud) {
        // they say the SIM65M-C uses $PAIR864 for setting baud
        // but 115200 minimum?
        // 2.3.147 Packet Type:860 PAIR_IO_OPEN_PORT Open a GNSS data port
        // PAIR860 checksum was wrong! try again 4/30/26
        case 4800:   // supported or ??
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,4800*10"   CR LF
                                   : "$PAIR860,0,0,37,4800,0*20" CR LF; break;
        case 9600:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,9600*13"   CR LF
                                   : "$PAIR860,0,0,37,9600,0*23" CR LF; break;
        case 19200:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,19200*26"  CR LF
                                   : "$PAIR860,0,0,37,19200,0*16" CR LF; break;
        case 38400:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,38400*23"  CR LF
                                   : "$PAIR860,0,0,37,38400,0*13" CR LF; break;
        case 57600:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,57600*28"  CR LF
                                   : "$PAIR860,0,0,37,57600,0*18" CR LF; break;
        case 115200:
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,115200*1B" CR LF
                                   : "$PAIR860,0,0,37,115200,0*2B" CR LF; break;
        default:
            *usedBaud = 9600;
            s = SIM65M_USE_PAIR864 ? "$PAIR864,0,0,9600*13"   CR LF
                                   : "$PAIR860,0,0,37,9600,0*23" CR LF; break;
    }
    strncpy(out, s, outSize);
}

// -----------------------------------------------------------------------------
// Pick the non-SIM65M (PCAS / PMTK) baud-change NMEA sentence for a given baud.
// Falls back to 9600 for any unsupported baud (and updates *usedBaud).
//
// History notes (kept from original code):
//   - 9600  $PCAS01,1   didn't work / now it worked? seems OK if chip is in 9600 state.
//   - 9600  $PMTK251    worked once but then broke; needed Arduino restart.
//   - 19200 $PCAS01,2   worked.   $PMTK251,19200 didn't work.
//   - 38400 $PCAS01,3   worked.   $PMTK251,38400 didn't work.
//   - 57600 $PCAS01,4   didn't work. $PMTK251,57600 untested.
//   - 115200 $PCAS01,5  prints, but lots of Rx buffer-overrun ERRORs.
// -----------------------------------------------------------------------------
static void buildPCASBaudSentence(int *usedBaud, char *out, size_t outSize) {
    const char *s;
    switch (*usedBaud) {
        case 4800:   s = "$PCAS01,0*1C" CR LF; break;
        case 9600:   s = "$PCAS01,1*1D" CR LF; break;
        case 19200:  s = "$PCAS01,2*1E" CR LF; break;
        case 38400:  s = "$PCAS01,3*1F" CR LF; break;
        case 57600:  s = "$PCAS01,4*18" CR LF; break;
        case 115200: s = "$PCAS01,5*19" CR LF; break;
        default:
            *usedBaud = 9600;
            // (legacy alternative) "$PMTK251,9600*17" CR LF
            s = "$PCAS01,1*1D" CR LF;
            break;
    }
    // Original used a hard length of 21 here
    strncpy(out, s, outSize);
}

// -----------------------------------------------------------------------------
// Send an NMEA sentence over Serial2 and wait long enough for the GPS to
// finish processing it.
// https://forum.arduino.cc/t/solved-proper-way-to-change-baud-rate-after-initial-setup/419860/5
// -----------------------------------------------------------------------------
static void sendNmeaAndWait(const char *sentence) {
    Serial2.print(sentence);
    Serial2.flush();
    // have to wait for the sentence to get out and complete at the GPS
    busy_wait_ms(1000);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void setGpsBaud(int desiredBaud) {
    V1_printf("setGpsBaud START %d" EOL, desiredBaud);
    updateStatusLED();
    Watchdog.reset();

    // Clamp desiredBaud to a baud the chip actually supports.
    int usedBaud = checkGpsBaudRate(desiredBaud);

    // Build the chip-specific baud-change sentence. usedBaud may be reset
    // to 9600 inside the helper if the requested rate is unsupported.
    char nmeaBaudSentence[64] = { 0 };
    if (USE_SIM65M) {
        buildSIM65MBaudSentence(&usedBaud, nmeaBaudSentence, sizeof(nmeaBaudSentence));
    } else {
        buildPCASBaudSentence(&usedBaud, nmeaBaudSentence, sizeof(nmeaBaudSentence));
    }

    // Drain anything still sitting in the CPU output buffer before we send.
    Serial2.flush();
    busy_wait_ms(1000);

    sendNmeaAndWait(nmeaBaudSentence);
    V1_printf("setGpsBaud for usedBaud %d, sent %s" EOL, usedBaud, nmeaBaudSentence);

    // 7/10/25 spec says to reboot after baud rate change (SIM65M only).
    if (USE_SIM65M) {
        strncpy(nmeaBaudSentence, "$PAIR023*3B" CR LF, sizeof(nmeaBaudSentence));
        sendNmeaAndWait(nmeaBaudSentence);
        V1_printf("setGpsBaud reboot. gps sent %s" EOL, nmeaBaudSentence);
    }

    // Serial2.end() Disables serial communication, allowing the RX and TX
    // pins to be used for general input and output.
    // To re-enable serial communication, call Serial2.begin().

    // FIX! we could try RP2040 using different bauds and see what baud rate
    // it's at, then send the command to change baud rate.
    // But really, how come we can't cold reset it to 9600?
    // When it's in a not-9600 state it's like vbat keeps the old baud rate
    // on reset and/or power cycle?? makes it dangerous to use anything
    // other than 9600 baud.
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);

    // If the GPS-side command didn't actually change baud, you'd need to
    // change this Serial2.begin() to match what the GPS is really using.
    Serial2.begin(usedBaud);
    V1_printf("setGpsBaud did Serial2.begin(%d)" EOL, usedBaud);
    gpsSleepForMillis(1000, false);
    V1_printf("setGpsBaud END %d" EOL, usedBaud);
}

// =============================================================================
// GpsINIT
// -----------------------------------------------------------------------------
// One-time GPS initialization:
//   1. Configure the GPIOs that control GPS power, reset, and on/off.
//   2. Drive them to known states (power off, NRESET deasserted, ON low).
//   3. Bring up Serial2 at the chip's expected default baud.
//   4. Run a full cold reset, which also sets target baud and balloon mode.
//
// The GPS power pin is floating at boot? Likely GPS is off but unknown.
// Should turn GPS off before doing init if you know it's been initted already.
//
// Equations and schematic for the gate resistor on the mosfet:
//   https://electronics.stackexchange.com/questions/666204/gate-resistors-on-the-mosfet
//
// HEY! can't I control the source impedance of RP2040 GPIO outputs?
// What if I change the GPIO slew rate controls? I could use a 2MA output
// drive and a slow slew rate.
//   https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
//
// GpsPwr is the mosfet control GPIO for the GPS power control.
// A03401A is a p-channel mosfet:
//   https://www.aosmd.com/sites/default/files/res/datasheets/AO3401A.pdf
//
// Doug's inrush-current-limiting design (affecting gate resistance with
// the mosfet):
//   https://docs.google.com/document/d/1b1TdheBbXtl7U7HTO9mjVkThD9QHASqJirI2kSnv4LE/edit?tab=t.0#heading=h.lcp1i04bgm4q
//
// pico-sdk gpio API reference:
//   https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_gpio/include/hardware/gpio.h
// On the RP2040, setting both pulls enables a "bus keep" function, i.e. a
// weak pull to whatever the current high/low state of the GPIO is.
//   void gpio_set_pulls(uint gpio, bool up, bool down);
//   static inline void gpio_pull_down(uint gpio)    { gpio_set_pulls(gpio, false, true);  }
//   static inline void gpio_disable_pulls(uint gpio){ gpio_set_pulls(gpio, false, false); }
//
// hmm.. the weak pullup will change the slew rate down also?
// =============================================================================

// -----------------------------------------------------------------------------
// Configure the GpsPwr (mosfet gate) pin with low drive / slow slew so we
// limit inrush current when powering up the GPS.
// gpio_init defaults to 8mA drive strength; we override it below.
// -----------------------------------------------------------------------------
static void initGpsPwrPin(void) {
    gpio_init(GpsPwr);  // defaults to 8mA drive strength
    pinMode(GpsPwr, OUTPUT);
    // this will be undone by the next thing that uses gpio_set_pulls()
    gpio_pull_up(GpsPwr);
    gpio_put(GpsPwr, HIGH);  // deassert (power off)
    // if you set them to input after this, they will be high impedance!
    gpio_set_slew_rate(GpsPwr, GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(GpsPwr, GPIO_DRIVE_STRENGTH_2MA);
}

// -----------------------------------------------------------------------------
// Configure GPS_NRESET_PIN as an output with a pull-up. Caller drives the
// initial level explicitly later.
// -----------------------------------------------------------------------------
static void initGpsResetPin(void) {
    gpio_init(GPS_NRESET_PIN);
    pinMode(GPS_NRESET_PIN, OUTPUT);
    gpio_pull_up(GPS_NRESET_PIN);
    // gpio_put(GPS_NRESET_PIN, HIGH); // deassert
}

// -----------------------------------------------------------------------------
// Configure GPS_ON_PIN as an output with a pull-down. Caller drives the
// initial level explicitly later.
// FIX! should toggle this for low power operation, instead of powering
// GpsPwr on/off (even if VBAT gives a hot fix).
// -----------------------------------------------------------------------------
static void initGpsOnPin(void) {
    gpio_init(GPS_ON_PIN);
    pinMode(GPS_ON_PIN, OUTPUT);
    gpio_pull_down(GPS_ON_PIN);
    // gpio_put(GPS_ON_PIN, LOW); // deassert
}

// -----------------------------------------------------------------------------
// Drive the three GPS control pins to their known initial states and log it.
// Updated: Do a full reset since VBAT may have kept old settings -- don't
// know if that includes baud rate, maybe?
// -----------------------------------------------------------------------------
static void driveGpsPinsToInitialState(void) {
    digitalWrite(GpsPwr, HIGH);
    V1_printf("set GpsPwr %d HIGH (power off)" EOL, GpsPwr);

    digitalWrite(GPS_NRESET_PIN, HIGH);
    V1_printf("set GPS_NRESET_PIN %d HIGH" EOL, GPS_NRESET_PIN);

    digitalWrite(GPS_ON_PIN, LOW);
    V1_printf("set GPS_ON_PIN %d LOW" EOL, GPS_ON_PIN);
}

// -----------------------------------------------------------------------------
// Print which pins we're using for GPS UART and control.
// -----------------------------------------------------------------------------
static void printGpsPinAssignments(void) {
    V1_printf("GPS_UART1_RX_PIN %d" EOL, GPS_UART1_RX_PIN);
    V1_printf("GPS_UART1_TX_PIN %d" EOL, GPS_UART1_TX_PIN);
    V1_printf("GpsPwr %d" EOL, GpsPwr);
    V1_printf("GPS_NRESET_PIN %d" EOL, GPS_NRESET_PIN);
    V1_printf("GPS_ON_PIN %d" EOL, GPS_ON_PIN);
}

// -----------------------------------------------------------------------------
// Bring Serial2 up at the GPS chip's expected boot-time default baud.
// SIM65M defaults to 115200; ATGM (etc.) default to 9600.
//
// For ATGM, we'll first talk at 9600 -- but GpsFullColdReset() will do
// that anyway, so this is a bit redundant.
// -----------------------------------------------------------------------------
static void beginSerial2AtDefaultBaud(void) {
    Serial2.setPollingMode(true);
    // Tried making bigger...seems like 32 is the reality though.
    // Serial2.setFIFOSize(SERIAL2_FIFO_SIZE);
    Serial2.flush();
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);

    if (USE_SIM65M) {
        // default uart baud rate for SIM65
        Serial2.begin(115200);
    } else {
        Serial2.begin(9600);
    }
    gpsSleepForMillis(2000, false);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void GpsINIT(void) {
    V1_println(F(EOL "GpsINIT START"));
    updateStatusLED();
    Watchdog.reset();
    CONSTELLATIONS_GROUP = atoi(cc._const_group);
    // -------------------------------------------------------------------------
    // GPIO setup -- power, reset, on/off
    // -------------------------------------------------------------------------
    initGpsPwrPin();
    initGpsResetPin();
    initGpsOnPin();
    // Drive pins to known starting state (power off, reset deasserted, on=LOW).
    driveGpsPinsToInitialState();
    // -------------------------------------------------------------------------
    // UART setup
    // -------------------------------------------------------------------------
    printGpsPinAssignments();
    // FIX! is it okay that RX is powered on while gps chip is powered off?
    Serial2.setRX(GPS_UART1_RX_PIN);
    Serial2.setTX(GPS_UART1_TX_PIN);
    beginSerial2AtDefaultBaud();
    // -------------------------------------------------------------------------
    // Full cold reset: also sets baud to the target rate and applies
    // setGpsBalloonMode. After this returns, the GPS is powered up.
    //
    // FIX! hmm will sim65 reset to 9600?
    // Will it stay at the new baud rate thru hot reset and cold reset
    // like ATGM366N (weird), or will it default to 115200 again?
    // -------------------------------------------------------------------------
    GpsFullColdReset();
    // Drain the rx buffer.
    // while (Serial2.available()) Serial2.read();
    gpsSleepForMillis(2000, false);
    V1_println(F("GpsINIT END" EOL));
}
// =============================================================================
// pwmGpsPwrOn
// -----------------------------------------------------------------------------
// Soft power-on for GpsPwr (active-low: drives a p-mosfet that gates GPS power).
//
// The point of this routine is to ramp the GPS rail up gradually rather than
// slamming the mosfet fully on, which keeps inrush current low.
//
// Note: vbat doesn't go through the mosfet, so vbat will be high as soon as
// power is available regardless of what we do here.
//
// Two strategies were tried; only the second is used in production.
// =============================================================================

// Strategy A (DISABLED -- doesn't work):
// Try to use the RP2040's internal pull-down (50-80 kohm) as a weak driver
// against the 10k external pull-up on the PCB. The external pull-up wins,
// so the rail never actually gets pulled low this way.
#define WEAK_PULLDOWN_FOR_ASSERT false

// Strategy B parameters: PWM the mosfet, ramping duty cycle up over ~2 sec.
// Starts at 1us on / 200us off and walks toward 200us on / 0us off in 20us
// steps, ~10 iterations, ~200 usec per iteration.
#define PWM_INITIAL_ON_USECS    1
#define PWM_INITIAL_OFF_USECS   200
#define PWM_STEP_USECS          20

// Set true to log duty_cycle at every 10% boundary. Off in production
// because the USB PLL is off during cold-reset GPS power-up, so prints
// wouldn't make it out anyway.
#define PWM_PRINT_DUTY_CYCLE    false

// -----------------------------------------------------------------------------
// Strategy A: weak-pulldown soft assert (does not work in this hardware,
// kept for reference). The external 10k pull-up on the PCB overrides the
// RP2040's internal pull-down.
// -----------------------------------------------------------------------------
static void gpsPwrOn_weakPulldown(void) {
    // Assumed current GpsPwr state on entry (the GPIO driving the mosfet
    // for the GPS chip power):
    //   output, driven with 1, with pullup (set by init) -> active deassert.
    pinMode(GpsPwr, INPUT);
    // Pulldown is 50 to 80 kohms on the rp2040 (so is pullup).
    gpio_pull_down(GpsPwr);  // this also disables the pullup
    sleep_ms(5000);          // 5 seconds; should have RC-ramped low by now.

    // Make sure we end with GpsPwr asserted using an active driver.
    pinMode(GpsPwr, OUTPUT);
    gpio_put(GpsPwr, LOW);
}

// -----------------------------------------------------------------------------
// Strategy B: bit-banged PWM that ramps from low duty to high duty over ~2s.
// LOW on the GPIO asserts the mosfet (turns GPS power on).
// -----------------------------------------------------------------------------
static void gpsPwrOn_pwmRamp(void) {
    uint64_t on_usecs  = PWM_INITIAL_ON_USECS;
    uint64_t off_usecs = PWM_INITIAL_OFF_USECS;

    // Loop until off_usecs has been stepped down to 0 (the on time has
    // taken over the full period).
    while (off_usecs > 0) {
        Watchdog.reset();

        digitalWrite(GpsPwr, LOW);   // assert mosfet (GPS power on)
        sleep_us(on_usecs);          // lower-power light sleep
        digitalWrite(GpsPwr, HIGH);  // deassert mosfet
        sleep_us(off_usecs);         // lower-power light sleep

        // Compute duty cycle as a percent, scaled. Shifts/scaling chosen
        // to keep some accuracy through integer division:
        // the delta (1000000/10000) gives a percent (*100).
        // 100x pct
        uint64_t duty_cycle =
            (on_usecs * 1000000UL) / ((on_usecs + off_usecs) * 10000UL);

        // Print duty_cycle at every 10% boundary (disabled -- USB PLL is
        // off during GPS cold-reset power-up, so prints wouldn't emit).
        if (PWM_PRINT_DUTY_CYCLE && (duty_cycle % 10) == 0) {
            V1_printf("pwmGpsPwrOn() duty_cycle (pct) %" PRIu64 EOL, duty_cycle);
        }

        on_usecs  += PWM_STEP_USECS;
        off_usecs -= PWM_STEP_USECS;
    }

    // Make sure we end with GpsPwr fully on (asserted).
    digitalWrite(GpsPwr, LOW);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void pwmGpsPwrOn(void) {
    if (WEAK_PULLDOWN_FOR_ASSERT) {
        gpsPwrOn_weakPulldown();
    } else {
        gpsPwrOn_pwmRamp();
    }
}
//************************************************
// =============================================================================
// GpsFullColdReset
// -----------------------------------------------------------------------------
// Performs a full cold reset of the GPS module:
//   1. Power down GPS, assert reset, drop Serial2.
//   2. Slow the RP2040 way down (and optionally drop core voltage / disable
//      USB PLL) to minimize our own power draw while the GPS rail is in its
//      high-inrush window.
//   3. Power GPS back on (optionally via PWM soft-start), deassert reset.
//   4. Sleep ~5 sec while the GPS comes up.
//   5. Restore RP2040 clocks/voltage.
//   6. Bring Serial2 up at the right baud, configure constellations,
//      balloon mode, PPS, broadcast filter, etc.
//   7. Read initial NMEA, return whether anything was seen.
//
// BUG: can't seem to reset the baud rate to 9600 when the GPS chip has a
// non-working baud rate.
//
// A full cold reset reverts to 9600 baud (as do standby modes -- don't use).
// =============================================================================

// -----------------------------------------------------------------------------
// Phase 1: Drive the GPS into a known powered-off, in-reset state.
//
// IDEA! since we KNOW the power demand will be high for 1 minute after poweron
// just go into light sleep to reduce rp2040 power demand for 1 minute --
// i.e. guarantee that cold reset takes 1 minute?
//
// FIX! what if we power on with GPS_ON_PIN LOW and GPS_NRESET_PIN HIGH
//
// NOTE: do I have to keep inputs like GPS_ON_PIN low until after power is on?
// What about reset? To avoid latchup of LNA, see:
// https://www.eevblog.com/forum/rf-microwave/gps-lna-overheating-on-custom-pcb/
// -----------------------------------------------------------------------------
static void powerOffGpsAndAssertReset(void) {
    V1_println(F("Turn off the serial2..float at gps? no UART stuff at poweron?"));
    V1_flush();
    Serial2.end();

    // experiment idea: can we float the rx/tx pins of Serial2? maybe switch it
    // to other pins temporarily?
    // https://github.com/earlephilhower/arduino-pico/discussions/199

    V1_println(F("Doing Gps COLD POWER_ON (GPS_ON_PIN off with first power)"));

    digitalWrite(GPS_ON_PIN, LOW);       // deassert
    digitalWrite(GPS_NRESET_PIN, LOW);   // assert reset
    digitalWrite(GpsPwr, HIGH);          // deassert mosfet (power off)
    // 7/10/25
    // Serial2.end();
    // full 2 secs off?
    gpsSleepForMillis(2000, false);
}

// -----------------------------------------------------------------------------
// Phase 3a: Apply power to the GPS rail. Either soft-PWM ramp the mosfet
// (preferred, lower inrush) or just slam it on.
// -----------------------------------------------------------------------------
static void applyGpsPower(void) {
    // CHANGE: 7/10/25..don't pwm sim65m?
    // if (PWM_GPS_POWER_ON_MODE && !USE_SIM65M) {
    if (PWM_GPS_POWER_ON_MODE) {
        // this is probably at least 2 secs. let's measure
        uint32_t start_millis2 = millis();
        pwmGpsPwrOn();
        uint32_t duration_millis2 = millis() - start_millis2;
        V1_printf("Used pwmGpsPwrOn() and took %lu millisecs" EOL, duration_millis2);
        // soft power-on for GpsPwr (assert low, controls mosfet)
        // note that vbat doesn't have mosfet control, so it will be high right
        // away with availability of power
    } else {
        digitalWrite(GpsPwr, LOW);  // assert to mosfet
        gpsSleepForMillis(500, false);
    }
}

// -----------------------------------------------------------------------------
// Phase 3b: Deassert NRESET after power has come up.
// new 12/7/24 disable Serial2 while powering on!
// should we float the rx/tx also somehow?
// -----------------------------------------------------------------------------
static void deassertGpsReset(void) {
    gpsSleepForMillis(500, false);
    digitalWrite(GPS_NRESET_PIN, HIGH);  // deassert
    gpsSleepForMillis(1000, false);
    Watchdog.reset();
}

// -----------------------------------------------------------------------------
// Phase 2/4: Print which low-power modes are enabled, then drop RP2040 clocks
// (and optionally core voltage / USB PLL) before the GPS power-up window.
//
// IDEA! since we KNOW the power demand will be high for 1 minute after poweron
// just go into light sleep to reduce rp2040 power demand for 1 minute --
// i.e. guarantee that cold reset takes 1 minute?
// hmm. we're stalling things now. maybe only sleep for 15 secs.
// FIX! this apparently makes the Serial2 dysfunctional so the gps chip can't
// send output -- it backs up on the initial TXT broadcast (revisions) and
// then hits a power peak right after we fix the clock back to normal
// (50Mhz min tried). So: is that worth it? dunno.
//
// UPDATE: is the broadcast right after power on the issue?
// other power saving: disable usb pll (and restore).
//
// https://github.com/earlephilhower/arduino-pico/discussions/1544
// We had to make sure we reset the watchdog now in gpsSleepForMillis;
// we already wakeup periodically to update led, so fine.
// -----------------------------------------------------------------------------
static void enterLowPowerForGpsBringup(void) {
    Watchdog.reset();
    if (!BALLOON_MODE) measureMyFreqs();

    V1_print(F("GPS power demand high during cold reset..try to minimize rp2040 power" EOL));

    if (ALLOW_LOWER_CORE_VOLTAGE_MODE) {
        V1_print(F("Also lowering core voltage to 0.95v" EOL));
    }
    if (ALLOW_USB_DISABLE_MODE) {
        V1_print(F("No keyboard interrupts will work because will disable USB PLL too" EOL));
    }
    if (ALLOW_KAZU_12MHZ_MODE) {
        V1_print(F("Will stay in 12Mhz using xosc, after things are restored" EOL));
    }
    if (ALLOW_TEMP_12MHZ_MODE) {
        V1_printf("Switch pll_sys PLL_SYS_MHZ %lu to xosc 12Mhz then sleep" EOL, PLL_SYS_MHZ);
    } else {
        V1_printf("Switch pll_sys PLL_SYS_MHZ %lu to pll 18Mhz then sleep" EOL, PLL_SYS_MHZ);
    }
    V1_flush();

    // The global IGNORE_KEYBOARD_CHARS guarantees core1 won't be interrupted
    // while we've messed with clocks during the GPS aggressive power-on control.
    // It should always be re-enabled after 15 secs.
    // Worst case to recover: unplug power and plug in again.
    // hmm core0 has to know to drain garbage chars if we assert this? then deassert?
    IGNORE_KEYBOARD_CHARS = true;

    // DRASTIC measures, do before sleep!
    // FIX! does the flush above not wait long enough?
    // Wait another second before shutting down serial.
    // Sleep may be problematic in this transition.
    // Some more thoughts about low power rp2040 and clocks; was wondering
    // what measureMyFreqs() sees differing ring osc and rtc freqs:
    // https://forums.raspberrypi.com/viewtopic.php?t=342156
    busy_wait_ms(500);

    // remember not to touch Serial if in BALLOON_MODE!!
    if (!BALLOON_MODE) {
        Serial.flush();
        // gets here
        // V1_printf("kevin1");
        // remove 3/8/26..was causing abort with new idea?
        // Serial.end();
        busy_wait_ms(500);
    }
    if (!BALLOON_MODE && ALLOW_USB_DISABLE_MODE) {
        // maybe only end if in USB disable mode?
        Serial.end();
        busy_wait_ms(500);
    }

    // no
    // V1_printf("kevin1");
    // includes deinit of the usb pll now?
    // 3/8/26 test?
    Watchdog.reset();
    kazuClocksSlow();

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__pll.html
    // There are two PLLs in RP2040:
    //   pll_sys - up to a 133MHz (actually more) system clock
    //   pll_usb - 48MHz USB reference clock
    //
    // pll_deinit(pll) releases/uninitializes the specified PLL, turning off
    // its power. pll_deinit(pll_usb) does NOT check if the PLL is in use
    // before powering it off -- use care.
    //
    // examples: https://sourcevu.sysprogs.com/rp2040/picosdk/symbols/pll_deinit
    // sidenote: pi pico sdk has set_sys_clock_48mhz() which sets sys clk to
    // 48MHz with peripheral clock matched. Example:
    // https://sourcevu.sysprogs.com/rp2040/examples/clocks/hello_48MHz/files/hello_48MHz.c#tok293
    // 18 is the slowest legal I can go for the sys pll.
    //
    // voltage enums:
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h

}

// -----------------------------------------------------------------------------
// Phase 5: Restore RP2040 clocks back to normal after the GPS bringup window.
// -----------------------------------------------------------------------------
static void exitLowPowerAfterGpsBringup(uint32_t pll_sys_mhz_restore) {
    Watchdog.reset();
    busy_wait_ms(500);
    // FIX! this restores/keeps sys clk to 12mhz and sys pll off.
    // The problem is _clock_speed doesn't have 12Mhz, and we need PLL_SYS_MHZ correct 
    // for PWM div/wrap calcs. 
    // Can we just change PLL_SYS_MHZ here?
    int currentGpsBaud = USE_SIM65M ? SIM65M_BAUD_RATE : ATGM336H_BAUD_RATE;
    kazuClocksRestore(pll_sys_mhz_restore, currentGpsBaud);
    // V1_print(F("Restored core voltage back to 1.1v" EOL));
    V1_flush();
    if (!BALLOON_MODE) measureMyFreqs();
    IGNORE_KEYBOARD_CHARS = false;
}

// -----------------------------------------------------------------------------
// Phase 6: Bring Serial2 up and step the GPS to the desired baud rate.
//
// hmm. we get a power surge here then? Is it because the Serial2 data was
// backed up in the gps chip and busy waiting? Drain it? (usually it's the
// TXT stuff (versions) at power on. Don't care.)
//
// We should be able to start talking to it. GPS should come up at 9600 so
// look with our uart at 9600.
//
// FIX! if we're stuck at 4800 -- okay, this won't matter.
// initially talking to it at what baud rate?
// But then we'll be good when we transition to the target rate also.
// -----------------------------------------------------------------------------
static void bringUpSerial2AndSetBaud(void) {
    int BAUD_RATE = USE_SIM65M ? SIM65M_BAUD_RATE : ATGM336H_BAUD_RATE;
    int desiredBaud = checkGpsBaudRate(BAUD_RATE);

    // FIX! does SIM65M sometimes come up in 115200 and sometimes in the
    // last BAUD_RATE set? Do both?
    // it either comes up in desiredBaud from some memory, or comes up in 115200?
    beginSerial2AtDefaultBaud();
    setGpsBaud(desiredBaud);

    if (USE_SIM65M) {
        // for old chips that are stuck at 9600 that we want to try faster
        Serial2.begin(9600);
        busy_wait_ms(500);
        setGpsBaud(desiredBaud);
    }

    gpsSleepForMillis(2000, false);
}

// -----------------------------------------------------------------------------
// Phase 6 (cont.): Optionally write the current "no broadcast / GPS only"
// config to GPS flash so subsequent cold resets boot in a calmer state.
//
// From CASIC_ProtocolSpecification_english.pdf page 24:
// CAS00 -- Save the current configuration information to FLASH. Even if the
// receiver is completely powered off, the FLASH contents persist.
//   Format:  $PCAS00*CS<CR><LF>
//   Example: $PCAS00*01<CR><LF>
//
// Could be dangerous, since it's writing a baud rate to the power-off/on
// reset config state -- could change it from 9600 and we'd lose track.
// As long as we stick with 9600 we should be safe.
//
// Will this help us boot in a better config so we don't get the power demand
// peaks we see (on subsequent boots)? Maybe we shouldn't do this all the time
// -- just once. Does the FLASH have a max # of writes issue (100k or ?)?
// We only do gps cold reset at start of day. Don't do it in BALLOON_MODE;
// that should fix the issue.
// -----------------------------------------------------------------------------
static void maybeWriteGpsConfigToFlash(void) {
    if (!ALLOW_UPDATE_GPS_FLASH_MODE || BALLOON_MODE) return;

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

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
bool GpsFullColdReset(void) {
    GpsIsOn_state = false;
    PPS_countDisable();

    V1_println(F(EOL "GpsFullColdReset START"));
    uint32_t start_millis = millis();
    Watchdog.reset();

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    // -------------------------------------------------------------------------
    // Phase 1: drive GPS off and into reset
    // -------------------------------------------------------------------------
    powerOffGpsAndAssertReset();

    // -------------------------------------------------------------------------
    // Cold-start command notes (for reference -- we rely on the power cycle
    // above rather than sending these):
    //
    //   $PMTK103*30                -- does this PMTK form work or not?
    //   $PCAS10,3*1F  factory      -- clear all data, reset rcvr
    //   $PCAS10,2*1E  cold start   -- no init info, clear all data except config
    //   $PCAS10,1*1D  warm start   -- no init info, all data valid; clear ephemeris
    //   $PCAS10,0*1C  hot start    -- no init info, all data valid
    //
    // Some say PCAS10 is just a boot-mode configuration (i.e. send before
    // power cycle), and that you have to toggle power off/on to get the
    // full-cold-start effect anyway.
    //
    // We do the full power cycle and don't wait for ack. We always commit
    // to driving Serial2.begin/end correctly rather than relying on
    // GpsIsOn() bookkeeping.
    //
    // FIX! does driving the uart rx/tx to gps while gps is powering up
    // change its behavior? What if we left them floating until after
    // powerup? Seems like the gps backs up on the serial data.
    // -------------------------------------------------------------------------

    // we still have usb pll on, and default clock frequency at this point

    // -------------------------------------------------------------------------
    // Phase 2: cut RP2040 clocks (and possibly voltage / USB PLL) for
    // the high-power GPS bringup window. Save current sys freq so we can
    // undo this temporary lower clock setting.
    // -------------------------------------------------------------------------
    uint32_t PLL_SYS_MHZ_restore = PLL_SYS_MHZ;
    enterLowPowerForGpsBringup();

    // -------------------------------------------------------------------------
    // Phase 3a: power GPS back on
    // -------------------------------------------------------------------------
    applyGpsPower();

    // -------------------------------------------------------------------------
    // Phase 3b: deassert NRESET (okay in both normal and experimental case)
    // -------------------------------------------------------------------------
    deassertGpsReset();

    // Finally turn on the GPS here (if we didn't already above in
    // experimental mode).
    digitalWrite(GPS_ON_PIN, HIGH);  // assert

    // FIX! still getting intermittent cases where we don't come back
    // (running 60Mhz). This should have no printing either.
    gpsSleepForMillis(5000, false);  // 5 secs

    // -------------------------------------------------------------------------
    // Phase 4: restore RP2040 clocks
    // -------------------------------------------------------------------------
    exitLowPowerAfterGpsBringup(PLL_SYS_MHZ_restore);

    Watchdog.reset();

    // -------------------------------------------------------------------------
    // Phase 5: Serial2 up, set baud, configure modes
    // -------------------------------------------------------------------------
    bringUpSerial2AndSetBaud();

    // this is all done earlier in the experimental mode
    // FIX! we don't need to toggle power to get the effect?
    if (USE_SIM65M) setGpsBalloonMode();
    if (USE_SIM65M && !BALLOON_MODE) setGpsPPSMode();

    maybeWriteGpsConfigToFlash();

    setGpsConstellations(CONSTELLATIONS_GROUP);
    // no ANT TXT (NMEA sentences) after this:
    setGpsBroadcast();

    // I guess it doesn't power on with location service on
    if (USE_SIM65M) setGnssOn_SIM65M();

    // -------------------------------------------------------------------------
    // Phase 6: read initial NMEA, decide whether the GPS is alive
    // -------------------------------------------------------------------------
    bool sentencesFound = getInitialGpsOutput();

    // flush out any old state in TinyGPSplus, so we don't get a valid fix
    // that's got a big fix_age
    invalidateTinyGpsState();
    GpsStartTime = get_absolute_time();  // usecs

    if (sentencesFound) {
        GpsIsOn_state = true;
        PPS_countEnable(true);  // reset
    }

    uint32_t duration_millis = millis() - start_millis;
    V1_print(F("GpsFullColdReset END"));
    V1_printf(" sentencesFound %u", sentencesFound);
    V1_printf(" duration_millis %lu" EOL, duration_millis);
    return sentencesFound;
}

// =============================================================================
// GpsHotReset
// -----------------------------------------------------------------------------
// Performs a hot reset of the GPS module. Mirrors the structure of
// GpsFullColdReset but is faster and skips the RP2040 low-power dance.
//
//   Hot start:  No initialization information is used and all the data is valid.
//   Warm start: Do not use initialization information and clear ephemeris.
//
// FIX! this doesn't have option of doing warm reset nmea command to gps chip.
// Would have to do that to support a warm reset that only clears ephemeris.
// Right now we just have hot and cold; we send the warm-reset command and
// retry if the hot reset doesn't get any sentences back.
//
// FIX! SIM65M spec says when the power supply is off, settings are reset to
// factory config and the receiver performs a cold start on next power up.
// I suppose we should just switch to idle mode instead of powering the GPS
// chip off?
//
// Hot reset doesn't change the baud rate from the prior config -- but what
// if we lost VBAT and config isn't preserved?
// =============================================================================

// -----------------------------------------------------------------------------
// Power-cycle the GPS chip without asserting reset (that's the "hot" part).
// reorganized to match GpsFullColdReset()
//
// NOTE: should we start with NRESET_PIN low also until powered (latchup)?
// NOTE: do we need to start low until powered to avoid latchup of LNA?
// FIX! what if we power on with GPS_ON_PIN LOW and GPS_NRESET_PIN HIGH
// -----------------------------------------------------------------------------
static void hotResetPowerCycle(void) {
    V1_println(F("Doing Gps HOT POWER_ON (GPS_ON_PIN off with power off-on)"));
    // Off (don't assert reset during power off)
    digitalWrite(GPS_ON_PIN, LOW);
    digitalWrite(GPS_NRESET_PIN, HIGH);
    digitalWrite(GpsPwr, HIGH);
    Serial2.end();
    // 2/16/2025 try faster for faster gps hot reset
    gpsSleepForMillis(1000, false);  // no early out

    // On -- match the PWM soft-start used for cold reset
    if (PWM_GPS_POWER_ON_MODE) {
        // this is probably at least 2 secs. let's measure
        uint32_t start_millis2 = millis();
        pwmGpsPwrOn();
        uint32_t duration_millis2 = millis() - start_millis2;
        V1_printf("Used pwmGpsPwrOn() and took %lu millisecs" EOL, duration_millis2);
        // soft power-on for GpsPwr (assert low, controls mosfet)
        // note that vbat doesn't have mosfet control, so it will be high
        // right away with availability of power
    } else {
        digitalWrite(GpsPwr, LOW);   // assert to mosfet
        gpsSleepForMillis(500, false);
    }

    // 2/16/2025 try faster for faster gps hot reset
    gpsSleepForMillis(1000, false);  // no early out

    // now assert the on/off pin
    digitalWrite(GPS_ON_PIN, HIGH);
    gpsSleepForMillis(1000, false);  // no early out
}

// -----------------------------------------------------------------------------
// Bring Serial2 up at the chip's power-on baud and step it to the desired baud.
// Should come up in the last programmed baud rate (from cold reset), but
// reality has been less reliable than that -- so we always re-set baud here.
// -----------------------------------------------------------------------------
static void hotResetBringUpSerial2(void) {
    int BAUD_RATE = USE_SIM65M ? SIM65M_BAUD_RATE : ATGM336H_BAUD_RATE;
    int desiredBaud = checkGpsBaudRate(BAUD_RATE);

    if (USE_SIM65M) {
        // Hmm. did we have failures where it didn't come up in the right baud rate?
        // it either comes up in desiredBaud from some memory (9600?),
        // or comes up in 115200?
        Serial2.begin(115200);
        gpsSleepForMillis(500, false);  // no early out
        setGpsBaud(desiredBaud);
    } else {
        // it either comes up in desiredBaud from some memory, or comes up in 9600?
        // Used to not set ATGM baud rate! now we do..above
        Serial2.begin(9600);
        gpsSleepForMillis(500, false);  // no early out
        setGpsBaud(desiredBaud);
    }
    gpsSleepForMillis(500, false);  // no early out
}

// -----------------------------------------------------------------------------
// Reapply config (constellations, broadcast filter, balloon mode, PPS).
// Optional -- skipped unless HOT_RESET_REDO_CONFIG is set.
//
// Always reconfig balloon mode -- there were known issues with ublox losing
// this, and it's hard to realize if we lost it unless we read the mode.
// Don't reconfig PPS in BALLOON_MODE; it's unnecessary because no prints.
// -----------------------------------------------------------------------------
static void hotResetReapplyConfig(void) {
    if (!HOT_RESET_REDO_CONFIG) return;

    setGpsConstellations(CONSTELLATIONS_GROUP);
    // we don't need no ZDA/TXT
    setGpsBroadcast();

    if (USE_SIM65M) setGpsBalloonMode();
    if (USE_SIM65M && !BALLOON_MODE) setGpsPPSMode();
}

// -----------------------------------------------------------------------------
// SIM65M: read back the navigation mode (sanity check across hot reset) and
// make sure GNSS is enabled. We could change the default config to power up
// with GNSS off, so we always re-enable it here.
// -----------------------------------------------------------------------------
static void hotResetSIM65MPostConfig(void) {
    if (!USE_SIM65M) return;

    // Always read it to make sure it's right thru hot reset.
    V1_println(F("Read the navigation mode: $PAIR081*33"));
    // Packet Type:081 PAIR_COMMON_GET_NAVIGATION_MODE
    Serial2.print("$PAIR081*33" CR LF);
    // 2/16/2025 try faster for faster gps hot reset
    nmeaBufferFastPoll(1500, true);  // duration_millis, printIfFull

    setGnssOn_SIM65M();
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
bool GpsHotReset(void) {
    GpsIsOn_state = false;
    PPS_countDisable();

    V1_println(F(EOL "GpsHotReset START"));
    uint32_t start_millis = millis();

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();
    // turn off the serial
    V1_flush();
    // -------------------------------------------------------------------------
    // Power-cycle the GPS (no reset assertion)
    // -------------------------------------------------------------------------
    hotResetPowerCycle();
    // -------------------------------------------------------------------------
    // Serial2 + baud
    // -------------------------------------------------------------------------
    hotResetBringUpSerial2();
    // -------------------------------------------------------------------------
    // Optional reconfig
    // -------------------------------------------------------------------------
    hotResetReapplyConfig();
    hotResetSIM65MPostConfig();
    // -------------------------------------------------------------------------
    // Read initial NMEA, decide whether the GPS is alive
    // -------------------------------------------------------------------------
    bool sentencesFound = getInitialGpsOutput();

    // flush out any old state in TinyGPSplus, so we don't get a valid fix
    // that's got a big fix_age
    invalidateTinyGpsState();

    uint32_t duration_millis = millis() - start_millis;
    V1_print(F("GpsHotReset END"));
    V1_printf(" sentencesFound %u", sentencesFound);
    V1_printf(" duration_millis %lu" EOL, duration_millis);

    if (sentencesFound) {
        GpsIsOn_state = true;
        PPS_countEnable(true);  // reset
    }
    GpsStartTime = get_absolute_time();  // usecs
    return sentencesFound;
}
// =============================================================================
// writeGpsConfigNoBroadcastToFlash
// -----------------------------------------------------------------------------
// Persist a "quiet" GPS configuration to the chip's flash so that on the
// next cold reset the receiver wakes up with broadcast disabled and only
// GPS constellation enabled. This reduces the power demand spike right
// after power-on.
//
// Sequence:
//   1. Stage the desired persistent config in volatile state:
//        - disable broadcast
//        - reduce constellations to GPS only (1)
//        - SIM65M: turn GNSS subsystem OFF (so the SAVE command is allowed
//          in multi-Hz mode -- spec only permits saving when GNSS is off)
//   2. Send the chip-specific "save current config to flash" NMEA command.
//   3. Restore working runtime state:
//        - SIM65M: turn GNSS back ON
//        - restore configured constellations
//        - re-enable broadcast
//
// Risk note (from original): do we ever power on and not do this full cold
// reset that sets up broadcast? The hot GPS reset shouldn't pull new state
// from config. So changing the FLASH to "no broadcast" should be safe --
// cold reset power-on will come up quiet and we set broadcast back during
// the rest of the cold reset flow.
//
// Reference (SIM65M PAIR commands):
//
//   $PAIR002*38  PAIR_GNSS_SUBSYS_POWER_ON
//     Power on the GNSS system. Include DSP/RF/Clock and other GNSS
//     modules. Please send this command before using any location service.
//
//   $PAIR003*39  PAIR_GNSS_SUBSYS_POWER_OFF
//     Power off GNSS system. Include DSP/RF/Clock and other GNSS modules.
//     CM4 can still receive commands after this (AT, race, PAIR commands
//     not dependent on DSP). Location service is unavailable. The system
//     can still receive configuration PAIR commands.
//
//   $PAIR513*3D  PAIR_NVRAM_SAVE_SETTING
//     Save the current configuration from RTC RAM to flash.
//     In multi-Hz, this can only be set when GNSS is powered off; 1Hz has
//     no such limit. (We power GNSS off above, so OK either way.)
//
//   $PAIR514*3A  PAIR_NVRAM_RESTORE_DEFAULT_SETTING (in case we mess up flash)
//     Clear current config and restore default settings. Does not support
//     run-time restore when GNSS is on -- send PAIR_GNSS_SUBSYS_POWER_OFF
//     first.
//
// FIX! we'll have to figure this out for SIM65M
// FIX! just GPS -- what about 0 (no constellations)? would that save more
//      power at gps power on?
// FIX! this doesn't change SIM65M default constellations (yet).
// =============================================================================

// -----------------------------------------------------------------------------
// Stage the "quiet" config we want to persist into flash.
// -----------------------------------------------------------------------------
static void stageQuietConfigForFlash(void) {
    disableGpsBroadcast();
    setGpsConstellations(1);  // GPS only
    // SIM65M only: turn GNSS subsystem off so the SAVE command is permitted
    // in multi-Hz mode. Also: powering up with GNSS off means a future cold
    // reset boots into a lower-power state.
    if (USE_SIM65M) setGnssOff_SIM65M();
}

// -----------------------------------------------------------------------------
// Send the chip-specific "save current config to flash" NMEA command.
// -----------------------------------------------------------------------------
static void sendSaveConfigToFlashCommand(void) {
    char nmeaSentence[64] = { 0 };
    if (USE_SIM65M) {
        strncpy(nmeaSentence, "$PAIR513*3D" CR LF, 64);
    } else {
        strncpy(nmeaSentence, "$PCAS00*01" CR LF, 64);
    }
    V1_printf("%s" EOL, nmeaSentence);
    Serial2.print(nmeaSentence);
    Serial2.flush();
    sleep_ms(1000);
}

// -----------------------------------------------------------------------------
// Restore working runtime state (GNSS back on, configured constellations,
// broadcast re-enabled).
// -----------------------------------------------------------------------------
static void restoreRuntimeConfig(void) {
    // we could change the default config to power up with GNSS off?
    if (USE_SIM65M) setGnssOn_SIM65M();
    // FIX! this doesn't change SIM65M default constellations (yet)
    setGpsConstellations(CONSTELLATIONS_GROUP);
    setGpsBroadcast();
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void writeGpsConfigNoBroadcastToFlash(void) {
    V1_println(F("writeGpsConfigNoBroadcastToFlash() START"));
    stageQuietConfigForFlash();
    sendSaveConfigToFlashCommand();
    restoreRuntimeConfig();
    V1_println(F("writeGpsConfigNoBroadcastToFlash END"));
}

// =============================================================================
// GpsON
// -----------------------------------------------------------------------------
// Bring the GPS up. Tries up to 3 times. If `GpsColdReset` is true, uses
// GpsFullColdReset(); otherwise uses GpsHotReset() and falls back to cold
// reset after 3 hot-reset failures.
//
// On each failed attempt (other than the last), also pokes the GPS chip
// directly with the software cold/warm-start NMEA command, in case the pin
// assertions in {Hot,FullCold}Reset didn't take effect.
//
// Assumes GpsINIT was already done (pins etc). The chip could be off or on
// already on entry.
// =============================================================================

#define GPS_RESET_MAX_TRIES 3

// -----------------------------------------------------------------------------
// Logging helper -- match the original behavior:
// no print of `GpsColdReset` if it's not set, so you can grep for
// "GpsColdReset" as a special case only.
// -----------------------------------------------------------------------------
static void logGpsOnBoundary(const char *boundary,  // "START" or "END"
                             bool gpsColdReset,
                             const char *trailing) {  // "" or " <EOL>"
    if (!gpsColdReset) {
        V1_printf("GpsON %s GpsIsOn_state %u%s" EOL,
                  boundary, GpsIsOn_state, trailing);
    } else {
        V1_printf("GpsON %s GpsIsOn_state %u GpsColdReset %u%s" EOL,
                  boundary, GpsIsOn_state, gpsColdReset, trailing);
    }
}

// -----------------------------------------------------------------------------
// Send the software-command fallback to nudge the GPS into a cold/warm start
// when pin-driven reset didn't produce any NMEA sentences. Depends on the
// baud setup being right.
//
// FIX! I do warm start here, not hot start! This will clear ephemeris,
// unlike normal hot starts.
// -----------------------------------------------------------------------------
static void sendSoftwareResetCommand(bool gpsColdReset) {
    if (USE_SIM65M) {
        if (gpsColdReset) {
            // Packet Type:007 PAIR_GNSS_SUBSYS_FULL_COLD_START
            V1_print(F("ERROR: no sentencesFound, send command for PAIR_GNS_SUBSYS_FULL_COLD_START"));
            Serial2.print("$PAIR001,007,0*3C" CR LF);
        } else {
            // Packet Type:005 PAIR_GNSS_SUBSYS_WARM_START
            // this will clear ephemeris, unlike normal hot starts
            V1_print(F("ERROR: no sentencesFound, send command for PAIR_GNS_SUBSYS_WARM_START (clear ephemeris)"));
            Serial2.print("$PAIR001,005,0*3E" CR LF);
        }
    } else {
        if (gpsColdReset) {
            // PCAS10 factory start
            V1_print(F("ERROR: no sentencesFound, send command PCAS10 factory start"));
            Serial2.print("$PCAS10,3*1F" CR LF);
        } else {
            // PCAS10 warm start.
            // This will clear ephemeris, unlike normal hot starts.
            V1_print(F("ERROR: no sentencesFound, send command PCAS10 warm start (clear ephemeris)"));
            Serial2.print("$PCAS10,1*1D" CR LF);
            Serial2.print("" CR LF);
        }
    }
    Serial2.flush();
    busy_wait_ms(1000);
}

// -----------------------------------------------------------------------------
// Run one reset attempt (cold or hot) and return whether NMEA was seen.
// -----------------------------------------------------------------------------
static bool tryOneGpsReset(bool gpsColdReset) {
    return gpsColdReset ? GpsFullColdReset() : GpsHotReset();
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void GpsON(bool GpsColdReset) {
    logGpsOnBoundary("START", GpsColdReset, "");
    Watchdog.reset();

    // Fast path: if we weren't asked for a cold reset and the GPS is already
    // on, fake `sentencesFound = true` so we skip the hot-reset attempt.
    // (For cold reset we don't care what the initial state is.)
    bool sentencesFound = false;
    if (!GpsColdReset && GpsIsOn()) {
        V1_println(F("do nothing because GpsIsOn()"));
        sentencesFound = true;
    }

    // -------------------------------------------------------------------------
    // Retry loop: up to GPS_RESET_MAX_TRIES attempts. If hot resets keep
    // failing, escalate to cold reset and reset the try counter.
    // -------------------------------------------------------------------------
    uint32_t tryCnt = 0;
    while (!sentencesFound) {
        tryCnt += 1;

        // Hit the retry ceiling -- either give up (cold) or escalate (hot).
        if (tryCnt >= GPS_RESET_MAX_TRIES) {
            if (GpsColdReset) {
                V1_print(F("ERROR: tryCnt 3 on GpsFullColdReset.. not retrying any more "));
                printSystemDateTime();
                V1_print(F(EOL));
                break;
            } else {
                V1_print(F("ERROR: tryCnt 3 on GpsHotReset.. switch to trying GpsColdReset "));
                printSystemDateTime();
                V1_print(F(EOL));
                GpsColdReset = true;
                tryCnt = 0;
            }
        }

        sentencesFound = tryOneGpsReset(GpsColdReset);

        // Software-command fallback: if the pin-driven reset produced
        // nothing and we still have retries left, poke the chip via NMEA.
        if (!sentencesFound && (tryCnt < GPS_RESET_MAX_TRIES)) {
            sendSoftwareResetCommand(GpsColdReset);
        }
    }

    logGpsOnBoundary("END", GpsColdReset, EOL);
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

    // 4/26/25 hmm. altitude and speed doesn't get flushed.
    // maybe we should flush those too, in case a valid 0 gets stuck in there?
    // we don't flush sat count stuff?
    gps.location.flush();
    gps.location.fixQualityFlush();
    gps.location.fixModeFlush();

    gps.speed.flush();
    gps.altitude.flush();

    gps.date.flush();
    gps.time.flush();
    V1_println(F("invalidateTinyGpsState END"));
}

//************************************************
void GpsOFF() {
    GpsIsOn_state = false;
    GpsStartTime = 0;
    PPS_countDisable();

    V1_printf("GpsOFF START GpsIsOn_state %u" EOL, GpsIsOn_state);
    digitalWrite(GpsPwr, HIGH);
    // Serial2.end() Disables serial communication,
    // To re-enable serial communication, call Serial2.begin().
    // FIX! do we really need or want to turn off Serial2?
    // Remember to Serial2.begin() when we turn it back on
    // normally the serial pins default to low
    // but after calling Serial2.begin() tx idles high
    Serial2.end();
    // delay between end and begin?
    gpsSleepForMillis(1000, false);
    // unlike i2c to vfo, we don't tear down the Serial2 definition...just .end()
    // so we can just .begin() again later
    // have to flush everything.
    // Can't keep enqueued time. not worth saving altitude/lat/lon..we have to
    // wait for time to get set again?
    invalidateTinyGpsState();

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    V1_flush();
    V1_printf("GpsOFF END GpsIsOn_state %u" EOL, GpsIsOn_state);
}


//************************************************
uint32_t updateGpsDataAndTime(int ms) {
    // to make sure we get some update, even if fix_age is larger than 1 sec.
    V1_println(F("updateGpsDataAndTime START"));
    Watchdog.reset();

    // ms has to be positive?
    // grab data for no more than ms milliseconds
    // stop if no data for 50 milliseconds
    // all the durations below won't start counting until we get the first char
    // (sets start_millis())
    const uint32_t entry_millis = millis();
    uint32_t start_millis       = 0;
    uint32_t last_char_millis   = 0;
    int incomingCharCnt         = 0;

    // inc on '$'
    // static: keep the count since the first time we start doing this function
    // can visually compare these in the stdout...ideally should always be the same
    static int sentenceStartCnt = 0;
    // inc on '*' (comes before the checksum)
    static int sentenceEndCnt = 0;
    static uint32_t last_gcp, last_gswf, last_gfc;

    V1_printf(
        "updateGpsDataAndTime started looking for NMEA current_millis %lu" EOL,
        millis());

    // clear the StampPrintf buffer, in case it had anything.
    if (VERBY[1]) DoLogPrint();

    GpsON(false);

    // Drain any stale bytes already in the UART FIFO before listening.
    // FIX! 7/10/25 change from 30 to 1, to get more room for 115200 baud of sim65m cb parts?
    getChar();
    if (charsAvailable >= 1) {
        if (VERBY[1])
            StampPrintf("INFO: initially drained NMEA chars because rx has stuff. uart rx initially %d" EOL,
                (int)charsAvailable);
        Watchdog.reset();
        // should be at most 31 to drain
        for (int n = charsAvailable; n > 0; n--) getChar();
    }

    // don't start sending to TinyGPS until we get $|CR|LF so we know we're aligned
    // can't do in another loop, because of delays getting chars. Have
    // to have it in this main timeout loop
    bool aligned = false;

    // the time of the last $..for setting system time to the secs in the nmea sentence
    // with less variation (rather than time at the end of the checksum)
    uint32_t dollar_millis     = 0;
    // double buffering so no race condition with '$'
    uint32_t dollarStar_millis = 0;
    uint32_t time_dollarStar_millis = 0;
    // only one that causes gps.time.updated
    bool     doDelayedTimeUpdate  = false;
    uint32_t timeUpdate_sentences = 0;
    // bool     timeUpdateDone       = false;
    gps.time.updated = false;
    gps.date.updated = false;

    // replaces multi-level break: set when we have enough time updates
    bool     finished             = false;

    while (!finished && (millis() - entry_millis) < (uint64_t)ms) {
        while (!finished && charsAvailable > 0) {
            uint32_t now = millis();
            // start the duration timing when we get the first char
            if (start_millis == 0) start_millis = now;
            last_char_millis = now;
            // we count all chars, even CR LF etc
            incomingCharCnt++;
            // timeUpdateDone = false;
            // shouldn't happen any more?
            if (VERBY[1] && charsAvailable >= 31)
                StampPrintf("ERROR: full. uart rx depth %d incomingCharCnt %d" EOL,
                    (int)charsAvailable, incomingCharCnt);
            char c      = incomingChar;
            bool isCrlf = (c == '\r' || c == '\n');

            // FIX! ignoring unprintables. Do we even get any? maybe in error?
            // either a number (0123456789),
            // an uppercase letter ABCDEFGHIJKLMNOPQRSTUVWXYZ
            // a lowercase letter  abcdefghijklmnopqrstuvwxyz
            // a punctuation character !"#$%&'()*+,-./:;<=>?@[\]^_`{|}~ ,
            // or <space>, or any character classified as printable by the current C locale.
            bool isPrint = !isCrlf && isprint(c);
            // always strip non-printable, non-CRLF chars here and continue the loop
            // CR and LF isprint() is false
            if (!isPrint && !isCrlf) {
                getChar();
                continue;
            }
            // Update sentence-alignment and timing state.
            // crlf falls through to gps.encode() below but isPrint stays false,
            // so it won't be written to the nmea buffer for printing.
            // TinyGPS++ expects the CR LF between sentences.
            switch (c) {
                case '$':
                    aligned = true;
                    sentenceStartCnt++;
                    dollar_millis = now;
                    break;
                case '\r': case '\n':
                    aligned = true;
                    break;
                case '*':
                    sentenceEndCnt++;
                    // clear time updated state, right before any TinyGPS term/commit event
                    // always need checksum before a commit event
                    // we use date in the routine. so both should be updated?
                    gps.time.updated = false;
                    // timeUpdateDone   = false;
                    // save dollar_millis to avoid race condition with next '$'
                    dollarStar_millis = dollar_millis;
                    break;
                default: break;
            }

            if (!aligned) { 
                getChar();  
                continue; 
            }

            // unload each char to TinyGPS++ object as it arrives
            // https://arduino.stackexchange.com/questions/13452/tinygps-plus-library
            gps.encode(c);
            // updated has to transition before we get the next dollar_millis??
            // we use dollarStar_millis to make sure no race condition with '$'
            // RMC should be last sentence in the burst? has date.
            // GGA is first in the burst. we use that for time.
            if (gps.time.updated && gps.date.updated) {
                // TinyGPS should be setup so only one time update trigger per burst
                doDelayedTimeUpdate = true;
                // save the earlies millis from the time update
                time_dollarStar_millis = dollarStar_millis;
            }
            // Note we disabled the GPTXT broadcast to reduce the NMEA load (for here)
            // Do we get any unprintable? ignore unprintable chars, just in case.
            if (VERBY[1] && isPrint) nmeaBufferAndPrint(c, false);  // no print if full
            getChar();
        }

        if (finished) break;

        // keep as close as possible to the NMEA sentence arrival?
        // I suppose we'll see gps.time.updated every time?
        updateStatusLED();
        // did we wait more than ?? millis() since good data read?
        // we wait until we get at least one char or go past the ms total wait
        // break out when we don't get the next char right away
        uint32_t gapMs = last_char_millis ? (millis() - last_char_millis) : 0;

        // FIX! should the two delays used be dependent on baud rate?
        // if we got slowed down by doing a timeUpdate, don't do this
        // FIX! if the time update took more than 32ms the rx fifo would back up, full
        // in any case, we don't break on this if we did a time update
        // situation probably doesn't happen now.
        // was 25
        // if (gapMs >= 10 && !timeUpdateDone) break;
        // no more timeUpdate delay in the loop
        if (gapMs >= 10) break;

        // stop the wait early if Serial2.available
        // was 25
        gpsSleepForMillis(10, true);
        getChar();
    }

    // how long does a burst take? 1 sec? this could e done one sec late relative to gps time
    if (doDelayedTimeUpdate) {
        // if we get two, we've gone too long on the burst
        timeUpdate_sentences++;
        checkUpdateTimeFromGps(time_dollarStar_millis);
        gps.time.updated = false;
        gps.date.updated = false;
        // trying to synchronize so GGA is always first
        // timeUpdateDone = true;
        // should only be one time update trigger now
        if (timeUpdate_sentences >= 2) finished = true;
    }
    
    // Reporting
    uint32_t duration_millis = start_millis ? (millis() - start_millis) : 0;
    if (VERBY[1]) {
        // print/clear any accumulated NMEA sentence stuff
        nmeaBufferPrintAndClear();
        V1_print(F(EOL));
        DoLogPrint();  // dump the StampPrintf if any
    }

    // This will be lower than a peak rate.
    // It includes dead time at start, dead time at end...
    // With some constant rate in the middle? but sentences could be split..
    // start_millis is the first char. so duration_millis will
    // include the end stall detect (25 millis).
    // So it's an average over that period.
    float avgCharRateSec = (duration_millis == 0)
        ? 0.0f
        : 1000.0f * ((float)incomingCharCnt / (float)duration_millis);
    // can it get too big?
    if (avgCharRateSec > 999999.9f) avgCharRateSec = 999999.9f;

    // the time for chars to arrive should never be more than 1 sec?
    // too much stuff in one burst?
    if (duration_millis > 1000)
        V1_printf("ERROR: NMEA sentences duration_millis %lu > 1000 milliseconds" EOL,
            duration_millis);

    V1_printf(
        "NMEA sentences: AvgCharRateSec %.f duration_millis %lu incomingCharCnt %d" EOL,
        avgCharRateSec, duration_millis, incomingCharCnt);

    // checksum errors at TinyGPS?
    uint32_t gcp  = gps.charsProcessed();
    uint32_t gswf = gps.sentencesWithFix();
    uint32_t gfc  = gps.failedChecksum();

    V1_printf("TinyGPS       charsProcessed %10lu sentencesWithFix %5lu failedChecksum %5lu" EOL,
        gcp, gswf, gfc);
    // can visually compare to prior sentences received and printed and see if tinygps
    // is reporting checksum errors that shouldn't exist?
    V1_printf("TinyGPS delta charsProcessed %10lu sentencesWithFix %5lu failedChecksum %5lu" EOL,
        gcp - last_gcp, gswf - last_gswf, gfc - last_gfc);
    last_gcp = gcp; last_gswf = gswf; last_gfc = gfc;

    updateStatusLED();
    uint32_t total_millis = millis() - entry_millis;
    // will be interesting to compare total_millis to duration_millis
    V1_printf("updateGpsDataAndTime END total_millis %lu" EOL EOL, total_millis);
    return total_millis;
}

// =============================================================================
// checkUpdateTimeFromGps
// -----------------------------------------------------------------------------
// Periodically resync the system clock to the GPS time, applying a PPS-based
// skew correction so the resulting system time aligns as closely as possible
// to the NMEA timestamp's true wall-clock instant.
//
// Called once per NMEA burst (rate-limited internally). Returns silently when:
//   - GPS year is out of range, or GPS date/time isn't valid
//   - We were called less than one burst-interval since the last call
//   - The current fix is too stale (fix_age > 250 normally, > 500 if forced)
//   - Hundredths of a second is non-zero (PPS skew is then unreliable)
//   - GPS date/time fields are out of range (and we're not forcing)
//
// On success, calls setTimeWithMillis() with a millis() timestamp adjusted
// backward by the best-guess PPS skew (so the captured "now" represents
// the moment the GPS started transmitting the NMEA sentence, not the
// moment we finished receiving it).
//
// UPDATE: since the first sentence (GNGGA for ATGM336H) has the least
// difference in time from being sent from GPS, it's most accurate.
// The burst takes about 1 sec so the last (GNGST?) might be delayed by
// 1 sec because of uart transmission delays.
// UPDATE: just use GNGGA (TinyGPS mode) ..first one.
// Change the quiet zone to be time of a burst -- that way we always update
// on the last burst before wspr tx?
// Is the quiet zone especially important for a long burst?
// What if the burst interval was 5 secs -- could there be staleness?
// Don't use 5 sec broadcast interval (now we're using?)
// =============================================================================

// Days-per-month lookup (Feb=28; leap years are not validated since we only
// glitch-filter, not date-arithmetic).
static const uint8_t monthDays[] =
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// -----------------------------------------------------------------------------
// Quick prefilter: do we have any business trying to update time?
// Returns true if the GPS year is in a sane range and the date/time fields
// claim to be valid.
// -----------------------------------------------------------------------------
static bool gpsTimeIsUsableForUpdate(void) {
    uint16_t gps_year = gps.date.year();
    bool gps_year_valid = (gps_year >= 2025 && gps_year <= 2035);
    return gps_year_valid && !GpsInvalidAll
           && gps.date.isValid() && gps.time.isValid();
}

// -----------------------------------------------------------------------------
// fix_age gate: refuse to use a stale fix. Tighter limit normally,
// looser limit if we're forcing an update.
// Returns true to keep going, false to abort the whole update.
// -----------------------------------------------------------------------------
static bool fixAgeIsAcceptable(bool forceUpdate, uint32_t timeUpdateCnt) {
    uint32_t fix_age = gps.time.age();
    if (forceUpdate) {
        // always take the first one, to be sure of getting something
        if (timeUpdateCnt != 0) {
            // (caller has already noted forceUpdate elapsed_millis2)
            // loose fix_age constraint, if we're forcing, just in case?
            if (fix_age > 500) {
                V1_printf("WARN: bad setTime forceUpdate. fix_age %lu" EOL, fix_age);
                return false;
            }
        }
        return true;
    }
    if (fix_age > 250) {
        V1_printf("WARN: bad try. fix_age %lu" EOL, fix_age);
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// Sanity check the GPS date/time fields. Returns true if anything's bogus.
// Also prints the offending values when bad.
//
// All fields are uint8_t so we don't have to check negatives.
// We don't validate by month-specific leap-year arithmetic -- glitches that
// way are unlikely.
// -----------------------------------------------------------------------------
static bool gpsDateTimeIsBad(uint8_t gps_hour, uint8_t gps_minute, uint8_t gps_second,
                             uint8_t gps_day, uint8_t gps_month, uint16_t gps_year) {
    bool bad = false;
    if (gps_hour > 23)   bad = true;
    if (gps_minute > 59) bad = true;
    if (gps_second > 59) bad = true;
    // should we validate by the month? forget about that. unlikely to glitch that way.
    if (gps_day > 31)    bad = true;
    if (gps_month > 12)  bad = true;
    if (gps_month < 1)   bad = true;

    // was year range already validated? but do it here too.
    // will have to remember to update this in 10 years (and above too!)
    if (gps_year < 2025 || gps_year > 2035) bad = true;

    // check the days in a month, only if we have a valid month for the array
    if (!bad) {
        uint8_t validDays = monthDays[gps_month - 1];  // 0..11 by check above
        if (gps_day > validDays) {
            V1_print(F("ERROR: gps time is bad."));
            V1_printf(" day %u too big. max %u for the month" EOL,
                      gps_day, validDays);
            bad = true;
        }
    }

    if (bad) {
        V1_print(F("ERROR: gps time is bad. Maybe no received gps time yet."));
        V1_printf(" gps_hour %u gps_minute %u gps_second %u",
                  gps_hour, gps_minute, gps_second);
        V1_printf(" gps_day %u gps_month %u gps_year %u" EOL,
                  gps_day, gps_month, gps_year);
    }
    return bad;
}

// -----------------------------------------------------------------------------
// Compute (or update) the best-guess offset from the last PPS rising edge to
// the captured setTime millis. We use this to push setTime_millis backwards
// so it represents the moment the NMEA sentence really started.
//
// elapsed_millis3 % 1000 is used so we still get a useful number even if we
// missed a PPS rise update (e.g. because the GPS was off briefly).
// -----------------------------------------------------------------------------
static void updateBestGuessSkewFromPPS(uint32_t setTime_millis,
                                       uint32_t *bestGuessSkewFromPPS,
                                       bool forceUpdate) {
    if (!PPS_rise_valid) {
        V1_printf("WARN: setTime PPS_rise_valid false. bestGuessSkewFromPPS %lu" EOL,
                  *bestGuessSkewFromPPS);
        return;
    }
    // we should be using this at least once per rollover.
    uint32_t elapsed_millis3 = setTime_millis - PPS_rise_millis;
    uint32_t elapsed_millis3_modulo = elapsed_millis3 % 1000;
    // print the modulo 1 sec also, if the last PPS was a while ago
    // (gps being reset or ?)
    uint32_t fix_age = gps.time.age();
    V1_printf("setTime elapsed_millis3 %lu %lu after PPS. PPS_rise_cnt %lu",
              elapsed_millis3, elapsed_millis3_modulo, PPS_rise_cnt);
    V1_printf(" fix_age %lu forceUpdate %u" EOL, fix_age, forceUpdate);
    // range check it..otherwise leave as it was (default 100 on first call)
    if (elapsed_millis3_modulo > 10 && elapsed_millis3_modulo < 500) {
        *bestGuessSkewFromPPS = elapsed_millis3_modulo;
    } else {
        V1_printf("ERROR: setTime elapsed_millis3_modulo %lu out of range, ignoring" EOL,
                  elapsed_millis3_modulo);
    }
    V1_printf("setTime PPS_rise_valid true, using bestGuessSkewFromPPS %lu" EOL,
              *bestGuessSkewFromPPS);
}

// -----------------------------------------------------------------------------
// Compare system time to gps time and emit a GOOD/WARN/ERROR drift line.
// secondDelta == 0 is GOOD, == 1 is WARN, > 1 is ERROR.
// Skipped on the first update since the first comparison doesn't matter.
// -----------------------------------------------------------------------------
static void reportDrift(int secondDelta, bool forceUpdate, uint32_t timeUpdateCnt) {
    if (timeUpdateCnt == 0) return;
    // V1_printf("system vs gps: total secondDelta %d" EOL, secondDelta);
    if (abs(secondDelta) > 1) {
        V1_printf("ERROR: excess drift. abs(secondDelta)>1:  secondDelta %d forceUpdate %u ",
                  secondDelta, forceUpdate);
    } else if (abs(secondDelta) == 1) {
        V1_printf("WARN: drift. abs(secondDelta)==1:  secondDelta %d forceUpdate %u ",
                  secondDelta, forceUpdate);
    } else {
        V1_printf("GOOD: no drift. secondDelta %d forceUpdate %u ",
                  secondDelta, forceUpdate);
    }
    printSystemDateTime();
    V1_print(F(EOL));
}

// -----------------------------------------------------------------------------
// Diagnostic: how full is the Serial2 RX FIFO right now? Helps us understand
// whether all this print/work is causing characters to back up.
//
// Seems like all this takes 11ms or so max. If chars are arriving 1 per ms,
// the rx fifo needs room for ~11 when we do time, to absorb char backup.
// Should be okay if we started with empty rx fifo, kept up with it, and
// then only do this time update once per burst.
// FIX! should we drain if > 30 ?
// -----------------------------------------------------------------------------
static void reportRxFifoBackup(uint32_t fix_age_entry) {
    uint32_t fix_age = gps.time.age();
    int charsAvailable = Serial2.available();
    if (charsAvailable > 25) {
        V1_print(F("ERROR: rx fifo backup (2): "));
    } else if (charsAvailable > 21) {
        V1_print(F("WARN: rx fifo backup (2): "));
    }
    V1_printf("gps fix_age_entry %lu fix_age now %lu charsAvailable %d" EOL,
              fix_age_entry, fix_age, charsAvailable);
}

// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
void checkUpdateTimeFromGps(uint32_t dollarStar_millis) {
    static bool     forceUpdate     = true;
    static uint64_t lastUpdate_millis = 0;
    static uint64_t lastCheck_millis  = 0;
    static uint32_t timeUpdateCnt    = 0;

    // positive skew. make negative when adjusting millis time.
    static uint32_t bestGuessSkewFromPPS = 100;
    uint32_t fix_age_entry = gps.time.age();
    // -------------------------------------------------------------------------
    // Early-out gate 1: GPS year/date/time validity
    // -------------------------------------------------------------------------
    // can't do anything if this isn't good!
    if (!gpsTimeIsUsableForUpdate()) return;
    // -------------------------------------------------------------------------
    // Early-out gate 2: rate limit -- once per burst at most.
    // We modified TinyGPS to only use GGA to commit time so we always see
    // the first sentence in the burst (most accurate timestamp).
    // -------------------------------------------------------------------------
    uint32_t elapsed_millis1 = millis() - lastCheck_millis;
    lastCheck_millis = millis();
    // but this burst max time is slightly bigger than the burst interval
    // so we'll only update every other at most?
    if (elapsed_millis1 < GPS_WAIT_FOR_NMEA_BURST_MAX) return;
    // -------------------------------------------------------------------------
    // Decide whether this is a forced update.
    // Force at least every burst. (1 or 5 secs?) With 2 wsprs plus cw, this could
    // otherwise be delayed up to every 5-6 minutes easily.
    // -------------------------------------------------------------------------
    uint32_t elapsed_millis2 = millis() - lastUpdate_millis;
    forceUpdate = elapsed_millis2 > GPS_WAIT_FOR_NMEA_BURST_MAX;
    if (forceUpdate && timeUpdateCnt != 0) {
        V1_printf("setTime forceUpdate. elapsed_millis2 %lu" EOL, elapsed_millis2);
    }

    // -------------------------------------------------------------------------
    // Early-out gate 3: fix_age must be acceptable.
    // Try to get as close to the NMEA timestamp as possible.
    // -------------------------------------------------------------------------
    if (!fixAgeIsAcceptable(forceUpdate, timeUpdateCnt)) return;
    // -------------------------------------------------------------------------
    // Early-out gate 4: hundredths must be 0.
    // PPS skew seems wrong often if not -- it goes to zero once we get a fix
    // and stays 0 for the repeated broadcast.
    // -------------------------------------------------------------------------
    uint8_t gps_hundredths = gps.time.centisecond();
    if (gps_hundredths > 0) {
        V1_printf("ERROR: won't setTime ..non-zero gps_hundredths %u ..PPS skew often bad" EOL,
                  gps_hundredths);
        printGpsDateTime(gps.date, gps.time, true);
        V1_print(F(EOL));
        return;
    }

    // -------------------------------------------------------------------------
    // Snapshot all GPS fields together (they should be stable as we collect).
    // -------------------------------------------------------------------------
    uint16_t gps_year   = gps.date.year();
    uint8_t  gps_month  = gps.date.month();
    uint8_t  gps_day    = gps.date.day();
    uint8_t  gps_hour   = gps.time.hour();
    uint8_t  gps_minute = gps.time.minute();
    uint8_t  gps_second = gps.time.second();

    // Snapshot the system time at the same moment for the drift comparison.
    // https://stackoverflow.com/questions/6636793/what-are-the-general-rules-for-comparing-different-data-types-in-c
    time_t t = now();
    // uint16_t y = (uint16_t) year(t);
    // uint8_t m = (uint8_t) month(t);
    uint8_t d  = (uint8_t) day(t);
    uint8_t hh = (uint8_t) hour(t);
    uint8_t mm = (uint8_t) minute(t);
    uint8_t ss = (uint8_t) second(t);

    // okay to just compare monthSecs and not roll up into total seconds.
    uint32_t monthSecs =
        (d * 24 * 3600) + (hh * 3600) + (mm * 60) + ss;
    uint32_t gps_monthSecs =
        (gps_day * 24 * 3600) + (gps_hour * 3600) + (gps_minute * 60) + gps_second;

    // UPDATE: always update time if we got this far (we'll only do this once
    // per sentence burst -- might have better skew numbers).
    // if ((!forceUpdate) && y == gps_year && m == gps_month && monthSecs == gps_monthSecs) return;

    // -------------------------------------------------------------------------
    // Early-out gate 5: range-check the GPS fields.
    // Not strictly an early-out if forceUpdate -- we proceed anyway, but
    // solar calcs will be wrong if date is wrong.
    // FIX! don't use it? or use it? solar calcs will be wrong if date is wrong.
    // -------------------------------------------------------------------------
    bool dateTimeBad = gpsDateTimeIsBad(gps_hour, gps_minute, gps_second,
                                        gps_day, gps_month, gps_year);
    if (dateTimeBad && !forceUpdate) return;

    // -------------------------------------------------------------------------
    // Compute PPS-based skew correction and commit the time update.
    // year can be given as full 4-digit or 2-digit (2010 or 10 for 2010);
    // it is converted to years since 1970.
    // -------------------------------------------------------------------------
    // setTime_millis is extern
    setTime_millis = dollarStar_millis;
    updateBestGuessSkewFromPPS(setTime_millis, &bestGuessSkewFromPPS, forceUpdate);

    // pushes back the Time prevMillis (captured by setTime) to align more
    // with when the gps chip sent out the time NMEA sentence. Probably have
    // to do this closely after setTime.
    //
    // Adjust less with USE_DOLLAR_TIME_MODE because it's time at the
    // beginning of the NMEA sentence, closer to the PPS edge (real time).
    if (bestGuessSkewFromPPS != 0) {
        // V1_printf("Adjusting setTime_millis %lu with bestGuessSkewFromPPS %lu" EOL,
        //     setTime_millis, bestGuessSkewFromPPS);
        setTime_millis -= bestGuessSkewFromPPS;
    }

    // setTime_millis should always be before or = current millis()
    setTimeWithMillis(gps_hour, gps_minute, gps_second,
                      gps_day, gps_month, gps_year,
                      setTime_millis);

    // -------------------------------------------------------------------------
    // (Disabled) verbose pre/post snapshot. Left in for debug toggling.
    // -------------------------------------------------------------------------
    if (false) {
        V1_print(F("GOOD: system setTime() with"));
        // V1_printf(" %u gps_month %u gps_year %u",
        //     gps_month, gps_year);
        V1_printf(" gps_day %u gps_hour %u gps_minute %u gps_second %u" EOL,
                  gps_day, gps_hour, gps_minute, gps_second);

        V1_print(F("system time before: (should be gps time):"));
        // V1_printf(" month %d year %d", m, y);
        V1_printf(" day %d hour %d minute %d second %d", d, hh, mm, ss);
        V1_printf(" forceUpdate %u now: ", forceUpdate);
        // this will be current system time
        printSystemDateTime();
        V1_print(F(EOL));
    }

    // -------------------------------------------------------------------------
    // Drift report (covers minute transitions via monthSecs comparison).
    // Don't print the first time through since that doesn't matter.
    // -------------------------------------------------------------------------
    int secondDelta = ((int) monthSecs) - ((int) gps_monthSecs);
    reportDrift(secondDelta, forceUpdate, timeUpdateCnt);
    // -------------------------------------------------------------------------
    // Diagnostic: rx fifo backup at this point. Might give an indication of
    // how long it takes to do all this work.
    // -------------------------------------------------------------------------
    reportRxFifoBackup(fix_age_entry);
    // -------------------------------------------------------------------------
    // Bookkeeping
    // -------------------------------------------------------------------------
    lastUpdate_millis = millis();
    forceUpdate = false;
    timeUpdateCnt += 1;
    V1_print(EOL);
    // FIX! does the uart rx fifo get backed up when we update and do all this printing?
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
    // don't have valid bits, just enum encoded for invalid
    // enum fq {Invalid = '0', GPS = '1', DGPS = '2', PPS = '3', RTK = '4', 
    //  FloatRTK = '5', Estimated = '6', Manual = '7', Simulated = '8' };
    // enum fm { N = 'N', A = 'A', D = 'D', E = 'E'};

    // can use this? gpsDebug will print the char?
    // is there no isValid() for these?
    bool validI = (gps.location.FixQuality() != TinyGPSLocation::Quality::Invalid);
    bool validJ = (gps.location.FixMode() != TinyGPSLocation::Mode::N);

    // can compare char and int? 
    // this includes Estimated as valid?
    // bool validI = gps.location.FixQuality() != 0;
    // bool validJ = gps.location.FixMode() != 'N';

    V1_printf("gps valids: %u %u %u %u %u %u %u %u %u %u %u" EOL,
        !GpsInvalidAll, validA, validB, validC, validD, validE, validF, validG, validH, validI, validJ);

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
    printStr("Degs.", true, 6);
    printStr("Course", true, 7);
    printStr("Speed", true, 6);
    printStr("ChrsRx", true, 10);
    printStr("SentsWfix", true, 10);
    printStr("failCksum", true, 10);
    printStr("FixQual", true, 7);
    printStr("FixMode", true, 7);

    V1_print(F(EOL));

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
        printFloat(gps.course.deg(), validE, 6, 2);
        printStr(TinyGPSPlus::cardinal(gps.course.value()), validE, 7);
        printFloat(gps.speed.knots(), validF, 6, 2);
        // FIX! does this just wrap wround if it's more than 6 digits?
        printInt(gps.charsProcessed(), true, 10);
        // FIX! does this just wrap wround if it's more than 10 digits?
        printInt(gps.sentencesWithFix(), true, 10);
        printInt(gps.failedChecksum(), true, 10);

        // FIX! use the enums?
        // enum Quality { Invalid = '0', GPS = '1', DGPS = '2', PPS = '3', RTK = '4', FloatRTK = '5', Estimated = '6', Manual = '7', Simulated = '8' };
        // enum Mode { N = 'N', A = 'A', D = 'D', E = 'E'};
        char fq[2] = { 0 };
        fq[0] = gps.location.FixQuality();
        printStr(fq, true, 7);

        char fm[2] = { 0 };
        fm[0] = gps.location.FixMode();
        printStr(fm, true, 7);
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
// 2) Enable the ON_OFF pin (HIGH signal from an STM32 - powered by separate 3.0V LDO linear regulator);
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
