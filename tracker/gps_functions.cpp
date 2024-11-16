// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

//*******************************************
// Reference docs (for SIM28 but should apply)
// can download all from https://simcom.ee/documents/?sort_by=size&sort_as=desc&dir=SIM28/
// SIM28M
// can download all from https://simcom.ee/documents/?sort_by=size&sort_as=desc&dir=SIM28M/
// SIM28ML
// can download all from https://simcom.ee/documents/?sort_by=size&sort_as=desc&dir=SIM28ML/

// my local copy
// xdg-open /home/kevin/Downloads/'SIM28@SIM68R@SIM68V_NMEA Messages Specification_V1.00.pdf'
// according to
// https://forum.arduino.cc/t/configuration-of-chinese-atgm336-gnss/1265640

//*******************************************
// ATGM336H uses AT6558 silicon ??
// AT6558 BDS/GNSS Full Constellation SOC Chip Data Sheet Version 1.14
// AT6558-5N-3X is GPS + BDS
// QFN package 40 pin 5x5x0.8mm
// https://www.icofchina.com/d/file/xiazai/2016-12-05/b1be6f481cdf9d773b963ab30a2d11d8.pdf


//*******************************************
// 'SIM28_Hardware Design_V1.07.pdf'
// https://simcom.ee/documents/SIM28/SIM28_Hardware%20Design_V1.07.pdf

// 'SIM28@SIM68R@SIM68V_NMEA Messages Specification_V1.00.pdf'
// https://simcom.ee/documents/SIM28/SIM28%40SIM68R%40SIM68V_NMEA%20Messages%20Specification_V1.00.pdf

// 'MT3339 Platform NMEA Message Specification_V1.00.pdf'
// https://simcom.ee/documents/SIM28/MT3339%20Platform%20NMEA%20Message%20Specification_V1.00.pdf

// 'SIM28 Specification_1407.pdf'
// https://simcom.ee/documents/SIM28/SIM28%20Specification_1407.pdf

// 'SIM28ML SPEC_V1604.pdf'
// https://simcom.ee/documents/SIM28ML/SIM28ML%20SPEC_V1604.pdf
// 'SIM28M SPEC 140716.pdf'

// 'SIM28 reference design with passive antenna(SAW+LNA+SAW).pdf'
// https://simcom.ee/documents/SIM28/SIM28%20reference%20design%20with%20passive%20antenna%28SAW%2BLNA%2BSAW%29.pdf

// 'SIM28M_Hardware Design_V1.00.pdf'
// https://simcom.ee/documents/SIM28M/SIM28M_Hardware%20Design_V1.00.pdf
// 'SIM28SIM68V_SMT_Application Note_V1.00.pdf'
// 'https://simcom.ee/documents/SIM28/SIM28SIM68V_SMT_Application%20Note_V1.00.pdf'

// Others:
// old: MTK NMEA Packet User Manual Revision 0.3 2006/05/02
// https://www.sparkfun.com/datasheets/GPS/Modules/PMTK_Protocol.pdf

// 2016.05.30 revision 1.2 GlobalTop Tech Inc
// has GPS and GLONASS BEIDOU GALILEO
// https://cdn.sparkfun.com/assets/parts/1/2/2/8/0/PMTK_Packet_User_Manual.pdf

// 2017-07-11 specification v1.03
// SIM868_NME_Message_Specification_V1.03
// https://simcom.ee/documents/SIM868E/SIM868_NMEA%20Message%20Specification_V1.03.pdf

//*******************************************
// Printing too much
// Many programmers run into trouble because they try to print too much debug info. The Arduino Serial.print function will "block" until those output characters can be stored in a buffer. While the sketch is blocked at Serial.print, the GPS device is probably still sending data. The input buffer on an Arduino is only 64 bytes, about the size of one NMEA sentence. After 64 bytes have been received stored, all other data is dropped! Depending on the GPS baud rate and the Serial Monitor baud rate, it may be very easy to lose GPS characters.

// It is crucial to call gps.available( gps_port ) or serial.read() frequently, and never to call a blocking function that takes more than (64*11/baud) seconds. If the GPS is running at 9600, you cannot block for more than 70ms. If your debug Serial is also running at 9600, you cannot write more than 64 bytes consecutively (i.e., in less than 70ms).

#include <Arduino.h>
// for isprint()
#include <ctype.h>
#include "gps_functions.h"
#include "debug_functions.h"
#include "defines.h"

// in libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// for setTime()
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time

// need this file to be .cpp because this uses type class
// for Watchdog.*
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

#include "led_functions.h"

// in the *ino
// how to reference?
// should we declare it here?
// refer to gps.* stuff?
// doesn't work

bool ublox_high_alt_mode_enabled = false;
extern TinyGPSPlus gps;
extern bool DEVMODE;

extern const int GpsPwr;
extern const int GPS_NRESET_PIN;
extern const int GPS_ON_PIN;

// input..not used..calibration?
extern const int GPS_1PPS_PIN;

extern const int GPS_UART1_RX_PIN;
extern const int GPS_UART1_TX_PIN;

extern const int SERIAL2_FIFO_SIZE;
extern const int SERIAL2_BAUD_RATE;

// for tracking gps fix time. we only power gps on/off..we don't send it gps reset commands
extern absolute_time_t GpsStartTime;  // usecs

extern char _verbosity[2];

// FIX! gonna need an include for this? maybe note
// # include <TimeLib.h>

// ************************************************
static bool GpsIsOn_state = false;
bool GpsIsOn(void) {
    return GpsIsOn_state;
}

//************************************************
// Is the maximum length of any in or out packet = 255 bytes?
#define NMEA_BUFFER_SIZE 8 * 255
static char nmeaBuffer[NMEA_BUFFER_SIZE] = { 0 };

// Outputs the content of the nmea buffer to stdio (UART and/or USB)
void nmeaBufferPrintAndClear(void) {
    if (nmeaBuffer[0] != 0) {
        // don't add an EOL to the print since we can accumulate multiple to look good?
        // Might have been missing a EOL. Add one
        Serial.println(nmeaBuffer);
        nmeaBuffer[0] = 0;  // Clear the buffer
    }
    // whenever something might have taken a long time like printing the big buffer
    updateStatusLED();
    Watchdog.reset();
}

// add one char at a time
void nmeaBufferAndPrint(const char charToAdd, bool printIfFull) {
    // we might add a EOL before a '$' that begins a sentence. so check for +2
    // EOL might be /r /n or /r/n (two chars). so check for 3.
    // possible 2 in front. 0 null term at end
    if ( (strlen(nmeaBuffer) + 3) >= NMEA_BUFFER_SIZE) {
        // make NMEA_BUFFER_SIZE bigger or
        // can just do more nmeaBufferPrint() if we run into a problem realtime
        // we shouldn't have to add EOL to the sentences. they come with CR LF ?
        Serial.printf(
            "WARNING: with NMEA_BUFFER_SIZE %d strlen(nmeaBuffer) %d "
            "there is no room for char %c <newline>",
            NMEA_BUFFER_SIZE, strlen(nmeaBuffer), charToAdd);
        Serial.println(F("..flushing by emptying first (no print)"));
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
void sleepForMilliSecs(int n, bool enableEarlyOut) {
    // FIX! should we do this here or where?
    Watchdog.reset();
    if (n < 0 || n > 5000) {
        Serial.printf("ERROR: sleepForMilliSecs() n %d too big. Using 1000" EOL, n);
        n = 1000;
    }
    int milliDiv = n / 10;

    // sleep approx. n secs
    for (int i = 0; i < milliDiv ; i++) {
        if (enableEarlyOut) {
            if (Serial2.available()) break;
        }
        // https://docs.arduino.cc/language-reference/en/functions/time/delay/
        // check for update every 10 milliseconds
        if ((milliDiv % 10) == 0) updateStatusLED();

        // faster recovery with delay?
        delay(10);
    }
}

//************************************************
int checkGpsBaudRate(int desiredBaud) {
    int usedBaud = desiredBaud;
    switch (desiredBaud) {
        case 9600: break;
        case 19200: break;
        case 39400: break;
        default: usedBaud = 9600;
    }
    return usedBaud;
}

//************************************************
void checkInitialGpsOutput(void) {
    if (DEVMODE) Serial.println(F("checkInitialGpsOutput START"));

    // FIX! rely on watchdog reset in case we stay here  forever?
    if (DEVMODE) Serial.println(F("drain any Serial2 garbage first"));
    // drain any initial garbage
    while (Serial2.available()) {Serial2.read();}
    if (DEVMODE) Serial.println(F("now look for some Serial2 bytes"));

    int i;
    char incomingChar = { 0 };
    // we drain during the GpsINIT now, oh. we should leave Gps ON so we get chars
    for (i = 0; i < 5; i++) {
        Watchdog.reset();
        if (!Serial2.available()) {
            Serial.println(F("no Serial2.available() ..sleep and reverify"));
        }
        else {
            while (Serial2.available()) {
                incomingChar = Serial2.read();
                // buffer it up like we do normally below, so we can see sentences
                nmeaBufferAndPrint(incomingChar, true); // print if full
            }
        }
        sleepForMilliSecs(2000, true); // return early if Serial2.available()
    }
    nmeaBufferPrintAndClear();
    updateStatusLED();
    Watchdog.reset();
    if (DEVMODE) Serial.println(F("checkInitialGpsOutput END"));
}

//************************************************
void setGpsBalloonMode(void) {
    if (DEVMODE) Serial.println(F("setGpsBalloonMode START"));
    // FIX! should we not worry about setting balloon mode (3) for ATGM336?
    // Serial2.print("$PSIMNAV,W,3*3A\r\n");
    // normal mode
    // Serial2.print("$PSIMNAV,W,0*39\r\n");
    // have to wait for the sentence to get out, and also complete
    // sleepForMilliSecs(1000, false);
    if (DEVMODE) Serial.println(F("setGpsBalloonMode END"));
}

void setGpsBaud(int desiredBaud) {
    // FIX! works 9600. doesn't work any other baud rate?
    if (DEVMODE) Serial.printf("setGpsBaud START %d" EOL, desiredBaud);
    updateStatusLED();
    Watchdog.reset();
    // after power on, start off talking at 9600 baud. when we change it

    // have to send CR and LF and correct checksum
    // CR and LF are in defines.h. they are not part of the checksum, nor is the $
    // Example: $PMTK251,38400*27<CR><LF>
    // just pre-calculate the checksums here and hardwire them in the static sentences used.
    // https://www.meme.au/nmea-checksum.html
    // should just get legal ones here
    int usedBaud = checkGpsBaudRate(desiredBaud);
    char nmeaBaudSentence[21] = { 0 };
    switch (usedBaud) {
        // this is default. must be 9600?
        // should it be the p
        // weird: what is this case gonna do?
        // case 0:      strncpy(nmeaBaudSentence, "$PMTK251,0*28" CR LF, 21); break;
        /*
        case 9600:   strncpy(nmeaBaudSentence, "$PMTK251,9600*17" CR LF, 21); break;
        case 19200:  strncpy(nmeaBaudSentence, "$PMTK251,19200*22" CR LF, 21); break;
        case 39400:  strncpy(nmeaBaudSentence, "$PMTK251,38400*27" CR LF, 21); break;
        case 57600:  strncpy(nmeaBaudSentence, "$PMTK251,57600*2C" CR LF, 21); break;
        case 115200: strncpy(nmeaBaudSentence, "$PMTK251,115200*1F" CR LF, 21); break;
        */

        case 4800:   strncpy(nmeaBaudSentence, "$PCAS01,0*1C" CR LF, 21); break;
        // this worked
        case 9600:   strncpy(nmeaBaudSentence, "$PCAS01,1*1D" CR LF, 21); break;
        // this worked
        case 19200:  strncpy(nmeaBaudSentence, "$PCAS01,2*1E" CR LF, 21); break;
        // this didn't work?
        case 38400:  strncpy(nmeaBaudSentence, "$PCAS01,3*1F" CR LF, 21); break;
        case 57600:  strncpy(nmeaBaudSentence, "$PCAS01,4*18" CR LF, 21); break;
        case 115200: strncpy(nmeaBaudSentence, "$PCAS01,5*19" CR LF, 21); break;

        default:
            usedBaud = 9600;
            // strncpy(nmeaBaudSentence, "$PMTK251,9600*17" CR LF, 21);
            strncpy(nmeaBaudSentence, "$PCAS01,1*1D" CR LF, 21);
    }
    Serial2.print(nmeaBaudSentence);
    if (DEVMODE) Serial.printf("setGpsBaud for usedBaud %d, sent %s" EOL, usedBaud, nmeaBaudSentence);

    // have to wait for the sentence to get out and complete at the GPS
    delay(3000);

    // Note:
    // Serial2.end() Disables serial communication,
    // allowing the RX and TX pins to be used for general input and output.
    // To re-enable serial communication, call Serial.begin().
    Serial2.end();
    Serial2.begin(usedBaud);
    if (DEVMODE) Serial.printf("setGpsBaud did Serial2.begin(%d)" EOL, usedBaud);
    // then have to change Serial2.begin() to agree
    sleepForMilliSecs(1000, false);
    if (DEVMODE) Serial.printf("setGpsBaud END %d" EOL, usedBaud);
}

//************************************************
void GpsINIT(void) {
    if (DEVMODE) Serial.println(F("GpsINIT START"));
    updateStatusLED();
    Watchdog.reset();

    //****************
    // The gps power pin is floating? likely GPS is off but unknown
    // Should turn gps off before doing init if you know it's been
    // initted already? 

    gpio_init(GpsPwr);
    pinMode(GpsPwr, OUTPUT);
    gpio_pull_up(GpsPwr);
    gpio_put(GpsPwr, HIGH); // init with gps power on?
    Serial.printf("GpsPwr %d (power off)" EOL, GpsPwr);

    //****************
    // FIX! are these doing anything?
    // can you turn 'run' off for lower power mode? don't use, rely on vbat for gps hot reset
    gpio_init(GPS_NRESET_PIN);
    pinMode(GPS_NRESET_PIN, OUTPUT);
    gpio_pull_up(GPS_NRESET_PIN);
    gpio_put(GPS_NRESET_PIN, HIGH);

    gpio_init(GPS_ON_PIN);
    pinMode(GPS_ON_PIN, OUTPUT);
    gpio_pull_up(GPS_ON_PIN);
    gpio_put(GPS_ON_PIN, HIGH);

    // FIX! is this necessary?
    digitalWrite(GPS_NRESET_PIN, HIGH);
    digitalWrite(GPS_ON_PIN, HIGH);
    //****************

    if (DEVMODE) {
        Serial.printf("GPS_UART1_RX_PIN %d" EOL, GPS_UART1_RX_PIN);
        Serial.printf("GPS_UART1_TX_PIN %d" EOL, GPS_UART1_TX_PIN);
        Serial.printf("(gpio) GpsPwr %d" EOL, GpsPwr);
        Serial.printf("(gpio) GPS_NRESET_PIN %d" EOL, GPS_NRESET_PIN);
        Serial.printf("(gpio) GPS_ON_PIN %d" EOL, GPS_ON_PIN);
    }

    Serial2.setRX(GPS_UART1_RX_PIN);
    Serial2.setTX(GPS_UART1_TX_PIN);
    Serial2.setPollingMode(true);

    //****************
    Serial2.end();
    // try making bigger (see tracker.ino)
    Serial2.setFIFOSize(SERIAL2_FIFO_SIZE);
    // first talk at 9600
    Serial2.begin(9600);
    sleepForMilliSecs(500, false);
    // drain the rx buffer. GPS is off, but shouldn't keep
    while (Serial2.available()) Serial2.read();
    gpio_put(GpsPwr, LOW); // leave INIT with gps power on
    // sleep 3 secs
    sleepForMilliSecs(3000, false);

    //****************
    checkInitialGpsOutput();
    setGpsBalloonMode();

    // FIFO is big enough to hold output while we send more input here
    int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
    // then up the speed to desired (both gps chip and then Serial2
    setGpsBaud(desiredBaud);
    Serial.println(F("Should get some GPS output now at the new baud rate"));
    checkInitialGpsOutput();

    if (DEVMODE) Serial.println(F("GpsINIT END"));
}

//************************************************
void GpsON(bool GpsColdReset) {
    // FIX! do the cold reset regardless of current state? I think no?
    // so cold reset is only done if the GPS is currently off?..for the off/on transition?
    if (DEVMODE) Serial.println(F("GpsON START"));

    if (GpsColdReset) {
        if (DEVMODE) Serial.printf("GpsON GpsIsOn_state %u GpsColdReset true" EOL, GpsIsOn_state);
    }
    else {
        if (DEVMODE) Serial.printf("GpsOn GpsIsOn_state %u GpsColdReset false" EOL, GpsIsOn_state);
    }
    // could be off or on already
    // Assume GpsINIT was already done

    // Just in case: wait for serial port to connect.
    // do we need these two each time?
    // yes if we did a Serial2.end() when off

    Watchdog.reset();
    updateStatusLED();
    if (!GpsColdReset && !GpsIsOn()) {
        setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
        GpsStartTime = get_absolute_time();  // usecs
        // always do this just in case the GpsIsOn() got wrong?
        // but we're relying on the Serial2.begin/end to be correct?
        // might as well commit to being right!
        digitalWrite(GpsPwr, LOW);
        // don't know what baud rate it was at. gps comes up at 9600
        Serial.end();
        Serial.begin(9600);
        // wait 2 seconds for normal power before sending more commands
        sleepForMilliSecs(2000, false);
        setGpsBalloonMode();
        // resets to 9600. set to new baud rate
        // FIFO is big enough to hold output while we send more input here
        int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
        // then up the speed to desired (both gps chip and then Serial2
        setGpsBaud(desiredBaud);
        checkInitialGpsOutput();
    }


    if (GpsColdReset) {
        if (DEVMODE) Serial.println(F("GpsON full cold reset START"));
        // turn it off first. may be off or on currently
        // turn off the serial
        Serial.end();
        Serial.flush();

        digitalWrite(GpsPwr, HIGH);
        setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
        sleepForMilliSecs(1000, false);
        // Cold Start. doesn't clear any system/user configs
        // Serial2.print("$PMTK103*30\r\n");
        // Full Cold Start. any system/user configs (back to factory status)
        // FIX! should we wait for ack or no?
        // have to toggle power off/on to get this effect? no?

        // always do this just in case the GpsIsOn() got wrong?
        // but we're relying on the Serial2.begin/end to be correct?
        // might as well commit to being right!
        //******************
        digitalWrite(GpsPwr, LOW);
        sleepForMilliSecs(1000, false);

        // don't know what baud rate it was at. gps comes up at 9600
        Serial.begin(9600);
        sleepForMilliSecs(1000, false);

        // send it twice??
        Serial.print("$PMTK104*37" CR LF);
        Serial.print("$PMTK104*37" CR LF);
        sleepForMilliSecs(1000, false);

        // FIX! do we have to toggle power off/on to get the cold reset?
        digitalWrite(GpsPwr, HIGH);
        sleepForMilliSecs(1000, false);
        digitalWrite(GpsPwr, LOW);
        sleepForMilliSecs(1000, false);

        // FIX! we don't need to toggle power to get the effect?
        setGpsBalloonMode();
        GpsStartTime = get_absolute_time();  // usecs

        //******************
        // resets to 9600. set to new baud rate
        // FIFO is big enough to hold output while we send more input here
        int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
        // then up the speed to desired (both gps chip and then Serial2
        setGpsBaud(desiredBaud);
        checkInitialGpsOutput();
        //******************
        // resets to 9600. set to new baud rate
        if (DEVMODE) Serial.println(F("GpsON full cold reset END"));
    }

    GpsIsOn_state = true;

    if (GpsColdReset) {
        if (DEVMODE) Serial.printf("GpsON END GpsIsOn_state %u GpsColdReset true" EOL, GpsIsOn_state);
    }
    else {
        if (DEVMODE) Serial.printf("GpsOn END GpsIsOn_state %u GpsColdReset false" EOL, GpsIsOn_state);
    }
}


//************************************************
/*
This used to be in the LightAPRS version of TinyGPSPlus-0.95
instead updated TinyGPSPlus (latest) in libraries to make them public, not private
< #if defined(ARDUINO_ARCH_RP2040)
< void TinyGPSDate::clear()
< {
<    valid = updated = false;
<    date = 0;
< }b
< #endif
*/

//************************************************
void GpsOFF() {
    if (DEVMODE) Serial.printf("GpsOFF START GpsIsOn_state %u" EOL, GpsIsOn_state);

    digitalWrite(GpsPwr, HIGH);
    // Serial2.end()
    // Disables serial communication, allowing the RX and TX pins to be used for general input and output.
    // To re-enable serial communication, call Serial.begin().
    // FIX! do we really need or want to turn off Serial2? Remember to Serial2.begin() when we turn it back on
    // (only if it was off)
    Serial2.end();

    // gps.date.clear(); // kazu had done this method
    // are these declared private?
    // FIX! how can we clear these? Do we change the library to make them public?

    /* from TinyGPS++.h in libraries..modify it and move to public? (in struct TinyGPSDate
    what about location etc? they have valid and updated

    private:
       bool valid, updated;
       uint32_t date, newDate;
       uint32_t lastCommitTime;
       void commit();
       void setDate(const char *term);

    TinyGPS++.h:   bool valid, updated;
    */

    // these three are the initial value
    gps.date.valid = false;
    gps.date.updated = false;
    gps.date.date = 0;

    // also has lastCommitTime = millis()
    // we don't change that?

    ublox_high_alt_mode_enabled = false;

    GpsIsOn_state = false;
    GpsStartTime = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    if (DEVMODE) Serial.printf("GpsOFF END GpsIsOn_state %u" EOL, GpsIsOn_state);
}

//************************************************
// FIX! why was this static void before?
void updateGpsDataAndTime(int ms) {
    uint64_t current_millis = millis();
    uint64_t entry_millis = current_millis;

    Watchdog.reset();
    // ms has to be positive?
    // grab data for no more than ms milliseconds
    // stop if no data for 50 milliseconds
    // all the durations below won't start counting until we get the first char (sets start_millis())
    uint64_t start_millis = 0;
    uint64_t last_serial2_millis = 0;
    uint64_t timeSinceLastChar_millis = 0;
    uint64_t duration_millis = 0;


    // drain the buffer, so we start with empty, so we aren't behind below
    // i.e as long as we keep up, we'll be okay
    // the rx buffer only has 32 entries! So we should never have to drain more than 32
    // it's key that we can drain here faster than uart can fill!
    // this is to avoid the ERROR msgs about the rx buffer below
    int drainCnt = 0;
    while (Serial2.available() > 0) { 
        drainCnt++;
        Serial2.read();
        if (drainCnt >= 32) { break; }
    }


    if (DEVMODE) Serial.println(F("updateGpsDataAndTime START"));
    if (DEVMODE & (_verbosity[0] >= '8'))
        Serial.printf("updateGpsDataAndTime started looking for NMEA current_millis %" PRIu64 EOL, current_millis);

    // clear the StampPrintf buffer, in case it had anything.
    DoLogPrint();

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
    static int sentenceEndCnt = 0;

    // unload each char to TinyGps++ object as it arrives and print it (actually into nmeaBuffer)
    // so we can see NMEA sentences for a period of time.
    // assume 1 sec broadcast rate
    // https://arduino.stackexchange.com/questions/13452/tinygps-plus-library

    // Could keep the sum for all time, and track total busy time..
    // but I think better not to average..just track this particular call.
    int incomingCharCnt = 0; 

    bool stopPrinting = false;
    bool last_stopPrinting = false;
    bool nullChar = false;
    bool spaceChar = false;
    bool notprintable = false;
    do {
        current_millis = millis();
        char incomingChar;
        while (Serial2.available() > 0) {
            // start the duration timing when we get the first char
            if (start_millis==0) start_millis = current_millis;

            // can't have the logBuffer fill up, because the unload is delayed
            int charsAvailable = (int) Serial2.available();
            if (DEVMODE) {
                if (charsAvailable > 20)
                    // might lose some if we can't keep up
                    // we were under 12 here for 9600 baud
                    // at 19200 baud, we hit 20 chars here with no checksum errors
                    StampPrintf("ERROR: NMEA incoming chars are backing up: 32 deep uart rx buffer has %d)" EOL,
                        (int) charsAvailable);
            }

            incomingChar = Serial2.read();
            // always send everything to TinyGPS++ ??
            // does it expect the CR LF ?
            gps.encode(incomingChar);
            // we count all chars, even CR LF etc
            incomingCharCnt++;
            // do we get any null chars?
            // are CR LF unprintable?
            stopPrinting = last_stopPrinting;
            spaceChar = false;
            nullChar = false;
            notprintable = !isprint(incomingChar);
            switch (incomingChar) {
                case '$':  sentenceStartCnt++; stopPrinting = false; break;
                case '*':  sentenceEndCnt++; stopPrinting = false; break;
                case '\n': stopPrinting = true; break;
                case '\r': stopPrinting = true; break;
                // don't change the stopPrinting flow if get a unprintable or these
                case '\0': nullChar = true; break;
                case ' ':  spaceChar = true; break;
                default: { ; }
            }
            // always strip these here
            bool enableStripping = true;
            if (enableStripping && (spaceChar || nullChar || notprintable)) continue;

            // stopPrinting: don't put CR LF in the nmeaBuffer. will add one on the transition
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

            // FIX! could disable the GPTXT broadcase to reduce data broadcast

            // make the nmeaBuffer big enough so that we never print while getting data?
            // and we never throw it away (lose data) ?? (for debug only though)
            // this should eliminate duplicate CR LF sequences and just put one in the stream
            // FIX! might be odd if a stop is spread over two different calls here?
            // always start printing again on inital call to this function (see inital state)
            if (DEVMODE & (_verbosity[0] >= '8')) {
                if (enableStripping && 
                        (last_stopPrinting && !stopPrinting && !notprintable && !nullChar && !spaceChar)) {
                    // false: don't print if full, just empty
                    nmeaBufferAndPrint('\r', false); 
                    nmeaBufferAndPrint('\n', false);
                }
            }

            // FIX! do we get any unprintable? ignore unprintable chars. 
            if (!enableStripping || 
                (!stopPrinting && !notprintable && !nullChar && !spaceChar)) {
                // FIX! can we not send CR LF? don't care. but performance-wise, might be good?
                // moved above
                // gps.encode(incomingChar);
                if (DEVMODE & (_verbosity[0] >= '8')) {
                    nmeaBufferAndPrint(incomingChar, false); 
                }
            }

            current_millis = millis();
            last_serial2_millis = current_millis;
            last_stopPrinting = stopPrinting;
        }

        // did we wait more than 50 millis() since good data read?
        // we wait until we get at least one char or go past the ms total wait
        // break out when we don't the next char right away 
        updateStatusLED();

        if (last_serial2_millis == 0) timeSinceLastChar_millis = 0;
        else timeSinceLastChar_millis = current_millis - last_serial2_millis;

        // FIX! should the two delays used be dependent on baud rate?
        if (timeSinceLastChar_millis >= 25) {
            // FIX! could the LED blinking have gotten delayed? ..we don't check in the available loop above.
            // save the info in the StampPrintf buffer..don't print it yet
            duration_millis = current_millis - start_millis;
            if (DEVMODE) StampPrintf(
                "updateGpsDataAndTime early out: ms %d " 
                "loop break at %" PRIu64 " millis,  duration %" PRIu64  EOL,
                 ms, current_millis, duration_millis);
            break;
        }

        // sleep for 50 milliseconds? will we get buffer overflow?
        // 32 symbols at 9600 baud = 33 milliseconds?
        // shouldn't sleep here..faster to just delay
        // I guess here we're trying to sync with a burst? but how long to wait?
        // if we just completed a burst, we should wait for 1 sec - total burst delay?
        sleepForMilliSecs(25, true); // stop the wait early if symbols arrive

    } while ( (current_millis - entry_millis) < (uint64_t) ms); // works if ms is 0


    // FIX! condition some of these with verbosityj
    // print/clear any accumulated NMEA sentence stuff
    if (DEVMODE & (_verbosity[0] >= '8')) {
        // print should only get dumped here?
        nmeaBufferPrintAndClear(); // print and clear
        Serial.print(F(EOL));
        // dump/flush the StampPrintf log_buffer
        DoLogPrint();
    }

    if (DEVMODE & (_verbosity[0] >= '8')) {
        // seems like we get 12 sentences every time we call this function
        // should stay steady
        int diff = sentenceStartCnt - sentenceEndCnt;
        Serial.printf(
            "start_millis %" PRIu64 " current_millis %" PRIu64 
            " sentenceStartCnt %d sentenceEndCnt %d diff %d" EOL,
            start_millis, current_millis, sentenceStartCnt, sentenceEndCnt, diff);

        // we can round up or down by adding 500 before the floor divide
        duration_millis = current_millis - start_millis;

        // This will be lower than a peak rate
        // It includes dead time at start, dead time at end...
        // With some constant rate in the middle? but sentences could be split..
        // fixed: entry_millis is the entrance to the function
        // star_millis is the first char. so duration_millis will include the end stall detect (25 millis)
        // So it's an average over that period.
        float AvgCharRateSec;
        if (duration_millis == 0) AvgCharRateSec = 0; 
        else AvgCharRateSec = 1000.0 * (((float) incomingCharCnt / (float) duration_millis));
        Serial.printf(
            "NMEA AvgCharRateSec %.1f duration_millis %" PRIu64 " incomingCharCnt %d" EOL,  
            AvgCharRateSec, duration_millis, incomingCharCnt);
    }

    if (DEVMODE & (_verbosity[0] >= '8')) {
        Serial.printf("gps.time.isValid():%u" EOL, gps.time.isValid());
    }

    if (gps.time.isValid()) {
        // Update the arduino (cpu) time. setTime is in the Time library.
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 0, 0, 0);
        if (DEVMODE & (_verbosity[0] >= '8'))
            Serial.printf("setTime(%02u:%02u:%02u)" EOL,
                gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    updateStatusLED();
    uint64_t total_millis = millis() - entry_millis;
    // will be interesting to see how much bigger this is compared to the duration_millis
    if (DEVMODE) Serial.printf("updateGpsDataAndTime END total_millis %" PRIu64 EOL EOL, total_millis);
}


//************************************************
// don't use the UBX chip, so just leaving this in for potential
// issues like this in other gps chips (balloon mode or ?)
// FIX!  I do send a full cold start NMEA request above, but
// don't look for an ACK..maybe long term will make it a full req/ack
// so I know it's really doing something (like these two routines)
void sendUBX(uint8_t *MSG, uint8_t len) {
    Serial2.write(0xFF);
    delay(500);
    for (int i = 0; i < len; i++) Serial2.write(MSG[i]);
}

//************************************************
// don't use the UBX chip, so just leaving this in for potential
// issues like this in other gps chips (balloon mode or ?)
boolean getUBX_ACK(uint8_t *MSG) {
    uint8_t b;
    uint8_t ackByteID = 0;
    uint8_t ackPacket[10];
    uint64_t startTime = millis();
    boolean status = false;

    // Construct the expected ACK packet
    ackPacket[0] = 0xB5;  // header
    ackPacket[1] = 0x62;  // header
    ackPacket[2] = 0x05;  // class
    ackPacket[3] = 0x01;  // id
    ackPacket[4] = 0x02;  // length
    ackPacket[5] = 0x00;
    ackPacket[6] = MSG[2];  // ACK class
    ackPacket[7] = MSG[3];  // ACK id
    ackPacket[8] = 0;  // CK_A
    ackPacket[9] = 0;  // CK_B

    // Calculate the checksums
    for (uint8_t ubxi=2; ubxi < 8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
    }

    while (true) {
        // Test for success
        if (ackByteID > 9) {
            // All packets in order!
            status = true;
            break;
        }

        // Timeout if no valid response in 3 seconds
        if (millis() - startTime > 3000) {
            status = false;
            break;
        }

        // Make sure data is available to read
        if (Serial2.available()) {
            b = Serial2.read();
            // Check that bytes arrive in sequence as per expected ACK packet
            if (b == ackPacket[ackByteID]) ackByteID++;
            else ackByteID = 0;  // Reset and look again, invalid order
        }
    }
    return status;
}

//************************************************
// following GPS code from : https://github.com/HABduino/HABduino/blob/master/Software/habduino_v4/habduino_v4.ino
void setGPS_DynamicModel6() {
    int gps_set_success = 0;
    uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };

    while (!gps_set_success) {
        if (DEVMODE) {
            Serial.println(F("ublox DynamicModel6 try..."));
        }
        sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
        gps_set_success = getUBX_ACK(setdm6);
    }
}

//************************************************
void gpsDebug() {
    // https://github.com/StuartsProjects/GPSTutorial

    if (!DEVMODE) return;
    if (DEVMODE) Serial.println(F("GpsDebug START"));

    Serial.print(F(EOL EOL));
    Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars FixSents Checksum"));
    Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  RX    RX        Fail"));
    Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));

    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
    printFloat(gps.location.lat(), gps.location.isValid(), 12, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

    printInt(gps.location.age(), gps.location.isValid(), 5);
    printDateTime(gps.date, gps.time);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
    printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);
    printInt(gps.charsProcessed(), true, 6); // FIX! does this just wrap wround if it's more than 6 digits?
    printInt(gps.sentencesWithFix(), true, 10); // FIX! does this just wrap wround if it's more than 10 digits?
    printInt(gps.failedChecksum(), true, 9);

    Serial.print(F(EOL EOL));

    if (DEVMODE) Serial.println(F("GpsDebug END"));
    if (DEVMODE) Serial.println(F("GpsINIT END"));
}


//*****************
// Notes:
// Arduino IDE allows function definitions after the point they are used
// used without needing an explicit function prototype before hand.

// The Arduino build creates these prototypes but not always correctly,
// leading to errors which are not obvious.

// Example: if the function argument list contains user defined data types and
// the automatically created function prototype is placed before the declaration of that data type.

//*****************
// alternative GPS
// SIM28ML
// SIM28ML/9600 9.7x10.1mm

// ATGM336H-5N11 9.7x10.1mm
// https://www.lcsc.com/datasheet/lcsc_datasheet_2304140030_ZHONGKEWEI-ATGM336H-5N11_C90769.pdf
// 5N-1X is gps only. saw + lnda
// Ipeak = 100mA
// ATGM336H-5N31 9.7x10.1mm
// https://www.lcsc.com/datasheet/lcsc_datasheet_1810261521_ZHONGKEWEI-ATGM336H-5N31_C90770.pdf
// 5N-3X is gps + bda. saw + lna

// Ipeak = 100mA
// ATGM336H-5NR32 10.1x9.7mm
// https://www.lcsc.com/datasheet/lcsc_datasheet_2411041759_ZHONGKEWEI-ATGM336H-5NR32_C5117921.pdf
// gps + bds. lna + saw

// 6N-32 is gps + bd2/3. -74 has GLO. 115200baud
// Ipeak = 100mA
// ATGM336H-6N-74 10.1x9.7mm
// https://www.lcsc.com/datasheet/lcsc_datasheet_2401121833_ZHONGKEWEI-ATGM336H-6N-74_C5804601.pdf

// Quectel L70-RL 10.1x9.7x2.5mm
// 18ma tracking compare to max-6x
// 18 pin lcc
// # protocol 2016 https://auroraevernet.ru/upload/iblock/6e6/6e624183292772f8dad0a6c327153eff.pdf
// # presentation
// https://auroraevernet.ru/upload/iblock/4bd/4bd89e299765c46248256cf6d9b8e0a7.pdf
// 5H Quectel L70 GP* only?

// L70B-M39 C6072279 SMD-18P
// hard to find the datasheets. need user account
// https://jlcpcb.com/partdetail/QUECTEL-L70BM39/C6072279
// L70REL-M37 C5745045 SMD-18P End of Life but L70REL-M37-EIT 10.1x9.7x2.5mm is not EOL?
// https://jlcpcb.com/partdetail/Quectel-L70RELM37/C5745045
// mouser uses this datasheet (for the L70_R)
// https://www.mouser.com/datasheet/2/1052/Quectel_L70_R_GPS_Specification_V2_2-1829911.pdf
// L70RE-M37 L70REL-M37 L70REL-M37-EIT
// https://www.quectel.com/gnss-iot-modules/
/// L76 L76G single band

//*******************
// ATGM336 might not support this:
// could use as PMTK_API_SET_PWR_SAV_MODE (elsewhere) rather than powering off?
// would have to wait for ack or time, after changing
// https://www.meme.au/nmea-checksum.html
// power saving mod off: (should this really be checksum 2F?)
// $PMTK320,0*26\r\n" manual has wrong checksum here?
// power saving mod on:
// $PMTK320,1*2E\r\n"
//*******************
