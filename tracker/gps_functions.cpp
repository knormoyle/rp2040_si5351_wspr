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

// says VDD_POR going low causes internal reset (nRESET)
// nRST pin going low causes internal reset (nRESET)
// nRST can be low while power is transition on, or it can be asserted/deasserted afer power on
// tcxo_xref needs to be running

//*******************************************
// Printing too much
// Many programmers run into trouble because they try to print too much debug info. 
// The Arduino Serial.print function will "block" until those output characters can be stored in a buffer. 
// While the sketch is blocked at Serial.print, the GPS device is probably still sending data. 
// The input buffer on an Arduino is only 64 bytes, about the size of one NMEA sentence. 
// FIX! is is really just 32 bytes?
// After 64 bytes have been received stored, all other data is dropped! 
// Depending on the GPS baud rate and the Serial Monitor baud rate, it may be very easy to lose GPS characters.

// It is crucial to call gps.available( gps_port ) or serial.read() frequently, 
// and never to call a blocking function that takes more than (64*11/baud) seconds. 
// If the GPS is running at 9600, you cannot block for more than 70ms. 

#include <Arduino.h>
// for isprint()
#include <ctype.h>
#include "gps_functions.h"
#include "debug_functions.h"
#include "print_functions.h"
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

extern TinyGPSPlus gps;

extern const int GpsPwr;
extern const int GPS_NRESET_PIN; // connected!
extern const int GPS_ON_PIN;

// input..not used..calibration?
extern const int GPS_1PPS_PIN;

extern const int GPS_UART1_RX_PIN;
extern const int GPS_UART1_TX_PIN;

extern const int SERIAL2_FIFO_SIZE;
extern const int SERIAL2_BAUD_RATE;

// for tracking gps fix time. we only power gps on/off..we don't send it gps reset commands
extern absolute_time_t GpsStartTime;  // usecs

// extern char _verbose[2];
// decode of _devmode
extern bool DEVMODE;
// decode of verbose 0-9
extern bool VERBY[10];

extern uint32_t GpsInvalidAllCnt;
extern bool GpsInvalidAll;

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
        Serial.printf("ERROR: sleepForMilliSecs() n %d too big (5000 max). Using 1000" EOL, n);
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
void checkInitialGpsOutput(void) {
    if (VERBY[0]) Serial.println(F("checkInitialGpsOutput START"));

    // FIX! rely on watchdog reset in case we stay here  forever?
    if (VERBY[9]) Serial.println(F("drain any Serial2 garbage first"));
    // drain any initial garbage
    while (Serial2.available()) {Serial2.read();}
    if (VERBY[9]) Serial.println(F("now look for some Serial2 bytes"));

    int i;
    char incomingChar = { 0 };
    // we drain during the GpsINIT now, oh. we should leave gps on so we get chars
    for (i = 0; i < 1; i++) {
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
        sleepForMilliSecs(1000, true); // return early if Serial2.available()
    }
    nmeaBufferPrintAndClear();
    updateStatusLED();
    Watchdog.reset();
    if (VERBY[0]) Serial.println(F("checkInitialGpsOutput END"));
}

//************************************************
void setGpsBalloonMode(void) {
    if (VERBY[0]) Serial.println(F("setGpsBalloonMode START"));
    // FIX! should we not worry about setting balloon mode (3) for ATGM336?
    // Serial2.print("$PSIMNAV,W,3*3A\r\n");
    // normal mode
    // Serial2.print("$PSIMNAV,W,0*39\r\n");
    // have to wait for the sentence to get out, and also complete
    // sleepForMilliSecs(1000, false);
    if (VERBY[0]) Serial.println(F("setGpsBalloonMode END"));
}

//************************************************
// always GGA GLL GSA GSV RMC
// nver ZDA TXT
void setGpsBroadcast() {
    if (VERBY[0]) Serial.print(F("setGpsBroadcast START" EOL));
    updateStatusLED();
    Watchdog.reset();
    // room for a 60 char sentence with CR and LF also
    char nmeaSentence[62] = { 0 };

    //*************************************************
    // ZDA
    // this time info is in other sentences also?
    // $–ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx
    // hhmmss.ss = UTC
    // xx = Day, 01 to 31
    // xx = Month, 01 to 12
    // xxxx = Year
    // xx = Local zone description, 00 to +/- 13 hours
    // xx = Local zone minutes description (same sign as hours)

    //*************************************************
    // from the latest CASIC_ProtocolSpecification_english.pdf
    // $PCAS03 string nGGA value Message ID, sentence header

    // no 0 or 1 fields?

    // 2  GGA output frequency, statement output frequency is based on positioning update rate
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

    // 20 CSvalue Hexadecimal value checksum, XOR result of all characters between $ and * (excluding $ and *)
    // 21 <CR><LF> charactersCarriage return and line feed

    // hmm this didn't work? still got zda and ANT txt. this was a forum posting. wrong apparently
    // strncpy(nmeaSentence, "$PCAS03,1,1,1,1,1,1,0,0*02" CR LF, 21);

    // spec has more/new detail. see below
    // FIX! still getting GNZDA and GPTXT ANTENNAOPEN with this
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
    delay(1000);

    Serial2.print(nmeaSentence);
    Serial2.flush();
    delay(1000);

    if (VERBY[9]) Serial.printf("setGpsBroadcast sent %s" EOL, nmeaSentence);
    if (VERBY[0]) Serial.print(F("setGpsBroadcast END" EOL ));

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
// Indicates the model of the chip or chipset (baseband chip model ATGB03, RF chip model ATGR201)
// $GPTXT,01,01,02,SW=URANUS2,V2.2.1.0*1D
// Indicates the software name and version number (software name URANUS2, version number V2.2.1.0)
// $GPTXT,01,01,02,TB=2013-06-20,13:02:49*43
// Indicates the code compilation time (June 20, 2013, 13:02:49)
// $GPTXT,01,01,02,MO=GB*77
// Indicates the working mode of the receiver at this startup (GB indicates the dual-mode mode of GPS+BDS)
// $GPTXT,01,01,02,CI=00000000*7A
// Indicates the customer number (the customer number is 00000000)

//************************************************
void setGpsConstellations(int desiredConstellations) {
    if (VERBY[0]) Serial.printf("setConstellations START %d" EOL, desiredConstellations);
    updateStatusLED();
    Watchdog.reset();
    // FIX! should we ignore desiredConstellations and force 3 (BDS + GPS
    int usedConstellations = desiredConstellations;
    char nmeaSentence[62] = { 0 };

    switch (usedConstellations) {
        case 1: strncpy(nmeaSentence, "$PCAS04,1*18" CR LF, 62); break;
        case 2: strncpy(nmeaSentence, "$PCAS04,2*1B" CR LF, 62); break;
        case 3: strncpy(nmeaSentence, "$PCAS04,3*1A" CR LF, 62); break;
        case 4: strncpy(nmeaSentence, "$PCAS04,4*1D" CR LF, 62); break;
        case 5: strncpy(nmeaSentence, "$PCAS04,5*1C" CR LF, 62); break;
        case 6: strncpy(nmeaSentence, "$PCAS04,6*AF" CR LF, 62); break;
        case 7: strncpy(nmeaSentence, "$PCAS04,7*1E" CR LF, 62); break;
        default:
            usedConstellations = 3;
            strncpy(nmeaSentence, "$PCAS04,3*1D" CR LF, 62);
    }

    // FIX! was it not passing unless I did this?
    if (false) {
        // alternative experiment
        // what about this rumored PMTK353 sentence?
        // $PMTK353,1,1,1,0,1*2B : Search GPS BEIDOU GLONASS and GALILEO satellites
        strncpy(nmeaSentence, "$PMTK353,1,1,1,0,1*2B" CR LF, 62);
        Serial2.print(nmeaSentence);
        Serial2.flush();
        delay(1000);
    }

    if (VERBY[9]) Serial.printf("setGpsConstellations for usedConstellations %d, sent %s" EOL, desiredConstellations, nmeaSentence);
    if (VERBY[0]) Serial.printf("setGpsConstellations END %d" EOL, desiredConstellations);
}

//************************************************
void setGpsBaud(int desiredBaud) {
    // Assumes we can talk to gps already at some existing agreed
    // on Serial2/gps chip setup (setup by int/warm reset/full cold reset)
    if (VERBY[0]) Serial.printf("setGpsBaud START %d" EOL, desiredBaud);
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
    // BAUD
    switch (usedBaud) {
        // this is default. must be 9600?
        // should it be the p
        // weird: what is this case gonna do?
        // case 0:      strncpy(nmeaBaudSentence, "$PMTK251,0*28" CR LF, 21); break;
        // case 9600:   strncpy(nmeaBaudSentence, "$PMTK251,9600*17" CR LF, 21); break;
        // case 19200:  strncpy(nmeaBaudSentence, "$PMTK251,19200*22" CR LF, 21); break;
        // case 38400:  strncpy(nmeaBaudSentence, "$PMTK251,38400*27" CR LF, 21); break;
        // case 57600:  strncpy(nmeaBaudSentence, "$PMTK251,57600*2C" CR LF, 21); break;
        // case 115200: strncpy(nmeaBaudSentence, "$PMTK251,115200*1F" CR LF, 21); break;

        // PMTK_SET_NMEA_BAUDRATE per SIM28 manual: 
        // Set NMEA port baudrate. 
        // Using PMTK251 command to setup baud rate setting, the setting will be back
        // to default value in the two conditions.
        // 1. Full cold start command is issued !! reverts to 9600?
        // 2. Enter standby mode (is this when vcc off, vbat on?
        // PMTK_CMD_STANDY_MODE enters standby mode (one of two SLEEP stages)

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

    // https://forum.arduino.cc/t/solved-proper-way-to-change-baud-rate-after-initial-setup/419860/5
    // get rid of anything still in the cpu output buffer
    Serial2.flush();
    delay(1000);
    Serial2.print(nmeaBaudSentence);
    if (VERBY[9]) Serial.printf("setGpsBaud for usedBaud %d, sent %s" EOL, usedBaud, nmeaBaudSentence);
    // have to wait for the sentence to get out and complete at the GPS
    delay(3000);

    // Note:
    // Serial2.end() Disables serial communication,
    // allowing the RX and TX pins to be used for general input and output.
    // To re-enable serial communication, call Serial2.begin().

    // FIX! we could try RP2040 using different bauds and see what baud rate it's at, then send
    // the command to change baud rate. But really, how come we can't cold reset it to 9600?
    // when it's in a not-9600 state? it's like vbat keeps the old baud rate on reset and/or power cycle??
    // makes it dangerous to use anything other than 9600 baud.
    Serial2.end();
    Serial2.begin(usedBaud);
    if (VERBY[9]) Serial.printf("setGpsBaud did Serial2.begin(%d)" EOL, usedBaud);
    // then have to change Serial2.begin() to agree
    sleepForMilliSecs(1000, false);
    if (VERBY[0]) Serial.printf("setGpsBaud END %d" EOL, usedBaud);
}

//************************************************
void GpsINIT(void) {
    if (VERBY[0]) Serial.println(F("GpsINIT START"));
    updateStatusLED();
    Watchdog.reset();

    //****************
    // The gps power pin is floating? likely GPS is off but unknown
    // Should turn gps off before doing init if you know it's been
    // initted already? 

    //****************
    // should have the default 8ma drive strength?
    gpio_init(GpsPwr);
    pinMode(GpsPwr, OUTPUT);
    gpio_pull_up(GpsPwr);
    gpio_put(GpsPwr, HIGH);

    gpio_init(GPS_NRESET_PIN);
    pinMode(GPS_NRESET_PIN, OUTPUT);
    gpio_pull_up(GPS_NRESET_PIN);
    gpio_put(GPS_NRESET_PIN, HIGH);

    gpio_init(GPS_ON_PIN);
    pinMode(GPS_ON_PIN, OUTPUT);
    gpio_pull_up(GPS_ON_PIN);
    gpio_put(GPS_ON_PIN, HIGH);
    //****************

    // Updated: Do a full reset since vbat may have kept old settings
    // don't know if that includes baud rate..maybe?
    digitalWrite(GpsPwr, HIGH);
    Serial.printf("set GpsPwr %d HIGH (power off)" EOL, GpsPwr);
    digitalWrite(GPS_NRESET_PIN, HIGH);
    Serial.printf("set GPS_NRESET_PIN %d HIGH" EOL, GPS_NRESET_PIN);
    digitalWrite(GPS_ON_PIN, HIGH);
    Serial.printf("set GPS_ON_PIN %d HIGH" EOL, GPS_ON_PIN);
    //****************

    if (VERBY[9]) {
        Serial.printf("GPS_UART1_RX_PIN %d" EOL, GPS_UART1_RX_PIN);
        Serial.printf("GPS_UART1_TX_PIN %d" EOL, GPS_UART1_TX_PIN);
        Serial.printf("(gpio) GpsPwr %d" EOL, GpsPwr);
        Serial.printf("(gpio) GPS_NRESET_PIN %d" EOL, GPS_NRESET_PIN);
        Serial.printf("(gpio) GPS_ON_PIN %d" EOL, GPS_ON_PIN);
    }

    Serial2.setRX(GPS_UART1_RX_PIN);
    Serial2.setTX(GPS_UART1_TX_PIN);

    //****************
    Serial2.setPollingMode(true);
    // try making bigger (see tracker.ino)..seems like 32 is the reality?
    Serial2.setFIFOSize(SERIAL2_FIFO_SIZE);
    Serial2.flush();
    Serial2.end();

    // first talk at 9600..but GpsFullColdReset() will do..so a bit redundant
    Serial2.begin(9600);
    sleepForMilliSecs(500, false);
    //****************

    // full cold reset, plus set baud to target baud rate, and setGpsBalloonMode done
    GpsFullColdReset();
    // gps is powered up now

    //****************
    // drain the rx buffer. GPS is off, but shouldn't keep
    while (Serial2.available()) Serial2.read();
    // sleep 3 secs
    sleepForMilliSecs(3000, false);

    if (VERBY[0]) Serial.println(F("GpsINIT END"));
}

//************************************************
void GpsFullColdReset(void) {
    // BUG: can't seem to reset the baud rate to 9600 when
    // the GPS chip has a non-working baud rate?

    // a full cold reset reverts to 9600 baud
    // as does standby modes? (don't use)
    if (VERBY[0]) Serial.println(F("GpsFullColdReset START"));
    GpsIsOn_state = false;
    GpsStartTime = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    // turn it off first. may be off or on currently
    // turn off the serial
    Serial.flush();
    Serial.end();

    // assert reset during power off
    digitalWrite(GPS_ON_PIN, HIGH);
    digitalWrite(GPS_NRESET_PIN, LOW);
    digitalWrite(GpsPwr, HIGH);
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

    // now power on with reset asserted
    digitalWrite(GpsPwr, LOW);
    sleepForMilliSecs(1000, false);
    // deassert NRESET after power on
    digitalWrite(GPS_NRESET_PIN, HIGH);


    // experiment ..2nd reset assertion after power is good
    // belt and suspenders: 
    // wait 2 secs and assert/deassert NRESET
    if (false) {
        sleepForMilliSecs(2000, false);
        digitalWrite(GPS_NRESET_PIN, LOW);
        sleepForMilliSecs(1000, false);
        digitalWrite(GPS_NRESET_PIN, HIGH);
        sleepForMilliSecs(1000, false);
    }

    if (false) {
        // TOTAL HACK experiment
        // since vbat seems to preserve the baud rate, even with NRESET assertion
        // try sending the full cold reset command at all reasonable baud rates
        // whatever baud rate the GPS was at, it should get one?
        Serial.println(F("In case reset isn't everything: try full cold reset NMEA cmd at 9600 baud"));
        Serial.begin(9600);
        Serial.print("$PMTK104*37" CR LF);
        Serial.flush();
        sleepForMilliSecs(1000, false);

        Serial.println(F("In case reset isn't everything, try full cold reset NMEA cmd at 19200 baud"));
        Serial.begin(19200);
        Serial.print("$PMTK104*37" CR LF);
        Serial.flush();
        sleepForMilliSecs(1000, false);

        Serial.println(F("In case reset isn't everything, try full cold reset NMEA cmd at 38400 baud"));
        Serial.begin(38400);
        Serial.print("$PMTK104*37" CR LF);
        Serial.flush();
        sleepForMilliSecs(1000, false);

        // We know we would have never told GPS a higher baud rate because we get data rx overruns

        // FIX! do we have to toggle power off/on to get the cold reset?
        // vbat is kept on when we toggle vcc
        if (true) {
            digitalWrite(GpsPwr, HIGH);
            sleepForMilliSecs(1000, false);
            digitalWrite(GpsPwr, LOW);
            sleepForMilliSecs(1000, false);
        }

    }

    //******************
    // resets to 9600. set to new baud rate
    // FIFO is big enough to hold output while we send more input here
    int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
    // then up the speed to desired (both gps chip and then Serial2

    // if it came up already in our target baud rate
    // this will be a no-op (seems it happens if vbat stays on)
    // if so, since we only have one target baud rate hardwired in, 
    // we should be able to start talking to it 

    // gps shold come up at 9600 so look with our uart at 9600
    Serial2.begin(9600);
    // wait for 1 secs before sending commands
    sleepForMilliSecs(1000, false);
    Serial.println(F("Should get some output at 9600 after reset?"));
    // we'll see if it's wrong baud rate or not, at this point
    checkInitialGpsOutput();
    
    // But then we'll be good when we transition to the target rate also
    setGpsBaud(desiredBaud);
    checkInitialGpsOutput();

    //******************
    // FIX! we don't need to toggle power to get the effect?
    setGpsBalloonMode();

    // all constellations
    setGpsConstellations(7); 
    // no ZDA/TXT
    setGpsBroadcast(); 

    GpsIsOn_state = true;
    GpsStartTime = get_absolute_time();  // usecs

    //******************
    if (VERBY[0]) Serial.println(F("GpsFullColdReset END"));
}

//************************************************
void GpsWarmReset(void) {
    if (VERBY[0]) Serial.println(F("GpsWarmReset START"));
    GpsIsOn_state = false;
    GpsStartTime = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();
    // warm reset doesn't change baud rate from prior config?

    // turn it off first. may be off or on currently
    // turn off the serial
    Serial.flush();
    Serial.end();

    // don't assert reset during power off
    digitalWrite(GPS_NRESET_PIN, HIGH);
    digitalWrite(GPS_ON_PIN, HIGH);
    digitalWrite(GpsPwr, HIGH);
    sleepForMilliSecs(1000, false);

    // now power on with reset still off
    // digitalWrite(GPS_NRESET_PIN, HIGH);
    // digitalWrite(GPS_ON_PIN, HIGH);
    digitalWrite(GpsPwr, LOW);
    sleepForMilliSecs(2000, false);

    GpsIsOn_state = true;
    GpsStartTime = get_absolute_time();  // usecs

    // don't know what baud rate it was at. gps comes up at 9600
    // maybe just assume it's the same as whatever setup was agreed on before
    // Serial2.end();
    // Serial2.begin(9600);

    // hmm.. just leave it like it was? vbat will keep the old baud rate?
    // resets to 9600. set to new baud rate
    // FIFO is big enough to hold output while we send more input here
    // the old reset if you want to change baud rate  
    // should be the same from init, so this should work? (unecessary ?)
    // don't change or ?? should be don't care?

    // if it comes back up in out desired BAUD rate already, 
    // everything will be fine and we'll start talking to it
    // as long as the tracker only got one other Baud rate other than 9600 
    // all will be fine
    if (true) {
        int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
        // then up the speed to desired (both gps chip and then Serial2
        setGpsBaud(desiredBaud);
    }
    // wait 2 seconds for normal power before sending more commands
    setGpsBalloonMode();
    // all constellations
    setGpsConstellations(7); 
    // no ZDA/TXT
    setGpsBroadcast(); 

    checkInitialGpsOutput();
    if (VERBY[0]) Serial.println(F("GpsWarmReset END"));
}

//************************************************
void GpsON(bool GpsColdReset) {
    if (VERBY[0]) {
        // no print if no cold reset request. So I can grep on GpsColdReset as a special case only
        if (!GpsColdReset) Serial.printf("GpsON START GpsIsOn_state %u" EOL, GpsIsOn_state);
        else Serial.printf("GpsON START GpsIsOn_state %u GpsColdReset %u" EOL, 
            GpsIsOn_state, GpsColdReset);
    }

    // could be off or on already
    // Assume GpsINIT was already done (pins etc)
    Watchdog.reset();

    // don't care what the initial state is, for cold reset
    if (GpsColdReset) GpsFullColdReset();
    // does nothing if already on
    else if (!GpsIsOn()) GpsWarmReset();

    if (VERBY[0]) {
        if (!GpsColdReset) Serial.printf("GpsON END GpsIsOn_state %u" EOL, GpsIsOn_state);
        else Serial.printf("GpsON END GpsIsOn_state %u GpsColdReset %u" EOL, 
            GpsIsOn_state, GpsColdReset);
    }
    Serial.flush();
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
bool ublox_high_alt_mode_enabled = false;

void GpsOFF(void) {
    if (VERBY[0]) Serial.printf("GpsOFF START GpsIsOn_state %u" EOL, GpsIsOn_state);

    digitalWrite(GpsPwr, HIGH);
    // Serial2.end()
    // Disables serial communication, allowing the RX and TX pins to be used for general input and output.
    // To re-enable serial communication, call Serial2.begin().
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
    // this should work without changing the TinyGPS++ library
    if (true) {
        GpsInvalidAllCnt = 2; // should get us at least 2 GPS broadcasts
        GpsInvalidAll = true;
    } else {
        // how do we clear him?
        // do we wait until we turn GpsON() on, beforre clearing GPSInvalidAll
        // we can decrement the count only if gps is on? (in setup1())
        gps.date.valid = false;
        gps.date.updated = false;
        gps.date.date = 0;
    }

    // also has lastCommitTime = millis()
    // we don't change that?

    ublox_high_alt_mode_enabled = false;

    GpsIsOn_state = false;
    GpsStartTime = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    Serial.flush();
    if (VERBY[0]) Serial.printf("GpsOFF END GpsIsOn_state %u" EOL, GpsIsOn_state);
}

//************************************************
// FIX! why was this static void before?
void updateGpsDataAndTime(int ms) {
    if (VERBY[0]) Serial.println(F("updateGpsDataAndTime START"));
    Watchdog.reset();

    // ms has to be positive?
    // grab data for no more than ms milliseconds
    // stop if no data for 50 milliseconds
    // all the durations below won't start counting until we get the first char (sets start_millis())
    uint64_t start_millis = 0;
    uint64_t last_serial2_millis = 0;
    uint64_t timeSinceLastChar_millis = 0;
    uint64_t duration_millis = 0;
    uint64_t current_millis = millis();
    uint64_t entry_millis = current_millis;

    if (VERBY[0]) Serial.printf(
        "updateGpsDataAndTime started looking for NMEA current_millis %" PRIu64 EOL, 
        current_millis);

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
            if (VERBY[9]) {
                if (charsAvailable > 28)
                    // this the case where we started this function with something in the buffer
                    // we unload each in less than 1ms..so we catch up
                    // compare to 28 so we only get 4 (32 -28) ERROR messages as we catch up
                    StampPrintf("WARN: NMEA incoming chars backing up? 32 deep uart rx buffer has %d)" EOL,
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
            if (VERBY[9]) {
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
                // FIX! can we not send CR LF? don't care. Performance-wise, might be good?
                // moved above to send everything to TinyGPS++
                // gps.encode(incomingChar);
                if (VERBY[0]) nmeaBufferAndPrint(incomingChar, false); 
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
            if (false && VERBY[9]) StampPrintf(
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


    // print/clear any accumulated NMEA sentence stuff
    if (VERBY[0]) {
        // print should only get dumped here?
        nmeaBufferPrintAndClear(); // print and clear
        Serial.print(F(EOL));
        // dump/flush the StampPrintf log_buffer
        DoLogPrint();
    }

    if (VERBY[9]) {
        // seems like we get 12 sentences every time we call this function
        // should stay steady
        int diff = sentenceStartCnt - sentenceEndCnt;
        // these 3 form a oneliner
        Serial.print("updateGpsDataAndTime:");
        Serial.printf(
            " start_millis %" PRIu64 " current_millis %" PRIu64, start_millis, current_millis);
        Serial.printf(
            " sentenceStartCnt %d sentenceEndCnt %d diff %d" EOL, sentenceStartCnt, sentenceEndCnt, diff);
        Serial.flush();
        

        // we can round up or down by adding 500 before the floor divide
        duration_millis = current_millis - start_millis;

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
        if (true) { // FIX! why is this crashing?
            Serial.printf(
            "updateGpsDataAndTime: NMEA AvgCharRateSec %.f duration_millis %" PRIu64 " incomingCharCnt %d" EOL,  
            AvgCharRateSec, duration_millis, incomingCharCnt);
            Serial.flush();
        }
    }

    if (VERBY[9])
        Serial.printf("updateGpsDataAndTime: GpsInvalidAll:%u gps.time.isValid():%u" EOL, 
            GpsInvalidAll, gps.time.isValid());
        Serial.flush();

    if (gps.time.isValid() && !GpsInvalidAll) {
        // FIX! don't be updating this every time
        // this will end up checking every time we get a burst?

        // do we need to check every iteration?
        // Update the arduino (cpu) time. setTime is in the Time library.
        // we don't care about comparing that the date is right??
        // we actually don't care about hour..but good to be aligned with that
        // uint8_t for gps data
        // the Time things are int
        if (hour() != gps.time.hour() || minute() != gps.time.minute() || second() != gps.time.second()) {
            setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 0, 0, 0);
            if (VERBY[0]) Serial.printf("setTime(%02u:%02u:%02u)" EOL,
                gps.time.hour(), gps.time.minute(), gps.time.second());
        }
    }

    updateStatusLED();
    uint64_t total_millis = millis() - entry_millis;

    // will be interesting to see how much bigger this is compared to the duration_millis
    if (VERBY[0]) 
        Serial.printf("updateGpsDataAndTime END total_millis %" PRIu64 EOL EOL, 
            total_millis);
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
        if (VERBY[9]) Serial.println(F("ublox DynamicModel6 try..."));
        sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
        gps_set_success = getUBX_ACK(setdm6);
    }
}
//******************************************************
// Enabling output:
// Checksums:
// ; Odd number of "ones": 03
// ; Even number of "ones": 02

// $PCAS03,1,0,0,0,1,1,0,0*03
// $PCAS03,1,1,1,1,1,1,0,0*02
//         | | | | | | | |
//         '-|-|-|-|-|-|-|--> GGA
//           '-|-|-|-|-|-|--> GLL
//             '-|-|-|-|-|--> GSA
//               '-|-|-|-|--> GSV
//                 '-|-|-|--> RMC
//                   '-|-|--> VTG
//                     '-|--> ZDA
//                       '--> TXT

//************************************************
void gpsDebug() {
    // https://github.com/StuartsProjects/GPSTutorial
    if (!VERBY[0]) return;
    Serial.println(F("GpsDebug START"));

    Serial.print(F(EOL EOL));
    Serial.println(F("Sats HDOP Latitude      Longitude   Fix  Date       Time     Date Alt      Course  Speed Card Chars FixSents  Checksum"));
    Serial.println(F("          (deg)         (deg)       Age                      Age  (m)      --- from GPS ----   RX    RX        Fail"));
    Serial.println(F("---------------------------------------------------------------------------------------------------------------------"));

    printInt(gps.satellites.value(), gps.satellites.isValid() && !GpsInvalidAll, 5);
    printInt(gps.hdop.value(), gps.hdop.isValid() && !GpsInvalidAll, 5);
    printFloat(gps.location.lat(), gps.location.isValid() && !GpsInvalidAll, 12, 6);
    printFloat(gps.location.lng(), gps.location.isValid() && !GpsInvalidAll, 12, 6);

    printInt(gps.location.age(), gps.location.isValid() && !GpsInvalidAll, 5);
    printDateTime(gps.date, gps.time);
    printFloat(gps.altitude.meters(), gps.altitude.isValid() && !GpsInvalidAll, 7, 2);
    printFloat(gps.course.deg(), gps.course.isValid() && !GpsInvalidAll, 7, 2);
    printFloat(gps.speed.kmph(), gps.speed.isValid() && !GpsInvalidAll, 6, 2);
    printStr((gps.course.isValid() && !GpsInvalidAll)? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);
    printInt(gps.charsProcessed(), true, 6); // FIX! does this just wrap wround if it's more than 6 digits?
    printInt(gps.sentencesWithFix(), true, 10); // FIX! does this just wrap wround if it's more than 10 digits?
    printInt(gps.failedChecksum(), true, 9);

    Serial.print(F(EOL EOL));
    Serial.println(F("GpsDebug END"));
}

// Was wondering why the HDOP was so high. it seems the 90 really means 90 / 100 = .9 HDOP 
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
// It was a design decision—possibly flawed—to deliver the HDOP exactly as reported by the NMEA string. 
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

// Example: if the function argument list contains user defined data types and
// the automatically created function prototype is placed before the declaration of that data type.

