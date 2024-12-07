// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// REMEMBER: no references to Serial.* in BALLOON_MODE! (no usb)

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

#include <Arduino.h>
// for isprint()
#include <ctype.h>
#include "gps_functions.h"
#include "debug_functions.h"
#include "print_functions.h"
#include "defines.h"
// enums for voltage at:
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h
#include "hardware/vreg.h"
// for disabling pll 
// https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__pll.html
#include "hardware/pll.h"
// ugh, do we need to include this for tusb_init()
#include "tusb.h"

// in libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// for setTime()
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog
#include "led_functions.h"

// object for TinyGPSPlus state
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
// decode of verbose 0-9
extern bool VERBY[10];

extern uint32_t GpsInvalidAllCnt;
extern bool GpsInvalidAll;
// to avoid servicing keyboard while in aggressive power transition
extern bool IGNORE_KEYBOARD_CHARS;

// FIX! gonna need an include for this? maybe note
// # include <TimeLib.h>

// we make RP2040 to 18mhz during the long gps cold reset fix time, then restore to this
extern uint32_t PLL_SYS_MHZ; // decode of _clock_speed
extern bool BALLOON_MODE;

// ************************************************
// false and true work here
bool EXPERIMENTAL_WARM_POWER_ON = true;
bool EXPERIMENTAL_COLD_POWER_ON = true;
bool LOWEST_POWER_TURN_ON_MODE = true;
bool ALLOW_USB_DISABLE_MODE = true;
bool ALLOW_UPDATE_GPS_FLASH_MODE = true;

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
    if ( (strlen(nmeaBuffer) + 3) >= NMEA_BUFFER_SIZE) {
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
void sleepForMilliSecs(int n, bool enableEarlyOut) {
    // FIX! should we do this here or where?
    Watchdog.reset();
    if (n < 0 || n > 120000) {
        V1_printf("ERROR: sleepForMilliSecs() n %d too big (120000 max). Using 1000" EOL, n);
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
        if ((milliDiv % 10) == 0) {
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
void drainInitialGpsOutput(void) {
    V1_println(F("drainInitialGpsOutput START"));

    // FIX! rely on watchdog reset in case we stay here  forever?
    V1_println(F("drain any Serial2 garbage first"));
    // drain any initial garbage
    while (Serial2.available()) {Serial2.read();}
    V1_println(F("now look for some Serial2 bytes"));

    int i;
    char incomingChar = { 0 };
    // we drain during the GpsINIT now, oh. we should leave gps on so we get chars
    for (i = 0; i < 1; i++) {
        Watchdog.reset();
        if (!Serial2.available()) {
            V1_println(F("no Serial2.available() ..sleep and reverify"));
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
    V1_println(F("drainInitialGpsOutput END"));
}

//************************************************
void setGpsBalloonMode(void) {
    V1_println(F("setGpsBalloonMode START"));
    // FIX! should we not worry about setting balloon mode (3) for ATGM336?
    // Serial2.print("$PSIMNAV,W,3*3A\r\n");
    // normal mode
    // Serial2.print("$PSIMNAV,W,0*39\r\n");
    // have to wait for the sentence to get out, and also complete
    // sleepForMilliSecs(1000, false);
    V1_println(F("setGpsBalloonMode END"));
}

//************************************************
// always GGA GLL GSA GSV RMC
// nver ZDA TXT
void setGpsBroadcast(void) {
    V1_print(F("setGpsBroadcast START" EOL));
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
    // xxxx = Year // xx = Local zone description, 00 to +/- 13 hours
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
    // delay(1000);

    V1_printf("setGpsBroadcast sent %s" EOL, nmeaSentence);
    V1_print(F("setGpsBroadcast END" EOL ));

}
//************************************************
void disableGpsBroadcast(void) {
    V1_print(F("disableGpsBroadcast START" EOL));
    updateStatusLED();
    Watchdog.reset();
    // room for a 60 char sentence with CR and LF also
    char nmeaSentence[62] = { 0 };
    // checksum from https://www.meme.au/nmea-checksum.html
    strncpy(nmeaSentence, "$PCAS03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*02" CR LF, 62);
    Serial2.print(nmeaSentence);
    Serial2.flush();
    // delay(1000);
    V1_printf("disableGpsBroadcast sent %s" EOL, nmeaSentence);
    V1_print(F("disableGpsBroadcast END" EOL ));
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
    V1_printf("setConstellations START %d" EOL, desiredConstellations);
    updateStatusLED();
    Watchdog.reset();
    // FIX! should we ignore desiredConstellations and force 3 (BDS + GPS
    int usedConstellations = desiredConstellations;
    char nmeaSentence[62] = { 0 };

    switch (usedConstellations) {
        // case 0 isn't defined in the CASIC_ProtocolSpecification.pdf?
        case 1: strncpy(nmeaSentence, "$PCAS04,1*18" CR LF, 62); break; // GPS
        case 2: strncpy(nmeaSentence, "$PCAS04,2*1B" CR LF, 62); break; // BDS
        case 3: strncpy(nmeaSentence, "$PCAS04,3*1A" CR LF, 62); break; // GPS+BDS
        case 4: strncpy(nmeaSentence, "$PCAS04,4*1D" CR LF, 62); break; // GLONASS
        case 5: strncpy(nmeaSentence, "$PCAS04,5*1C" CR LF, 62); break; // GPS+GLONASS
        case 6: strncpy(nmeaSentence, "$PCAS04,6*AF" CR LF, 62); break; // BDS+GLONASS
        case 7: strncpy(nmeaSentence, "$PCAS04,7*1E" CR LF, 62); break; // GPS+BDS+GLONASS
        default:
            usedConstellations = 3;
            strncpy(nmeaSentence, "$PCAS04,3*1D" CR LF, 62); // GPS+BDS
    }

    // FIX! does the above not do anything? is this the only way?
    if (true) {
        // alternative experiment
        // what about this rumored PMTK353 sentence?
        // $PMTK353,1,1,1,0,1*2B : Search GPS BEIDOU GLONASS and GALILEO satellites
        strncpy(nmeaSentence, "$PMTK353,1,1,1,0,1*2B" CR LF, 62);
        Serial2.print(nmeaSentence);
        Serial2.flush();
        delay(1000);
    }

    V1_printf("setGpsConstellations for usedConstellations %d, sent %s" EOL, desiredConstellations, nmeaSentence);
    V1_printf("setGpsConstellations END %d" EOL, desiredConstellations);
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
    Serial2.flush();
    V1_printf("setGpsBaud for usedBaud %d, sent %s" EOL, usedBaud, nmeaBaudSentence);
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
    V1_printf("setGpsBaud did Serial2.begin(%d)" EOL, usedBaud);
    // then have to change Serial2.begin() to agree
    sleepForMilliSecs(1000, false);
    V1_printf("setGpsBaud END %d" EOL, usedBaud);
}

//************************************************
void GpsINIT(void) {
    V1_println(F("GpsINIT START"));
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
    // gpio_put(GpsPwr, HIGH); // deassert

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
    V1_println(F("GpsINIT END"));
}

//************************************************
void GpsFullColdReset(void) {
    // BUG: can't seem to reset the baud rate to 9600 when
    // the GPS chip has a non-working baud rate?

    // a full cold reset reverts to 9600 baud
    // as does standby modes? (don't use)
    V1_println(F("GpsFullColdReset START"));
    Watchdog.reset();

    GpsIsOn_state = false;
    GpsStartTime = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    // turn it off first. may be off or on currently
    V1_println(F("Turn off the serial2..does it float at the gps then? gps doesn't try to do UART stuff at poweron?"));
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
    if (EXPERIMENTAL_COLD_POWER_ON) {
        V1_println(F("Doing Gps EXPERIMENTAL_COLD_POWER_ON (GPS_ON_PIN off with first power)"));
        // NOTE: do I have to keep inputs like GPS_ON_PIN low 
        // until after power is on? What about reset?
        // to avoid latchup of LNA? see
        // https://www.eevblog.com/forum/rf-microwave/gps-lna-overheating-on-custom-pcb/
        digitalWrite(GPS_ON_PIN, LOW); // deassert
        digitalWrite(GPS_NRESET_PIN, LOW); // assert
        digitalWrite(GpsPwr, HIGH); // deassert
    } else {
        V1_println(F("Doing Gps NORMAL_COLD_POWER_ON (GPS_ON_PIN on with first power)"));
        digitalWrite(GPS_ON_PIN, HIGH); // assert
        digitalWrite(GPS_NRESET_PIN, LOW); // assert
        digitalWrite(GpsPwr, HIGH); // deassert
    }

    sleepForMilliSecs(500, false);

    //******************
    // Cold Start. doesn't clear any system/user configs
    // Serial2.print("$PMTK103*30\r\n");
    // Full Cold Start. any system/user configs (back to factory status)
    // FIX! should we wait for ack or no?
    // have to toggle power off/on to get this effect? 

    // always do this just in case the GpsIsOn() got wrong?
    // but we're relying on the Serial2.begin/end to be correct?
    // might as well commit to being right!

    // FIX! hmm. does driving the uart rx/tx to gps while gps is powering up
    // change it's behavior. What if we left them floating until after powerup?
    // seems like the gps backs up on the serial data?

    digitalWrite(GpsPwr, LOW); // assert
    sleepForMilliSecs(500, false);

    // deassert NRESET after power on (okay in both normal and experimental case)
    // new 12/7/24 disable Serial2 while powering on!
    // should we float the rx/tx also somehow?
    Serial2.end();
    sleepForMilliSecs(500, false);

    if (EXPERIMENTAL_COLD_POWER_ON) {
        digitalWrite(GPS_NRESET_PIN, HIGH); // deassert
        if (!LOWEST_POWER_TURN_ON_MODE) {
            // turn this on later in lowest power mode. while rp2040 is reduced power
            digitalWrite(GPS_ON_PIN, HIGH); // assert
        }
    } else {
        digitalWrite(GPS_NRESET_PIN, HIGH);
    }
        
    sleepForMilliSecs(1000, false);

    // old experiment..do reset assertion (again or solely? after power is good
    // belt and suspenders: 
    // wait 2 secs and assert/deassert NRESET
    // still can't seem to get back to 9600 baud after higher baud rate set
    if (false) {
        sleepForMilliSecs(2000, false);
        digitalWrite(GPS_NRESET_PIN, LOW);
        sleepForMilliSecs(1000, false);
        digitalWrite(GPS_NRESET_PIN, HIGH);
        sleepForMilliSecs(1000, false);
    }

    Watchdog.reset();
    // IDEA! since we KNOW the power demand will be high for 1 minute after poweron
    // just go into light sleep to reduce rp2040 power demand for 1 minute
    // i.e. guarantee that cold reset, takes 1 minute? 
    // hmm. we're stalling things now. maybe only sleep for 30 secs
    // FIX! this apparently makes the Serial2 dysfunctional so the gps chip can't send output
    // it backs up on the initial TXT broadcast (revisions) and then hits a power peak
    // right after we fix the clock back to normal (50Mhz min tried)
    // so: is that worth it? dunno.

    // other power saving: disable usb pll (and restore)
    
    // https://github.com/earlephilhower/arduino-pico/discussions/1544
    // we had to make sure we reset the watchdog, now, in sleepForMilliSecs
    // we already wakeup periodically to update led, so fine

    //******************
    // so we can undo the testing
    uint32_t freq_khz = PLL_SYS_MHZ * 1000UL;
    if (LOWEST_POWER_TURN_ON_MODE) {
        // the global IGNORE_KEYBOARD_CHARS is used to guarantee no interrupting of core1
        // while we've messed with clocks during the gps agressive power on control
        // it should always be re-enabled after 30 secs. 
        // Worst case to recover: unplug power and plug in again

        Watchdog.reset();
        measureMyFreqs();
        V1_print(F(RED));
        V1_print(F("GPS power demand is high until first fix after cold reset..sleep for 30 secs" EOL));
        V1_printf("Going to slow PLL_SYS_MHZ from %lu to 18Mhz before long sleep" EOL, PLL_SYS_MHZ);
        V1_print("No keyboard interrupts will work because disabling USB PLL too or minimally: Serial.end()" EOL);
        V1_print("Also lowering core voltage to 0.95v" EOL);
        V1_print(F(NORMAL));
        V1_flush();

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
        // remember not to touch Serial if in BALOOON_MODE!!
        if (!BALLOON_MODE) Serial.end();

        if (ALLOW_USB_DISABLE_MODE) {
            busy_wait_ms(500);
            // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__pll.html
            // There are two PLLs in RP2040. They are:
            // pll_sys - Used to generate up to a 133MHz (actually more) system clock
            // pll_usb - Used to generate a 48MHz USB reference clock

            // void pll_deinit (PLL	pll)	
            // Release/uninitialise specified PLL.This will turn off the power to the specified PLL. 
            // Note this function does not check if the PLL is in use before powering it off. (use care)
            // this seems to crap out
            // pll_deinit(pll_usb);

            // examples: https://sourcevu.sysprogs.com/rp2040/picosdk/symbols/pll_deinit
            // sidenote: pi pico sdk has set_sys_clock_48mhz() function
            // Initialise the system clock to 48MHz Set the system clock to 48MHz, 
            // and set the peripheral clock to match.
            // example:
            // https://sourcevu.sysprogs.com/rp2040/examples/clocks/hello_48MHz/files/hello_48MHz.c#tok293


            // HACK: doesn't work?
            // pll_deinit(pll_usb);
        }
        set_sys_clock_khz(18000, true);
        // enums for voltage at:
        // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h
        // vreg_set_voltage(VREG_VOLTAGE_0_95 ); // 0_85 crashes for him. 0.90 worked for him

        // finally turn on the gps here! (if we didn't already above (experimental mode)
        digitalWrite(GPS_ON_PIN, HIGH); // assert

    } else {
        V1_print(F("GPS power demand is high until first fix after cold reset..sleep for 30 secs" EOL));
    }

    // can't do this while USB is disabled? but if we can't disable deinit USB (see above), we can?
    measureMyFreqs();
    // FIX! still getting intermittent cases where we don't come back (running 60Mhz)
    sleepForMilliSecs(30000, false); // 30 secs

    //******************
    // DRASTIC measures, undo after sleep!
    Watchdog.reset();
    if (LOWEST_POWER_TURN_ON_MODE) {
        // maybe try not changing core voltage (here and above)
        // vreg_set_voltage(VREG_VOLTAGE_1_10 ); // normal core voltage
        busy_wait_ms(1000);
        set_sys_clock_khz(freq_khz, true);

        // don't bother reinitting if in BALLOON_MODE (no usb)
        // especially: don't try touching Serial.*
        if (ALLOW_USB_DISABLE_MODE && !BALLOON_MODE) {
        busy_wait_ms(1000);
            // pll_init() Parameters
            // pll	pll_sys or pll_usb
            // ref_div	Input clock divider.
            // vco_freq	Requested output from the VCO (voltage controlled oscillator)
            // post_div1	Post Divider 1 - range 1-7. Must be >= post_div2
            // post_div2	Post Divider 2 - range 1-7
            pll_init(pll_usb, 1, 1440000000, 6, 5); // return USB pll to 48mhz
            busy_wait_ms(1000);
            // High-level Adafruit TinyUSB init code, 
            // does many things to get USB back online
            tusb_init();
            Serial.begin(115200);
            busy_wait_ms(1000);
        }

        V1_printf("After long sleep, Restored sys_clock_khz() and PLL_SYS_MHZ to %lu" EOL, PLL_SYS_MHZ);
        V1_print(F("Restored USB pll to 48Mhz, and Serial.begin()" EOL));
        V1_print(F("Restored core voltage back to 1.1v" EOL));
        V1_flush();
        measureMyFreqs();

        IGNORE_KEYBOARD_CHARS = false;
    }

    //******************
    Watchdog.reset();

    if (false) {
        // TOTAL HACK experiment
        // since vbat seems to preserve the baud rate, even with NRESET assertion
        // try sending the full cold reset command at all reasonable baud rates
        // whatever baud rate the GPS was at, it should get one?
        V1_println(F("In case reset isn't everything: try full cold reset NMEA cmd at 9600 baud"));
        Serial2.begin(9600);
        Serial2.print("$PMTK104*37" CR LF);
        Serial2.flush();
        sleepForMilliSecs(1000, false);

        V1_println(F("In case reset isn't everything, try full cold reset NMEA cmd at 19200 baud"));
        Serial2.begin(19200);
        Serial2.print("$PMTK104*37" CR LF);
        Serial2.flush();
        sleepForMilliSecs(1000, false);

        V1_println(F("In case reset isn't everything, try full cold reset NMEA cmd at 38400 baud"));
        Serial2.begin(38400);
        Serial2.print("$PMTK104*37" CR LF);
        Serial2.flush();
        sleepForMilliSecs(1000, false);

        // We know we would have never told GPS a higher baud rate because we get data rx overruns

        // FIX! do we have to toggle power off/on to get the cold reset?..i.e. the 
        // nmea request just says what happens on the next power off/on?
        // vbat is kept on when we toggle vcc
        digitalWrite(GpsPwr, HIGH);
        sleepForMilliSecs(1000, false);
        digitalWrite(GpsPwr, LOW);
        sleepForMilliSecs(1000, false);

    }

    // hmm. we get a power surge here then? Is it because the Serial2 data was backed up 
    // in the gps chip and busy waiting? Drain it? (usually it's the TXT stuff (versions)
    // at power on. Don't care.

    // we should be able to start talking to it 
    // gps shold come up at 9600 so look with our uart at 9600?
    Serial2.begin(9600);
    // wait for 1 secs before sending commands
    sleepForMilliSecs(1000, false);
    V1_println(F("Should get some output at 9600 after reset?"));
    // we'll see if it's wrong baud rate or not, at this point
    drainInitialGpsOutput();

    // But then we'll be good when we transition to the target rate also
    int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
    setGpsBaud(desiredBaud);
    // this is all done earlier in the experimental mode
    // FIX! we don't need to toggle power to get the effect?
    setGpsBalloonMode();
    // all constellations GPS/BaiDou/Glonass
    // setGpsConstellations(7); 
    // FIX! try just gps to see effect on power on current
    setGpsConstellations(7); 
    // no ZDA/ANT TXT (NMEA sentences) after this:
    setGpsBroadcast(); 

    drainInitialGpsOutput();

    GpsIsOn_state = true;
    GpsStartTime = get_absolute_time();  // usecs

    V1_println(F("GpsFullColdReset END"));
}

//************************************************
void GpsWarmReset(void) {
    V1_println(F("GpsWarmReset START"));
    GpsIsOn_state = false;
    GpsStartTime = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();
    // warm reset doesn't change baud rate from prior config?

    // turn it off first. may be off or on currently
    // turn off the serial
    V1_flush();
    Serial2.end();

    // don't assert reset during power off

    // FIX! what if we power on with GPS_ON_PIN LOW and GPS_NRESET_PIN HIGH
    if (EXPERIMENTAL_WARM_POWER_ON) {
        V1_println(F("Doing Gps EXPERIMENTAL_WARM POWER_ON (GPS_ON_PIN off with power off-on)"));
        // NOTE: should we start with NRESET_PIN low also until powered (latchup?)?
        digitalWrite(GPS_NRESET_PIN, HIGH);
        // NOTE: do we need to start low until powered to avoid latchup of LNA?
        digitalWrite(GPS_ON_PIN, LOW);
        digitalWrite(GpsPwr, HIGH);
    } else {
        V1_println(F("Doing Gps NORMAL_WARM POWER_ON (GPS_ON_PIN off with power off-on)"));
        digitalWrite(GPS_NRESET_PIN, HIGH);
        digitalWrite(GPS_ON_PIN, HIGH);
        digitalWrite(GpsPwr, HIGH);
    }
    sleepForMilliSecs(1000, false);

    // now power on with reset still off
    // digitalWrite(GPS_NRESET_PIN, HIGH);
    // digitalWrite(GPS_ON_PIN, HIGH);
    digitalWrite(GpsPwr, LOW);
    sleepForMilliSecs(2000, false);

    if (EXPERIMENTAL_WARM_POWER_ON) {
        digitalWrite(GPS_ON_PIN, HIGH);
        sleepForMilliSecs(2000, false);
    }
    GpsIsOn_state = true;
    GpsStartTime = get_absolute_time();  // usecs

    // if it comes back up in out desired BAUD rate already, 
    // everything will be fine and we'll start talking to it
    // as long as the tracker only got one other Baud rate other than 9600 
    // all will be fine

    // or will this come out of warm reset at 9600 baud and we can't talk to it?
    
    // FIX! this is a don't care then? whatever it was? or ??
    // but since we serial2.end() above, we have to restart it on the rp2040
    int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
    // then up the speed to desired (both gps chip and then Serial2
    setGpsBaud(desiredBaud);
    setGpsBalloonMode();

    // all constellations
    // setGpsConstellations(7); 
    // FIX! try just gps to see effect on power on current
    setGpsConstellations(1); 
    // no ZDA/TXT
    // do this after the config write now
    // setGpsBroadcast(); 

    // from the CASIC_ProtocolSpecification_english.pdf page 24
    // I suppose this could be dangerous, since it's writing a baud rate to the power off/on reset config state?
    // could change it from 9600 and we'd lose track of what it is? As long as we stick with 9600 we should be safe
    // CAS00. Description Save the current configuration information to FLASH. 
    // Even if the receiver is completely powered off, the information in FLASH will not be lost.
    // Format $PCAS00*CS<CR><LF>
    // Example $PCAS00*01<CR><LF>

    // will this help us to boot in a better config so we don't get the power demand peaks we see (on subsequent boots)
    // maybe we shouldn't do this all the time? just once. Does the FLASH have a max # of writes issue? (100k or ??)
    // we only do gps cold reset at start of day. Don't do it in BALLOON_MODE. that should fix the issue
    if (ALLOW_UPDATE_GPS_FLASH_MODE && !BALLOON_MODE) {
        // risk: do we ever power on and not do this full cold reset that sets up broadcast?
        // the warm gps reset shouldn't get new state from config?
        disableGpsBroadcast(); 
        // HMM! should we change it to no broadcast, in the FLASH, so cold reset power on might try to do no broadcast
        char nmeaBaudSentence[21] = { 0 };
        strncpy(nmeaBaudSentence, "$PCAS00*01" CR LF, 21);
        V1_println(F("Write GPS current config state to GPS Flash (for use in next GPS cold reset?)"));
        V1_printf("%s" EOL, nmeaBaudSentence);
        Serial2.print(nmeaBaudSentence);
        Serial2.flush();
    }
    setGpsBroadcast(); 

    drainInitialGpsOutput();
    V1_println(F("GpsWarmReset END"));
}

//************************************************
void GpsON(bool GpsColdReset) {
    // no print if no cold reset request. So I can grep on GpsColdReset as a special case only
    if (!GpsColdReset) {
        V1_printf("GpsON START GpsIsOn_state %u" EOL, GpsIsOn_state);
    } else {
        V1_printf("GpsON START GpsIsOn_state %u GpsColdReset %u" EOL, GpsIsOn_state, GpsColdReset);
    }

    // could be off or on already
    // Assume GpsINIT was already done (pins etc)
    Watchdog.reset();

    // don't care what the initial state is, for cold reset
    if (GpsColdReset) GpsFullColdReset();
    // does nothing if already on
    else if (!GpsIsOn()) GpsWarmReset();

    if (!GpsColdReset) {
        V1_printf("GpsON END GpsIsOn_state %u" EOL, GpsIsOn_state);
    }
    else {
        V1_printf("GpsON END GpsIsOn_state %u GpsColdReset %u" EOL, GpsIsOn_state, GpsColdReset);
    }

    V1_flush();
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

void invalidateTinyGpsState(void) {
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

    // TinyGPS also has lastCommitTime = millis()
    // we don't change that?

    // these three are the initial value
    // this should work without changing the TinyGPS++ library
    // did this not work?
    if (false) {
        // should get us ignoring least 2 GPS broadcasts? Two cycles through loop1() ?
        GpsInvalidAllCnt = 2; 
        GpsInvalidAll = true;
    } else {
        // how do we clear him?
        // do we wait until we turn GpsON() on, beforre clearing GPSInvalidAll
        // we can decrement the count only if gps is on? (in setup1())
        gps.date.valid = false;
        gps.date.updated = false;
        gps.date.date = 0;
    }
}

//************************************************
void GpsOFF(bool keepTinyGpsState) {

    V1_printf("GpsOFF START GpsIsOn_state %u" EOL, GpsIsOn_state);

    digitalWrite(GpsPwr, HIGH);
    // Serial2.end() Disables serial communication,
    // To re-enable serial communication, call Serial2.begin().
    // FIX! do we really need or want to turn off Serial2? 
    // Remember to Serial2.begin() when we turn it back on
    Serial2.end();

    // unlike i2c to vfo, we don't tear down the Serial2 definition...just .end() 
    // so we can just .begin() again later
    if (!keepTinyGpsState)
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
void updateGpsDataAndTime(int ms) {
    V1_println(F("updateGpsDataAndTime START"));
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

    V1_printf(
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
            if (VERBY[1]) {
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
            if (VERBY[1]) {
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
                if (VERBY[1]) nmeaBufferAndPrint(incomingChar, false); 
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
            if (false && VERBY[1]) StampPrintf(
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
    if (VERBY[1]) {
        // print should only get dumped here?
        nmeaBufferPrintAndClear(); // print and clear
        V1_print(F(EOL));
        // dump/flush the StampPrintf log_buffer
        DoLogPrint();
    }
    if (VERBY[1]) {
        // seems like we get 12 sentences every time we call this function
        // should stay steady
        int diff = sentenceStartCnt - sentenceEndCnt;
        // these 3 form a oneliner
        V1_print("updateGpsDataAndTime:");
        V1_printf(
            " start_millis %" PRIu64 " current_millis %" PRIu64, start_millis, current_millis);
        V1_printf(
            " sentenceStartCnt %d sentenceEndCnt %d diff %d" EOL, sentenceStartCnt, sentenceEndCnt, diff);
        V1_flush();

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
            V1_printf(
            "updateGpsDataAndTime: NMEA AvgCharRateSec %.f duration_millis %" PRIu64 " incomingCharCnt %d" EOL,  
            AvgCharRateSec, duration_millis, incomingCharCnt);
            V1_flush();
        }
    }

    V1_printf("updateGpsDataAndTime: GpsInvalidAll:%u gps.time.isValid():%u" EOL, 
        GpsInvalidAll, gps.time.isValid());
    V1_flush();

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
            V1_printf("setTime(%02u:%02u:%02u)" EOL,
                gps.time.hour(), gps.time.minute(), gps.time.second());
        }
    }

    updateStatusLED();
    uint64_t total_millis = millis() - entry_millis;

    // will be interesting to compare total_millis to duration_millis
    V1_printf("updateGpsDataAndTime END total_millis %" PRIu64 EOL EOL, 
        total_millis);
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
    if (!VERBY[1]) return;
    V1_println(F("GpsDebug START"));

    V1_print(F(EOL EOL));
    V1_println(F("Sats HDOP Latitude      Longitude   Fix  Date       Time     Date Alt      Course  Speed Card Chars FixSents  Checksum"));
    V1_println(F("          (deg)         (deg)       Age                      Age  (m)      --- from GPS ----   RX    RX        Fail"));
    V1_println(F("---------------------------------------------------------------------------------------------------------------------"));

    // https://github.com/StuartsProjects/GPSTutorial
    if (VERBY[1]) {
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
    }

    V1_print(F(EOL EOL));
    V1_println(F("GpsDebug END"));
}

//*****************
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

//*****************
// some notes on bad power on of ATGM336
// https://www.eevblog.com/forum/rf-microwave/gps-lna-overheating-on-custom-pcb/

// So the magic sequence to turn the LNA into a toaster is:
// 1) Have the 3V3 power off;
// 2) Enable the ON_OFF pin (HIGH signal from an STM32 - powered by a separate 3.0V LDO Linear Regulator);
// 3) Turn 3V3 power on.
// 4) Hot LNA!
// After that, the only thing that cools down the LNA is turning 3V3 off again. 
// Disabling the ON_OFF pin does nothing.

// Yes, this sounds like a classic example of latch up caused by an input voltage exceeding a power rail.
// https://en.wikipedia.org/wiki/Latch-up

// https://forum.arduino.cc/t/gps-power-management-reset-loop/529253/5
// someone using the NDB6020P, which had a good suggestion by @MarkT to simply increase 
// the value of the gate resistor to slow the switching of the MOSFET. 
// So, I increased my gate resistor to 47 kOhm, watched as my turn on time increased to 150 μs 
// and was pleasantly surprised to discover the Arduino didn't reset when the GPS was turned on! 
// Out of curiousity, I increased the value of my gate resistor again to 100 kOhm, 
// which further increased the turn on time to 250 μs and also seemed to fix the reset problem. 
// With these higher value gate resistors, I put my scope leads back on the 3.3 V source, 
// and measured the voltage dropping to only ~3.0 V when the GPS is turned on (~250 mV).
