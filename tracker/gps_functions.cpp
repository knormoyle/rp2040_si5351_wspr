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
// https://www.sparkfun.com/datasheets/GPS/Modules/PMTK_Protocol.pdf


//*******************************************

#include <Arduino.h>
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
extern uint64_t GpsTimeToLastFix;  // milliseconds

// FIX! gonna need an include for this? maybe note
// # include <TimeLib.h>

// ************************************************
static bool GpsIsOn_state = false;

bool GpsIsOn(void) {
    return GpsIsOn_state;
}

// ************************************************
// Is the maximum length of any in or out packet = 255 bytes?
#define NMEA_BUFFER_SIZE 4 * 255
static char nmeaBuffer[NMEA_BUFFER_SIZE] = { 0 };

// Outputs the content of the nmea buffer to stdio (UART and/or USB)
void nmeaBufferPrintAndClear(void) {
    if (nmeaBuffer[0] != 0) {
        // don't add an EOL to the print since we can accumulate multiple to look good?
        Serial.print(nmeaBuffer);
        nmeaBuffer[0] = 0;  // Clear the buffer
    }
    // whenever something might have taken a long time like printing the big buffer
    updateStatusLED();
    Watchdog.reset();
}

// add one char at a time
void nmeaBufferAndPrint(const char charToAdd) {
    // we might add a EOL before a '$' that begins a sentence. so check for +2
    // EOL might be /r /n or /r/n (two chars). so check for 3.
    // possible 2 in front. 0 null term at end
    if ( (strlen(nmeaBuffer) + 3) >= NMEA_BUFFER_SIZE) {
        // make NMEA_BUFFER_SIZE bigger or
        // can just do more nmeaBufferPrint() if we run into a problem realtime
        Serial.printf(
            "WARNING: with NMEA_BUFFER_SIZE %d strlen(nmeaBuffer) %d "
            "there is no room for char %c <newline>" EOL,
            NMEA_BUFFER_SIZE, strlen(nmeaBuffer), charToAdd);
        Serial.println(F("..flushing with nmeaBufferPrint first"));
        nmeaBufferPrintAndClear();
    }

    int n = strlen(nmeaBuffer);
    if (charToAdd == '$') {
        // put a EOL in first!
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
void sleepForSecs(int n) {
    if (n < 0 || n > 10) {
        Serial.printf("ERROR: sleep_for_n_secs() n %d too big. Using 2" EOL, n);
        n = 2;
    }
    // sleep approx. n secs
    for (int i = 0; i < n * 2; i++) {
        sleep_ms(500); // 1/2 sec per iteration
        updateStatusLED();
    }
    Watchdog.reset();
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
    // FIX! rely on watchdog reset in case we stay here  forever?
    if (DEVMODE) Serial.println(F("look for some Serial2 bytes"));
    int i;
    char incomingChar = { 0 };
    for (i = 0; i < 10; i++) {
        Watchdog.reset();
        if (!Serial2.available()) {
            Serial.println(F("no Serial2.available() ..sleep and reverify"));
            sleepForSecs(2);
        }
        else {
            while (Serial2.available()) {
                incomingChar = Serial2.read();
                // buffer it up like we do normally below, so we can see sentences
                nmeaBufferAndPrint(incomingChar);
            }
        }
        sleep_ms(50);
    }
    nmeaBufferPrintAndClear();
    updateStatusLED();
    Watchdog.reset();
}

//************************************************
void setGpsBalloonMode(void) {
    if (DEVMODE) Serial.println(F("setGpsBalloonMode START"));
    // FIX! should we not worry about setting balloon mode (3) for ATGM336?
    // Serial2.print("$PSIMNAV,W,3*3A\r\n");
    // normal mode
    // Serial2.print("$PSIMNAV,W,0*39\r\n");
    Serial2.print("$PMTK104*37" CR LF);
    sleepForSecs(1);
    if (DEVMODE) Serial.println(F("setGpsBalloonMode END"));
}

void setGpsBaud(int desiredBaud) {
    if (DEVMODE) Serial.printf("setGpsBaud START %d" EOL, desiredBaud);
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
        case 0:      strncpy(nmeaBaudSentence, "$PMTK251,0*28" CR LF, 21); break;
        case 9600:   strncpy(nmeaBaudSentence, "$PMTK251,9600*17" CR LF, 21); break;
        case 19200:  strncpy(nmeaBaudSentence, "$PMTK251,19200*22" CR LF, 21); break;
        case 39400:  strncpy(nmeaBaudSentence, "$PMTK251,38400*27" CR LF, 21); break;
        case 57600:  strncpy(nmeaBaudSentence, "$PMTK251,57600*2C" CR LF, 21); break;
        case 115200: strncpy(nmeaBaudSentence, "$PMTK251,115200*1F" CR LF, 21); break;
        default:
            usedBaud = 9600;
            strncpy(nmeaBaudSentence, "$PMTK251,9600*17" CR LF, 21);
    }
    Serial2.print(nmeaBaudSentence);


    // Note:
    // Serial2.end() Disables serial communication,
    // allowing the RX and TX pins to be used for general input and output.
    // To re-enable serial communication, call Serial.begin().
    Serial2.begin(usedBaud);
    // then have to change Serial2.begin() to agree
    sleepForSecs(1);
    if (DEVMODE) Serial.printf("setGpsBaud END %d" EOL, usedBaud);
}

//************************************************
void GpsINIT(void) {
    if (DEVMODE) Serial.println(F("GpsINIT START"));
    updateStatusLED();
    Watchdog.reset();

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

    // try making bigger
    Serial2.setFIFOSize(SERIAL2_FIFO_SIZE);
    // first talk at 9600
    Serial2.begin(9600);

    gpio_init(GpsPwr);
    pinMode(GpsPwr, OUTPUT);
    gpio_pull_up(GpsPwr);
    gpio_put(GpsPwr, LOW);

    //****************
    // FIX! are these doing anything?
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
    // sleep 3 secs
    sleepForSecs(3);
    //****************

    checkInitialGpsOutput();
    setGpsBalloonMode();

    // FIFO is big enough to hold output while we send more input here
    int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
    // then up the speed to desired (both gps chip and then Serial2
    setGpsBaud(desiredBaud);
    Serial.println(F("Should get some GPS output now at the new baud rate"));
    checkInitialGpsOutput();

    if (DEVMODE) Serial.println(F("GpsINIT END" EOL));
}

//************************************************
void GpsON(bool GpsColdReset) {
    if (DEVMODE) Serial.println(F("GpsON START" EOL));
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
        sleepForSecs(2);
        setGpsBalloonMode();
        // resets to 9600. set to new baud rate
        // FIFO is big enough to hold output while we send more input here
        int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
        // then up the speed to desired (both gps chip and then Serial2
        setGpsBaud(desiredBaud);
        checkInitialGpsOutput();
    }

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


    // Do the cold reset regardless of current state?
    if (GpsColdReset) {
        if (DEVMODE) Serial.println(F("GpsON full cold reset START"));
        setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
        // Cold Start. doesn't clear any system/user configs
        // Serial2.print("$PMTK103*30\r\n");
        // Full Cold Start. any system/user configs (back to factory status)
        // FIX! should we wait for ack or no?
        // have to toggle power off/on to get this effect? no?

        GpsStartTime = get_absolute_time();  // usecs
        // always do this just in case the GpsIsOn() got wrong?
        // but we're relying on the Serial2.begin/end to be correct?
        // might as well commit to being right!
        digitalWrite(GpsPwr, LOW);
        sleepForSecs(2);
        Serial.print("$PMTK104*37" CR LF);
        sleepForSecs(2);

        // FIX! we don't need to toggle power to get the effect?
        setGpsBalloonMode();

        // resets to 9600. set to new baud rate
        // FIFO is big enough to hold output while we send more input here
        int desiredBaud = checkGpsBaudRate(SERIAL2_BAUD_RATE);
        // then up the speed to desired (both gps chip and then Serial2
        setGpsBaud(desiredBaud);
        checkInitialGpsOutput();
        if (DEVMODE) Serial.println(F("GpsON full cold reset END"));
    }

    GpsIsOn_state = true;
    GpsTimeToLastFix = 0;

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
    GpsTimeToLastFix = 0;
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    if (DEVMODE) Serial.printf("GpsOFF END GpsIsOn_state %u" EOL, GpsIsOn_state);
}

//************************************************
// FIX! why was this static void before?
void updateGpsDataAndTime(int ms) {
    if (DEVMODE) Serial.println(F("updateGpsDataAndTime START"));
    // ms has to be positive?
    // grab data for no more than ms milliseconds
    // stop if no data for 10 milliseconds
    Watchdog.reset();
    GpsON(false);

    // don't need to wait for serial port to connect?
    // while (!Serial) {delay(1);}

    uint64_t start_millis = millis();
    uint64_t current_millis = start_millis;
    uint64_t last_serial2_millis = 0;

    Serial.printf("updateGpsDataAndTime started looking at %" PRIu64 " millis" EOL, current_millis);
    // unload each char as it arrives and prints it (with buffering, now)
    // so we can see NMEA sentences for a period of time.
    // assume 1 sec broadcast rate
    // https://arduino.stackexchange.com/questions/13452/tinygps-plus-library

    do {
        // FIX! what is this..unload gps sentences?
        current_millis = millis();
        // FIX! too much printing? lose GPS?
        if (false and DEVMODE) {
            if (Serial2.available())
                Serial.println(F("found some Serial2.available() (NMEA)"));
            else
                Serial.println(F("did not find any Serial2.available() (NMEA)"));
        }

        while (Serial2.available() > 0) {
            // FIX! in DEVMODE can we print all the sentences?
            char incomingChar = Serial2.read();
            gps.encode(incomingChar);
            if (DEVMODE) nmeaBufferAndPrint(incomingChar);
            current_millis = millis();
            // could the LED blinking have gotten delayed?
            updateStatusLED();
            last_serial2_millis = current_millis;
        }
        // did we wait more than 10 millis() since good data read?
        // did we wait more than 50 millis() since good data read?
        // did we wait more than 100 millis() since good data read?
        // early out (so we don't wait for the long ms time)
        if ((last_serial2_millis != 0) && (current_millis > last_serial2_millis + 100)) {
            Serial.printf("updateGpsDataAndTime stopped looking at %" PRIu64 " millis" EOL, current_millis);
            break;
        }
        // sleep for 1 second? will we get buffer overflow?
        sleepForSecs(1);

    } while ( (current_millis - start_millis) < (uint64_t) ms); // works if ms is 0

    // print/clear any accumulated NMEA sentence stuff
    if (DEVMODE) {
        nmeaBufferPrintAndClear(); // print and clear
        Serial.print(F(EOL));
    }

    if (DEVMODE) {
        Serial.printf("gps.time.isValid():%u" EOL, gps.time.isValid());
    }

    if (gps.time.isValid()) {
        // setTime is in the Time library.
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 0, 0, 0);
        if (DEVMODE) Serial.printf("setTime(%02u:%02u:%02u)" EOL,
                gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    updateStatusLED();
    if (DEVMODE) Serial.println(F("updateGpsDataAndTime END"));
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

    Serial.println();
    Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars FixStncs. Checksum"));
    Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  RX    RX        Fail"));
    Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));

    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    printInt(gps.hdop.value(), gps.hdop.isValid(), 5);

    // printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    // update to 12 to match
    printFloat(gps.location.lat(), gps.location.isValid(), 12, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);

    printInt(gps.location.age(), gps.location.isValid(), 5);
    printDateTime(gps.date, gps.time);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
    printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

    printInt(gps.charsProcessed(), true, 6); // does this just wrap wround if it's more than 6 digits?
    printInt(gps.sentencesWithFix(), true, 10); // does this just wrap wround if it's more than 10 digits?
    printInt(gps.failedChecksum(), true, 9);
    Serial.println();
    if (DEVMODE) Serial.println(F("GpsDebug END"));
    if (DEVMODE) Serial.println(F("GpsINIT END"));
}


// Notes:
// Arduino IDE allows function definitions after the point they are used
// used without needing an explicit function prototype before hand.

// The Arduino build creates these prototypes but not always correctly,
// leading to errors which are not obvious.

// Example: if the function argument list contains user defined data types and
// the automatically created function prototype is placed before the declaration of that data type.
