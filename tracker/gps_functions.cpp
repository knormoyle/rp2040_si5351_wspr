// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// Watch out for:
// In the Arduino IDE, to reference a variable declared within a function from another part of
// your code, you need to declare the variable as "global" by defining it outside of any specific function,
// allowing access to it from anywhere in your sketch

// It is a special feature of the Arduino IDE that you can define functions after the point
// at which they are used without needing an explicit function prototype before hand.

// The Arduino build routine creates these prototypes but, unhappily, not always correctly,
// leading to errors which are not obvious.

// This is especially so if the function argument list contains user defined data types and
// the automatically created function prototype is placed before the declaration of that data type.

#include <Arduino.h>
#include "gps_functions.h"
#include "debug_functions.h"

// in libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// for setTime()
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time

// need this file to be .cpp because this uses type class
// for Watchdog.*
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// extern void updateStatusLED(void)
// instead, should we have
#include "led_functions.h"

// in the *ino
// how to reference?
// should we declare it here?
// refer to gps.* stuff?
// doesn't work

extern bool ublox_high_alt_mode_enabled;
extern TinyGPSPlus gps;
extern bool DEVMODE;

extern const int GpsPwr;
extern const int GPS_NRESET_PIN;
extern const int GPS_ON_PIN;

// input..not used..calibration?
extern const int GPS_1PPS_PIN;

extern const int GPS_UART1_RX_PIN;
extern const int GPS_UART1_TX_PIN;

extern bool GpsIsOn;
// for tracking gps fix time. we only power gps on/off..we don't send it gps reset commands
extern absolute_time_t GpsStartTime;  // usecs
extern uint64_t GpsTimeToLastFix;  // milliseconds

// FIX! gonna need an include for this? maybe note
// # include <TimeLib.h>

void GpsINIT() {
    Serial2.setRX(GPS_UART1_RX_PIN);
    Serial2.setTX(GPS_UART1_TX_PIN);
    Serial2.setPollingMode(true);
    Serial2.setFIFOSize(512);
    Serial2.begin(9600);  // GPS

    gpio_init(GpsPwr);
    gpio_pull_up(GpsPwr);
    gpio_put(GpsPwr, 0);

    gpio_init(GPS_NRESET_PIN);
    gpio_pull_up(GPS_NRESET_PIN);
    gpio_put(GPS_NRESET_PIN, 1);

    gpio_init(GPS_ON_PIN);
    gpio_pull_up(GPS_ON_PIN);
    gpio_put(GPS_ON_PIN, 1);

    // FIX! is this necessary?
    digitalWrite(GPS_NRESET_PIN, HIGH);
    digitalWrite(GPS_ON_PIN, HIGH);
}


void GpsON(bool GpsColdReset) {
    if (DEVMODE) {
        if (!GpsColdReset) printf("GpsON\n");
        else printf("GpsON with full gps cold start\n");*
    }
    // could be off or on already
    // Assume GpsINIT was already done

    // Just in case: wait for serial port to connect.
    // do we need these two each time?

    // while (!Serial2) { delay(1); }
    // Serial2.begin(9600);

    digitalWrite(GpsPwr, LOW);
    GpsStartTime = get_absolute_time();  // usecs

    // alternative GPS
    // SIM28ML
    // SIM28ML/9600 9.7x10.1mm

    // ATGM336H-5N11 9.7x10.1mm https://www.lcsc.com/datasheet/lcsc_datasheet_2304140030_ZHONGKEWEI-ATGM336H-5N11_C90769.pdf 5N-1X is gps only. saw + lnda
    // Ipeak = 100mA
    // ATGM336H-5N31 9.7x10.1mm https://www.lcsc.com/datasheet/lcsc_datasheet_1810261521_ZHONGKEWEI-ATGM336H-5N31_C90770.pdf 5N-3X is gps + bda. saw + lna

    // Ipeak = 100mA
    // ATGM336H-5NR32 10.1x9.7mm https://www.lcsc.com/datasheet/lcsc_datasheet_2411041759_ZHONGKEWEI-ATGM336H-5NR32_C5117921.pdf gps + bds. lna + saw

    // 6N-32 is gps + bd2/3. -74 has GLO. 115200baud
    // Ipeak = 100mA
    // ATGM336H-6N-74 10.1x9.7mm https://www.lcsc.com/datasheet/lcsc_datasheet_2401121833_ZHONGKEWEI-ATGM336H-6N-74_C5804601.pdf


    // https://www.sparkfun.com/datasheets/GPS/Modules/PMTK_Protocol.pdf
    // Cold Start. doesn't clear any system/user configs
    // Serial2.print("$PMTK103*30\r\n");

    // Full Cold Start. any system/user configs (back to factory status)
    // FIX! should we wait for ack or no?
    // have to toggle power off/on to get this effect? no?
    if (GpsColdReset) {
        sleep_ms(2000);
        Serial2.print("$PMTK104*37\r\n");
        sleep_ms(2000);
        // don't worry about setting balloon mode (3) for ATGM336?
        // Serial2.print("$PSIMNAV,W,3*3A\r\n");
        // normal mode
        // Serial2.print("$PSIMNAV,W,0*39\r\n");

        // don't need?
        // digitalWrite(GpsPwr, HIGH);
        // sleep_ms(2000);
        // digitalWrite(GpsPwr, LOW);
        // sleep_ms(2000);

        // ATGM336 might not support this:
        // could use as PMTK_API_SET_PWR_SAV_MODE (elsewhere) rather than powering off?
        // would have to wait for ack or time, after changing
        // https://www.meme.au/nmea-checksum.html
        // power saving mod off: (should this really be checksum 2F?)
        // $PMTK320,0*26\r\n" manual has wrong checksum here?
        // power saving mod on:
        // $PMTK320,1*2E\r\n"

        // FIX! do we need any config of the ATGM336?
        // just leaving this in for now. don't need for ATGM336
        if (!ublox_high_alt_mode_enabled) {
            // enable ublox high altitude mode if we have ublox
            /*
            setGPS_DynamicModel6();
            if (DEVMODE) {
                Serial.println(F("ublox DynamicModel6 enabled..."));
            }
            */
            ublox_high_alt_mode_enabled = true;
        }
        GpsIsOn = true;
        GpsTimeToLastFix = 0;
    }
}


/*
This used to be in the LightAPRS version of TinyGPSPlus-0.95
instead updated TinyGPSPlus (latest) in libraries to make them public, not private
< #if defined(ARDUINO_ARCH_RP2040)
< void TinyGPSDate::clear()
< {
<    valid = updated = false;
<    date = 0;
< }
< #endif
*/

void GpsOFF() {
    digitalWrite(GpsPwr, HIGH);
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

    /*printf("GpsOFF\n");*/
    GpsIsOn = false;
    GpsStartTime = 0;
    GpsTimeToLastFix = 0;
}

// FIX! why was this static void before?
void updateGpsDataAndTime(int ms) {
    // ms has to be positive?
    // grab data for no more than ms milliseconds
    // stop if no data for 10 milliseconds
    Watchdog.reset();
    GpsON();
    // while (!Serial) {delay(1);} // wait for serial port to connect.

    uint64_t start = millis();
    uint64_t bekle = 0;
    // this unloads each char as it arrives and prints it
    // so we can see NMEA sentences for a period of time. Should we do it for 2 secs?
    // assume 1 sec broadcast rate
    // https://arduino.stackexchange.com/questions/13452/tinygps-plus-library
    do {
        // FIX! what is this..unload gps sentences?
        while (Serial2.available() > 0) {
            // FIX! in DEVMODE can we print all the sentences?
            char c;
            // should this be readln?
            c = Serial2.read();
            gps.encode(c);
            bekle = millis();
        }
        // did we wait more than 10 millis() good data read?
        if ((bekle != 0) && (millis() > bekle+10)) break;
    } while ( (millis() - start) < (uint64_t) ms);

    if (DEVMODE) {
        printf("gps.time.isValid():%u\n", gps.time.isValid());
    }

    if (gps.time.isValid()) {
        // 11_6_24 0 instead of NULL (not pointer)
        // causes problems with the routines declares?
        // setTime is in the Time library.
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 0, 0, 0);
        if (DEVMODE) {
            printf("setTime(%02u:%02u:%02u)\n", gps.time.hour(), gps.time.minute(), gps.time.second());
        }
    }
}


void sendUBX(uint8_t *MSG, uint8_t len) {
    Serial2.write(0xFF);
    delay(500);
    for (int i = 0; i < len; i++) Serial2.write(MSG[i];
}

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

void gpsDebug() {
    if (!DEVMODE) return;

    Serial.println();
    Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars Sentences Checksum"));
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

    printInt(gps.charsProcessed(), true, 6);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 9);
    Serial.println();
}

