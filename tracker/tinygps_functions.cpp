// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"

// object for TinyGPSPlus state
extern TinyGPSPlus gps;

extern bool VERBY[10];
extern bool BALLOON_MODE;

extern uint64_t GpsTimeToLastFix = 0;  // milliseconds

extern uint32_t GpsInvalidAllCnt;
extern bool GpsInvalidAll;

//************************************************
// TinyGPSPlus can extract custom fields from any NMEA sentence.  
// TinyGPSPlus has built-in facilities for extracting latitude, longitude, altitude, etc., 
// from the $GPGGA and $GPRMC sentences.  
// With TinyGPSCustom type, can extract other NMEA fields, 
// even from non-standard NMEA sentences.

// $GPGGA - Global Positioning System Fix Data
// $GPGLL - Geographic position, latitude / longitude
// $GPGSA - GPS DOP and active satellites 
// $GPGSV - GPS Satellites in view

// Odd we only get the GNGGA, GNGGL when all 3 are enabled?
// ATGM336H: GNGGA, GNGGL
// ATGM336H: BDGSA, GLGSA, GPGSA
// ATGM336H: BDGSV, GLGSV, GPGSV
// FIX! expand to Galileo for USE_SIM65M ?

// By declaring TinyGPSCustom objects, we announce that we
// are interested in the 15th, 16th, and 17th fields in the $GPGSA sentence.
// Counting starts with the field immediately following the sentence name, 
// i.e. $GPGSA.  see http://aprs.gids.nl/nmea  and gps chip docs.

void tinyGpsCustomInit() {
    V1_println(F("tinyGpsCustomInit START"));
    TinyGPSCustom gp_sats(gps, "GPGSV", 3);
    TinyGPSCustom gp_pdop(gps, "GPGSA", 15); // 15th element
    TinyGPSCustom gp_hdop(gps, "GPGSA", 16);
    TinyGPSCustom gp_vdop(gps, "GPGSA", 17);

    TinyGPSCustom bd_sats(gps, "BDGSV", 3);
    TinyGPSCustom bd_pdop(gps, "BDGSA", 15);
    TinyGPSCustom bd_hdop(gps, "BDGSA", 16);
    TinyGPSCustom bd_vdop(gps, "BDGSA", 17);

    TinyGPSCustom gl_sats(gps, "GLGSV", 3);
    TinyGPSCustom gl_pdop(gps, "GLGSA", 15);
    TinyGPSCustom gl_hdop(gps, "GLGSA", 16);
    TinyGPSCustom gl_vdop(gps, "GLGSA", 17);

    V1_println(F("tinyGpsCustomInit End"));
}

//************************************************
void tinyGpsCustom() {
    V1_println(F("tinyGpsCustom START"));
    // From http://aprs.gids.nl/nmea/:
    // $GPGSV
    // GPS Satellites in view
    //
    // eg. $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
    //   $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
    //   $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D
    // 
    // 1    = Total number of messages of this type in this cycle
    // 2    = Message number
    // 3    = Total number of SVs in view
    // 4    = SV PRN number
    // 5    = Elevation in degrees, 90 maximum
    // 6    = Azimuth, degrees from true north, 000 to 359
    // 7    = SNR, 00-99 dB (null when not tracking)
    // 8-11 = Information about second SV, same as field 4-7
    // 12-15= Information about third SV, same as field 4-7
    // 16-19= Information about fourth SV, same as field 4-7
    // Every time anything is updated, print everything.
    if (gps.altitude.isUpdated() || gps.satellites.isUpdated() ||
        pdop.isUpdated() || hdop.isUpdated() || vdop.isUpdated()) {

        V1_print(F(EOL));
        V1_print(F(EOL));

        bool validA = gps.satellites.isValid() && !GpsInvalidAll;
        bool validB = gps.hdop.isValid() && !GpsInvalidAll;
        bool validB_gp = gps.gp_hdop.isValid() && !GpsInvalidAll;
        bool validB_bd = gps.bd_hdop.isValid() && !GpsInvalidAll;
        bool validB_gl = gps.gl_hdop.isValid() && !GpsInvalidAll;
        bool validC = gps.location.isValid() && !GpsInvalidAll;
        bool validD = gps.altitude.isValid() && !GpsInvalidAll;

        V1_printf("gps valids: %u %u %u %u %u %u %u %u" EOL,
            !GpsInvalidAll, validA, validB, validB_gp, validB_bd, validB_gl, validC, validD);

        printStr("Date", 11);
        printStr("Time", 9);
        printStr("Alt", 8);

        printStr("sats", 8);
        printStr("hdop", 8);
        printStr("vdop", 8);
        printStr("pdop", 8);

        printStr("gp_sats", 8);
        printStr("gp_hdop", 8);
        printStr("gp_vdop", 8);
        printStr("gp_pdop", 8);

        printStr("bd_sats", 8);
        printStr("bd_hdop", 8);
        printStr("bd_vdop", 8);
        printStr("bd_pdop", 8);

        printStr("gl_sats", 8);
        printStr("gl_hdop", 8);
        printStr("gl_vdop", 8);
        printStr("gl_pdop", 8);

        V1_print(F(EOL));

        // char debugMsg1[] = "Before printInt/Float/String tinyGpsCustom prints";
        // realPrintFlush(debugMsg1, false);  // no print

        if (VERBY[1]) {
            printGpsDateTime(gps.date, gps.time);  // gps.time.age() exists?
            printFloat(gps.altitude.meters(), validD, 8, 2);

            printInt(gps.satellites.value(), validA, 8);
            printInt(gps.hdop.value(), validB, 8);
            printInt(gps.vdop.value(), validB, 8);
            printInt(gps.pdop.value(), validB, 8);

            printInt(gps.gp_sats.value(), validB_gp, 8);
            printInt(gps.gp_hdop.value(), validB_gp, 8);
            printInt(gps.gp_vdop.value(), validB_gp, 8);
            printInt(gps.gp_pdop.value(), validB_gp, 8);

            printInt(gps.bd_sats.value(), validB_bd, 8);
            printInt(gps.bd_hdop.value(), validB_bd, 8);
            printInt(gps.bd_vdop.value(), validB_bd, 8);
            printInt(gps.bd_pdop.value(), validB_bd, 8);

            printInt(gps.gl_sats.value(), validB_gl, 8);
            printInt(gps.gl_hdop.value(), validB_gl, 8);
            printInt(gps.gl_vdop.value(), validB_gl, 8);
            printInt(gps.gl_pdop.value(), validB_gl, 8);
        }
        V1_print(F(EOL));
        V1_print(F(EOL));

        // char debugMsg1[] = "After printInt/Float/String tinyGpsCustom prints";
        // realPrintFlush(debugMsg1, false);  // no print

    V1_println(F("tinyGpsCustom END"));
}
