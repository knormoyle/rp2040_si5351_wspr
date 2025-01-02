// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"
#include "solar5_functions.h"
#include "time_functions.h"
// for time_t things..used to get epoch time from rtc

// will this give me epoch time from rtc?
// #include <RTClib> // https://github.com/adafruit/RTClib

#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// don't rely on math.h?
// #define PI 3.1415926535897932384626433832795
// #define HALF_PI 1.5707963267948966192313216916398
// #define TWO_PI 6.283185307179586476925286766559

// #define DEG_TO_RAD 0.017453292519943295769236907684886
// #define RAD_TO_DEG 57.295779513082320876798154814105

extern bool VERBY[10];
extern TinyGPSPlus gps;

// #include <cmath>

#include "other_cpp/suncalc2.cpp"

//****************************************
// Solar zenith and azimuth angle (SZA, SAA) computation
// longitude longitude in degrees
// latitude latitude in degrees
// time_stamp time in UTC
// return SZA, SAA

// The calculations are based on geographic latitude and longitude of the observer 
// Positive numbers for Northern latitude and Eastern longitude, 
// Negative numbers for Southern latitude and Western longitude, and UTC time.

// forked from https://github.com/mourner/suncalc
// https://github.com/mourner/suncalc/blob/master/suncalc.js

// Return zenith and azimuth per standard api. Can change to be elevation.
// Elevation is corrected for refraction here, before being returned as zenith (sza)

void solarZenithAndAzimuthAngle5(double *sza, double *saa, double longitude, double latitude, time_t timestamp) {
    double elevation = 0;
    double azimuth = 0;
    *sza = 90 - elevation;
    *saa = azimuth;
}

//***********************************************************************

void calcSolarElevation5(double *solarElevation, double *solarAzimuth, double *solarDistance) {
    V1_print(F("calcSolarElevation4 START" EOL));

    // we check before setting rtc. assuming these are all valid ranges
    // no double-check!
    // int rtc_hour = hour();
    // int rtc_minute = minute();
    // int rtc_second = second();
    // int rtc_day = day();
    // int rtc_month = month();
    // int rtc_year = year();

    // secs since year 2000
    // best to subtract time objects on arduino?

    // I guess we're not going to qualify this by whether a 3d fix is valid,
    // or by it's age. we don't send down telemetry based on those qualifications
    // but alright to calc this to allow a 2d fix..but no calc if not valid
    // FIX! return 0 then??? we don't have any "valid" bit here?
    // we could return a big number?
    // knots telemetry # goes 0 to 41.
    // so maybe we'll just return 0, since 1 to 41 could be interesting
    // we'll flatline at 41 i.e. "knots/2"
    // does solar elevation go 0 to 180? I think not?

    // gps.location.lat() qualified by gps.location.isValid()
    // gps.location.lng() qualified by gps.location.isValid()
    // gps.location.age() qualified by gps.location.isValid()
    bool badSolar = false;
    if (!gps.location.isValid()) badSolar = true;

    // range check lat and lon?
    double lat = gps.location.lat();
    // FIX! is both 90 and -90 legal?. clamp to max ..and set bad
    if (lat < -90.0) {
        lat = -90.0;
        badSolar = true;
    }
    if (lat > 90.0) {
        lat = 90.0;
        badSolar = true;
    }

    double lon = gps.location.lng();
    // FIX! is both 180 and -180 legal. clamp to max ..and set bad
    if (lon < -180.0) {
        lon = -180.0;
        badSolar = true;
    }
    if (lon > 180.0) {
        lon = 180.0;
        badSolar = true;
    }

    // TimeLib has this. Is this epoch secs?
    // julian date of 1/1/2000 is JD 245,1545.0
    // #define SECS_YR_2000  (946684800UL) // the time at the start of y2k
    // yes, it's epoch time.
    // https://www.epochconverter.com/
    // 946684800
    // Timestamp to Human date
    // Assuming that this timestamp is in seconds:
    // GMT: Saturday, January 1, 2000 12:00:00 AM
    // Your time zone: Friday, December 31, 1999 5:00:00 PM GMT-07:00

    // FIX! what will the solar elevation be at night when I'm
    // testing at home qth at night!

    // Epoch Time (known as: Unix epoch, Unix time, POSIX time or Unix timestamp)
    // is the number of seconds that have elapsed since January 1, 1970
    // (midnight UTC/GMT),
    // not counting leap seconds (in ISO 8601: 1970-01-01T00:00:00Z).
    // SECS_YR_2000,

    // should be able to handle it from an 'unsigned long' for time_t ?
    uint64_t epochTime = getEpochTime();
    V1_printf("calculateSolarPosition4 epochTime %" PRIu64 " lat %.7f lon %.7f" EOL,
        epochTime, lat, lon);

    double sza;
    double saa;

    // lat lon are double. epochTime is time_t (uint64_t)
    // epochTime is the number of seconds since the Unix epoch (January 1, 1970, 00:00:00 UTC).
    solarZenithAndAzimuthAngle5(&sza, &saa, lon, lat, epochTime);
    // The solar zenith angle is the zenith angle of the sun, 
    // i.e., the angle between the sun's rays and the vertical direction. 
    // It is the complement to the solar altitude or solar elevation, 
    // which is the altitude angle or elevation angle between the sun's rays and a horizontal plane.
    double elevation = 90 - sza;
    double azimuth = saa;
    // FIX! printing before badSolar forces to 0?
    V1_printf("solarZenithAndAzimuthAngle5 elevation %.3f azimuth %.3f" EOL, elevation, azimuth);

    // returns degrees, not radians
    // solarElevation: can this be negative?.
    // solarAzimuth:   can this be negative?.
    // solarDistance:  can this be negative?
    double solarElevation_here = elevation;
    double solarAzimuth_here   = saa;
    double solarDistance_here  = 0;

    if (badSolar) {
        solarElevation_here = 0.0;
        solarAzimuth_here   = 0.0;
        solarDistance_here  = 0.0;
    }

    *solarElevation = solarElevation_here;
    *solarAzimuth   = solarAzimuth_here;
    *solarDistance  = solarDistance_here;

    V1_print(F("calcSolarElevation4 END" EOL));
}
