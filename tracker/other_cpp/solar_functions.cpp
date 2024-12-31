// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"
#include "solar_functions.h"
// for time_t things..used to get epoch time from rtc

// will this give me epoch time from rtc?
// #include <RTClib> // https://github.com/adafruit/RTClib

#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// forked from SolarPosition.cpp
// https://github.com/KenWillmott/SolarPosition/blob/master/SolarPosition.cpp
// 2019 Ken Willmott
// Arduino library based on the program "Arduino Uno and Solar Position Calculations"
// (c) David R. Brooks, which can be found at http://www.instesre.org/ArduinoDocuments.htm
// paper describing:
// https://instesre.org/ArduinoUnoSolarCalculations.pdf

// Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License
// https://creativecommons.org/licenses/by-nc-nd/4.0/

// don't rely on math.h?
// #define PI 3.1415926535897932384626433832795
// #define HALF_PI 1.5707963267948966192313216916398
// #define TWO_PI 6.283185307179586476925286766559

// #define DEG_TO_RAD 0.017453292519943295769236907684886
// #define RAD_TO_DEG 57.295779513082320876798154814105

extern bool VERBY[10];
extern TinyGPSPlus gps;

//***********************************************************************
// pointer to external sync function
getExternalTime SolarPosition::getExtTimePtr = NULL;

// constructor
SolarPosition::SolarPosition(const double Lat, const double Lon) {
    Latitude = Lat * DEG_TO_RAD;
    Longitude = Lon * DEG_TO_RAD;
}

//***********************************************************************
// public methods:
// assign a Time provider function
void SolarPosition::setTimeProvider(getExternalTime getTimeFunction) {
    getExtTimePtr = getTimeFunction;
}

//***********************************************************************
// Get current Position
SolarPosition_t SolarPosition::getSolarPosition() {
    const SolarPosition_t nullPos;
    if (getExtTimePtr != NULL) {
        time_t timeNow = getExtTimePtr();
        SolarPosition_t pos;
        result = calculateSolarPosition(timeNow, Latitude, Longitude);
        pos.elevation = result.elevation * RAD_TO_DEG;
        pos.azimuth = result.azimuth * RAD_TO_DEG;
        pos.distance = result.distance * KM_PER_AU;
        pos.time = timeNow;
        return pos;
    } else {
        return nullPos;
    }
}

//***********************************************************************
// Get Position for specified time
SolarPosition_t SolarPosition::getSolarPosition(time_t t) {
    SolarPosition_t pos;
    result = calculateSolarPosition(t, Latitude, Longitude);
    pos.elevation = result.elevation * RAD_TO_DEG;
    pos.azimuth = result.azimuth * RAD_TO_DEG;
    pos.distance = result.distance * KM_PER_AU;
    pos.time = t;
    return pos;
}

//***********************************************************************
// Get current Elevation
double SolarPosition::getSolarElevation() {
    if (getExtTimePtr != NULL) {
        result = calculateSolarPosition(getExtTimePtr(), Latitude, Longitude);
        return result.elevation * RAD_TO_DEG;
    } else {
        return 0;
    }
}

//***********************************************************************
// Get Elevation for specified time
double SolarPosition::getSolarElevation(time_t t) {
    result = calculateSolarPosition(t, Latitude, Longitude);
    return result.elevation * RAD_TO_DEG;
}

//***********************************************************************
// Get current Azimuth
double SolarPosition::getSolarAzimuth() {
    if (getExtTimePtr != NULL) {
        result = calculateSolarPosition(getExtTimePtr(), Latitude, Longitude);
        return result.azimuth * RAD_TO_DEG;
    } else {
        return 0;
    }
}

//***********************************************************************
// Get Azimuth for specified time
double SolarPosition::getSolarAzimuth(time_t t) {
    result = calculateSolarPosition(t, Latitude, Longitude);
    return result.azimuth * RAD_TO_DEG;
}

//***********************************************************************
// Get current Solar distance in AU
double SolarPosition::getSolarDistance() {
    if (getExtTimePtr != NULL) {
        result = calculateSolarPosition(getExtTimePtr(), Latitude, Longitude);
        return result.distance * KM_PER_AU;
    } else {
        return 0;
    }
}

//***********************************************************************
// Get Solar distance in AU for specified time
double SolarPosition::getSolarDistance(time_t t) {
    result = calculateSolarPosition(t, Latitude, Longitude);
    return result.distance * KM_PER_AU;
}

// end of public methods

// beginning of utility functions
//***********************************************************************
uint64_t JulianDate(uint32_t year, uint32_t month, uint32_t day) {
    uint64_t JD_whole;
    uint32_t A, B;
    if (month <= 2) {
        year--;
        month += 12;
    }
    A = year / 100;
    B = 2 - A + (A / 4);
    JD_whole =
        (uint64_t)( 365.25  * (year + 4716) ) +
        (uint32_t)( 30.6001 * (month + 1) ) +
        (day + B - 1524);

    return JD_whole;
}

//***********************************************************************
SolarPosition_t calculateSolarPosition(time_t tParam, double Latitude, double Longitude) {
    const double DAYS_PER_JULIAN_CENTURY = 36525.0;
    const uint64_t Y2K_JULIAN_DAY = 2451545;

    tmElements_t timeCandidate;
    static time_t timePrevious = 0;
    static double latPrevious;
    static double lonPrevious;
    static SolarPosition_t result;

    uint64_t JD_whole;
    uint64_t JDx;

    double JD_frac;
    double rightAscension;
    double Declination;
    double hourAngle;
    double GreenwichHourAngle;
    double elapsedT;
    double solarLongitude;
    double solarMeanAnomaly;
    double earthOrbitEccentricity;
    double sunCenter;
    double solarTrueLongitude;
    double solarTrueAnomaly;
    double equatorObliquity;

    // only calculate if time or location has changed
    if (tParam != timePrevious || Latitude != latPrevious || Longitude != lonPrevious) {
        breakTime(tParam, timeCandidate);

        // this was broken up so 32 bit fp would work well? still fine with 64-bit fp
        JD_whole = JulianDate(
            tmYearToCalendar(timeCandidate.Year),
            timeCandidate.Month,
            timeCandidate.Day);

        JD_frac = (
                (timeCandidate.Hour +
                (timeCandidate.Minute / 60.0) +
                (timeCandidate.Second / 3600.0)
            ) / (24.0 - 0.5));

        elapsedT = JD_whole - Y2K_JULIAN_DAY;
        elapsedT = (elapsedT + JD_frac) / DAYS_PER_JULIAN_CENTURY;
        solarLongitude   = DEG_TO_RAD * fmod(280.46645 + (36000.76983 * elapsedT), 360);
        solarMeanAnomaly = DEG_TO_RAD * fmod(357.5291  + (35999.0503  * elapsedT), 360);

        earthOrbitEccentricity = 0.016708617 - (0.000042037 * elapsedT);
        sunCenter = DEG_TO_RAD * (
            ((1.9146   - (0.004847 * elapsedT)) * sin(1 * solarMeanAnomaly)) +
            ((0.019993 - (0.000101 * elapsedT)) * sin(2 * solarMeanAnomaly)) +
            (0.00029                            * sin(3 * solarMeanAnomaly)) );

        solarTrueAnomaly = solarMeanAnomaly + sunCenter;

        // original code relied on left to right multiply divider precedence?
        // parens to be obvious
        // https://www.tutorialspoint.com/cprogramming/c_operators_precedence.htm
        equatorObliquity = DEG_TO_RAD * (
            23 +
            (26 / 60.0) +
            (21.448 / 3600.0) -
            ((46.815 / 3600) * elapsedT) );

        JDx = JD_whole - Y2K_JULIAN_DAY;
        GreenwichHourAngle = 280.46061837 + (
            ((360 * JDx) % 360) +
            (0.98564736629 * JDx) +
            (360.98564736629 * JD_frac));

        GreenwichHourAngle = fmod(GreenwichHourAngle, 360.0);
        solarTrueLongitude = fmod(sunCenter + solarLongitude, TWO_PI);

        rightAscension = atan2(
            sin(solarTrueLongitude) * cos(equatorObliquity),
            cos(solarTrueLongitude));

        Declination = asin(sin(equatorObliquity) * sin(solarTrueLongitude));
        hourAngle = (DEG_TO_RAD * GreenwichHourAngle) + Longitude - rightAscension;

        // results:
        result.distance = 1.000001018 * (
            (1 - (earthOrbitEccentricity * earthOrbitEccentricity)) /
            (1 + (earthOrbitEccentricity * cos(solarTrueAnomaly)) ) );

        // elevation from the horizon
        result.elevation = asin(
            (sin(Latitude) * sin(Declination)) +
            (cos(Latitude) * cos(Declination) * cos(hourAngle)));

        // Azimuth measured eastward from north.
        result.azimuth = PI +
            atan2(sin(hourAngle),
                (cos(hourAngle) * sin(Latitude)) - (tan(Declination) * cos(Latitude)));

        // copy the time
        result.time = tParam;

        // remember the parameters
        timePrevious = tParam;
        latPrevious = Latitude;
        lonPrevious = Longitude;
    }
    return result;
}

// solar position object. Means we can store the position. Might be interesting.
SolarPosition_t savedPosition;

//***********************************************************************
void calcSolarElevation(double *solarElevation, double *solarAzimuth, double *solarDistance) {
    V1_print(F("calcSolarElevation START" EOL));

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
    V1_printf("calculateSolarPosition epochTime %" PRIu64 " lat %.7f lon %.7f" EOL,
        epochTime, lat, lon);

    savedPosition = calculateSolarPosition(epochTime, lat * DEG_TO_RAD, lon * DEG_TO_RAD);

    // returns degrees, not radians
    // solarElevation: can this be negative?. decimal, integer accuracy
    // solarAzimuth:   can this be negative?. decimal, integer accuracy
    // solarDistance:  can this be negative? maybe error case. kilometers.
    double solarElevation_here = (savedPosition.elevation * RAD_TO_DEG);
    double solarAzimuth_here   = (savedPosition.azimuth * RAD_TO_DEG);
    double solarDistance_here  = (savedPosition.distance * KM_PER_AU);

    // The solar zenith angle is the zenith angle of the sun,
    // i.e., the angle between the sun's rays and the vertical direction.
    // It is the complement to the solar altitude or solar elevation,
    // which is the altitude angle or elevation angle between the sun's rays and a horizontal plane.
    // FIX! this actually gives us elevation?
    double elevation = solarElevation_here;
    double azimuth = solarAzimuth_here;
    // FIX! printing before badSolar forces to 0?
    V1_printf("solarZenithAndAzimuthAngle elevation %.3f azimuth %.3f" EOL, elevation, azimuth);

    if (badSolar) {
        solarElevation_here = 0.0;
        solarAzimuth_here   = 0.0;
        solarDistance_here  = 0.0;
    }

    *solarElevation = solarElevation_here;
    *solarAzimuth   = solarAzimuth_here;
    *solarDistance  = solarDistance_here;

    V1_print(F("calcSolarElevation END" EOL));
}

//***********************************************************************
// Function that gets current epoch time
time_t getEpochTime() {
    // from PaulStoffregen Timelib.h
    // https://github.com/PaulStoffregen/Time
    // now()  returns the current time as seconds since Jan 1 1970

    // now() will be wrong if we didn't set time correctly from gps
    int rtc_year = year();
    int rtc_month = month();
    int rtc_day = day();
    int rtc_hour = hour();
    int rtc_minute = minute();
    int rtc_second = second();

    // this is a unsigned long?
    time_t epoch_now = now();
    V1_print("getEpochTime (utc)"); 
    V1_printf(" year %d month %d day %d hour %d minute %d second %d epoch_now %" PRIu64 EOL,
        rtc_year, rtc_month, rtc_day, rtc_hour, rtc_minute, rtc_second, epoch_now);

    return epoch_now;
}
//***********************************************************************


