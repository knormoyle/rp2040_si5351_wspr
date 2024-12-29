// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"

#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// forked from SolarPosition.cpp
// https://github.com/KenWillmott/SolarPosition/blob/master/SolarPosition.cpp
// 2019 Ken Willmott
// Arduino library based on the program "Arduino Uno and Solar Position Calculations"
// (c) David R. Brooks, which can be found at http://www.instesre.org/ArduinoDocuments.htm
// issued under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License:
// https://creativecommons.org/licenses/by-nc-nd/4.0/

#include "SolarPosition.h"

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
        (uint64_t) ( 365.25  * (year + 4716) ) +
        (uint32_t)  ( 30.6001 * (month + 1) ) +
        ( day + B - 1524) ;

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

        JD_whole = JulianDate(
            tmYearToCalendar(timeCandidate.Year),
            timeCandidate.Month,
            timeCandidate.Day
        );

        JD_frac = (
            (timeCandidate.Hour + (timeCandidate.Minute / 60.0) + (timeCandidate.Second / 3600.0)) /
            (24.0 - 0.5);
        );

        elapsedT = JD_whole - Y2K_JULIAN_DAY;
        elapsedT = (elapsedT + JD_frac) / DAYS_PER_JULIAN_CENTURY;
        solarLongitude   = DEG_TO_RAD * fmod(280.46645 + (36000.76983 * elapsedT), 360);
        solarMeanAnomaly = DEG_TO_RAD * fmod(357.5291  + (35999.0503  * elapsedT), 360);

        earthOrbitEccentricity = 0.016708617 - (0.000042037 * elapsedT);
        sunCenter = DEG_TO_RAD * (
            ( (1.9146   - (0.004847 * elapsedT)) * sin(    solarMeanAnomaly) ) +
            ( (0.019993 - (0.000101 * elapsedT)) * sin(2 * solarMeanAnomaly) ) +
            ( 0.00029                            * sin(3 * solarMeanAnomaly) )
        );

        solarTrueAnomaly = solarMeanAnomaly + sunCenter;

        // original code relied on left to right multiply divider precedence?
        // parens to be obvious
        // https://www.tutorialspoint.com/cprogramming/c_operators_precedence.htm
        equatorObliquity = DEG_TO_RAD * (
            23 +
            (26 / 60.0) +
            (21.448 / 3600.0) -
            ((46.815 / 3600) * elapsedT)
        );

        JDx = JD_whole - Y2K_JULIAN_DAY;
        GreenwichHourAngle = 280.46061837 + (
            ((360 * JDx) % 360) +
            (0.98564736629 * JDx) +
            (360.98564736629 * JD_frac)
        );
        GreenwichHourAngle = fmod(GreenwichHourAngle, 360.0);
        solarTrueLongitude = fmod(sunCenter + solarLongitude, TWO_PI);

        rightAscension = atan2(
            sin(solarTrueLongitude) * cos(equatorObliquity),
            cos(solarTrueLongitude)
        );

        Declination = asin(sin(equatorObliquity) * sin(solarTrueLongitude));
        hourAngle = (DEG_TO_RAD * GreenwichHourAngle) + Longitude - rightAscension;

        // results:
        result.distance = 1.000001018 *
            (1 - earthOrbitEccentricity * earthOrbitEccentricity) /
            (1 + earthOrbitEccentricity * cos(solarTrueAnomaly);

        // elevation from the horizon
        result.elevation = asin(
            (sin(Latitude) * sin(Declination)) +
            (cos(Latitude) * (cos(Declination) * cos(hourAngle))
        );

        // Azimuth measured eastward from north.
        result.azimuth = PI +
            atan2(
                sin(hourAngle),
                (cos(hourAngle) * sin(Latitude)) - (tan(Declination) * cos(Latitude))
            );

        // copy the time
        result.time = tParam;

        // remember the parameters
        timePrevious = tParam;
        latPrevious = Latitude;
        lonPrevious = Longitude;
    }
    return result;
}

