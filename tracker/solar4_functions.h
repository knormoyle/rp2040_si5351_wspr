// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php 
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef SOLAR4_FUNCTIONS_H
#define SOLAR4_FUNCTIONS_H

// moved to short_solarfunctions.cpp
// #define INVALID_VALUE 999999999.999999d

void solarZenithAndAzimuthAngle4(double *sza, double *saa, double longitude, double latitude, time_t timeStamp);
void calcSolarElevation4(int *solarElevation, int *solarAzimuth, int *solarDistance);
time_t getEpochTime4();

// from solarfunctions.cpp originally
double degToRad(double degrees);
double radToDeg(double radians);
double normalizeDegrees180(double angle);
double normalizeDegrees360(double angle);
double normalizeRadiansPi(double angle);
double normalizeRadians2Pi(double angle);
double calculateJulianDate(long timestamp_utc);
double calculateJulianCenturyNumber(long timestamp_utc);
double calculateEccentricity(double julianCenturyNumber);
double calculateObliquity(double julianCenturyNumber);
double calculateCorrectedObliquity(double julianCenturyNumber);
double calculateCorrectedObliquity(double obliquity, double omega);
double calculateOmega(double julianCenturyNumber);
double calculateSolarEquationOfCenter(double julianCenturyNumber, double meanAnomaly);
double calculateSunMeanLongitude(double julianCenturyNumber);
double calculateDeclination(double julianCenturyNumber);
double calculateEquationOfTime(double julianCenturyNumber);
double calculateLocalSolarTime(long timestamp_utc, double equationOfTime, double longitude);
double calculateHourAngle(double localSolarTime);
double calculateSolarElevation(double hourAngle, double declination, double latitude);
double calculateSolarAzimuth(double hourAngle, double declination, double elevation, double latitude);
double calculateSolarZenithAngle(double trueelevation);
double calculateCorrectedSolarElevation(double elevation);
double calculateApproximateAtmosphericRefraction(double elevation);


#endif  // SOLAR4_FUNCTIONS_H
