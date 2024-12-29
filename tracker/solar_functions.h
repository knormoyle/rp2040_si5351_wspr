// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php 
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef SOLAR_FUNCTIONS_H
#define SOLAR_FUNCTIONS_H
// #include <stdint.h>

// forked from original SolarPosition.h
// 2019 Ken Willmott
// https://github.com/KenWillmott/SolarPosition/blob/master/SolarPosition.h

// Arduino library based on the program "Arduino Uno and Solar Position Calculations"
// (c) David R. Brooks, which can be found at http://www.instesre.org/ArduinoDocuments.htm
// and issued under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License:
// https://creativecommons.org/licenses/by-nc-nd/4.0/

#include <Arduino.h>
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time

// kilometers per astronomical unit
const double KM_PER_AU = 149597870.7;  
typedef time_t(*getExternalTime)();

// data structure to store solar position results
struct SolarPosition_t {
  double elevation = 0;
  double azimuth = 0;
  double distance = 0;
  time_t time = 0;
};

// utility functions
uint64_t JulianDate(uint32_t year, uint32_t month, uint32_t day);
SolarPosition_t calculateSolarPosition(time_t tParam, double Latitude, double Longitude);

// class interface
class SolarPosition {
  private:
    // shared pointer to external sync function
    static getExternalTime getExtTimePtr;  
    // current values:
    double Latitude;
    double Longitude;
    // results:
    SolarPosition_t result;

  public:
    SolarPosition(double Latitude, double Longitude);
    static void setTimeProvider(getExternalTime getTimeFunction);
    SolarPosition_t getSolarPosition();
    SolarPosition_t getSolarPosition(time_t t);

    double getSolarElevation();
    double getSolarElevation(time_t t);
    double getSolarAzimuth();
    double getSolarAzimuth(time_t t);
    double getSolarDistance();
    double getSolarDistance(time_t t);
};

#endif  // SOLAR_FUNCTIONS_H
