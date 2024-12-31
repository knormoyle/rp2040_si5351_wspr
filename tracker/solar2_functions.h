// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php 
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef SOLAR2_FUNCTIONS_H
#define SOLAR2_FUNCTIONS_H

#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time

// forked from SolarAngles.cpp

// https://github.com/david-salac/Fast-SZA-and-SAA-computation/tree/master
// https://github.com/david-salac/Fast-SZA-and-SAA-computation/blob/master/CPP_version/SolarAngles.cpp
// Fast algorithm for the computation of the SZA and SAA
// Author: David Salac https://github.com/david-salac/
// The implementation of the fast algorithm for computation of the Solar Zenith Angle (aka SZA)
// and Solar Azimut Angle (aka SAA) based on the logic proposed by Roberto Grena in 2012
// https://doi.org/10.1016/j.solener.2012.01.024.
// The precision of the algorithm is 0.3 degrees for the SZA and 0.5 degrees f
// or the SAA (mean-average error).

void solarZenithAndAzimuthAngle2(double *sza, double *saa, double longitude, double latitude, time_t timeStamp);
void calcSolarElevation2(double *solarElevation, double *solarAzimuth, double *solarDistance);

time_t getEpochTime2();

#endif  // SOLAR2_FUNCTIONS_H
