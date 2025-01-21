// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef TIME_FUNCTIONS_H
#define TIME_FUNCTIONS_H
#include <stdint.h>
#include <TinyGPS++.h>
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time

void printGpsDateTime(TinyGPSDate &d, TinyGPSTime &t, bool printAge);
time_t getEpochTime();

#endif
