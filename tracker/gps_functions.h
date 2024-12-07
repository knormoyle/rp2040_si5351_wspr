// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef GPS_FUNCTIONS_H
#define GPS_FUNCTIONS_H
#include <stdint.h>

// why was this here?
// #include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
// what about the tiny gps library? already included in the .ino?

//************************************************
bool GpsIsOn(void);
void GpsINIT(void);
void GpsFullColdReset(void);
void GpsWarmReset(void);
void invalidateTinyGpsState(void);
void GpsON(bool GpsColdReset);
void GpsOFF(bool keepTinyGpsState);
void updateGpsDataAndTime(int ms);
void gpsDebug(void);

// these probably shouldn't be used outside gps_functions.cpp ?
void nmeaBufferPrintAndClear(void);
void nmeaBufferAndPrint(const char charToAdd, bool printIfFull);
void sleepForMilliSecs(int n, bool enableEarlyOut);
void drainInitialGpsOutput(void);
void setGpsBalloonMode(void);
void setGpsBroadcast(void);
void disableGpsBroadcast(void);
void setGpsConstellations(int desiredConstellations);
void setGpsBaud(int desiredBaud);


#endif
