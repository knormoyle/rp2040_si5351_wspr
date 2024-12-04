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
void invalidateTinyGpsState(void);
void GpsON(bool GpsColdReset);
void GpsOFF(bool keepTinyGpsState);
void updateGpsDataAndTime(int ms);
void sendUBX(uint8_t *MSG, uint8_t len);
void setGPS_DynamicModel6();
void gpsDebug(void);

#endif
