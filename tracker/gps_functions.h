// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef GPS_FUNCTIONS_H
#define GPS_FUNCTIONS_H
#include <stdint.h>

bool GpsIsOn(void);
void GpsINIT(void);
bool GpsFullColdReset(void);
bool GpsWarmReset(void);
// why did this not compile if void in param?
void invalidateTinyGpsState();
void GpsON(bool GpsColdReset);
void GpsOFF();
uint32_t updateGpsDataAndTime(int ms);
void checkUpdateTimeFromGps(uint32_t dollarStarTime);
void gpsDebug(void);
void writeGpsConfigNoBroadcastToFlash(void);

// these probably shouldn't be used outside gps_functions.cpp ?
void nmeaBufferPrintAndClear(void);
void nmeaBufferAndPrint(const char charToAdd, bool printIfFull);
void nmeaBufferFastPoll(uint32_t duration_millis, bool printIfFull);

void gpsSleepForMillis(int n, bool enableEarlyOut);
bool getInitialGpsOutput(void);
void setGpsBalloonMode(void);
void setGpsBroadcast(void);
void disableGpsBroadcast(void);
void setGpsConstellations(int desiredConstellations);
void setGpsBaud(int desiredBaud);
void kazuClocksSlow(void);
void kazuClocksRestore(uint32_t PLL_SYS_MHZ_restore);

#endif // GPS_FUNCTIONS_H
