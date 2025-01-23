// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef LED_FUNCTIONS_H
#define LED_FUNCTIONS_H
#include <stdint.h>

//*********************************
extern const int STATUS_LED_PIN;

// these are the short blinks or long blinks, depending on value
const int LED_STATUS_NO_GPS = 1;        // 1 short
const int LED_STATUS_GPS_TIME = 2;      // 2 short
const int LED_STATUS_GPS_FIX = 3;       // 3 short
const int LED_STATUS_TX_WSPR = 4;       // 4 short
const int LED_STATUS_TX_TELEMETRY = 5;  // 1 long
const int LED_STATUS_TX_TELEN1 = 6;     // 2 long
const int LED_STATUS_TX_TELEN2 = 7;     // 3 long
const int LED_STATUS_TX_CW = 8;         // 4 long
const int LED_STATUS_REBOOT_NO_SERIAL = 8;  // 5 long
const int LED_STATUS_USER_CONFIG = 9;   // 6 long

void initStatusLED(void);
void setStatusLEDBlinkCount(int cnt);
void updateStatusLED(void);
void turnOnLED(bool turn_on);
bool isLEDOn(void);
void flipLED(void);
void blockingLongBlinkLED(uint8_t count);

#endif

