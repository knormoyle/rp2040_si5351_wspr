// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef LED_FUNCTIONS_H
#define LED_FUNCTIONS_H
#include <stdint.h>

extern const int STATUS_LED_PIN;

extern const int LED_STATUS_NO_GPS;
extern const int LED_STATUS_GPS_TIME;
extern const int LED_STATUS_GPS_FIX;
extern const int LED_STATUS_TX_WSPR;
extern const int LED_STATUS_TX_TELEMETRY;
extern const int LED_STATUS_TX_TELEN1;
extern const int LED_STATUS_TX_TELEN2;
extern const int LED_STATUS_TX_CW;
extern const int LED_STATUS_REBOOT_NO_SERIAL;
extern const int LED_STATUS_USER_CONFIG;

void initStatusLED(void);
void setStatusLEDBlinkCount(int cnt);
void updateStatusLED(void);
void turnOnLED(bool turn_on);
bool isLEDOn(void);
void flipLED(void);
void blockingLongBlinkLED(uint8_t count);

#endif

