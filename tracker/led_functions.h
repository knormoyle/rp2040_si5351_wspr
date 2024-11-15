// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

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
extern const int LED_STATUS_REBOOT_NO_SERIAL;
extern const int LED_STATUS_USER_CONFIG;

void initStatusLED(void);
void setStatusLEDBlinkCount(int cnt);
void updateStatusLED(void);
void turnOnLED(bool turn_on);
bool isLEDOn(void);
void flipLED(void)       ;

#endif

