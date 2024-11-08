/* 
 * led_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

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

void initStatusLED(void);
void setStatusLEDBlinkCount(int cnt);
void updateStatusLED(void);
void turnOnLED(bool turn_on);
bool isLEDOn(void);
void flipLED(void)       ;

#endif

