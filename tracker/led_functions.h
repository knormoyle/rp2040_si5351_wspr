/* 
 * led_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#ifndef LED_FUNCTIONS_H
#define LED_FUNCTIONS_H
#include <stdint.h>

void initStatusLED(void);
void setStatusLEDBlinkCount(int cnt);
void updateStatusLED(void);
void turnOnLED(bool turn_on);
bool isLEDOn(void);
void flipLED(void)       ;

#endif

