/*
 * led_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#include <Arduino.h>
#include "led_functions.h"

extern const int STATUS_LED_PIN;
extern const int LED_STATUS_NO_GPS;
extern const int LED_STATUS_GPS_TIME;
extern const int LED_STATUS_GPS_FIX;
extern const int LED_STATUS_TX_WSPR;
extern const int LED_STATUS_TX_TELEMETRY;
extern const int LED_STATUS_TX_TELEN1;
extern const int LED_STATUS_TX_TELEN2:

const int LED_BLINK_ON_PERIOD_USEC=50000;
const int LED_BLINK_OFF_PERIOD_USEC=300000;
const int LED_BLINK_PAUSE_PERIOD_USEC=1000000;

int statusLEDBlinkCnt=0;

void initStatusLED(void)
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  turnOnLED(true);
}

void setStatusLEDBlinkCount(int cnt)
{
  statusLEDBlinkCnt = cnt;
}

void updateStatusLED(void)
{
  static uint32_t nextFlipUsec = 0;
  static int targetBlinkCnt = 0;
  static int currBlinkCnt = 0;

  uint32_t usec = time_us_32();
  if ((int32_t)(nextFlipUsec - usec) <= 0) {
    if (isLEDOn() == false) {
      // OFF to ON
      if (targetBlinkCnt == 0) {
        targetBlinkCnt = statusLEDBlinkCnt;
        currBlinkCnt = 0;
      }
      if (++currBlinkCnt <= targetBlinkCnt) {
        turnOnLED(true);
      }
      nextFlipUsec = usec + LED_BLINK_ON_PERIOD_USEC;
    }
    else {
      // ON to OFF
      turnOnLED(false);
      if (currBlinkCnt >= targetBlinkCnt) {
        nextFlipUsec = usec + LED_BLINK_PAUSE_PERIOD_USEC;
        targetBlinkCnt = 0;
      }
      else {
        nextFlipUsec = usec + LED_BLINK_OFF_PERIOD_USEC;
      }
    }
  }
}