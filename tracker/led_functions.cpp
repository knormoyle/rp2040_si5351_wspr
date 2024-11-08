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

#include <Arduino.h>
#include "led_functions.h"

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

void turnOnLED(bool turn_on)  
{
    digitalWrite(STATUS_LED_PIN, (turn_on) ? HIGH : LOW);
}

bool isLEDOn(void) 
{
    return (digitalRead(STATUS_LED_PIN) ? true : false);
}

void flipLED(void) 
{
    turnOnLED(!isLEDOn());
}


