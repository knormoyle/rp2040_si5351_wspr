// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#include <Arduino.h>
#include "led_functions.h"

const int LED_BLINK_ON_PERIOD_USEC = 50000;
const int LED_BLINK_OFF_PERIOD_USEC = 300000;
const int LED_BLINK_PAUSE_PERIOD_USEC = 1000000;

int statusLEDBlinkCnt = 0;

void initStatusLED(void) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    turnOnLED(true);
}

void setStatusLEDBlinkCount(int cnt) {
    statusLEDBlinkCnt = cnt;
}

void updateStatusLED(void) {
    static uint32_t nextFlipUsec = 0;
    static int targetBlinkCnt = 0;
    static int currBlinkCnt = 0;

    uint32_t usec = time_us_32();
    if ((int32_t) (nextFlipUsec - usec) <= 0) {
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
        } else {
            // ON to OFF
            turnOnLED(false);
            if (currBlinkCnt >= targetBlinkCnt) {
                nextFlipUsec = usec + LED_BLINK_PAUSE_PERIOD_USEC;
                targetBlinkCnt = 0;
            } else {
                nextFlipUsec = usec + LED_BLINK_OFF_PERIOD_USEC;
            }
        }
    }
}

void turnOnLED(bool turn_on) {
    digitalWrite(STATUS_LED_PIN, (turn_on) ? HIGH : LOW);
}

bool isLEDOn(void) {
    return (digitalRead(STATUS_LED_PIN) ? true : false);
}

void flipLED(void) {
    turnOnLED(!isLEDOn());
}

// other non-blocking examples to look at
// parallel led blinking code
// Output library supports HIGH, LOW, TOGGLE, PULSE, BLINK_WITHOUT_DELAY.
// https://github.com/ArduinoGetStarted/output
// https://github.com/ArduinoGetStarted/output/tree/master/examples

