// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#include <Arduino.h>
#include "led_functions.h"
#include <Adafruit_SleepyDog.h> 

const int LED_BLINK_ON_PERIOD_USEC = 50000;
const int LED_BLINK_OFF_PERIOD_USEC = 300000;
const int LED_BLINK_PAUSE_PERIOD_USEC = 1000000;

const int LONG_LED_BLINK_ON_PERIOD_USEC = LED_BLINK_ON_PERIOD_USEC * 3;
const int LONG_LED_BLINK_OFF_PERIOD_USEC = LED_BLINK_OFF_PERIOD_USEC * 3;
const int LONG_LED_BLINK_PAUSE_PERIOD_USEC = LONG_LED_BLINK_PAUSE_PERIOD_USEC * 3;

// note this is global ..so static
int statusLEDBlinkCnt = 0;

//********************
void initStatusLED(void) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    turnOnLED(true);
}

//********************
void setStatusLEDBlinkCount(int cnt) {
    statusLEDBlinkCnt = cnt;
}
//********************
void updateStatusLED(void) {
    // no prints allowed here. can be used while USB printing doesn't work
    // but not protected by BALLOON_MODE or VERBY
    static uint32_t nextFlipUsec = 0;
    static int targetBlinkCnt = 0;
    static int currBlinkCnt = 0;

    bool longBlinks = false;
    int blinkCntInit;

    if (statusLEDBlinkCnt >= 8) {
        // threshold for indicating 3, 4, ... long blinks
        blinkCntInit = statusLEDBlinkCnt - 5;
        longBlinks = true;
    } else {
        // short blinks
        blinkCntInit = statusLEDBlinkCnt;
        longBlinks = false;
    }

    uint32_t usec = time_us_32();
    if ((int32_t) (nextFlipUsec - usec) <= 0) {
        if (isLEDOn() == false) {
            // OFF to ON
            if (targetBlinkCnt == 0) {
                targetBlinkCnt = blinkCntInit;
                currBlinkCnt = 0;
            }
            if (++currBlinkCnt <= targetBlinkCnt) {
                turnOnLED(true);
            }
            if (longBlinks) nextFlipUsec = usec + LONG_LED_BLINK_ON_PERIOD_USEC;
            else nextFlipUsec = usec + LED_BLINK_ON_PERIOD_USEC;

        } else {
            // ON to OFF
            turnOnLED(false);
            if (currBlinkCnt >= targetBlinkCnt) {
                targetBlinkCnt = 0;
                if (longBlinks) nextFlipUsec = usec + LONG_LED_BLINK_PAUSE_PERIOD_USEC;
                else nextFlipUsec = usec + LED_BLINK_PAUSE_PERIOD_USEC;
            } else {
                if (longBlinks) nextFlipUsec = usec + LONG_LED_BLINK_OFF_PERIOD_USEC;
                else nextFlipUsec = usec + LED_BLINK_OFF_PERIOD_USEC;
            }
        }
    }
}

//********************
void turnOnLED(bool turn_on) {
    digitalWrite(STATUS_LED_PIN, (turn_on) ? HIGH : LOW);
}

bool isLEDOn(void) {
    return (digitalRead(STATUS_LED_PIN) ? true : false);
}

void flipLED(void) {
    turnOnLED(!isLEDOn());
}

//********************
void blockingLongBlinkLED(uint8_t count) {
    for (int i = 0; i < count ; i++ ) {
        Watchdog.reset();
        turnOnLED(true);
        sleep_ms(1000);
        turnOnLED(false);
        sleep_ms(1000);
    }
}

// other non-blocking examples to look at
// parallel led blinking code
// Output library supports HIGH, LOW, TOGGLE, PULSE, BLINK_WITHOUT_DELAY.
// https://github.com/ArduinoGetStarted/output
// https://github.com/ArduinoGetStarted/output/tree/master/examples

