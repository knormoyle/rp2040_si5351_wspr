// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// https://arduino-pico.readthedocs.io/en/latest/usb.html

//********************************************
// what do we need here
#include <stdint.h>

// stdlib https://www.tutorialspoint.com/c_standard_library/stdlib_h.htm
// #include <stdlib.h>
// #include <string.h>
// #include <cstdio.h>

#include "defines.h"
#include "debug_functions.h"
#include "print_functions.h"
#include "led_functions.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// decode of _verbose 0-9
extern bool VERBY[10];
extern bool BALLOON_MODE;

//***********************************************************
char drainSerialTo_CRorNL (uint32_t millis_max) {
    // Support hitting <enter> frantically to get to config menu right away on boot
    // this approximates 100 millis per loop iteration
    int max_iter = millis_max / 100;
    char incomingByte = 0;
    Watchdog.reset();
    if (!BALLOON_MODE) {
        int i;
        for (i = 0; i <= max_iter ; i++) {
            if (!Serial.available()) {
                updateStatusLED();
            }
            else {
                incomingByte = Serial.read();
                V1_println(incomingByte);
                // FIX! 13 is ascii CR \r.
                // FIX! 10 is ascii LF \n.
                // we don't drain past CR/LF. so if you hit enter, the stuff after that stays as input
                // readStringUntil() reads characters from the serial buffer into a String. 
                // The function terminates if it times out (see setTimeout()).
                // Serial.setTimeout() sets the maximum milliseconds to wait for serial data. 
                // defaults to 1000 milliseconds.

                if (incomingByte != 13) {
                    Serial.setTimeout(100);
                    Serial.readStringUntil(13); // empty readbuffer if there's data
                }
                if (incomingByte == 13) {
                    V1_println(F("Found Serial incomingByte == 13 (CR)..will not drain the rest"));
                    // what happens if there is \r\n...I guess it will go to the setup menu with the \n
                    break;
                }
                if (incomingByte == 10) {
                    V1_println(F("Found Serial incomingByte == 10 (LF)..will not drain the rest"));
                    break;
                }
            }
            Watchdog.reset();
            sleep_ms(100);
        }
        if (i > max_iter) {
            // V1_println("Must have timed out looking for input char(s) on Serial");
            ;
        }

    }
    // will always return single byte '0' if none found
    // otherwise return single byte char
    return incomingByte;
}

//***********************************************************
char getOneChar (uint32_t millis_max) {
    // this approximates 1000 millis per loop iteration
    int max_iter = millis_max / 100;
    char incomingByte = 0;
    Watchdog.reset();
    if (!BALLOON_MODE) {
        int i;
        for (i = 0; i <= max_iter ; i++) {
            if (!Serial.available()) {
                updateStatusLED();
            }
            else {
                incomingByte = Serial.read();
                // keep waiting if CR or LF
                if (incomingByte == 13 || incomingByte == 10) {
                    ;
                } else {
                    V1_println(incomingByte);
                    break;
                }
            }
            Watchdog.reset();
            sleep_ms(100);
        }
        if (i > max_iter) {
            V1_println("Must have timed out looking for input char on Serial");
        }

    }
    // will always return single byte '0' if none found
    // otherwise return single byte char
    return incomingByte;
}

