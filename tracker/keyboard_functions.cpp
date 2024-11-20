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

extern bool DEVMODE;
// decode of _verbose 0-9
extern bool VERBY[10];

//***********************************************************
bool drainSerialTo_CRorNL (void) {
    // Support hitting <enter> frantically to get to config menu right away on boot
    int i;
    char incomingByte = { 0 };
    bool found_CRorNL = false;
    bool found_any = false;
    for (i = 0; i < 1; i++) {
        Watchdog.reset();
        if (!Serial.available()) {
            Serial.println(F("Good! no Serial.available() ..sleep and reverify"));
            updateStatusLED();
            sleep_ms(200);
        }
        else {
            found_any = true;
            incomingByte = Serial.read();
            Serial.println(incomingByte);
            // FIX! 13 is ascii CR \r.
            // FIX! 10 is ascii LF \n.
            // we don't drain past CR/LF. so if you hit enter, the stuff after that stays as input
            if (incomingByte == 13) {
                Serial.println(F("Uh-oh. Found Serial incomingByte == 13 (CR)..will not drain the rest"));
                // what happens if there is \r\n...I guess it will go to the setup menu with the \n
                found_CRorNL = true;
                break;
            }
            if (incomingByte == 10) {
                Serial.println(F("Uh-oh. Found Serial incomingByte == 10 (CR)..will not drain the rest"));
                found_CRorNL = true;
                break;
            }
        }
    }
    return found_CRorNL | found_any;
}

