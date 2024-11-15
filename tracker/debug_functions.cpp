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

// Incorporates work by: Roman Piksaykin R2BDY. Thank you.
// https://github.com/RPiks/pico-WSPR-tx

// Got rid of this and use Serial everywhere
// #define SerialUSB Serial

extern bool DEVMODE;
#include "debug_functions.h"
#include "led_functions.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// for TinyGPSDate definition
#include <TinyGPS++.h>
#include <Wire.h>

// why was this static?
// is this used for signed or unsigned?
void printInt(uint64_t val, bool valid, int len) {
    if (!DEVMODE) return;
    char sz[32] = "*****************";
    // FIX! should this really be %ld? why not %d
    // int64_t should use 
    // printf("%" PRId64 "\n", t);
    // uint64_t should use 
    // printf("%" PRIu64 "\n", t);
    // to print in hexadecimal
    // printf("%" PRIx64 "\n", t);

    //  https://stackoverflow.com/questions/9225567/how-to-portably-print-a-int64-t-type-in-c
    // complete list of types and formats
    // https://en.cppreference.com/w/cpp/types/integer


    if (valid) snprintf(sz, sizeof(sz), "%" PRIu64, val);
    sz[len] = 0;
    // puts blanks after the null char for the rest of the buffer?
    for (int i = strlen(sz); i < len; ++i) sz[i] = ' ';
    if (len > 0) sz[len - 1] = ' ';
    Serial.print(sz);

    // whenever something might have taken a long time like printing
    updateStatusLED();
}

// why was this static
// with arduino, can't we just use printf to stdout rather than Serial.print() ?
void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
    if (!DEVMODE) return;
    char sz[32];
    if (!d.isValid()) {
        Serial.print(F("********** "));
    } else {
        snprintf(sz, sizeof(sz), "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        Serial.print(sz);
    }

    if (!t.isValid()) {
        Serial.print(F("******** "));
    } else {
        snprintf(sz, sizeof(sz), "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        Serial.print(sz);
    }
    printInt(d.age(), d.isValid(), 5);
    // whenever something might have taken a long time like printing
    updateStatusLED();
}

// why was this static
// with arduino, can't we just use printf to stdout rather than Serial.print() ?
void printStr(const char *str, int len) {
    if (!DEVMODE) return;
    int slen = strlen(str);
    for (int i = 0; i < len; ++i) Serial.print(i < slen ? str[i] : ' ');
    // whenever something might have taken a long time like printing
    updateStatusLED();
}


// why was this static?
// with arduino, can't we just use printf to stdout rather than Serial.print() ?
void printFloat(float val, bool valid, int len, int prec) {
    if (!DEVMODE) return;

    if (!valid) {
    while (len-- > 1) Serial.print('*');
        Serial.print(' ');
    } else {
        Serial.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i = flen; i < len; ++i) Serial.print(' ');
    }
    // whenever something might have taken a long time like printing
    updateStatusLED();

}

//********************************************
// how many of these do we need here?

#include <stdint.h>
#include <stdarg.h>

// do we need so we can slam in some printf?
// #include <cstdio.h>
// does stdio_init_all imply we want this
#include <stdio.h>

// stdlib https://www.tutorialspoint.com/c_standard_library/stdlib_h.htm
// #include <stdlib.h>
// #include <string.h>

// #include "hardware/clocks.h"
// #include "pico/stdlib.h"

// maybe 10 lines?
#define BUFFER_SIZE 1024

static char logBuffer[BUFFER_SIZE] = {0};

// Buffers the log information to the log buffer
// it is much faster than direct UART output
// pformat: printf style format
// ... argument list to print
void StampPrintf(const char* pformat, ...) {
    // background on i/o
    // interesting info on tracking GPIO transitions with IRQ and callback function
    // https://ceworkbench.wordpress.com/2023/01/04/using-the-raspberry-pi-pico-gpio-with-the-c-c-sdk/

    static uint32_t sTick = 0;

    uint64_t tm_us = to_us_since_boot(get_absolute_time());
    const uint32_t tm_day = (uint32_t)(tm_us / 86400000000ULL);
    tm_us -= (uint64_t)tm_day * 86400000000ULL;
    const uint32_t tm_hour = (uint32_t)(tm_us / 3600000000ULL);
    tm_us -= (uint64_t)tm_hour * 3600000000ULL;
    const uint32_t tm_min = (uint32_t)(tm_us / 60000000ULL);
    tm_us -= (uint64_t)tm_min * 60000000ULL;
    const uint32_t tm_sec = (uint32_t)(tm_us / 1000000ULL);
    tm_us -= (uint64_t)tm_sec * 1000000ULL;

    char timestamp[64];
    // create timestamp
    snprintf(timestamp, sizeof(timestamp),
        "%02lud%02lu:%02lu:%02lu.%06llu [%04lu] ",
        tm_day, tm_hour, tm_min, tm_sec, tm_us, sTick++);

    // va_start stdarg https://www.tutorialspoint.com/c_standard_library/c_macro_va_start.htm
    va_list argptr;
    va_start(argptr, pformat);
    char message[BUFFER_SIZE];
    // vsnprintf https://cplusplus.com/reference/cstdio/vsnprintf/
    // format the message
    vsnprintf(message, sizeof(message), pformat, argptr);
    va_end(argptr);

    // kevin. cheap enough check as long as timestamp + message <= 150 bytes?
    if ( (strlen(timestamp) + strlen(message + 1)) >= (BUFFER_SIZE - (strlen(logBuffer) + 1)) ) {
    // make BUFFER_SIZE bigger or do more DoLogPrint() if we run into a problem realtime
    // FIX! should we detect when we're close to the logBuffer being full?
        printf("WARNING: with BUFFER_SIZE %d strlen(logBuffer) %d there is no room for timestamp %s message %s <newline>",
            BUFFER_SIZE, strlen(logBuffer), timestamp, message);
        printf("..flushing with DoLogPrint");
        DoLogPrint();
    }
    strncat(logBuffer, timestamp, BUFFER_SIZE - strlen(logBuffer) - 1);
    strncat(logBuffer, message, BUFFER_SIZE - strlen(logBuffer) - 1);
    strncat(logBuffer, "\n", BUFFER_SIZE - strlen(logBuffer) - 1);
}

// Outputs the content of the log buffer to stdio (UART and/or USB)
// Direct output to UART is very slow so we will do it in CPU idle times
// and not in time critical functions
void DoLogPrint() {
    Watchdog.reset();
    if (logBuffer[0] != '\0') {
        printf("%s", logBuffer);
        logBuffer[0] = '\0';  // Clear the buffer
    }
    // whenever something might have taken a long time like printing the big buffer
    updateStatusLED();
    Watchdog.reset();

}
