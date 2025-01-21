// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
//********************************************
#include "time_functions.h"
#include "print_functions.h"
#include "debug_functions.h"
#include "gps_functions.h"
#include "led_functions.h"

// decode of _verbose 0-9
extern bool VERBY[10];

// why was this static
// with arduino, can't we just use printf to stdout rather than V1_print() ?
void printGpsDateTime(TinyGPSDate &d, TinyGPSTime &t, bool printAge) {
    if (!VERBY[1]) return;
    char sz[32];
    if (d.isValid()) {
        snprintf(sz, sizeof(sz), "%04d-%02d-%02d ", d.year(), d.month(), d.day());
        V1_print(sz);
    } else {
        V1_print(F("********** "));
    }

    if (t.isValid()) {
        snprintf(sz, sizeof(sz), "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        V1_print(sz);
    } else {
        V1_print(F("******** "));
    }

    if (printAge) printInt(d.age(), d.isValid(), 6);
    // whenever something might have taken a long time like printing
    updateStatusLED();
}


//***********************************************************************
// Function that gets current epoch time
time_t getEpochTime() {
    // from PaulStoffregen Timelib.h
    // https://github.com/PaulStoffregen/Time
    // now()  returns the current time as seconds since Jan 1 1970

    // now() will be wrong if we didn't set time correctly from gps
    int rtc_year = year();
    int rtc_month = month();
    int rtc_day = day();
    int rtc_hour = hour();
    int rtc_minute = minute();
    int rtc_second = second();

    // this is a unsigned long?
    time_t epoch_now = now();
    V1_print(F("getEpochTime (utc)"));
    if (false) {
        V1_printf(" year %d month %d day %d hour %d minute %d second %d epoch_now %" PRIu64 EOL,
            rtc_year, rtc_month, rtc_day, rtc_hour, rtc_minute, rtc_second, epoch_now);
    } else {
        V1_printf(" %04d-%02d-%02d", rtc_year, rtc_month, rtc_day);
        V1_printf(" %02d:%02d:%02d", rtc_hour, rtc_minute, rtc_second);
        V1_printf(" epoch_now %" PRIu64 EOL, epoch_now);
    }

    return epoch_now;
}

