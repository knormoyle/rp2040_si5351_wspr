// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
//********************************************
// what do we need here?
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

// for measureMyFreqs?
#include "pico/stdlib.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

// stdlib https://www.tutorialspoint.com/c_standard_library/stdlib_h.htm
// #include <cstdio.h>
// #include <stdlib.h>
// #include <string.h>

#include "print_functions.h"
#include "debug_functions.h"
#include "led_functions.h"
#include "gps_functions.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// for TinyGPSDate definition
#include <TinyGPS++.h>

// decode of _verbose 0-9
extern bool VERBY[10];
extern bool GpsInvalidAll;
extern TinyGPSPlus gps;

//***********************************
// background on i/o
// interesting info on tracking GPIO transitions with IRQ and callback function
// https://ceworkbench.wordpress.com/2023/01/04/using-the-raspberry-pi-pico-gpio-with-the-c-c-sdk/

//***********************************
// got a "full" with 1024..had 996 things in it
// from updateGpsDataAndTime()
// #define LOG_BUFFER_SIZE 2048
#define LOG_BUFFER_SIZE 4096
// was this too big and running out of memory?
// #define LOG_BUFFER_SIZE 8192

// Individual message can be up to the full buffer size?
// wasteful to be that big. at most maybe 150 bytes?
#define MESSAGE_BUFFER_SIZE 150

static char logBuffer[LOG_BUFFER_SIZE] = { 0 };
char message[MESSAGE_BUFFER_SIZE] = { 0 };

// Buffers the log information to the log buffer
// it is much faster than direct UART output
// pformat: printf style format
// ... argument list to print
void StampPrintf(const char* pformat, ...) {
    if (!VERBY[1]) return;
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
    // FIX! clean up old stuff
    if (false) {
        snprintf(timestamp, sizeof(timestamp),
            "%02lud%02lu:%02lu:%02lu.%06llu [%04lu] ",
            tm_day, tm_hour, tm_min, tm_sec, tm_us, sTick++);
    } else {
        // the min/sec are not aligned with the gps synchonied time
        // but good for measuring delays between things
        // don't need day.
        snprintf(timestamp, sizeof(timestamp),
            "%02lu:%02lu.%06llu [%04lu] ",
            tm_min, tm_sec, tm_us, sTick++);
    }

    // va_start stdarg https://www.tutorialspoint.com/c_standard_library/c_macro_va_start.htm
    va_list argptr;
    va_start(argptr, pformat);
    // message can be up to the full buffer size?
    // wasteful to be that big. at most maybe 150 bytes?
    // vsnprintf https://cplusplus.com/reference/cstdio/vsnprintf/
    // format the message
    vsnprintf(message, sizeof(message), pformat, argptr);
    va_end(argptr);

    // make LOG_BUFFER_SIZE bigger and recompile
    // or do more DoLogPrint() if we run into a problem realtime

    // Put the 3 things in the logBuffer more efficiently than
    // doing successive strlen()' and strncat?
    int i = strlen(logBuffer);
    int j = strlen(timestamp);
    int j2 = strlen(message);
    // will the message fit even and empty buffer?
    bool ignore = false;
    if ((j + j2 + 1) > LOG_BUFFER_SIZE) {
        V1_printf(
            EOL "ERROR: LOG_BUFFER_SIZE %d, is too small for j + j2 + 1 = %d, "
            "timestamp %s message %s and EOL. ..ignoring" EOL EOL,
            LOG_BUFFER_SIZE, j + j2 + 1, timestamp, message);
        ignore = true;
    // Will the message fit without dumping the existing buffer?
    } else if ((i + j + j2 + 1) > LOG_BUFFER_SIZE) {
        // create timestamp
        snprintf(timestamp, sizeof(timestamp),
            "%02lud%02lu:%02lu:%02lu.%06llu [%04lu] ",
            tm_day, tm_hour, tm_min, tm_sec, tm_us, sTick++);
        V1_printf(
            EOL "ERROR: LOG_BUFFER_SIZE %d, i %d, adding j + j2 + 1 = %d, "
            "has no room for timestamp %s message %s (+ EOL)" EOL EOL,
            LOG_BUFFER_SIZE, i, j + j2 + 1, timestamp, message);
        V1_println(F("..flushing by discarding all of logBuffer first"));
        logBuffer[0] = 0;
        i = 0;
    }

    if (!ignore) {
        strncpy(logBuffer + i, timestamp, j);
        strncpy(logBuffer + i + j, message, j2);
        logBuffer[i + j + j2] = 0;  // null term
    }
}

//***********************************
// Outputs the content of the log buffer to stdio (UART and/or USB)
// Direct output to UART is very slow so we will do it in CPU idle times
// and not in time critical functions
void DoLogPrint() {
    if (!VERBY[1]) return;
    Watchdog.reset();
    if (logBuffer[0] != 0) {
        V1_println(logBuffer);
        logBuffer[0] = 0;  // Clear the buffer. no need to zero the whole thing.
    }
    // whenever something might have taken a long time like printing the big buffer
    updateStatusLED();
    Watchdog.reset();
}

//***********************************
// from: https://sourcevu.sysprogs.com/rp2040/examples/clocks/hello_48MHz/files/hello_48MHz.c#tok293
// don't namespace collide with measure_freqs() in SPI.h?
void measureMyFreqs(void) {
    if (!VERBY[1]) return;
    V1_print(F("measureMyFreqs START" EOL));

    // see for frequency_count_khz()
    // evidently uses a special frequency counter ??
    // https://sourcevu.sysprogs.com/rp2040/picosdk/files/src/rp2_common/hardware_clocks/clocks.c#tok966
    // what about this? interesting for something? what is clock_handle_t?
    // probably 'clk_sys' looking at people's use of clock_configure()
    // https://sourcevu.sysprogs.com/rp2040/picosdk/files/src/rp2_common/hardware_clocks/clocks.c#tok909
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_clocks/clocks.c

    // uint32_t clock_get_hz(clock_handle_t clock) {

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__clocks.html#gae78816cc6112538a12adcc604be4b344
    // clk_ref  Watchdog and timers reference clock.
    // clk_sys  Processors, bus fabric, memory, memory mapped registers.
    // clk_peri  Peripheral clock for UART and SPI.
    // clk_usb  USB clock.
    // clk_adc  ADC clock.
    // clk_rtc  Real time clock.
    uint32_t f_clk_sys_get_hz = clock_get_hz(clk_sys);
    uint32_t f_clk_peri_get_hz = clock_get_hz(clk_peri);
    uint32_t f_clk_usb_get_hz = clock_get_hz(clk_usb);
    uint32_t f_clk_ref_get_hz = clock_get_hz(clk_ref);

    // frequency_count_khz() is accurate to +-1Khz
    uint32_t f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint32_t f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint32_t f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);

    uint32_t f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint32_t f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint32_t f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint32_t f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
    // accuracy is just +-1Khz here?
    uint32_t f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
#endif

    V1_printf("clk_sys_get_hz  = %luHz" EOL, f_clk_sys_get_hz);
    V1_printf("clk_peri_get_hz = %luHz" EOL, f_clk_peri_get_hz);
    V1_printf("clk_usb_get_hz  = %luHz" EOL, f_clk_usb_get_hz);
    V1_printf("clk_ref_get_hz  = %luHz" EOL, f_clk_ref_get_hz);

    V1_printf("pll_sys  = %lukHz" EOL, f_pll_sys);
    V1_printf("pll_usb  = %lukHz" EOL, f_pll_usb);
    V1_printf("rosc     = %lukHz" EOL, f_rosc);
    V1_printf("clk_sys  = %lukHz" EOL, f_clk_sys);
    V1_printf("clk_peri = %lukHz" EOL, f_clk_peri);
    V1_printf("clk_usb  = %lukHz" EOL, f_clk_usb);
    V1_printf("clk_adc  = %lukHz" EOL, f_clk_adc);
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
    V1_printf("clk_rtc  = %lukHz" EOL, f_clk_rtc);
#endif

    // Can't measure clk_ref / xosc as it is the ref
    V1_print(F("measureMyFreqs END" EOL));
    V1_flush();
    // FIX! need a delay after flush also? (usb can be disabled soon)
    busy_wait_ms(1000);
}

//***********************************
// interesting strategies for detecting buffer sizes in linux
// https://stackoverflow.com/questions/15543193/retrieving-buffer-packet-payload-sizes-for-usb-serial-write-transfer-in-userspac

// default? (for usb serial)
#define SERIAL_TX_BUFFER_SIZE 256

// https://docs.arduino.cc/language-reference/en/functions/communication/serial/availableForWrite/
// Serial.availableForWrite()
// Get the number of bytes (characters) available for writing in the serial buffer
// without blocking the write operation
void realPrintFlush(char *debugMsg, bool print) {
    if (!VERBY[1]) return;
    uint32_t avail = Serial.availableForWrite();
    uint32_t avail_last = 0;
    if (print) {
        Serial.printf("%s" EOL, debugMsg);
        Serial.printf("realPrintFlush avail %lu" EOL, avail);
    }
    uint32_t iter = 0;
    // FIX! does it ever get bigger than 256?
    // get two consective samples of avail that are the same
    while (avail < 256 || (avail != avail_last)) {
        sleep_ms(100);
        avail_last = avail;
        avail = Serial.availableForWrite();
        // once a sec
        if (print && ((iter % 10) == 0))
            Serial.printf("realPrintFlush avail %lu iter %lu" EOL, avail, iter);
        updateStatusLED();
        // wait at most 10 secs
        iter++;
        if (iter > (10 * 10)) break;
    }
    Serial.flush();
    avail = Serial.availableForWrite();
    if (print) {
        Serial.printf("realPrintFlush after final Serial.flush(): avail %lu iter %lu" EOL, 
            avail, iter);
    }
}

//***********************************
// is this used for signed or unsigned?
// len should be < 32
void printInt(uint64_t val, bool valid, int len) {
    if (!VERBY[1]) return;
    // FIX! should this really be %ld? why not %d
    // int64_t should use
    // printf("%" PRId64 "\n", t);
    // uint64_t should use
    // printf("%" PRIu64 "\n", t);
    // to print in hexadecimal
    // printf("%" PRIx64 "\n", t);

    // https://stackoverflow.com/questions/9225567/how-to-portably-print-a-int64-t-type-in-c
    // complete list of types and formats
    // https://en.cppreference.com/w/cpp/types/integer
    char sz[32] = "*****************";
    int lenm1 = len - 1;
    // https://cplusplus.com/reference/cstdio/printf/
    if (!valid) {
        sz[lenm1] = 0; // terminator placed for len
        Serial.print(sz);
        // add a space on the right
        Serial.print(F(" "));
    } else {
        // variable in format. right justified is the default
        // https://www.geeksforgeeks.org/using-variable-format-specifier-c/
        // add a space on the right
        Serial.printf("%*" PRIu64 " ", lenm1, val);
    }

    // whenever something might have taken a long time like printing
    updateStatusLED();
}

// added valid for printStr too
void printStr(const char *str, bool valid, int len) {
    if (!VERBY[1]) return;
    char sz[32] = "*****************";
    int lenm1 = len - 1;
    if (!valid) {
        sz[lenm1] = 0; // terminator placed for len
        Serial.print(sz);
        // add a space on the right
        Serial.print(F(" "));
    } else {
        // variable printf in format. right justified is the default
        // https://www.geeksforgeeks.org/using-variable-format-specifier-c/
        // add a space on the right
        Serial.printf("%*s ", lenm1, str);
    }

    // whenever something might have taken a long time like printing
    updateStatusLED();
}

void printFloat(double val, bool valid, int len, int prec) {
    if (!VERBY[1]) return;
    // new way. 
    // variable in printf format. right justified is the default
    // https://cplusplus.com/reference/cstdio/printf/
    char sz[32] = "*****************";
    int lenm1 = len - 1;
    if (!valid) {
        sz[lenm1] = 0; // terminator placed for len
        Serial.print(sz);
        // add a space on the right
        Serial.print(F(" "));
    } else {
        // variable in format. right justified is the default
        // https://www.geeksforgeeks.org/using-variable-format-specifier-c/
        // add a space on the right
        Serial.printf("%*.*f ", lenm1, prec, val);
    }
    // whenever something might have taken a long time like printing
    Serial.flush();
    updateStatusLED();
}

//**********************************
void freeMem() {
    V1_print(F("freeMem START" EOL));
    if (!VERBY[1]) return;
    // Nice to use F() for strings that are constant
    // compiled string stays in flash.
    // does not get copied to SRAM during the C++ initialization
    // string it has the PROGMEM property and runs from flash.
    V1_print(F("Free RAM: "));
    V1_print(freeMemory(), DEC);
    V1_println(F(" byte. Why is this < 0?. Is it's calc wrong?"));

    V1_print(F("Free Heap: "));
    V1_print(rp2040.getFreeHeap(), DEC);
    V1_println(F(" byte"));

    V1_print(F("Used Heap: "));
    V1_print(rp2040.getUsedHeap(), DEC);
    V1_println(F(" byte"));

    V1_print(F("Total Heap: "));
    V1_print(rp2040.getTotalHeap(), DEC);
    V1_println(F(" byte"));

    // https://forum.arduino.cc/t/trying-to-make-sense-of-ram-usage/622666
    // char __stack = 0;

    // V1_print(F("__brkval="));
    // V1_println((unsigned int)__brkval);
    // V1_print(F("__malloc_heap_start="));
    // V1_println((unsigned int)__malloc_heap_start);
    // V1_print("__flp=");
    // V1_println((unsigned int)__flp);
    // V1_print("__stack=");
    // V1_println((unsigned int)&__stack);
    // V1_print("stack size=");
    // V1_println(RAM_end - (unsigned int)&__stack);
    // V1_print("Heap size=");
    // V1_println((unsigned int)__brkval - RAM_start);

    V1_print(F("freeMem END" EOL));
}

//**********************************
extern const int GPS_WAIT_FOR_NMEA_BURST_MAX; 
void gpsResetTest() {
    static uint32_t count = 0;
    // FIX! currently don't have cold reset support
    V0_print(F("do_gpsResetTest START" EOL));

    if (count == 0) {
        V0_print(F("need gps init and cold reset once to fully setup gps"));
        GpsINIT();
        GpsOFF(false);  // don't keep TinyGPS state
        GpsON(false);   // no gps cold reset
        GpsOFF(false);  // don't keep TinyGPS state
        count += 1;
        V0_print(F("do_gpsResetTest END"));
        V0_flush();
        return;
    }

    bool fix_valid_all = false;
    uint32_t tries = 0;
    uint64_t duration_millis = 0;
    uint64_t start_millis = millis();
    while (!fix_valid_all) {
        Watchdog.reset();
        tries += 1;
        // we can look at time/drift updates here
        updateGpsDataAndTime(GPS_WAIT_FOR_NMEA_BURST_MAX);
        fix_valid_all = !GpsInvalidAll &&
            gps.time.isValid() &&
            (gps.date.year() >= 2024 && gps.date.year() <= 2034) &&
            gps.satellites.isValid() && (gps.satellites.value() >= 3) &&
            gps.hdop.isValid() &&
            gps.altitude.isValid() &&
            gps.location.isValid() &&
            gps.speed.isValid() &&
            gps.course.isValid();

        duration_millis = millis() - start_millis;
        if (duration_millis > 240000) {  // 4 * 60 = 240 secs
            V0_printf("ERROR: gpsResetTest %lu: no fix after %" PRIu64 " millisecs" EOL, 
                count, duration_millis);
            break;
        }
        // by not sleeping, we should get better timing accuracy?
        // maybe check that at least 1 sec has elapsed for each loop iteration
        V0_flush();
        Watchdog.reset();
        // could be negative
        int64_t wait_millis =  1000 - (int64_t) duration_millis;
        if (wait_millis > 0) {
            V0_printf("will sleep for wait_millis %" PRId64 EOL, wait_millis);
            sleep_ms(wait_millis);
        }
    }

    if (fix_valid_all) {
        if (duration_millis < 60000) { // 60 secs
            V0_printf("GOOD: gpsResetTest %lu: fix took %" PRIu64 " millisecs" EOL, 
                count, duration_millis);
        } else {
            V0_printf("ERROR: gpsResetTest %lu: fix (too slow) took %" PRIu64 " millisecs" EOL, 
                count, duration_millis);
        }
        // prints out the summary info
        gpsDebug();
    }
    GpsOFF(false);  // don't keep TinyGPS state
    count += 1;
    V0_print(F("do_gpsResetTest END"));
    V0_flush();
}

