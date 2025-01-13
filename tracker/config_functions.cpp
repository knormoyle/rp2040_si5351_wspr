// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
// for the reboot to bootloader mode
#include "pico/bootrom.h"

// for isprint()
#include <ctype.h>

// my stuff
#include "print_functions.h"
#include "gps_functions.h"
#include "led_functions.h"
#include "u4b_functions.h"
#include "keyboard_functions.h"
// just so we can do some tests?
#include "si5351_functions.h"
#include "wspr_functions.h"
#include "i2c_functions.h"
#include "config_functions.h"
#include "cw_functions.h"

//*****************************************************
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// FIX! is the program bigger than 1M ?
// Once done, we can access this at XIP_BASE + FLASH_TARGET_OFFSET
#define FLASH_TARGET_OFFSET (4 * 256 * 1024)

// https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html
// https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html#ga2ad3247806ca16dec03e655eaec1775f


// https://github.com/raspberrypi/pico-examples/blob/master/flash/program/flash_program.c
// Flash is "execute in place" and so will be in use when any code that is stored in flash runs,
// e.g. an interrupt handler or code running on a different core.
//
// Calling flash_range_erase or flash_range_program at the same time 
// as flash is running code would cause a crash.
// flash_safe_execute disables interrupts and tries to cooperate with the 
// other core to ensure flash is not in use
// See the documentation for flash_safe_execute and its assumptions and limitations
// int flash_safe_execute (void(*) (void *) func,
// void * param,
// uint32_t enter_exit_timeout_ms
// )
// Returns
// PICO_OK on success (the function will have been called).
// PICO_TIMEOUT on timeout (the function may have been called).
// PICO_ERROR_NOT_PERMITTED if safe execution is not possible 
// (the function will not have been called).
// PICO_ERROR_INSUFFICIENT_RESOURCES if the method fails due to dynamic resource exhaustion
// (the function will not have been called)
// I'm getting -4 ??

// Execute a function with IRQs disabled and with the other core 
// also not executing/reading flash
// int rc = flash_safe_execute(call_flash_range_erase, (void*)FLASH_TARGET_OFFSET, UINT32_MAX);
// hard_assert(rc == PICO_OK);

// https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html
// https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html#ga2ad3247806ca16dec03e655eaec1775f
// Initialize a core such that the other core can lock it out during flash_safe_execute.
// see the dire need at the link above (around flash access)
// did this in setup1() and setup()
// flash_safe_execute_core_init();


// already defined?
// #define FLASH_SECTOR_SIZE 4096
// #define FLASH_PAGE_SIZE 256

//**************************************
extern char _callsign[7];
extern char _verbose[2];   // 0 is used to disable all. 1 is all printing for now. 2:9 same
extern char _TELEN_config[5];
extern char _clock_speed[4];
extern char _U4B_chan[4];  // 1 to 3 digits?
extern char _Band[3];      // string with 2, 10, 12, 15, 17, 20 legal. null at end
extern char _Band_cw[3];   // string with 2, 10, 12, 15, 17, 20 legal. null at end
// https://rfzero.net/documentation/rf/
// The table below shows the typical output power vs. current in the output stages 
// running in push-pull with a T1 transformer
// Current [mA] 
// 137 kHz    1 MHz     10 MHz    30 MHz    50 MHz    200 MHz
// 8 9,5 dBm  14,2 dBm  14,5 dBm  15,0 dBm  14,5 dBm  13,3 dBm
// 6 9,2 dBm  12,8 dBm  13,3 dBm  13,7 dBm  13,0 dBm  11,8 dBm
// 4 8,3 dBm  10,3 dBm  10,7 dBm  11,0 dBm  10,5 dBm   9,7 dBm
// 2 5,2 dBm   4,7 dBm   5,0 dBm   5,5 dBm   5,0 dBm   4,5 dBm

// Note running at 4ma output is just 4 dBm reduction. so maybe that should be our low power?
extern char _tx_high[2];   // 0 is 4mA si5351. 1 is 8mA si5351
extern char _testmode[2];  // currently: sweep telemetry

// don't allow more than approx. 43 hz "correction" on a band. leave room for 6 chars
extern char _correction[7];  // parts per billion -30000 to 30000. default 0
// traquito: 500 correction does  ~7 hz lower on 20M (14095.600 base freq)
// traquito: 500 correction does ~14 hz lower on 10M (28124.600 base freq)

// test only: 1 means you don't wait for starting minute from _U4B_chan
// does wait for any 2 minute alignment though
extern char _go_when_rdy[2];
extern char _factory_reset_done[2];
extern char _use_sim65m[2];
extern char _morse_also[2];   
extern char _solar_tx_power[2];

extern char _id13[3];
extern char _start_minute[2];
extern char _lane[2];

//**************************************
// decodes from _Band _U4B_chan
extern uint32_t XMIT_FREQUENCY;
extern const uint32_t DEFAULT_PLL_SYS_MHZ;
extern uint32_t PLL_SYS_MHZ;  // decode of _clock_speed

// PWM stuff gets recalc'ed if PLL_SYS_MHZ changes
extern uint32_t PWM_DIV;
extern uint32_t PWM_WRAP_CNT;
// this is fixed
extern const uint32_t INTERRUPTS_PER_SYMBOL;

extern bool TESTMODE;  // decode of _testmode
extern bool VERBY[10];  // decode of verbose 0-9. disabled if BALLOON_MODE
extern bool BALLOON_MODE;  // this is set by setup() when it detects no USB/Serial
extern bool USE_SIM65M;  // ATGM3365N-31 if false

//**************************************
// Verbosity:
// 0: none. no use of Serial
// 1: currently, all Serial.print*() and V0_flush()
// 2:9 same as 1

//***************************************
// Converts string to upper case
// string to convert <inout>
// No return value, string is converted directly in the parameter *str
void convertToUpperCase(char *str) {
    while (*str) {
        *str = toupper((unsigned char)*str);
        str++;
    }
}

//***************************************
// HACK for debug (force config)
void forceHACK(void) {
    static bool HACK = false;

    if (false) {
        // apparently 12-17Mhz illegal.
        // 18-19Mhz legal
        // 20-48Mhz legal (and more above that).
        // Has to do with divisors and such for the sys pll
        V1_println(F("Check some legal/illegal low frequencies for possible use with pll_sys later" EOL));
        for (uint32_t i = 12; i <= 48; i++)  {
            uint32_t freq_khz = i * 1000UL;
            if (set_sys_clock_khz(freq_khz, false)) {
                V1_printf("GOOD: set_sys_clock_khz() can change clock to %lu Mhz. legal" EOL, i);
            } else {
                V1_printf("ERROR: set_sys_clock_khz() can not change clock to %lu Mhz. illegal" EOL, i);
            }
        }
    }

    if (HACK && !BALLOON_MODE) {
        // https://stackoverflow.com/questions/2606539/snprintf-vs-strcpy-etc-in-c
        // recommends to always snprintf(buffer, sizeof(buffer), "%s", string);

        // HACK FIX! always 1 now for debug
        V1_println(F("Forcing _verbose to 1 (always for now)"));
        snprintf(_verbose, sizeof(_verbose), "1");
        decodeVERBY();

        V1_println(F("Forcing _go_when_rdy to 1 (always for now)"));
        snprintf(_go_when_rdy, sizeof(_go_when_rdy), "1");
    }
}

//***************************************
// Echo user input to stdout and set input_variable
// prompt: Prompt to display to user <input>
// input_variable: Variable to which we want to read input <output>
// max_length: Maximum length of input string <input>
void get_user_input(const char *prompt, char *input_variable, int max_length) {
    V0_println(F("get_user_input START"));
    Watchdog.reset();
    updateStatusLED();

    // BALLOON_MODE will just create an empty string, if we ever come here?
    int index = 0;
    int ch;
    // Display the prompt to the user
    V0_print(F("<enter> to end input" EOL));
    V0_printf("%s", prompt);
    V0_flush();
    while (!BALLOON_MODE) {
        int timeout_ms = 0;
        Watchdog.reset();
        // FIX! disable this if BALLOON_MODE
        while (!Serial.available()) {
            sleep_ms(100);
            timeout_ms += 100;
            // this will eventually watchdog reset timeout if no character?
            updateStatusLED();
            if (timeout_ms > 60 * 1000) {  // 60 secs
                V0_println(F("ERROR: exceeded 60 secs timeout waiting for input, rebooting" EOL));
                V0_flush();
                Watchdog.enable(5000);  // milliseconds
                while (true) tight_loop_contents();
            }
         }

        ch = Serial.read();
        if (ch == '\n' || ch == '\r') {  // Enter key pressed
            break;
        // Backspace key pressed (127 for most Unix, 8 for Windows)
        } else if (ch == 127 || ch == 8) {
            if (index > 0) {
                index--;
                V0_printf("\b \b");  // Move back, print space, move back again
            }
        } else if (isprint(ch)) {
            if (index < max_length - 1) {  // Ensure room for null terminator
                input_variable[index++] = ch;
                V0_printf("%c", ch);  // Echo character
            }
        }
        V0_flush();
        // whenever something might have taken a long time like printing the big buffer
        updateStatusLED();
    }

    input_variable[index] = '\0';  // Null-terminate the string
    V0_print(F(EOL));
    V0_println(F("get_user_input END"));
}

//***************************************
// Hex listing of the settings FLASH to stdout
// buf: Address of FLASH to list <input>
// len: Length of storage to list <input>
void printFLASH(const uint8_t *buf, size_t len) {
    V0_print(F(CLEAR_SCREEN BRIGHT BOLD_ON UNDERLINE_ON EOL));
    V0_print(F("printFLASH:" EOL));
    V0_print(F(BOLD_OFF UNDERLINE_OFF));

    for (size_t i = 0; i < len; ++i) {
        V0_printf("%02x", buf[i]);
        if (i % 16 == 15) {
            V0_print(F(EOL));
        } else {
            V0_print(F(" "));
        }
    }
    V0_print(F(NORMAL));
}

//***************************************
void config_intro(void) {
    V0_println(F("config_intro START"));
    setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
    updateStatusLED();

    // re-read the NVRAM just to see if we have any errors, and to see the VERBY decode
    V0_print(F(CLEAR_SCREEN CURSOR_HOME BRIGHT));
    for (int i = 0; i < 10; i++) V0_println();
    // V0_print(F(RED));
    V0_println("tracker: AD6Z firmware, AG6NS 0.04 pcb, JLCPCB with *kbn* or *kbn2* bom/cpl mod");
    V0_println("https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker");
    V0_println("https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/tree/main/pcb/tracker/v0.4_kbn");
    V0_println("tracker.ino firmware version: " __DATE__ " "  __TIME__);
    V0_println("support: knormoyle@gmail.com or https://groups.io/g/picoballoon");
    // V0_print(F(NORMAL));

    //***************
    // get all the _* config state set and fix any bad values (to defaults)
    // normally we only read it during setup() so we might not see prints from it in putty.log
    int result = read_FLASH();
    // if anything got fixed to defaults, no read again
    Watchdog.reset();
    if (result == -1) {
        V0_println(F("WARN: read_FLASH got result -1 first time, redo. ..errors were fixed to default"));
        result = read_FLASH();
    }
    Watchdog.reset();
    if (result == -1) {
        V0_println(F("ERROR: read_FLASH got result -1 a second time, ignore"));
    }

    //**************
    // FIX! what happens when BALLOON_MODE and Serial isn't there .. just timeout reboot?
    drainSerialTo_CRorNL(1000);
    // FIX! assume this is the state it was in before config menu?
    // not always right. but loop will self-correct?
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    Watchdog.reset();
    // string concat works here with defines.sh special strings
    V0_print(F("config_intro END" EOL CLEAR_SCREEN));
}

//***************************************
void show_TELEN_msg() {
    V0_print(F("show_TELEN_msg()"));
    V0_print(F(EOL "Telen Types:" EOL EOL));
    V0_print(F(BRIGHT UNDERLINE_ON));
    V0_print(F(EOL EOL EOL EOL "TELEN CONFIG INSTRUCTIONS:" EOL EOL));
    V0_print(F(UNDERLINE_OFF NORMAL));
    V0_print(F("* There are 4 possible TELEN values, corresponding to TELEN 1 value 1," EOL));
    V0_print(F("  TELEN 1 value 2, TELEN 2 value 1 and TELEN 2 value 2." EOL));
    V0_print(F("* Enter 4 characters (legal 0-9 or -) in TELEN_config." EOL));
    V0_print(F("  use a '-' (minus) to disable one or more values." EOL));
    V0_print(F("  example:" EOL));
    V0_print(F("  '----' disables all telen " EOL));
    V0_print(F("* example:" EOL));
    V0_print(F("  '01--'" EOL));
    V0_print(F("    Telen 1 value 1 to type 0" EOL));
    V0_print(F("    Telen 1 value 2 to type 1" EOL));
    V0_print(F("    disables all of TELEN 2" EOL));

    V0_print(F(BRIGHT UNDERLINE_ON));
    V0_print(F(EOL "Telen Types:" EOL EOL));
    V0_print(F(UNDERLINE_OFF NORMAL));
    V0_print(F("-: disabled, 0: ADC0, 1: ADC1, 2: ADC2, 3: ADC3" EOL));

    V0_print(F("4: minutes since boot, 5: minutes since GPS fix aquired" EOL));
    V0_print(F("6-9: OneWire temperature sensors 1 though 4" EOL));
    V0_print(F("A: custom: OneWire temperature sensor 1 hourly low/high" EOL));
    V0_print(F("B-Z: reserved for future I2C devices etc" EOL));
    V0_print(F(EOL "(ADC values are in units of mV)" EOL));
    V0_print(F("See the Wiki for more info.n" EOL));
}

//********************************************
// 'Z' command causes this to execute
void do_i2c_tests(void) {
    // quick and dirty way to execute some different tests
    if (true) {
        // picks a good HF freq for the config'ed _Band. 
        // uses t_callsign a and t_grid6 in the message
        // NOTE: turns GPS back on at the end..so it's assuming it's last after wspr
        while (true)  {
            V1_print(F(EOL "cw_send_message() with current _Band/_callsign/grid6/altitude" EOL));
            cw_send_message();
            V1_print(F(EOL "<enter> within 2 secs to abort cw test loop, otherwise repeats" EOL));
            char c_char = getOneChar(2000);  // 2 secs
            if (c_char != 0) break;
        }
    } else if (true) {
        i2c_scan_both();
    } else {
        V0_println(F(EOL "Can we read and write these?"));

        // FIX! should we update this to the actual read+write test in i2c_test
        V0_println(F(EOL "SI5351A_MULTISYNTH0_BASE reset state 0xe8 ?"));
        // FIX! are these comparing to expected read value?
        // or should I use i2c_function.cpp's i2cWrReadTest()
        i2cWrite(SI5351A_MULTISYNTH0_BASE, 0x55);
        i2cWrRead(SI5351A_MULTISYNTH0_BASE, 0x00);
        i2cWrite(SI5351A_MULTISYNTH0_BASE, 0xaa);
        i2cWrRead(SI5351A_MULTISYNTH0_BASE, 0x00);
        i2cWrite(SI5351A_MULTISYNTH0_BASE, 0xe8);
        i2cWrRead(SI5351A_MULTISYNTH0_BASE, 0x00);

        V0_println(F(EOL "Other reads:"));
        V0_println(F(EOL "SI5351A_MULTISYNTH1_BASE"));
        i2cWrRead(SI5351A_MULTISYNTH1_BASE, 0);
        V0_println(F(EOL "SI5351A_CLK0_CONTROL"));
        i2cWrRead(SI5351A_CLK0_CONTROL, 0);
        V0_println(F(EOL "SI5351A_CLK1_CONTROL"));
        i2cWrRead(SI5351A_CLK1_CONTROL, 0);
        V0_println(F(EOL "SI5351A_OUTPUT_ENABLE_CONTROL"));
        i2cWrRead(SI5351A_OUTPUT_ENABLE_CONTROL, 0);

        V0_println(F(EOL "SI5351A read reg 0x00"));
        i2cWrRead(0, 0x00);  // Device Status. D7 should be 1
        V0_println(F(EOL "SI5351A read reg 0x00"));
        i2cWrRead(0, 0x00);  // Device Status. D7 should be 1
    }
}

//***************************************
void user_interface(void) {
    // Does println do better compared to my V0_print() with LINEND as \r\n ??
    // V0_println() does \r\n line ending?
    V0_println(F("user_interface START"));
    sleep_ms(100);
    // FIX! do we change the led blink pattern during config?
    config_intro();
    show_values();
    show_commands();

    while (!BALLOON_MODE) {
        V0_print(F(UNDERLINE_ON BRIGHT UNDERLINE_OFF NORMAL));
        // no comma to concat strings
        // F() to keep string in flash, not ram
        V0_println(F("Enter single char command: Z, *, @, /, X, C, U, V, T, K, A, B, P, D, R, G, S, M"));
        V0_print(F(UNDERLINE_OFF NORMAL));

        Watchdog.reset();
        char c_char = getOneChar(15000);

        // V0_printf("%s" EOL, c_char);
        if (c_char == 0) {
            V0_print(F(CLEAR_SCREEN EOL));
            V0_print(F("(3) Timeout waiting for input, ..rebooting" EOL));
            sleep_ms(100);
            // milliseconds
            Watchdog.enable(500);
            while (true) tight_loop_contents();
        }

        // make char capital either way
        if (c_char > 90) c_char -= 32;

        // FIX! can we case the int32_t to char. we might lost data with the cast
        char confirm[2] = { 0 };
        switch ( c_char ) {
            case 'Z':
                V0_print(F("Will run test" EOL));
                do_i2c_tests();
                break;

            case '@':
                V0_print(F(EOL));
                V0_print(F("<DANGER> Only do this if you have seen you're in a good GPS state <DANGER!>" EOL));
                V0_print(F("<DANGER> Write current GPS config, no broadcast, 1 constellation to GPS FLASH? <DANGER!>" EOL));
                V0_print(F(EOL));
                get_user_input("Y to confirm, Anything else like <newline> to abort" EOL, confirm, sizeof(confirm));
                convertToUpperCase(confirm);
                if (false) {
                // Y is 89
                // if (confirm[0] == 'Y') {
                    V0_print(F("Copying config to GPS flash, no broadcast enabled (config for gps cold reset?)" EOL));
                    V0_flush();
                    // this changes constellations to just GPS
                    // and no broadcasts
                    writeGpsConfigNoBroadcastToFlash();
                    // after: it restores desired broadcast, restores constellations to desired
                } else {
                    V0_print(F("No GPS config write done." EOL));
                }

                break;

            case '/':
                V0_print(F("Rebooting to bootloader mode..drag/drop a uf2 per normal" EOL));
                V0_print(F("after reboot: drag/drop a uf2 the normal way to the drive that shows" EOL));
                V0_print(F("You should be able to leave usb connected. If not, disconnect/connect%" EOL));
                V0_print(F("Goodbye ..rebooting after '/' command" EOL));

                V0_print(F(CLEAR_SCREEN EOL));
                // #include "pico/bootrom.h"
                // doug: for 1.5 ide
                // doug: reset_usb_boot(0, 0); // sets pin 0 for led to be asserted?
                // doug: Looks like in the new SDK (2.1) it's now rom_reset_usb_boot()
                // I'm using arduino ide version 2.3.4
                // what about:
                // https://arduino-pico.readthedocs.io/en/latest/rp2040.html
                // rp2040.rebootToBootloader() 
                // Will reboot the RP2040 into USB UF2 upload mode.
                reset_usb_boot(0, 0);
                // rom_reset_usb_boot();
                // just in case it returns here
                Watchdog.enable(2000);  // milliseconds
                while (true) tight_loop_contents();

            case '*':
                V0_print(F("Do factory reset of config state to default values (and reboot)" EOL));
                doFactoryReset();  // doesn't return, reboots
                break;
            case 'X':
                V0_print(F("Goodbye ..rebooting after 'X' command" EOL));
                V0_print(F(CLEAR_SCREEN EOL));
                Watchdog.enable(500);  // milliseconds
                while (true) tight_loop_contents();
            case 'C':
                // FIX! will 1 char send wspr?
                get_user_input("Enter callsign: (3 to 6 chars: 1 to 3 [A-Z0-9] + 0 to 3 [A-Z]" EOL,
                    _callsign, sizeof(_callsign));
                convertToUpperCase(_callsign);
                write_FLASH();
                break;
            case 'U':
                get_user_input("Enter U4B channel (0-599): " EOL, _U4B_chan, sizeof(_U4B_chan));
                init_rf_freq(&XMIT_FREQUENCY, _Band, _lane);

                write_FLASH();
                break;
            case 'V':
                get_user_input("Enter Verbosity level (0-9): " EOL, _verbose, sizeof(_verbose));
                write_FLASH();
                break;
            case 'T':
                show_TELEN_msg();
                get_user_input("Enter TELEN config: " EOL, _TELEN_config, sizeof(_TELEN_config));
                convertToUpperCase(_TELEN_config);
                write_FLASH();
                break;
            case 'K':
                get_user_input("Enter clock speed (18, 20-48, 49-250 (not all)): " EOL, _clock_speed, sizeof(_clock_speed));
                write_FLASH();
                PLL_SYS_MHZ = atoi(_clock_speed);
                // frequencies like 205 mhz will PANIC,
                // System clock of 205000 kHz cannot be exactly achieved
                // should detect the failure and change the nvram, otherwise we're stuck even on reboot
                // this is the only config where we don't let something bad get into flash
                // don't change the pll, just check. change it on reboot
                if (PLL_SYS_MHZ < 18 || PLL_SYS_MHZ > 250) {
                    V0_printf("user_interface: _clock_speed %lu illegal. Using %lu instead" EOL,
                        PLL_SYS_MHZ, DEFAULT_PLL_SYS_MHZ);

                    // https://stackoverflow.com/questions/2606539/snprintf-vs-strcpy-etc-in-c
                    // recommends to always
                    // snprintf(buffer, sizeof(buffer), "%s", string);
                    PLL_SYS_MHZ = DEFAULT_PLL_SYS_MHZ;
                    snprintf(_clock_speed, sizeof(_clock_speed), "%lu",  PLL_SYS_MHZ);
                    write_FLASH();
                }
                makeSureClockIsGood();

                break;
            case 'A':
                get_user_input("Enter Band (2,10,12,15,17,20):" EOL, _Band, sizeof(_Band));
                // redo channel selection if we change bands,
                // since U4B definition changes per band
                write_FLASH();
                init_rf_freq(&XMIT_FREQUENCY, _Band, _lane);
                break;
            case 'B':
                get_user_input("Enter CW Band (2,10,12,15,17,20):" EOL, _Band_cw, sizeof(_Band));
                // redo channel selection if we change bands,
                // since U4B definition changes per band
                write_FLASH();
                break;
            case 'P':
                get_user_input("Enter Tx high: (0 or 1)" EOL, _tx_high, sizeof(_tx_high));
                write_FLASH();
                break;
            case 'D':
                get_user_input("Enter TESTMODE (currently affects nothing): (0 or 1)" EOL,
                    _testmode, sizeof(_testmode));
                write_FLASH();
                break;
            case 'R':
                V0_print(F("Don't cause more than approx. 43 hz 'correction' on a band." EOL));
                V0_print(F("effect varies per band?" EOL));
                get_user_input("Enter ppb Correction to si5351: (-30000 to 30000)" EOL,
                    _correction, sizeof(_correction));
                write_FLASH();
                break;
            case 'G':
                V0_print(F("test only: 1 Don't wait for starting minute from _U4B_chan" EOL));
                V0_print(F("does wait for any 2 minute alignment though" EOL));
                V0_print(F("ZERO THIS BEFORE BALLOON FLIGHT!! ignores BALLOON_MODE" EOL));
                get_user_input("Enter go_when_rdy for faster test..any 2 minute start: (0 or 1):",
                    _go_when_rdy, sizeof(_go_when_rdy));
                write_FLASH();
                break;
            case 'S':
                get_user_input("Enter use_sim65m: 1 if gps chip is SIM65, 0 if ATGM3365N-31:",
                    _use_sim65m, sizeof(_use_sim65m));
                write_FLASH();
                break;
            case 'M':
                get_user_input("Send morse also? 0 or 1: " EOL, _morse_also, sizeof(_morse_also));
                write_FLASH();
                break;
            case 'L':
                get_user_input("Dynamic solar elevation tx power? 0 or 1: " EOL, 
                    _solar_tx_power, sizeof(_solar_tx_power));
                write_FLASH();
                break;
            case 13:  break;
            case 10:  break;
            default:
                V0_printf("You pressed: %c - (0x%02x), invalid choice!" EOL, c_char, c_char);
                V0_print(F(CLEAR_SCREEN));
                sleep_ms(1000);
                break;
        }
        int result = check_data_validity_and_set_defaults();
        if (result == -1) {
            // this should include a fix of empty callsign?
            V0_print(F("ERROR: check_data_validity_and_set_defaults() fixed illegal value (2)"));
        }
        show_values();
        V0_println(F("user_interface END"));
    }
}

//********************************************
void makeSureClockIsGood(void) {
    // https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#gab3a273e837ba1947bb5fd8fc97cf47e5
    // says "Note that not all clock frequencies are possible;
    // it is preferred that you use src/rp2_common/hardware_clocks/scripts/vcocalc.py
    // to calculate the parameters for use with set_sys_clock_pll".
    // Probably want to have a read of section 2.15 of
    // https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
    // for more information about the PLLs and clock dividers.

    // do only full Mhz work? maybe can do more but we only need integer Mhz here.
    // This guy say: absolutely can do non Mhz frequencies (just not those between 125 and 126?)
    // https://github.com/raspberrypi/pico-sdk/issues/1450
    // None of the frequencies can be exactly matched exactly by the PLL so set_sys_clock_khz fails -
    // as per the docs, you can use vco_calc.py to find out settings
    // for set_sys_clock_pll for settings that are close.
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_clocks/scripts/vcocalc.py

    // bool check_sys_clock_khz (
    // uint32_t freq_khz, uint * vco_freq_out, uint * post_div1_out, uint * post_div2_out)

    uint32_t freq_khz = PLL_SYS_MHZ * 1000UL;
    V0_printf("user_interface: checking with set_sys_clock_khz(%lu, false)" EOL, freq_khz);
    // uint vco_freq_out[1];
    // uint post_div1_out[1];
    // uint post_div2_out[1];
    // good = check_sys_clock_khz(freq_khz, vco_freq_out, post_div1_out, post_div2_out);
    bool good = set_sys_clock_khz(freq_khz, false);
    if (good) {
        // V0_printf("user_interface: good with check_sys_clock_khz(%lu)" EOL, freq_khz);
        V0_printf("user_interface: good with set_sys_clock_khz(%lu, false)" EOL, freq_khz);
    } else {
        // V0_printf("user_interface: bad with check_sys_clock_khz(%lu)" EOL, freq_khz);
        V0_printf("user_interface: bad with set_sys_clock_khz(%lu, false)" EOL, freq_khz);
    }

    // can use vcocalc.py to calculate parameters with set_sys_clock_pll()
    // static bool set_sys_clock_khz ( uint32_t freq_khz, bool required
    if (!good) {
        V0_print(F("makeSureClockIsGood():"));
        V0_printf(" ERROR: RP2040 can't change clock to %lu Mhz.", PLL_SYS_MHZ);
        V0_printf(" Using %lu instead" EOL, DEFAULT_PLL_SYS_MHZ);
        PLL_SYS_MHZ = DEFAULT_PLL_SYS_MHZ;
        snprintf(_clock_speed, sizeof(_clock_speed), "%lu", PLL_SYS_MHZ);
        write_FLASH();
        // check this default?
        freq_khz = PLL_SYS_MHZ * 1000UL;
        if (!set_sys_clock_khz(freq_khz, false)) {
            V1_println("user_interface: ERROR: The DEFAULT_SYS_MHZ is not legal either. will use 125");
            PLL_SYS_MHZ = 125;
            snprintf(_clock_speed, sizeof(_clock_speed), "%lu", PLL_SYS_MHZ);
            write_FLASH();
        }
    }
}

//********************************************
// Reads flash where the user settings are saved
// prints hexa listing of data
// calls function which check data validity

// background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
// https://github.com/MakerMatrix/RP2040_flash_programming
// doesn't cover the fetch from FLASH exclusion issue?
// https://github.com/MakerMatrix/RP2040_flash_programming/blob/main/RP2040_flash/RP2040_flash.ino

// update whever you add a bit or more to flash used (the offsets used below)
#define FLASH_BYTES_USED 34
int read_FLASH(void) {
    Watchdog.reset();
    V1_print(F("read_FLASH START" EOL));
    // there is no internal eeprom on rp2040
    // Therefore, do not frequently update the EEPROM or you may prematurely wear out the flash.
    // https://arduino-pico.readthedocs.io/en/latest/eeprom.html

    // While the Raspberry Pi Pico RP2040 does not come with an EEPROM onboard,
    // we could use a simulated one by using a single 4K chunk of flash at the end of flash space.
    // April 13, 2023
    // https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

    // PICO_FLASH_SIZE_BYTES # The total size of the RP2040 flash, in bytes
    // FLASH_SECTOR_SIZE     # The size of one sector, in bytes (the minimum amount you can erase)
    // FLASH_PAGE_SIZE       # The size of one page, in bytes (the mimimum amount you can write)

    // expected:
    // PICO_FLASH_SIZE_BYTES is 2MB or 2097152 bytes.
    // FLASH_SECTOR_SIZE is 4K or 4096 bytes.
    // FLASH_PAGE_SIZE is 256 bytes.

    // So 256 bytes will be one page? does my config fit in that? yes!

    // pointer should be the last physical sector to eliminate the chance
    // that it will interfere with program code.
    // That sector starts at the address:
    // PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE
    // two functions to use
    // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html
    // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_flash

    // flash_range_erase(uint32_t flash_offs, size_t count);
    // flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);

    // https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_flash

    // https://stackoverflow.com/questions/890535/what-is-the-difference-between-char-const-and-const-char
    // const char * is a pointer to a const char. value can't change. pointer can change
    // these two are equivalent
    // const char *
    // char const *
    // char * const is a constant pointer to a char. value can change. pointer can't change
    // const char * const is a const pointer to a const char

    // FIX! why is this weird, re above defs?
    // was 12/18/24
    // const uint8_t *uflash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    uint8_t *uflash_target_contents = (uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    // printFLASH(uflash_target_contents, (int) FLASH_PAGE_SIZE);  // 256

    char flash_target_contents[FLASH_BYTES_USED] = { 0 };
    // FIX! Does the flash read ever get wrong results?
    for (int i = 0; i < FLASH_BYTES_USED ; i++) {
        flash_target_contents[i] = (char) uflash_target_contents[i];
    }
    // BE SURE YOU ONLY USE ONE PAGE: i.e. 256 bytes total
    // FIX! should we just use snprintf?
    strncpy(_callsign,     flash_target_contents + 0,  6); _callsign[6] = 0;
    strncpy(_verbose,      flash_target_contents + 6,  1); _verbose[1] = 0;
    strncpy(_TELEN_config, flash_target_contents + 7,  4); _TELEN_config[4] = 0;
    strncpy(_clock_speed,  flash_target_contents + 11, 3); _clock_speed[3] = 0;
    strncpy(_U4B_chan,     flash_target_contents + 14, 3); _U4B_chan[3] = 0;
    // FIX! change to _band everywhere?
    strncpy(_Band,         flash_target_contents + 17, 2); _Band[2] = 0;
    strncpy(_tx_high,      flash_target_contents + 19, 1); _tx_high[1] = 0;
    strncpy(_testmode,     flash_target_contents + 20, 1); _testmode[1] = 0;
    strncpy(_correction,   flash_target_contents + 21, 6); _correction[6] = 0;
    strncpy(_go_when_rdy,  flash_target_contents + 27, 1); _go_when_rdy[1] = 0;
    strncpy(_factory_reset_done,  flash_target_contents + 28, 1); _factory_reset_done[1] = 0;
    strncpy(_use_sim65m,   flash_target_contents + 29, 1); _use_sim65m[1] = 0;
    strncpy(_morse_also,   flash_target_contents + 30,  1); _morse_also[1] = 0;
    strncpy(_Band_cw,      flash_target_contents + 31,  2); _Band_cw[2] = 0;
    strncpy(_solar_tx_power,  flash_target_contents + 33,  1); _solar_tx_power[1] = 0;

    PLL_SYS_MHZ = atoi(_clock_speed);
    // recalc
    calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);

    // FIX! we should decode the _Band/_U4B_chan and set any ancillary decode vars?
    // any XMIT_FREQUENCY ?
    process_chan_num(_id13, _start_minute, _lane, _Band, _U4B_chan);

    // FIX! define this as extern?
    init_rf_freq(&XMIT_FREQUENCY, _Band, _lane);

    // fix anything bad! both in _* variables and FLASH (defaults)
    // -1 if anything got fixed
    int result = check_data_validity_and_set_defaults();
    if (result == -1) {
        // this should include a fix of empty callsign?
        V0_print(F("ERROR: check_data_validity_and_set_defaults() fixed some illegal value (1)"));
    }

    // forceHACK();

    if (_testmode[0] == '1') TESTMODE = true;
    else TESTMODE = false;
    if (_use_sim65m[0] == '1') USE_SIM65M = true;
    else USE_SIM65M = false;

    decodeVERBY();

    V1_print(F("read_FLASH END" EOL));
    return result;
}

//**************************************
void decodeVERBY(void) {
    // don't do any printing in here, in case BALLOON_MODE/VERBY not correct yet
    // V1_print(F("decodeVERBY START" EOL));
    // can't use Serial at all, if BALLOON_MODE
    // VERBY[0] guarantees that, even for config
    // Currently VERBY[1] covers everything else
    if (BALLOON_MODE) {
        for (int i = 0; i <= 9 ; i++) VERBY[i] = false;
        return;
    }
    // always set VERBY[0] if not BALLOON_MODE, so we
    // can see the config output
    VERBY[0] = true;

    // if _verbose is currently illegal (haven't updated NVRAM?, set everything
    if (_verbose[0] < '0' && _verbose[0] > '9') {
        // set everything if it's illegal ascii
        for (int i = 0; i < 10 ; i++) VERBY[i] = true;
        return;
    }
    int j = _verbose[0] - '0';  // '0' is 48
    // j and everything below
    // 0 is ascii 48
    // decode '1' to '9' to thermometer code VERBY
    for (int i = 1; i < 10 ; i++) {
        // so for verbose 9, we'' get 0 thru 9 set to true
        // for verbose 0 we'll just get 0
        if (i <= j) VERBY[i] = true;
        else VERBY[i] = false;
    }

    // V1_printf("decoded _verbose %s to VERBY[9:0]" EOL, _verbose);
    if (false) {
        for (int i = 0; i < 10 ; i++) {
            V0_printf("VERBY[%d] %x" EOL, i, VERBY[i]);
        }
    }

    // V1_print(F("decodeVERBY END" EOL));
}

//**************************************
// https://github.com/raspberrypi/pico-examples/blob/master/flash/program/flash_program.c
// This function will be called when it's safe to call flash_range_erase
static void call_flash_range_erase(void *param) {
    uint32_t offset = (uint32_t)param;
    flash_range_erase(offset, FLASH_SECTOR_SIZE);
}

//**************************************
// https://github.com/raspberrypi/pico-examples/blob/master/flash/program/flash_program.c
// This function will be called when it's safe to call flash_range_program
static void call_flash_range_program(void *param) {
    uint32_t offset = ((uintptr_t*)param)[0];
    const uint8_t *data = (const uint8_t *)((uintptr_t*)param)[1];
    // uint32_t offset = ((uint8_t*)param)[0];
    // const uint8_t *data = (const uint8_t *)((uint8_t*)param)[1];
    flash_range_program(offset, data, FLASH_PAGE_SIZE);
}

//***************************************
// Write the user entered data into FLASH
void write_FLASH(void) {
    Watchdog.reset();
    V1_print(F("write_FLASH START" EOL));
    // Flash is initially all zeroes
    char data_chunk[FLASH_BYTES_USED] = { 0 };  // enough to cover what we use here
    uint8_t udata_chunk[FLASH_PAGE_SIZE] = { 0 };  // 256 bytes

    // don't take the extra null term (but _callsign might be short!)
    strncpy(data_chunk + 0,  _callsign, 6);
    strncpy(data_chunk + 6,  _verbose, 1);
    strncpy(data_chunk + 7,  _TELEN_config, 4);
    strncpy(data_chunk + 11, _clock_speed, 3);
    strncpy(data_chunk + 14, _U4B_chan, 3);
    strncpy(data_chunk + 17, _Band, 2);
    strncpy(data_chunk + 19, _tx_high, 1);
    strncpy(data_chunk + 20, _testmode, 1);
    strncpy(data_chunk + 21, _correction, 6);
    strncpy(data_chunk + 27, _go_when_rdy, 1);
    strncpy(data_chunk + 28, _factory_reset_done, 1);
    strncpy(data_chunk + 29, _use_sim65m, 1);
    strncpy(data_chunk + 30, _morse_also, 1);
    strncpy(data_chunk + 31, _Band_cw, 2);
    strncpy(data_chunk + 33, _solar_tx_power, 1);

    // alternative for casting the array to uint8_t
    // https://stackoverflow.com/questions/40579902/how-to-turn-a-character-array-into-uint8-t
    // If you're on an architecture where uint8_t is a typedef to unsigned char,
    // then simply take the first char and cast it to uint8_t:
    // int length = (uint8_t)(udata_chunk[0]);
    for (int i = 0; i < FLASH_BYTES_USED ; i++) {
        udata_chunk[i] = (uint8_t) data_chunk[i];
    }
    // a "Sector" is 4096 bytes
    // FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE, FLASH_PAGE_SIZE = 040000x, 4096, 256
    // you could theoretically write 16 pages at once (a whole sector).

    V0_print(F("Erasing FLASH target region" EOL));
    uint32_t ints;
    int rc;
    if (false) {
        rc = flash_safe_execute(call_flash_range_erase, (void*)FLASH_TARGET_OFFSET, UINT32_MAX);
        V0_printf("flash_safe_execute call_flash_range_erase rc: %d" EOL, rc);
        // hard_assert(rc == PICO_OK);
    } else {
        // was 12/18/2024
        // don't interrupt..not enough? what about code fetch
        // uint32_t ints = save_and_disable_interrupts();
        ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
        // writes 256 bytes (one "page") (16 pages per sector)
    }

    V0_print(F("Writing FLASH target region" EOL));
    if (false) {
        uintptr_t params[] = {FLASH_TARGET_OFFSET, (uintptr_t) udata_chunk};
        // uintptr_t params[] = {FLASH_TARGET_OFFSET, (uint8_t) udata_chunk};
        rc = flash_safe_execute(call_flash_range_program, params, UINT32_MAX);
        V0_printf("flash_safe_execute call_flash_range_program() rc: %d" EOL, rc);
        // hard_assert(rc == PICO_OK);
    } else {
        // was 12/18/2024
        // was this supposed to be FLASH_SECTOR_SIZE?
        flash_range_program(FLASH_TARGET_OFFSET, udata_chunk, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
    }
    V1_print(F("write_FLASH END" EOL));
}


//**************************************
// Checks validity of user settings and if something is wrong,
// sets "factory defaults" and writes it back to FLASH
// create result to return
int check_data_validity_and_set_defaults(void) {
    int result = 1;
    // set reasonable defaults if memory was uninitialized or has bad values
    // create 'result' to return

    //*****************
    // do full legal callsign check? (including spaces at end)
    // be sure to null terminate so we can print the callsign
    // space is not legal in the callsign here? but can have nulls.
    // make shortest legal callsign == 4 chars?
    // https://dxplorer.net/wspr/msgtypes.html

    // 6 chars in callsign
    // 1 - can be a letter or number or left blank <space>
    // 2 - can be a letter or number
    // 3 - can only be a number
    // 4 - can only be a letter or left blank <space>
    // 5 - can only be a letter or left blank <space>
    // 6 - can only be a letter or left blank <space>

    // JTEncode handles the cases that need leading or trailing spaces for wspr validity
    // we don't create spaces here
    // A6ZZZ needs a leading space when you send it
    // 99 needs a leading space
    // 99A needs a leading space
    // 99ABC needs a leading space
    // A9ABC needs a leading space
    // A99BC is legal
    // 99ABC needs a leading space
    // 999ABC is legal

    // work from the back until you find the first number
    // don't allow <space> to be legal anywhere
    // ignore extra trailing nulls
    int clength = strlen(_callsign);
    bool callsignBad = false;
    // this also covers the null case (strlen 0)
    if (clength < 3) {
        callsignBad = true;
    } else if (clength > 6) {
        callsignBad = true;
    } else if (clength >= 3) {
        for (int i = 0; i <= 2; i++) {
            if ((_callsign[i] < 'A' && _callsign[i] > 'Z') &&
                (_callsign[i] < '0' && _callsign[i] > '9')) {
                callsignBad = true;
            }
        }
    } else if (clength >= 4) {
        if (_callsign[3] < 'A' || _callsign[3] > 'Z') callsignBad = true;
    } else if (clength >= 5) {
        if (_callsign[4] < 'A' || _callsign[4] > 'Z') callsignBad = true;
    } else if (clength == 6) {
        if (_callsign[5] < 'A' || _callsign[5] > 'Z') callsignBad = true;
    }

    if (callsignBad) {
        V0_printf(EOL "ERROR: _callsign %s is not supported/legal, initting to AB1CDE" EOL, _callsign);
        snprintf(_callsign, sizeof(_callsign), "AB1CDE");
        write_FLASH();
        result = -1;
    }

    //*****************
    // change to strcpy for null terminate
    if (_verbose[0] == 0 || _verbose[0] < '0' || _verbose[0] > '9') {
        V0_printf(EOL "ERROR: _verbose %s is not supported/legal, initting to 1" EOL, _verbose);
        snprintf(_verbose, sizeof(_verbose), "1");
        write_FLASH();
        result = -1;
    }

    //*****************
    // 0-9 and - are legal. _
    // make sure to null terminate
    bool bad = false;
    if (_TELEN_config[0] == 0) {
        bad = true;
    } else {
        for (int i = 0; i <= 3; i++) {
            if ((_TELEN_config[i] < '0' || _TELEN_config[i] > '9') && _TELEN_config[i] != '-') bad = true;
        }
    }
    if (bad) {
        V0_printf(EOL "ERROR: _TELEN_config %s is not supported/legal, initting to ----" EOL,
            _TELEN_config);
        snprintf(_TELEN_config, sizeof(_TELEN_config), "----");
        write_FLASH();
        result = -1;
    }

    //*****************
    // _clock_speed
    // keep the upper limit at 250 to avoid nvram getting
    // a freq that won't work. will have to load flash nuke uf2 to clear nram
    // if that happens, so that default clock will return?
    // if so: Download the [UF2 file]
    // https://datasheets.raspberrypi.com/soft/flash_nuke.uf2
    // code is
    // https://github.com/raspberrypi/pico-examples/blob/master/flash/nuke/nuke.c
    // may require some iterations of manually setting all the configs by hand
    // after getting the nuke uf2 (it autoruns) and then reloading pico-WSPRer.uf2
    // hmm. I suppose we could call this routine to fix nvram at the beginning, so if the
    // clock gets fixed, then the defaults will get fixed (where errors exist)
    // be sure to null terminate
    clength = strlen(_clock_speed);
    bool clock_speedBad = false;
    // this also covers the null case (strlen 0)
    if (clength > 3) {
        clock_speedBad = true;
    } else {
        for (int i = 0; i <= 2; i++) {
            if (_clock_speed[i] < '0' && _clock_speed[i] > '9') {
                V0_printf(EOL "ERROR: check_data_validity...(): (1) illegal _clock_speed: %s" EOL, _clock_speed);
                clock_speedBad = true;
            }
        }
    }
    if (!clock_speedBad) {
        PLL_SYS_MHZ = atoi(_clock_speed);
        if (PLL_SYS_MHZ == 0 || PLL_SYS_MHZ < 18 || PLL_SYS_MHZ > 250) {
            V0_printf(EOL "ERROR: check_data_validity...(): (2) illegal _clock_speed: %s" EOL, _clock_speed);
            clock_speedBad = true;
        }

        if (!set_sys_clock_khz(PLL_SYS_MHZ * 1000UL, false)) {
            // http://jhshi.me/2014/07/11/print-uint64-t-properly-in-c/index.html
            V0_printf(EOL "ERROR: check_data_validity...(): RP2040 can't change clock to %lu Mhz" EOL, PLL_SYS_MHZ);
            clock_speedBad = true;
        }
    }

    if (clock_speedBad) {
        V0_printf(EOL "ERROR: _clock_speed %s is not legal, initting to %lu" EOL, _clock_speed, DEFAULT_PLL_SYS_MHZ);
        PLL_SYS_MHZ = DEFAULT_PLL_SYS_MHZ;
        // recalc

        snprintf(_clock_speed, sizeof(_clock_speed), "%lu", PLL_SYS_MHZ);
        write_FLASH();
        result = -1;
    }
    

    // always recalc these for the current PLL_SYS_MHZ
    calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);

    //*****************
    // be sure to null terminate
    if (_U4B_chan[0] == 0 || atoi(_U4B_chan) < 0 || atoi(_U4B_chan) > 599) {
        V0_printf(EOL "ERROR: _U4B_chan %s is not supported/legal, initting to 599" EOL, _U4B_chan);
        snprintf(_U4B_chan, sizeof(_U4B_chan), "%s", "599");
        write_FLASH();
        // this will set _lane, _id13, _start_minute
        process_chan_num(_id13, _start_minute, _lane, _Band, _U4B_chan);
        init_rf_freq(&XMIT_FREQUENCY, _Band, _lane);
        result = -1;
    }
    //*****************
    // null returns 0
    switch (atoi(_Band)) {
        case 2: break;
        case 10: break;
        case 12: break;
        case 15: break;
        case 17: break;
        case 20: break;
        default:
            V0_printf("ERROR: _Band %s is not supported/legal, initting to 20" EOL, _Band);
            snprintf(_Band, sizeof(_Band), "20");
            write_FLASH();
            // figure out the XMIT_FREQUENCY for new band
            process_chan_num(_id13, _start_minute, _lane, _Band, _U4B_chan);
            init_rf_freq(&XMIT_FREQUENCY, _Band, _lane);
            result = -1;
            break;
    }
    //*****************
    // null returns 0
    switch (atoi(_Band_cw)) {
        case 2: break;
        case 10: break;
        case 12: break;
        case 15: break;
        case 17: break;
        case 20: break;
        default:
            V0_printf("ERROR: _Band_cw %s is not supported/legal, initting to 20" EOL, _Band);
            snprintf(_Band_cw, sizeof(_Band_cw), "20");
            write_FLASH();
            result = -1;
            break;
    }
    //*****************
    if (_tx_high[0] != '0' && _tx_high[0] != '1') {
        V0_printf(EOL "ERROR: _tx_high %s is not supported/legal, initting to 1" EOL, _tx_high);
        snprintf(_tx_high, sizeof(_tx_high), "1");
        write_FLASH();
        result = -1;
    }
    //*****************
    if (_testmode[0] != '0' && _testmode[0] != '1') {
        V0_printf(EOL "ERROR: _testmode %s is not supported/legal, initting to 0" EOL, _testmode);
        snprintf(_testmode, sizeof(_testmode), "0");
        write_FLASH();
        result = -1;
    }
    //*****************
    // what does atoi() when null is first char? returns 0
    // detect that case to get ascii 0 in there
    if (_correction[0] == 0 || atoi(_correction) < -30000 || atoi(_correction) > 30000) {
        // left room for 6 bytes
        V0_printf(EOL "ERROR: _correction %s is not supported/legal, initting to 0" EOL, _correction);
        snprintf(_correction, sizeof(_correction), "0");
        write_FLASH();
        result = -1;
    }
    //*****************
    if (_go_when_rdy[0] != '0' && _go_when_rdy[0] != '1') {
        V0_printf(EOL "ERROR: _go_when_rdy %s is not supported/legal, initting to 0" EOL, _go_when_rdy);
        snprintf(_go_when_rdy, sizeof(_go_when_rdy), "0");
        write_FLASH();
        result = -1;
    }
    //*****************
    if (_use_sim65m[0] != '0' && _use_sim65m[0] != '1') {
        V0_printf(EOL "ERROR: _use_sim65m %s is not supported/legal, initting to 0" EOL, _use_sim65m);
        snprintf(_use_sim65m, sizeof(_use_sim65m), "0");
        write_FLASH();
        result = -1;
    }
    //*****************
    if (_factory_reset_done[0] != '0' && _factory_reset_done[0] != '1') {
        V0_printf(EOL "ERROR: _factory_reset_done %s is not support/legal .. will doFactoryReset" EOL, _factory_reset_done);
        doFactoryReset();  // no return, reboots
    }
    //*****************
    if (_morse_also[0] != '0' && _morse_also[0] != '1') {
        V0_printf(EOL "ERROR: _morse_also %s is not supported/legal, initting to 0" EOL, _morse_also);
        snprintf(_morse_also, sizeof(_morse_also), "0");
        write_FLASH();
        result = -1;
    }
    //*****************
    if (_solar_tx_power[0] != '0' && _solar_tx_power[0] != '1') {
        V0_printf(EOL "ERROR: _solar_tx_power %s is not supported/legal, initting to 0" EOL, _solar_tx_power);
        snprintf(_solar_tx_power, sizeof(_solar_tx_power), "0");
        write_FLASH();
        result = -1;
    }
    return result;
}

//***************************************
// print the current config
void show_values(void) {
    V0_println(F("show_values START" EOL));

    // V0_print(F(EOL, CLEAR_SCREEN, UNDERLINE_ON, BRIGHT));
    // since these macros are "" strings in print_functions.h, they will just concat here
    // no commas necessary?

    V0_print(F("FLASH read values:" EOL));

    V0_printf("C: callsign: %s" EOL, _callsign);
    V0_printf("U: U4B channel: %s" EOL, _U4B_chan);
    V0_printf("A: band: %s (meters)" EOL, _Band);
    V0_printf("B: band for cw: %s (meters)" EOL, _Band_cw);
    V0_printf(" (id13: %s", _id13);
    V0_printf(" start Minute: %s", _start_minute);
    V0_printf(" lane: %s)" EOL, _lane);
    V0_printf("P: tx_high: %s" EOL, _tx_high);
    V0_printf("V: verbose: %s" EOL, _verbose);
    V0_printf("T: TELEN config: %s" EOL, _TELEN_config);
    V0_printf("K: clock speed: %s (Mhz)" EOL, _clock_speed);
    V0_printf("D: TESTMODE: %s" EOL, _testmode);
    V0_printf("R: correction: %s (Mhz)" EOL, _correction);
    V0_printf("G: go_when_rdy: %s" EOL, _go_when_rdy);
    V0_printf("S: use_sim65m: %s" EOL, _use_sim65m);
    V0_printf("M: morse_also: %s" EOL, _morse_also);
    V0_printf("L: dynamic tx power using solar elevation: %s" EOL, _solar_tx_power);
    V0_printf("*: factory_reset_done: %s" EOL, _factory_reset_done);

    V0_printf(EOL "XMIT_FREQUENCY: %lu (symbol 0)" EOL, XMIT_FREQUENCY);
    V0_print(F("SIE_STATUS: bit 16 is CONNECTED. bit 3:2 is LINE_STATE. bit 0 is VBUS_DETECTED" EOL));
    // see bottom of tracker.ino for details about memory mapped usb SIE_STATUS register
    #define sieStatusPtr ((uint32_t*)0x50110050)
    uint32_t sieValue = *sieStatusPtr;
    // https://stackoverflow.com/questions/43028865/how-to-print-hex-from-uint32-t
    V0_printf("SIE_STATUS: 0x%" PRIx32 EOL, sieValue);

    V0_println(F("show_values END" EOL));
}

// show list of valid commands
void show_commands(void) {
    V0_println(F("show_commands START" EOL));

    V0_println(F(EOL "Valid commands:" EOL));
    V0_println(F("X: exit config mode and reboot"));
    V0_println(F("/: reboot to drag/drop new .uf2 (not implemented)"));
    V0_println(F("@: write current gps config, no broadcast, 1 constellation to GPS Flash (for boot)"));
    V0_println(F("*: factory reset all config values"));
    V0_println(F("Z: run i2c test or scan: which, currently?"));
    V0_println(F("C: change Callsign (6 char max)"));
    V0_println(F("U: change U4b channel # (0-599)"));
    V0_println(F("A: change band (2,10,12,15,17,20 default 20)"));
    V0_println(F("B: change band for cw (2,10,12,15,17,20 default 20)"));
    V0_println(F("P: change tx power: 1 high, 0 lower default )"));
    V0_println(F("V: verbose (0 for no messages, 9 for all)"));
    V0_println(F("T: TELEN config"));
    V0_printf("K: clock speed  (default: %lu)" EOL,  DEFAULT_PLL_SYS_MHZ);
    V0_println(F("D: TESTMODE (current: sweep telemetry values) (default: 0)"));
    V0_println(F("R: si5351 ppb correction (-3000 to 3000) (default: 0)"));
    V0_println(F("G: go_when_rdy (callsign tx starts at any modulo 2 starting minute (default: 0)"));
    V0_println(F("S: sim65m: 1 sim65m, 0 atgm3365n-31 (default: 0)"));
    V0_println(F("M: morse_also: 1 tx cw msg after all wspr(default: 0)"));
    V0_println(F("L: solar_tx_power: 1 adjust power from solar elevation(default: 0)"));

    V0_print(F("show_commands END" EOL));
}

//*****************************************************
void doFactoryReset() {
    V0_println(F("doFactoryReset START"));
    snprintf(_callsign, sizeof(_callsign), "AB1CDE");
    snprintf(_verbose, sizeof(_verbose), "1");
    snprintf(_TELEN_config, sizeof(_TELEN_config), "----");
    snprintf(_clock_speed, sizeof(_clock_speed), "%lu", DEFAULT_PLL_SYS_MHZ);
    snprintf(_U4B_chan, sizeof(_U4B_chan), "%s", "599"); // always 3 chars?
    snprintf(_Band, sizeof(_Band), "20");
    snprintf(_Band_cw, sizeof(_Band), "20");
    snprintf(_tx_high, sizeof(_tx_high), "1");
    snprintf(_testmode, sizeof(_testmode), "0");
    snprintf(_correction, sizeof(_correction), "0");
    snprintf(_go_when_rdy, sizeof(_go_when_rdy), "0");
    // when we read_FLASH, if this is 0, we set everything to default
    // or if user command is '*'
    snprintf(_factory_reset_done, sizeof(_go_when_rdy), "1");
    snprintf(_use_sim65m, sizeof(_use_sim65m), "0");
    snprintf(_morse_also, sizeof(_morse_also), "0");
    snprintf(_solar_tx_power, sizeof(_solar_tx_power), "0");

    // What about the side decodes? Don't worry, just reboot
    write_FLASH();
    V0_println(F("doFactoryReset END"));

    // reboot
    V0_print(F("Goodbye ..rebooting after doFactorReset()" EOL));
    Watchdog.enable(500);  // milliseconds
    while (true) tight_loop_contents();
}
//*****************************************************
