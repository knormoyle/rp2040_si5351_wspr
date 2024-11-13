// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#include <Arduino.h>
#include <stdint.h>

// any of this needed
#include <stdio.h>
// for isprint()
#include <ctype.h>
#include <stdlib.h>

#include "defines.h"
#include "led_functions.h"
#include "config_functions.h"

// FIX! is the program bigger than 256K
#define FLASH_TARGET_OFFSET (256 * 1024) // leaves 256k of space for the program
#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGE_SIZE 256
#define kHz 1000U

extern uint32_t XMIT_FREQUENCY;
extern bool DEVMODE;
extern uint32_t PLL_SYS_MHZ;

extern char _callsign[7];
extern char _id13[3];
extern char _start_minute[2];
extern char _lane[2];
extern char _suffix[2];
extern char _verbosity[2];
extern char _TELEN_config[5];
extern char _clock_speed[4];
extern char _U4B_chan[4];
extern char _Band[3];     // string with 10, 12, 15, 17, 20 legal. null at end
extern char _tx_high[2];  // 0 is 2mA si5351. 1 is 8mA si5351
extern char _devmode[2];

extern bool DEVMODE;

// don't allow more than approx. 43 hz "correction" on a band. leave room for 6 chars
extern char _correction[6];  // parts per billion -3000 to 3000. default 0
// traquito: 500 correction does  ~7 hz lower on 20M (14095.600 base freq)
// traquito: 500 correction does ~14 hz lower on 10M (28124.600 base freq)

// test only: 1 means you don't wait for starting minute from _U4B_channel ;
// does wait for any 2 minute alignment though
extern char _go_when_rdy[2];

/*
Verbosity:
0: none
1: temp/volts every second, message if no gps
2: GPS status every second
3: messages when a Tx started
4: x-tended messages when a Tx started
5: dump context every 20 secs
6: show PPB every second
7: Display GxRMC and GxGGA messages
8: display ALL NMEA sentences from GPS module
9: same as 8
*/


// #include <string.h>
// #include <ctype.h>
// #include <defines.h>
// #include "pico/stdlib.h"

// Echo user input to stdout and set input_variable
// prompt: Prompt to display to user <input>
// input_variable: Variable to which we want to read input <output>
// max_length: Maximum length of input string <input>
void get_user_input(const char *prompt, char *input_variable, int max_length) {
    int index = 0;
    int ch;

    // Display the prompt to the user
    printf("%s", prompt);
    fflush(stdout);

    while (1) {
        ch = getchar();
        if (ch == '\n' || ch == '\r') {  // Enter key pressed
            break;
        // Backspace key pressed (127 for most Unix, 8 for Windows)
        } else if (ch == 127 || ch == 8) {
            if (index > 0) {
                index--;
                printf("\b \b");  // Move back, print space, move back again
            }
        } else if (isprint(ch)) {
            if (index < max_length - 1) {  // Ensure room for null terminator
                input_variable[index++] = ch;
                printf("%c", ch);  // Echo character
            }
        }
        fflush(stdout);
        // whenever something might have taken a long time like printing the big buffer
        updateStatusLED();
    }

    input_variable[index] = '\0';  // Null-terminate the string
    printf("\n");
}

// Hex listing of the settings FLASH to stdout
// buf: Address of FLASH to list <input>
// len: Length of storage to list <input>
void printFLASH(const uint8_t *buf, size_t len) {
    printf("%s%s%s%s\nFLASH dump: \n%s%s",
        CLEAR_SCREEN, BRIGHT, BOLD_ON, UNDERLINE_ON, BOLD_OFF, UNDERLINE_OFF);

    for (size_t i = 0; i < len; ++i) {
        printf("%02x", buf[i]);
        if (i % 16 == 15) printf("\n");
        else printf(" ");
    }
    printf("%s", NORMAL);
}

void display_intro(void) {
    printf("%s%s%s\n\n\n\n\n\n\n\n\n\n\n\n", CLEAR_SCREEN, CURSOR_HOME, BRIGHT);
    printf("=====================================================\n\n");
    printf("%stracker: AD6Z firmware for AG6NS 0.04 pcb", UNDERLINE_ON);
    printf("version: %s %s\n\n%s", __DATE__ , __TIME__, UNDERLINE_OFF);
    printf("\n===================================================\n");
    printf("%spress any key to continue%s", RED, NORMAL);

    // wait
    char c = getchar_timeout_us(60000000);
    printf("%s", CLEAR_SCREEN);
}

void show_TELEN_msg() {
    printf("%s%s\n\n\n\nTELEN CONFIG INSTRUCTIONS:\n\n%s%s",
        BRIGHT, UNDERLINE_ON, UNDERLINE_OFF, NORMAL);
    printf("* There are 4 possible TELEN values, corresponding to TELEN 1 value 1,\n");
    printf("  TELEN 1 value 2, TELEN 2 value 1 and TELEN 2 value 2.\n");
    printf("* Enter 4 characters (legal 0-9 or -) in TELEN_config.\n");
    printf("  use a '-' (minus) to disable one or more values.\n");
    printf("  example:\n");
    printf("  '----' disables all telen \n");
    printf("* example:\n");
    printf("  '01--'\n");
    printf("    Telen 1 value 1 to type 0\n");
    printf("    Telen 1 value 2 to type 1\n");
    printf("    disables all of TELEN 2\n");

    printf("%s%s\nTelen Types:\n\n%s%s",
        BRIGHT, UNDERLINE_ON, UNDERLINE_OFF, NORMAL);
    printf("-: disabled, 0: ADC0, 1: ADC1, 2: ADC2, 3: ADC3,\n");

    printf("4: minutes since boot, 5: minutes since GPS fix aquired \n");
    printf("6-9: OneWire temperature sensors 1 though 4 \n");
    printf("A: custom: OneWire temperature sensor 1 hourly low/high \n");
    printf("B-Z: reserved for future I2C devices etc \n");
d
    printf("\n(ADC values are in units of mV)\n");
    printf("See the Wiki for more info.\n\n");
}

// called if keystroke from terminal on USB detected during operation.
void user_interface(void) {
    int c;
    char str[10];

    sleep_ms(100);
    turnOnLED(true);
    display_intro();
    show_values();

    // background
    // https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
    for (;;) {
        printf("%s%s\nEnter single char command: X, C, U, V, T, K, A, P, D, R, G%s%s",
            UNDERLINE_ON, BRIGHT, UNDERLINE_OFF, NORMAL);
        // in case user setup menu entered during flight,
        // this will reboot after 60 secs
        c = getchar_timeout_us(60000000);
        printf("%c\n", c);
        if (c == PICO_ERROR_TIMEOUT) {
            printf("%s\n\n Timeout waiting for input, ..rebooting\n", CLEAR_SCREEN);
            sleep_ms(100);
            Watchdog.enable(500);  // milliseconds
            for (;;) {}}

        // make char capital either way
        if (c > 90) c -= 32;
        switch (c) {
            case 'X':
                printf("%s\n\nGoodbye ..rebooting", CLEAR_SCREEN);
                Watchdog.enable(500);  // milliseconds
                for (;;)    {}
            case 'C':
                // FIX! will 1 char send wspr?
                get_user_input("Enter callsign: (3 to 6 chars: 1 to 3 [A-Z0-9] + 0 to 3 [A-Z]", _callsign, sizeof(_callsign));
                convertToUpperCase(_callsign);
                write_FLASH();
                break;
            case 'U':
                get_user_input("Enter U4B channel (0-599): ", _U4B_chan, sizeof(_U4B_chan));
                process_chan_num();
                write_FLASH();
                break;
            case 'V':
                get_user_input("Enter Verbosity level (0-9): ", _verbosity, sizeof(_verbosity));
                write_FLASH();
                break;
            case 'T':
                show_TELEN_msg();
                get_user_input("Enter TELEN config: ", _TELEN_config, sizeof(_TELEN_config));
                convertToUpperCase(_TELEN_config);
                write_FLASH();
                break;
            case 'K':
                get_user_input("Enter clock speed (100-250): ", _clock_speed, sizeof(_clock_speed));
                write_FLASH();
                // frequencies like 205 mhz will PANIC,
                // System clock of 205000 kHz cannot be exactly achieved
                // should detect the failure and change the nvram, otherwise we're stuck even on reboot
                // this is the only config where we don't let something bad get into flash
                // don't change the pll, just check. change it on reboot
                if (atoi(_clock_speed) < 100 || atoi(_clock_speed) > 250) {
                    printf("%s\n_clock_speed %s is not supported/legal, initting to 133\n%s",
                        RED, _TELEN_config, NORMAL);
                    snprintf(_clock_speed, sizeof(_clock_speed), "133");
                    write_FLASH();
                }
                uint32_t clkhz =  atoi(_clock_speed) * 1000000L;

                if (!set_sys_clock_khz(clkhz / kHz, false)) {
                    printf("%s\n RP2040 can't change clock to %dMhz. Using 133 instead\n%s",
                        RED, PLL_SYS_MHZ, NORMAL);
                    snprintf(_clock_speed, sizeof(_clock_speed), "133");
                    write_FLASH();
                }
                break;

            case 'A':
                get_user_input("Enter Band (10,12,15,17,20): ", _Band, sizeof(_Band));
                // redo channel selection if we change bands, since U4B definition changes per band
                write_FLASH();
                process_chan_num();
                XMIT_FREQUENCY = init_rf_freq();
                break;

            case 'P':
                get_user_input("Enter Tx power: (0 or 1) ", _tx_power, sizeof(_tx_power));
                write_FLASH();
                break;
            case 'D':
                get_user_input("Enter DEVMODE to enable messaging: (0 or 1) ", _devmode, sizeof(_devmode));
                write_FLASH();
                break;
            case 'R':
                printf("Don't cause than approx. 43 hz 'correction' on a band. Effect varies per band?");
                get_user_input("Enter ppb Correction to si5351: (-3000 to 3000) ", _correction, sizeof(_correction));
                write_FLASH();
                break;
            case 'D':
                get_user_input("Enter go_when_rdy for faster test..any 2 minute start: (0 or 1) ",
                    _go_when_rdy, sizeof(_go_when_rdy));
                write_FLASH();
                break;
            case 'G':
                printf("test only: 1 means you don't wait for starting minute from _U4B_channel");
                printf("does wait for any 2 minute alignment though");
                get_user_input("Enter go_when_rdy for faster test..any 2 minute start: (0 or 1) ",
                    _go_when_rdy, sizeof(_go_when_rdy));
                write_FLASH();
                break;

            case 13:  break;
            case 10:  break;
            default:
                printf("%s\nYou pressed: %c - (0x%02x), invalid choice! ",
                    CLEAR_SCREEN, c, c);
                sleep_ms(1000);
                break;
        }
        int result = check_data_validity_and_set_defaults();
        show_values();
    }
}

// Reads flash where the user settings are saved
// prints hexa listing of data
// calls function which check data validity
// background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
void read_FLASH(void) {
    // pointer to a safe place after the program memory
    const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    printFLASH(flash_target_contents, FLASH_PAGE_SIZE);  // 256

    // null terminate in case it's printf'ed with %s
    // potentially has null also before the end?
    // FIX! shouldn't all of these (all chars) have null terminate?
    // FIX! left gaps (unused)
    // multichar that have room have a null in the flash?
    strncpy(_callsign,     flash_target_contents+0,  6); _callsign[6] = 0;
    strncpy(_verbosity,    flash_target_contents+6,  1); _verbosity[1] = 0;
    strncpy(_TELEN_config, flash_target_contents+7,  4); _TELEN_config[4] = 0;
    strncpy(_clock_speed,  flash_target_contents+11, 3); _clock_speed[3] = 0;
    strncpy(_U4B_chan,     flash_target_contents+14, 3); _U4B_chan[3] = 0;
    strncpy(_Band,         flash_target_contents+17, 2); _Band[2] = 0;
    strncpy(_tx_power,     flash_target_contents+19, 1); _tx_power[1] = 0;
    strncpy(_devmode,      flash_target_contents+20, 1); _devmode[1] = 0;
    strncpy(_correction,   flash_target_contents+21, 6); _correction[6] = 0;
    strncpy(_go_when_rdy,  flash_target_contents+27, 1); _go_when_rdy[1] = 0;

    PLL_SYS_MHZ = atoi(_clock_speed);

    // FIX! change to _band everywhere

    // FIX! we should decode the _Band/_U4B_chan and set any ancillary decode vars?
    // any XMIT_FREQUENCY ?
    process_chan_num();
    // _32_dialfreqhz not used any more
    // FIX! define this as extern?
    XMIT_FREQUENCY = init_rf_freq();
    if (_devmode[0] == 1) DEVMODE = true;
    else DEVMODE = false;
}

// Write the user entered data into FLASH
void write_FLASH(void) {
    // initializes all to zeroes
    uint8_t data_chunk[FLASH_PAGE_SIZE] = { 0 };  // 256 bytes

    strncpy(data_chunk+0,  _callsign, 6);
    strncpy(data_chunk+6,  _verbosity, 1);
    strncpy(data_chunk+7,  _TELEN_config, 4);
    strncpy(data_chunk+11, _clock_speed, 3);
    strncpy(data_chunk+14, _U4B_chan, 3);
    strncpy(data_chunk+17, _Band, 2);
    strncpy(data_chunk+19, _tx_power, 1);
    strncpy(data_chunk+20, _devmode, 1);
    strncpy(data_chunk+21, _correction, 6);

    // you could theoretically write 16 pages at once (a whole sector).
    // don't interrupt
    uint32_t ints = save_and_disable_interrupts();

    // a "Sector" is 4096 bytes
    // FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE, FLASH_PAGE_SIZE = 040000x, 4096, 256
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    // writes 256 bytes (one "page") (16 pages per sector)
    flash_range_program(FLASH_TARGET_OFFSET, data_chunk, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

// Checks validity of user settings and if something is wrong,
// sets "factory defaults" and writes it back to FLASH
// create result to return
int check_data_validity_and_set_defaults(void) {
    int result = 1;
    // set reasonable defaults if memory was uninitialized or has bad values
    // create 'result' to return
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

    // when you send to wspr
    // JTEncode handles this
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
    bool callsignBad = False;
    if (clength < 3) {
        callsignBad = true
    }

    if (clength > 6) {
        callsignBad = true
    } else if (clength >= 3) {
        for (i = 0; i <= 2; i--) {
            if ((_callsign[i] < 'A' && _callsign[i] > 'Z') &&
                (_callsign[i] < '0' && _callsign[i] > '9')) {
                callsignBad = true;
            }
        }
    } else if (clength >= 4) {
        i = 3;
        if (_callsign[i] < 'A' || _callsign[i] > 'Z') callsignBad = true;
    } else if (clength >= 5) {
        i = 4;
        if (_callsign[i] < 'A' || _callsign[i] > 'Z') callsignBad = true;
    } else if (clength == 6) {
        i = 5;
        if (_callsign[i] < 'A' || _callsign[i] > 'Z') callsignBad = true;
    }

    if (callsignBad) {
        printf("%s\n_callsign %s is not supported/legal, initting to AB1CDE\n%s",
            RED, _callsign, NORMAL);
        snprintf(_callsign, sizeof(_callsign), "AB1CDE");
        write_FLASH();
        result = -1;
    }

    // change to strcpy for null terminate
    if (_verbosity[0] < '0' || _verbosity[0] > '9') {
        printf("%s\n_verbosity %s is not supported/legal, initting to 1\n%s",
            RED, _verbosity, NORMAL);
        snprintf(_verbosity, sizeof(_verbosity), "1");
        write_FLASH();
        result = -1;
    }

    // 0-9 and - are legal. _
    // make sure to null terminate
    int i;
    for (i = 1; i <= 3; ++i) {
        if ((_TELEN_config[i] < '0' || _TELEN_config[i] > '9') && _TELEN_config[i] != '-') {
            printf("%s\n_TELEN_config %s is not supported/legal, initting to ---\n%s",
                RED, _TELEN_config, NORMAL);
            snprintf(_TELEN_config, sizeof(_TELEN_config), "----");
            write_FLASH();
            result = -1;
        }
    }

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
    if (atoi(_clock_speed) < 100 || atoi(_clock_speed) > 250) {
        printf("%s\n_clock_speed %s is not supported/legal, initting to 133\n%s", RED, _clock_speed, NORMAL);
        snprintf(_clock_speed, sizeof(_clock_speed), "133");
        write_FLASH();
        result = -1;
    }

    uint32_t clkhz =  atoi(_clock_speed) * 1000000L;
    if (!set_sys_clock_khz(clkhz / kHz, false)) {
        printf("%s\n RP2040 can't change clock to %dMhz. Using 133 instead\n%s", RED, PLL_SYS_MHZ, NORMAL);
        snprintf(_clock_speed, sizeof(_clock_speed), "133");
        write_FLASH();
        result = -1;
    }

    //*********
    // be sure to null terminate
    if (atoi(_U4B_chan) < 0 || atoi(_U4B_chan) > 599) {
        printf("%s\n_U4B_chan %s is not supported/legal, initting to 599\n%s", RED, _U4B_chan, NORMAL);
        snprintf(_U4B_chan, sizeof(_U4B_chan), "599");
        write_FLASH();
        // this will set _lane, _id13, _start_minute
        process_chan_num();
        XMIT_FREQUENCY = init_rf_freq();
        result = -1;
    }
    //****************
    // kevin 10_30_24
    switch (atoi(_Band)) {
        case 10: break;
        case 12: break;
        case 15: break;
        case 17: break;
        case 20: break;
        default:
            printf("%s\n_Band %s is not supported/legal, initting to 20\n%s", RED, _Band, NORMAL);
            snprintf(_Band, sizeof(_Band), "20");
            write_FLASH();
            // FIX! stop using _32_dialfreqhz
            // figure out the XMIT_FREQUENCY for new band, and set _32_dialfreqhz
            // have to do this whenever we change bands
            process_chan_num();
            XMIT_FREQUENCY = init_rf_freq();
            result = -1;
            break;
    }
    if (_tx_power[0] != '0' && _tx_power[0] > '1') {
        printf("%s\n_tx_power %s is not supported/legal, initting to 1\n%s",
            RED, _tx_power, NORMAL);
        snprintf(_tx_power, sizeof(_tx_power), "1");
        write_FLASH();
        result = -1;
    }
    if (_devmode[0] != '0' && _devmode[0] > '1') {
        printf("%s\n_devmode %s is not supported/legal, initting to 0\n%s",
            RED, _devmode, NORMAL);
        snprintf(_devmode, sizeof(_devmode), "0");
        write_FLASH();
        result = -1;
    }
    if (atoi(_correction) < -3000 || atoi(_correction) > 3000) {
        printf("%s\n_correction %s is not supported/legal, initting to 0\n%s",
            RED, _correction, NORMAL);
        snprintf(_correction, sizeof(_correction), "0");
        write_FLASH();
        result = -1;
    }
    if (_go_when_rdy[0] != '0' && _go_when_rdy[0] > '1') {
        printf("%s\n_go_when_rdy %s is not supported/legal, initting to 0\n%s",
            RED, _go_when_rdy, NORMAL);
        snprintf(_go_when_rdy, sizeof(_go_when_rdy), "0");
        write_FLASH();
        result = -1;
    }
    return result;
    //****************
}

// Function that writes out the current set values of parameters
void show_values(void) /* shows current VALUES  AND list of Valid Commands */ {
    printf("%s%s%s\n\nCurrent values:\n%s%s", CLEAR_SCREEN, UNDERLINE_ON, BRIGHT, UNDERLINE_OFF, NORMAL);

    printf("\n\tcallsign:%s\n\t", _callsign);
    printf("U4B channel:%s", _U4B_chan);
    printf(" (id13:%s", _id13);
    printf(" start Minute:%s", _start_minute);
    printf(" lane:%s)\n\t", _lane);
    printf("verbosity:%s\n\t", _verbosity);
    printf("TELEN config:%s\n\t", _TELEN_config);
    printf("clock speed:%sMhz\n\t", _clock_speed);
    printf("band:%s\n\t", _Band);
    printf("DEVMODE:%s\n\t", _devmode);
    printf("correction:%s\n\t", _correction);
    printf("go_when_rdy:%s\n\t", _go_when_rdy);
    printf("XMIT_FREQUENCY:%d\n\t", XMIT_FREQUENCY);

    printf("%s%sValid commands: %s%s", UNDERLINE_ON, BRIGHT, UNDERLINE_OFF, NORMAL);

    printf("\n\n\tX: eXit configuration and reboot\n\t");
    printf("C: change Callsign (6 char max)\n\t");
    printf("U: change U4b channel # (0-599)\n\t");
    printf("A: change band (10,12,15,17,20 default 20)\n\t");
    printf("V: verbosity (0 for no messages, 9 for all) \n\t");
    printf("T: TELEN config\n\t");
    printf("K: clock speed  (default: 133)\n\t");
    printf("D: DEVMODE to enable messaging (default: 0)\n\t");
    printf("R: si5351 ppb correction (-3000 to 3000) (default: 0)\n\t");
    printf("R: go_when_ready (callsign tx starts with any modulo 2 starting minute (default: 0)\n\t");
}

// Converts string to upper case
// string to convert <inout>
// No return value, string is converted directly in the parameter *str
void convertToUpperCase(char *str) {
    while (*str) {
        *str = toupper((unsigned char)*str);
        str++;
    }
}
