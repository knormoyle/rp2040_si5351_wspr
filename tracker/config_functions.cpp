// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#include <Arduino.h>

#include <stdint.h>
#include <stdio.h>
#include "pico/stdio.h"
// what about this?
#include "class/cdc/cdc_device.h"

#include <stdlib.h>
// for isprint()
#include <ctype.h>

#include"pico/stdlib.h"
#include"hardware/flash.h"

// my stuff
#include "defines.h"
#include "led_functions.h"
#include "u4b_functions.h"
#include "config_functions.h"

#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// FIX! is the program bigger than 256K

// this is 256K from start of flash
// #define FLASH_TARGET_OFFSET (256 * 1024) // leaves 256K of space for the program
// this is 1M from start of flash

#define FLASH_TARGET_OFFSET (4 * 256 * 1024) // leaves 1M of space for the program

// already defined?
// #define FLASH_SECTOR_SIZE 4096
// #define FLASH_PAGE_SIZE 256

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


// don't allow more than approx. 43 hz "correction" on a band. leave room for 6 chars
extern char _correction[7];  // parts per billion -3000 to 3000. default 0
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
    static bool HACK = true;
    if (HACK) {
        // HACK FIX! always true now for debug
        // https://stackoverflow.com/questions/2606539/snprintf-vs-strcpy-etc-in-c
        // recommends to always
        // snprintf(buffer, sizeof(buffer), "%s", string);

        Serial.println(F("Forcing DEVMODE true, _devmode 1 (always for now)"));
        strncpy(_devmode, "1", sizeof(_devmode));
        DEVMODE = true;  // set when _devmode is set
        // HACK FIX! always 9 now for debug

        Serial.println(F("Forcing _verbosity to 9 (always for now)"));
        strncpy(_verbosity, "9", sizeof(_verbosity));
    }
    else {
        if (_devmode[0] == '1') DEVMODE = true;
        else DEVMODE = true;
    }
}


//***************************************
// Echo user input to stdout and set input_variable
// prompt: Prompt to display to user <input>
// input_variable: Variable to which we want to read input <output>
// max_length: Maximum length of input string <input>
void get_user_input(const char *prompt, char *input_variable, int max_length) {
    Watchdog.reset();
    updateStatusLED();

    int index = 0;
    int ch;

    // Display the prompt to the user
    Serial.printf("%s", prompt);
    // FIX! does this work if printf doesn't
    // fflush(stdout);
    Serial.flush();

    while (1) {
        
        int timeout_ms = 0;
        while (!Serial.available()) { 
            sleep_ms(250);
            timeout_ms += 250;
            // this will eventually watchdog reset timeout if no character?
            updateStatusLED();
            if (timeout_ms > 15 * 1000) {
                Serial.println(F("ERROR: timeout waiting for input, rebooting" EOL));
                Watchdog.enable(50);  // milliseconds
                while (1) { ; }
            }
         }
        ch = Serial.read();
        
        if (ch == '\n' || ch == '\r') {  // Enter key pressed
            break;
        // Backspace key pressed (127 for most Unix, 8 for Windows)
        } else if (ch == 127 || ch == 8) {
            if (index > 0) {
                index--;
                Serial.printf("\b \b");  // Move back, print space, move back again
            }
        } else if (isprint(ch)) {
            if (index < max_length - 1) {  // Ensure room for null terminator
                input_variable[index++] = ch;
                Serial.printf("%c", ch);  // Echo character
            }
        }
        Serial.flush();
        // fflush(stdout);
        // whenever something might have taken a long time like printing the big buffer
        updateStatusLED();
    }

    input_variable[index] = '\0';  // Null-terminate the string
    Serial.printf("\n");
}

// Hex listing of the settings FLASH to stdout
// buf: Address of FLASH to list <input>
// len: Length of storage to list <input>
void printFLASH(const uint8_t *buf, size_t len) {
    Serial.printf("%s%s%s%s\nFLASH dump: \n%s%s",
        CLEAR_SCREEN, BRIGHT, BOLD_ON, UNDERLINE_ON, BOLD_OFF, UNDERLINE_OFF);

    for (size_t i = 0; i < len; ++i) {
        Serial.printf("%02x", buf[i]);
        if (i % 16 == 15) Serial.printf("\n");
        else Serial.printf(" ");
    }
    Serial.printf("%s", NORMAL);
}

//***************************************
void config_intro(void) {
    Serial.println(F("config_intro() START"));

    setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
    updateStatusLED();

    // should all do the same thing
    Serial.println(F("test Serial.println config_intro() Serial.println()"));
    // can't use F() for this case
    Serial.printf("test Serial.printf(\"..\", EOL) config_intro() Serial.printf%s", EOL);
    Serial.print(F("test Serial.print(.. EOL) config_intro()" EOL));

    Serial.print(F(CLEAR_SCREEN CURSOR_HOME BRIGHT));
    for (int i = 0; i < 10; i++) Serial.println();
    Serial.print(F(UNDERLINE_ON));
    Serial.println("tracker: AD6Z firmware, AG6NS 0.04 pcb, JLCPCB with *kbn* or *kbn2* bom/cpl mod");
    Serial.println("https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker");
    Serial.println("tracker.ino firmware version: " __DATE__ " "  __TIME__);
    Serial.println("support: knormoyle@gmail.com or https://groups.io/g/picoballoon");
    Serial.print(F(UNDERLINE_OFF));
    Serial.print(F(RED));
    Serial.println("press any key to continue");
    Serial.print(F(NORMAL));

    // wait..don't need the char user interrupted with 
    // int c = getchar_timeout_us(60000000);
    // PICO_ERROR_GENERIC PICO_ERROR_TIMEOUT ??
    // int c = getchar_timeout_us(0);

    int i;
    char incomingByte = { 0 };
    for (i = 0; i < 10*5; i++) {
        Watchdog.reset();
        if (!Serial.available()) {
            sleep_ms(200);
        }
        else {
            incomingByte = Serial.read();
            Serial.println(incomingByte);
            if (incomingByte != 13) {
                Serial.readStringUntil(13); // empty readbuffer after good data
            }
            break;
        }
    }
    if (i == 10*5) {
        Serial.println("(2) Must have timed out looking for input char(s) on Serial");
    }
    // FIX! assume this is the state it was in before config menu? 
    // not always right. but loop will self-correct?
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();

    Watchdog.reset();
    // string concat works here with defines.sh special strings
    Serial.print(F("config_intro() END" CLEAR_SCREEN));

}

//***************************************
void show_TELEN_msg() {
    Serial.println(F("show_TELEN_msg()"));
    Serial.printf("%s%s\n\n\n\nTELEN CONFIG INSTRUCTIONS:\n\n%s%s",
        BRIGHT, UNDERLINE_ON, UNDERLINE_OFF, NORMAL);
    Serial.printf("* There are 4 possible TELEN values, corresponding to TELEN 1 value 1,\n");
    Serial.printf("  TELEN 1 value 2, TELEN 2 value 1 and TELEN 2 value 2.\n");
    Serial.printf("* Enter 4 characters (legal 0-9 or -) in TELEN_config.\n");
    Serial.printf("  use a '-' (minus) to disable one or more values.\n");
    Serial.printf("  example:\n");
    Serial.printf("  '----' disables all telen \n");
    Serial.printf("* example:\n");
    Serial.printf("  '01--'\n");
    Serial.printf("    Telen 1 value 1 to type 0\n");
    Serial.printf("    Telen 1 value 2 to type 1\n");
    Serial.printf("    disables all of TELEN 2\n");

    Serial.printf("%s%s\nTelen Types:\n\n%s%s",
        BRIGHT, UNDERLINE_ON, UNDERLINE_OFF, NORMAL);
    Serial.printf("-: disabled, 0: ADC0, 1: ADC1, 2: ADC2, 3: ADC3,\n");

    Serial.printf("4: minutes since boot, 5: minutes since GPS fix aquired \n");
    Serial.printf("6-9: OneWire temperature sensors 1 though 4 \n");
    Serial.printf("A: custom: OneWire temperature sensor 1 hourly low/high \n");
    Serial.printf("B-Z: reserved for future I2C devices etc \n");
    Serial.printf("\n(ADC values are in units of mV)\n");
    Serial.printf("See the Wiki for more info.\n\n");
}

// called if keystroke from terminal on USB detected during operation.

//***************************************
void user_interface(void) {
    // Does println do better compared to my Serial.print() with LINEND as \r\n ??
    // Serial.println()
    // Prints data to the serial port as human-readable ASCII text 
    // followed by a carriage return character (ASCII 13, or '\r') 
    // and a newline character (ASCII 10, or '\n'). 
    // Serial.println(val)
    // Serial.println(val, format)
    // Serial: serial port object. 
    // val: the value to print - any data type

    // format can be DEC, HEX, OCT, BIN ..prints as ASCII-encoded.
    // format: specifies the number base (for integral data types) 
    // or number of decimal places (for floating point types)

    // seems like we can use it
    Serial.println(F("user_interface() START"));
    sleep_ms(100);
    // we have a different blinking pattern now?
    // turnOnLED(true);
    config_intro();
    show_values();

    for (;;) {
        Serial.print(F(UNDERLINE_ON BRIGHT UNDERLINE_OFF NORMAL));
        // no comma to concat strings
        // F() to keep string in flash, not ram
        Serial.print(F("(print) Enter single char command: /, X, C, U, V, T, K, A, P, D, R, G" EOL));
        Serial.println(F("(println) Enter single char command: /, X, C, U, V, T, K, A, P, D, R, G"));
        Serial.print(F(UNDERLINE_OFF NORMAL));

        // getchar_timeout_us(): doesn't work linker problem
        // int c = getchar_timeout_us(60000000);
        // PICO_ERROR_GENERIC PICO_ERROR_TIMEOUT ??
        // int c = getchar_timeout_us(0);
        // look at c for error

        int i;
        char incomingByte = '\0';
        for (i = 0; i < 100*5; i++) {
            Watchdog.reset();
            if (!Serial.available()) {
                sleep_ms(200);
            }
            else { 
                incomingByte = Serial.read();
                Serial.println(incomingByte);
                if (incomingByte != 13) {
                    Serial.readStringUntil(13); // empty readbuffer if there's data
                }
                break;
            }
        }
        if (i == 100*5) {
            Serial.println("(2) Must have timed out looking for input char(s) on Serial");
        }
        Watchdog.reset();
        char c_char = (char) incomingByte;

        // FIX! how does this timeout
        // Serial.printf("%s\n", c_char);

        // if (c == PICO_ERROR_TIMEOUT) {
        if (c_char == 0) {
            Serial.printf("%s\n\n (3) Timeout waiting for input, ..rebooting\n", CLEAR_SCREEN);
            sleep_ms(100);
            Watchdog.enable(500);  // milliseconds
            for (;;) { ; }
        }

        // make char capital either way
        if (c_char > 90) c_char -= 32;

        // regenerated from _clock_speed below
        uint32_t clkhz = 0;
        // FIX! can we case the int32_t to char. we might lost data with the cast

        switch ( c_char ) {
            case '/':
                Serial.print(F("Rebooting to bootloader mode..drag/drop a uf2 per normal\r\n"));
                Serial.print(F("after reboot: drag/drop a uf2 the normal way to the drive that shows\r\n"));
                Serial.print(F("You should be able to leave usb connected. If not, disconnect/connect\r\n%"));
                Serial.printf("Goodbye!%s", CLEAR_SCREEN);
                Watchdog.enable(2000);  // milliseconds
                for (;;) { ; }
            case 'X':
                Serial.printf("%s\n\nGoodbye ..rebooting", CLEAR_SCREEN);
                Watchdog.enable(500);  // milliseconds
                for (;;) { ; }
            case 'C':
                // FIX! will 1 char send wspr?
                get_user_input("Enter callsign: (3 to 6 chars: 1 to 3 [A-Z0-9] + 0 to 3 [A-Z]", 
                    _callsign, sizeof(_callsign));
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
                clkhz = atoi(_clock_speed) * 1000000UL;
                // frequencies like 205 mhz will PANIC,
                // System clock of 205000 kHz cannot be exactly achieved
                // should detect the failure and change the nvram, otherwise we're stuck even on reboot
                // this is the only config where we don't let something bad get into flash
                // don't change the pll, just check. change it on reboot
                if (atoi(_clock_speed) < 100 || atoi(_clock_speed) > 250) {
                    Serial.printf("%s\n_clock_speed %s is not supported/legal, initting to 133\n%s",
                        RED, _TELEN_config, NORMAL);
                    // https://stackoverflow.com/questions/2606539/snprintf-vs-strcpy-etc-in-c
                    // recommends to always
                    // snprintf(buffer, sizeof(buffer), "%s", string);
                    // I guess this is okay for when we source from fixed size string literals
                    snprintf(_clock_speed, sizeof(_clock_speed), "133");
                    write_FLASH();
                    clkhz = atoi(_clock_speed) * 1000000UL;
                }
                if (!set_sys_clock_khz(clkhz / kHz, false)) {
                    Serial.printf("%s\n RP2040 can't change clock to %luMhz. Using 133 instead\n%s",
                        RED, PLL_SYS_MHZ, NORMAL);
                    snprintf(_clock_speed, sizeof(_clock_speed), "133");
                    write_FLASH();
                    clkhz = atoi(_clock_speed) * 1000000UL;
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
                get_user_input("Enter Tx high: (0 or 1) ", _tx_high, sizeof(_tx_high));
                write_FLASH();
                break;
            case 'D':
                get_user_input("Enter DEVMODE to enable messaging: (0 or 1) ", 
                    _devmode, sizeof(_devmode));
                write_FLASH();
                break;
            case 'R':
                Serial.printf("Don't cause than approx. 43 hz 'correction' on a band. Effect varies per band?");
                get_user_input("Enter ppb Correction to si5351: (-3000 to 3000) ", 
                    _correction, sizeof(_correction));
                write_FLASH();
                break;
            case 'G':
                Serial.printf("test only: 1 means you don't wait for starting minute from _U4B_channel");
                Serial.printf("does wait for any 2 minute alignment though");
                get_user_input("Enter go_when_rdy for faster test..any 2 minute start: (0 or 1) ",
                    _go_when_rdy, sizeof(_go_when_rdy));
                write_FLASH();
                break;
            case 13:  break;
            case 10:  break;
            default:
                Serial.printf("%s\nYou pressed: %c - (0x%02x), invalid choice! ", CLEAR_SCREEN, c_char, c_char);
                sleep_ms(1000);
                break;
        }
        check_data_validity_and_set_defaults();
        show_values();
        Serial.println(F("user_interface() END"));
    }
}

//***************************************
// Prints out hex listing of the settings FLASH to stdio
// buf: address of FLASH to list <input>
// len: bytes of FLASH to list <input>
void print_buf(const uint8_t *buf, int len) {
    Serial.println(F("print_buf()"));
    Serial.printf("%s%s%s%s\nFLASH dump:\n%s%s", 
        CLEAR_SCREEN, BRIGHT, BOLD_ON, UNDERLINE_ON, BOLD_OFF, UNDERLINE_OFF);
    for (int i = 0; i < len; ++i) {
        Serial.printf("%02x", buf[i]);
        if (i % 16 == 15) Serial.printf("\n");
        else Serial.printf(" ");
    }
    Serial.printf("%s", NORMAL);
}

// Reads flash where the user settings are saved
// prints hexa listing of data
// calls function which check data validity


//***************************************
// background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
#define FLASH_BYTES_USED 28
void read_FLASH(void) {

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
    const uint8_t *uflash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    print_buf(uflash_target_contents, (int) FLASH_PAGE_SIZE);  // 256

    char flash_target_contents[FLASH_BYTES_USED] = { 0 };
    for (int i = 0; i < FLASH_BYTES_USED ; i++) {
        flash_target_contents[i] = (char) uflash_target_contents[i];
    }
    // BE SURE YOU ONLY USE ONE PAGE: i.e. 256 bytes total
    // FIX! should we just use snprintf?
    strncpy(_callsign,     flash_target_contents + 0,  6); _callsign[6] = 0;
    strncpy(_verbosity,    flash_target_contents + 6,  1); _verbosity[1] = 0;
    strncpy(_TELEN_config, flash_target_contents + 7,  4); _TELEN_config[4] = 0;
    strncpy(_clock_speed,  flash_target_contents + 11, 3); _clock_speed[3] = 0;
    strncpy(_U4B_chan,     flash_target_contents + 14, 3); _U4B_chan[3] = 0;
    // FIX! change to _band everywhere?
    strncpy(_Band,         flash_target_contents + 17, 2); _Band[2] = 0;
    strncpy(_tx_high,      flash_target_contents + 19, 1); _tx_high[1] = 0;
    strncpy(_devmode,      flash_target_contents + 20, 1); _devmode[1] = 0;
    strncpy(_correction,   flash_target_contents + 21, 6); _correction[6] = 0;
    strncpy(_go_when_rdy,  flash_target_contents + 27, 1); _go_when_rdy[1] = 0;

    PLL_SYS_MHZ = atoi(_clock_speed);

    // FIX! we should decode the _Band/_U4B_chan and set any ancillary decode vars?
    // any XMIT_FREQUENCY ?
    process_chan_num();
    // _32_dialfreqhz not used any more
    // FIX! define this as extern?
    XMIT_FREQUENCY = init_rf_freq();

    // fix anything bad! both in _* variables and FLASH (defaults)
    check_data_validity_and_set_defaults(); 

    // hack _devmode _verbosity DEVMODE
    forceHACK();

}

//***************************************
// Write the user entered data into FLASH
void write_FLASH(void) {
    // initializes all to zeroes
    char data_chunk[FLASH_BYTES_USED] = { 0 };  // enough to cover what we use here
    uint8_t udata_chunk[FLASH_PAGE_SIZE] = { 0 };  // 256 bytes

    // don't take the extra null term (but _callsign might be short!)
    strncpy(data_chunk + 0,  _callsign, 6);
    strncpy(data_chunk + 6,  _verbosity, 1);
    strncpy(data_chunk + 7,  _TELEN_config, 4);
    strncpy(data_chunk + 11, _clock_speed, 3);
    strncpy(data_chunk + 14, _U4B_chan, 3);
    strncpy(data_chunk + 17, _Band, 2);
    strncpy(data_chunk + 19, _tx_high, 1);
    strncpy(data_chunk + 20, _devmode, 1);
    strncpy(data_chunk + 21, _correction, 6);
    strncpy(data_chunk + 27, _go_when_rdy, 1);

    // you could theoretically write 16 pages at once (a whole sector).
    // don't interrupt
    uint32_t ints = save_and_disable_interrupts();

    // a "Sector" is 4096 bytes
    // FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE, FLASH_PAGE_SIZE = 040000x, 4096, 256

    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    // writes 256 bytes (one "page") (16 pages per sector)

    // alternative for casting the array to uint8_t
    // https://stackoverflow.com/questions/40579902/how-to-turn-a-character-array-into-uint8-t

    // If you're on an architecture where uint8_t is a typedef to unsigned char,
    // then simply take the first char and cast it to uint8_t:
    // int length = (uint8_t)(udata_chunk[0]);

    for (int i = 0; i < FLASH_BYTES_USED ; i++) {
        udata_chunk[i] = (uint8_t) data_chunk[i];
    }

    flash_range_program(FLASH_TARGET_OFFSET, udata_chunk, FLASH_PAGE_SIZE);
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
    bool callsignBad = false;
    if (clength < 3) {
        callsignBad = true;
    }

    int i;
    if (clength > 6) {
        callsignBad = true;
    } else if (clength >= 3) {
        for (i = 0; i <= 2; i++) {
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
        Serial.printf("%s\n_callsign %s is not supported/legal, initting to AB1CDE\n%s",
            RED, _callsign, NORMAL);
        snprintf(_callsign, sizeof(_callsign), "AB1CDE");
        write_FLASH();
        result = -1;
    }

    // change to strcpy for null terminate
    if (_verbosity[0] < '0' || _verbosity[0] > '9') {
        Serial.printf("%s\n_verbosity %s is not supported/legal, initting to 1\n%s",
            RED, _verbosity, NORMAL);
        snprintf(_verbosity, sizeof(_verbosity), "1");
        write_FLASH();
        result = -1;
    }

    // 0-9 and - are legal. _
    // make sure to null terminate
    for (int i = 1; i <= 3; ++i) {
        if ((_TELEN_config[i] < '0' || _TELEN_config[i] > '9') && _TELEN_config[i] != '-') {
            Serial.printf("%s\n_TELEN_config %s is not supported/legal, initting to ---\n%s",
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
        Serial.printf("%s\n_clock_speed %s is not supported/legal, initting to 133\n%s", 
            RED, _clock_speed, NORMAL);
        snprintf(_clock_speed, sizeof(_clock_speed), "133");
        write_FLASH();
        result = -1;
    }

    uint32_t clkhz = atoi(_clock_speed) * 1000000UL;
    if (!set_sys_clock_khz(clkhz / kHz, false)) {
        // http://jhshi.me/2014/07/11/print-uint64-t-properly-in-c/index.html
        Serial.printf("%s\n RP2040 can't change clock to %luMhz. Using 133 instead\n%s", 
            RED, PLL_SYS_MHZ, NORMAL);
        snprintf(_clock_speed, sizeof(_clock_speed), "133");
        write_FLASH();
        result = -1;
    }

    //*********
    // be sure to null terminate
    if (atoi(_U4B_chan) < 0 || atoi(_U4B_chan) > 599) {
        Serial.printf("%s\n_U4B_chan %s is not supported/legal, initting to 599\n%s", 
            RED, _U4B_chan, NORMAL);
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
            Serial.printf("%s\n_Band %s is not supported/legal, initting to 20\n%s", RED, _Band, NORMAL);
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
    if (_tx_high[0] != '0' && _tx_high[0] > '1') {
        Serial.printf("%s\n_tx_high %s is not supported/legal, initting to 1\n%s",
            RED, _tx_high, NORMAL);
        snprintf(_tx_high, sizeof(_tx_high), "1");
        write_FLASH();
        result = -1;
    }
    if (_devmode[0] != '0' && _devmode[0] > '1') {
        Serial.printf("%s\n_devmode %s is not supported/legal, initting to 0\n%s",
            RED, _devmode, NORMAL);
        snprintf(_devmode, sizeof(_devmode), "0");
        write_FLASH();
        result = -1;
    }
    if (atoi(_correction) < -3000 || atoi(_correction) > 3000) {
        // left room for 6 bytes
        Serial.printf("%s\n_correction %s is not supported/legal, initting to 0\n%s",
            RED, _correction, NORMAL);
        snprintf(_correction, sizeof(_correction), "0");
        write_FLASH();
        result = -1;
    }
    if (_go_when_rdy[0] != '0' && _go_when_rdy[0] > '1') {
        Serial.printf("%s\n_go_when_rdy %s is not supported/legal, initting to 0\n%s",
            RED, _go_when_rdy, NORMAL);
        snprintf(_go_when_rdy, sizeof(_go_when_rdy), "0");
        write_FLASH();
        result = -1;
    }
    return result;
    //****************
}

//***************************************
// Function that writes out the current set values of parameters
void show_values(void) /* shows current VALUES  AND list of Valid Commands */ {
    Serial.println(F("show_values() START"));

    // Serial.printf("%s%s%s\r\n", CLEAR_SCREEN, UNDERLINE_ON, BRIGHT);
    // since these macros are "" strings in defines.h, they will just concat here
    // no commas necessary?
    Serial.print(F(CLEAR_SCREEN UNDERLINE_ON BRIGHT "\r\n"));
    Serial.print(F("1" "2" "3" "testing print string concat" "\r\n"));

    Serial.print(F("Current values:\r\n"));
    Serial.printf("%s%s\r\n", UNDERLINE_OFF, NORMAL);

    Serial.printf("callsign:%s\r\n", _callsign);
    Serial.printf("U4B channel:%s", _U4B_chan);
    Serial.printf(" (id13:%s", _id13);
    Serial.printf(" start Minute:%s", _start_minute);
    Serial.printf(" lane:%s)\r\n", _lane);
    Serial.printf("verbosity:%s\r\n", _verbosity);
    Serial.printf("TELEN config:%s\r\n", _TELEN_config);
    Serial.printf("clock speed:%sMhz\r\n", _clock_speed);
    Serial.printf("band:%s\r\n", _Band);
    Serial.printf("DEVMODE:%s\r\n", _devmode);
    Serial.printf("correction:%s\r\n", _correction);
    Serial.printf("go_when_rdy:%s\r\n", _go_when_rdy);
    Serial.printf("XMIT_FREQUENCY:%lu\r\n", XMIT_FREQUENCY);

    Serial.printf("%s%sValid commands: %s%s\r\n", UNDERLINE_ON, BRIGHT, UNDERLINE_OFF, NORMAL);

    // FIX! could we use Serial.println and avoid the \r\n ?
    Serial.print(F("X: eXit configuration and reboot\r\n"));
    Serial.print(F("/: reboot to bootloader mode to drag/drop new .uf2\r\n"));
    Serial.print(F("C: change Callsign (6 char max)\r\n"));
    Serial.print(F("U: change U4b channel # (0-599)\r\n"));
    Serial.print(F("A: change band (10,12,15,17,20 default 20)\r\n"));
    Serial.print(F("V: verbosity (0 for no messages, 9 for all) \r\n"));
    Serial.print(F("T: TELEN config\r\n"));
    Serial.print(F("K: clock speed  (default: 133)\r\n"));
    Serial.print(F("D: DEVMODE to enable messaging (default: 0)\r\n"));
    Serial.print(F("R: si5351 ppb correction (-3000 to 3000) (default: 0)\r\n"));
    Serial.print(F("R: go_when_ready (callsign tx starts with any modulo 2 starting minute (default: 0)\r\n"));

    Serial.println(F("show_values() END"));
}

