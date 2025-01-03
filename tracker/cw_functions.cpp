// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// forked from https://github.com/prt459/Nano_Beacon/tree/main
// Nano_Beacon 29 Nov 2021 by Paul VK3HN.

// Nano_Beacon -- a basic CW beacon
// See READ.ME at url above for further background and code mod help.
// https://vk3hn.wordpress.com/

// reverse beacon network 10M
// https://www.reversebeacon.net/main.php?zoom=33.57,-97.40,3.05&rows=100&spotters=1&max_age=30,days&bands=10&spotted_call=EA1KT&hide=distance_km

// EA1KT being spotted at 28022.0

// https://groups.io/g/picoballoon/message/19326

// Yet ANOTHER way to do this (CW) is possible, and works without the PLL Reset. 
// Register 3 Output Enable Control contains a set of bits, when set to 1, 
// it disables the corresponding output. 

// So I have discovered that I can key CW by setting these bits to 1 for key-up (no output) 
// or 0 for key-down (has output). 
// When the output is disabled, it still preserves the proper phase relationship. 
// So you can key the CW transmitter WITHOUT those few milliseconds of mess at key-down where 
// you have a millisecond or two of the outputs not being both on and not being both 
// on the correct phase relationship, 
// then 1.46 milliseconds of delay while the PLL Reset does its stuff 
// (note: who knows, if that PLL reset duration may vary, 
// based on what other '5351 configuration parameters). 

// there are TWO ways to key a clock output on/off:
// Use the "power down" bit in the CLK0 Control and/or CLK1 Control registers (16 and 17 respectively). 
// This does NOT preserve the inverted (180-degree) phase relationship 
// when you re-enable by setting the power down bit to 0; 
// hence a PLL Reset is required, 
// If you want differential antenna drive (U4B High Power mode). 
// Use the Register 3 "Output Enable Control" documented in AN619 at page 17 
// see https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf. 
// This DOES preserve the phase relationships so no PLL reset is required 
// when using differential drive. 

// OK, official. 
// You DO need the PLL Reset, on switching on an output, 
// to get the correct 180-degree phase difference, 
// even when just using the inverted bit. 
// Tested using a little U4B BASIC program, and two-channel oscilloscope probes 
// attached to each of Clk0 and Clk1 outputs. 
// HP set to 1 to cause the inverted drive (high power) mode:
// withPLLReset.png: the standard U4B protection firmware version. 
// There is a PLL reset (register 177) after switching on the clock outputs 
// via their respective CLK0 Control and CLK1 Control registers (registers 16 and 17). 
// Clk0 and Clk1 have the proper 180-degree phase relationship. 

// WithoutPLLReset.png: After commenting out that line of code with the PLL reset, 
// I see a random phase relationship between Clk0 and Clk1 on each keydown. 
// Snapshot here demonstrates one such example. 

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog
#include "si5351_functions.h"
#include "print_functions.h"
#include "led_functions.h"
#include "gps_functions.h"
#include "cw_functions.h"

//********************************
// we the full differential clk0/clk1 set with this
#define CW_CLK_NUM WSPR_TX_CLK_0_NUM 

#define CW_DASH_LEN 3  // length of dash (in dots)
#define CW_LETTER_SPACE_LEN 4
#define CW_WORD_SPACE_LEN 7
// number of morse chars on Serial after which we newline
#define SERIAL_LINE_WIDTH 80

// #define KEYING_DELAY sleep_ms
#define KEYING_DELAY busy_wait_ms

//********************************
extern uint32_t XMIT_FREQUENCY;
extern bool VERBY[10];

extern char t_altitude[7];
extern char t_grid6[7];
extern char t_callsign[7];
extern char t_grid6[7];

extern char _Band[3];  // string with 10, 12, 15, 17, 20 legal. null at end
extern char _tx_high[2];  // 0 is 4mA si5351. 1 is 8mA si5351


// FIX! the clock disable mode doesn't work for keying the clk0/1 (ms5351m) ??
bool USE_CLK_POWERDOWN_MODE = true;

//********************************
// stay in first 91 khz of each cw band for rbn?
// alright if the frequency is not perfect. use whatever denom we have setup for wspr
uint32_t get_cw_freq(char *band) {
    uint32_t cw_freq;
    switch(atoi(band)) {
        case 10: cw_freq = 28050100; break;
        case 12: cw_freq = 24935100; break;
        case 15: cw_freq = 21050100; break;
        case 17: cw_freq = 18150100; break;
        case 20: cw_freq = 14050100; break;
        default: cw_freq = 28050100; break;
    }
    V1_printf("get_cw_freq band %s will use cw_freq %lu" EOL, band, cw_freq);
    return cw_freq;
}

//********************************
// morse reference table
struct morse_char_t {
    char ch[7];
};

morse_char_t MorseCode[] = {
    {'A', '.', '-',  0,   0,   0,   0},
    {'B', '-', '.', '.', '.',  0,   0},
    {'C', '-', '.', '-', '.',  0,   0},
    {'D', '-', '.', '.',  0,   0,   0},
    {'E', '.',  0,   0,   0,   0,   0},
    {'F', '.', '.', '-', '.',  0,   0},
    {'G', '-', '-', '.',  0,   0,   0},
    {'H', '.', '.', '.', '.',  0,   0},
    {'I', '.', '.',  0,   0,   0,   0},
    {'J', '.', '-', '-', '-',  0,   0},
    {'K', '-', '.', '-',  0,   0,   0},
    {'L', '.', '-', '.', '.',  0,   0},
    {'M', '-', '-',  0,   0,   0,   0},
    {'N', '-', '.',  0,   0,   0,   0},
    {'O', '-', '-', '-',  0,   0,   0},
    {'P', '.', '-', '-', '.',  0,   0},
    {'Q', '-', '-', '.', '-',  0,   0},
    {'R', '.', '-', '.',  0,   0,   0},
    {'S', '.', '.', '.',  0,   0,   0},
    {'T', '-',  0,   0,   0,   0,   0},
    {'U', '.', '.', '-',  0,   0,   0},
    {'V', '.', '.', '.', '-',  0,   0},
    {'W', '.', '-', '-',  0,   0,   0},
    {'X', '-', '.', '.', '-',  0,   0},
    {'Y', '-', '.', '-', '-',  0,   0},
    {'Z', '-', '-', '.', '.',  0,   0},
    {'0', '-', '-', '-', '-', '-',  0},
    {'1', '.', '-', '-', '-', '-',  0},
    {'2', '.', '.', '-', '-', '-',  0},
    {'3', '.', '.', '.', '-', '-',  0},
    {'4', '.', '.', '.', '.', '-',  0},
    {'5', '.', '.', '.', '.', '.',  0},
    {'6', '-', '.', '.', '.', '.',  0},
    {'7', '-', '-', '.', '.', '.',  0},
    {'8', '-', '-', '-', '.', '.',  0},
    {'9', '-', '-', '-', '-', '.',  0},
    {'/', '-', '.', '.', '-', '.',  0},
    {'?', '.', '.', '-', '-', '.', '.'},
    {'.', '.', '-', '.', '-', '.', '-'},
    {',', '-', '-', '.', '.', '-', '-'},
    {'(', '-', '.', '-', '.', '-', 0},
    {')', '.', '-', '.', '-', '.', 0}
};

#define MORSE_CHARS_MAX 80
char morse_msg[MORSE_CHARS_MAX + 1];

//**************************************
int morse_lookup(char c) {
// returns the index of parameter 'c' in MorseCode array, or -1 if not found
    for(uint32_t i = 0; i < sizeof(MorseCode); i++) {
        if (c == MorseCode[i].ch[0]) return i;
    }
    return -1;
}

//**************************************
uint8_t target_wpm = 12; // 12 wpm default
uint32_t dot_ms = 1000 * 60/(50 * 12); // 12 wpm default
// https://morsecode.world/international/timing.html
uint32_t cw_ch_counter = 0;

void cw_keyer_speed(uint8_t wpm) {
    if (wpm < 5 || wpm > 20) {
        V1_print(F(EOL "cw_keyer_speed illegal wpm, using 12" EOL));
        wpm = 12;
    }
    // scale to wpm (10 wpm == 60mS dot length)
    // https://morsecode.world/international/timing.html
    // seconds per dit = 60/(50 * wpm)
    // millisecs per dit = 1000 * 60/(50 * wpm)

    // globals
    dot_ms = 1000 * 60/(50 * wpm);
    target_wpm = 1000 * 60/(50 * dot_ms);
    V1_printf(EOL "cw_keyer_speed target_wpm %u dot_ms %lu" EOL, target_wpm, dot_ms);
}

//**************************************
key_state_e key_state = E_KEY_UP;
void cw_key_state(key_state_e k) {
    // 'cw_key_state' {E_KEY_DOWN, E_KEY_UP}
    // hm. not checking current state?
    switch (k) {
        case E_KEY_DOWN: if (key_state != E_KEY_DOWN)  {
            // this has to be fast
            // V1_print(F("cw_key_state E_KEY_DOWN"));
            key_state = E_KEY_DOWN;
            if (USE_CLK_POWERDOWN_MODE) {
                si5351a_power_up_clk01(); // this does a pllb reset too
            } else {
                i2cWrite(3, 0xfc); // just enable clk0/1
                // vfo_turn_on_clk_out(CW_CLK_NUM, false);  // don't print
                // si5351a_reset_PLLB(false);  // don't print
            }
        }
        break;

        case E_KEY_UP: if (key_state != E_KEY_UP)  {
            // this has to be fast
            // V1_print(F("cw_key_state E_KEY_UP"));
            key_state = E_KEY_UP;
            if (USE_CLK_POWERDOWN_MODE) {
                si5351a_power_down_clk01(); // this does a pllb reset too
            } else {
                i2cWrite(3, 0xff); // disable clk0-7
                // vfo_turn_off_clk_out(CW_CLK_NUM, false);  // don't print
                // si5351a_reset_PLLB(false);  // don't print
            }
        }
        break;
    }
}

//**************************************
tx_state_e tx_state = E_STATE_RX;
void cw_tx_state(tx_state_e s) {
    // if necessary, activate the receiver or the transmitter
    // changes global 'cw_state' {E_STATE_RX, E_STATE_TX}
    switch (s) {
        case E_STATE_RX: if (tx_state != E_STATE_RX) {
            V1_print(F(EOL "cw_tx_state E_STATE_RX" EOL));
            vfo_turn_off();
            // turn gps back on!
            // cw should be at the end of wspr (during the extended telemetry time slot)
            GpsON(false); // no full cold reset
            tx_state = E_STATE_RX;
            V1_print(F(EOL ">Rx" EOL));
            V1_flush();
        }
        break;

        // should we check if already in the state?
        case E_STATE_TX: if (tx_state != E_STATE_TX) {
            V1_print(F(EOL "cw_tx_state E_STATE_TX" EOL));
            GpsOFF(true); // keep TinyGPS state
            updateStatusLED();
            // start with the vfo off
            vfo_turn_off();
            KEYING_DELAY(2000);

            uint32_t hf_freq = get_cw_freq(_Band);
            XMIT_FREQUENCY = hf_freq;
            // uses XMIT_FREQUENCY

            // vfo_turn_on() does turn on the clk outputs!
            // does pll reset too
            vfo_turn_on(CW_CLK_NUM);
            V1_print(EOL "Should have been hearing sdr CW tone..RF should be on" EOL);
            V1_flush();

            V1_print(EOL "Should not hear sdr CW tone now for 5 secs..RF should be off" EOL);
            V1_flush();
            if (USE_CLK_POWERDOWN_MODE) {
                si5351a_power_down_clk01(); // this does a pllb reset too
            } else {
                // vfo_turn_off_clk_out(CW_CLK_NUM, false);  // don't print
                // i2cWrite(9, 0xff); // no oeb pin enable control
                // si5351a_reset_PLLB(false);  // don't print
                i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff); // disable clk0-7
            }

            Watchdog.reset();
            KEYING_DELAY(5000);

            tx_state = E_STATE_TX;
            V1_print(F(EOL ">Tx" EOL));
            V1_flush();
        }
        break;
    }
}

//**************************************

void cw_send_dot() {
    // if(cw_ch_counter % SERIAL_LINE_WIDTH == 0) V1_println();
    cw_key_state(E_KEY_DOWN);
    KEYING_DELAY(dot_ms);  // key down for one dot period
    cw_key_state(E_KEY_UP);
    KEYING_DELAY(dot_ms);  // wait for one dot period (space)
    // how much delay does this cause?
    // V1_print(".");
    cw_ch_counter++;
}

//**************************************
void cw_send_dash() {
    cw_key_state(E_KEY_DOWN);
    // if(cw_ch_counter % SERIAL_LINE_WIDTH == 0) V1_println();
    KEYING_DELAY(dot_ms * CW_DASH_LEN);  // key down for CW_DASH_LEN dot periods
    cw_key_state(E_KEY_UP);
    KEYING_DELAY(dot_ms);  // wait for one dot period (space)
    // how much delay does this cause?
    // V1_print("-");
    cw_ch_counter++;
}

//**************************************
void cw_send_letter_space() {
    KEYING_DELAY(dot_ms * CW_LETTER_SPACE_LEN);  // wait for 4 dot periods
    // how much delay does this cause?
    // V1_print(" ");
}

//**************************************
void cw_send_word_space() {
    KEYING_DELAY(dot_ms * CW_WORD_SPACE_LEN);  // wait for 7 dot periods
    // how much delay does this cause?
    // V1_print("  ");
}

//**************************************
void cw_send_morse_char(char c) {
    // 'c' is a '.' or '-' char, so send it
    if (c == '.') cw_send_dot();
    else if (c == '-') cw_send_dash();
    // ignore anything else, including 0s
}

//**************************************
void cw_send(char *m) {
    V1_print(F("cw_send START" EOL));
    Watchdog.reset();
    // sends the message in string 'm' as CW, with inter letter and word spacing
    // s is the speed to play at; if s == 0, use the current speed
    int m_len = strlen(m);
    if (m_len == 0 || m_len > MORSE_CHARS_MAX) {
        V1_printf("ERROR: bad strlen(m) %d for morse message" EOL, m_len);
        return;
    }

    V1_printf("%s" EOL, m);
    for (int i=0; i < m_len; i++) {
        Watchdog.reset();
        if(m[i] == ' ') {
            cw_send_word_space();
        } else {
            int n = morse_lookup(m[i]);
            if (n == -1) {
                // char not found, ignore it (but report it on Serial)
                V1_printf("ERROR: i %d char %c in message not found in MorseTable" EOL, i, m[i]);
            } else {
                // char found, so send it as dots and dashes
                for(int j=1; j<7; j++) cw_send_morse_char(MorseCode[n].ch[j]);
                cw_send_letter_space();  // send an inter-letter space
            }
        }
    }
    // EOL after all the dots and dashes..
    V1_print(EOL);
    V1_print(F("cw_send END" EOL));
}

//**************************************
void cw_init() {
    // assume vfo_init() has been done. si5351_functions.cpp
    tx_state  = E_STATE_RX;
    key_state = E_KEY_UP;
    cw_ch_counter = 0;

    /*
    // dump out the MorseCode table for diagnostic
    for(int i=0; i<40; i++) {
        V1_print(MorseCode[i].ch[0]);
        V1_print(' ');
        for(int j=1; j<7; j++) {
            V1_print(MorseCode[i].ch[j]);
        }
        V1_println();
    }
    */
}
//**************************************
void cw_high_drive_strength() {
    V1_print("cw_high_drive_strength START" EOL);
    // change the si5351a drive strength back to 8ma
    // it modifies s_vfo_drive_strength (in si5351a_functions.cpp)
    // which isn't used until a frequency is set. the s*ms_div_prev are cleared
    // so it will trigger a pll reset when you set a frequency
    // void vfo_set_drive_strength(uint8_t clk_num, uint8_t strength);
    vfo_set_drive_strength(CW_CLK_NUM, SI5351A_CLK01_IDRV_8MA);
    V1_print("cw_high_drive_strength END" EOL);
}

void cw_restore_drive_strength() {
    // change the si5351a drive strength back to configured value
    // I suppose we always set it on the next wspr, so okay if we do nothing here?
}

//**************************************
void cw_send_message() {
    V1_print(F("cw_send_message START" EOL));
    Watchdog.reset();
    // uint8_t wpm = 16;
    // this was getting me 9 wpm?
    uint8_t wpm = 12;
    // uint8_t wpm = 5;
    cw_keyer_speed(wpm);
    // up to 80 chars
    // has to be uppercase letters

    bool MEASURE_WPM = true;
    if (MEASURE_WPM) {
        snprintf(morse_msg, sizeof(morse_msg), "PARIS PARIS PARIS ");
    } else {
        snprintf(morse_msg, sizeof(morse_msg), "CQ CQ DE %s %s BALLOON %s %s %s %s K",
            t_callsign, t_callsign, t_grid6, t_grid6, t_altitude, t_altitude);
    }

    // More power scotty! best chance of someone spotting!
    cw_high_drive_strength();
    cw_tx_state(E_STATE_TX);
    // The neat thing about "PARIS " is that it's a nice even 50 units long. (with word space
    // It translates to ".--. .- .-. .. .../" so there are:
    // 10 dits: 10 units;
    // 4 dahs: 12 units;
    // 9 intra-character spaces: 9 units;
    // 4 inter-character spaces: 12 units;
    // 1 word space: 7 units.
    // A grand total of 50 units.

    if (MEASURE_WPM) {
        uint64_t start_millis = millis();
        // we send 'PARIS ' 3x above
        V1_print(EOL "Should start to hear CW now..RF should be toggling" EOL);
        cw_send(morse_msg);
        uint64_t actual = millis() - start_millis;
        uint64_t expected = 3 * 50 * dot_ms;
        V1_printf("duration (millis) actual %" PRIu64 " expected %" PRIu64 EOL, actual, expected);
        uint32_t actual_dot_ms = (actual / (3 * 50));
        uint8_t actual_wpm = 1000 * 60/(50 * actual_dot_ms);
        V1_printf("wpm actual %u expected %u" EOL, actual_wpm, target_wpm);
    
    } else {
        cw_send(morse_msg);
    }

    // time for PARIS should be 50 * dot_ms
    // turns gps back on!
    cw_tx_state(E_STATE_RX);
    V1_print(EOL "Shouldn't hear any CW now..RF should be off" EOL);
    KEYING_DELAY(5000);

    cw_restore_drive_strength();
    Watchdog.reset();
    V1_print(F("cw_send_message END" EOL));
}
