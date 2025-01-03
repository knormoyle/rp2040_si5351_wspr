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

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog
#include "si5351_functions.h"
#include "print_functions.h"
#include "led_functions.h"
#include "gps_functions.h"

#define CW_CLK_NUM WSPR_TX_CLK_0_NUM // we get the full differential clk0/clk1 with this
#define CW_DASH_LEN 3  // length of dash (in dots)
#define CW_LETTER_SPACE_LEN 4
#define CW_WORD_SPACE_LEN 7
// number of morse chars on Serial after which we newline
#define SERIAL_LINE_WIDTH 80

//********************************
extern uint32_t XMIT_FREQUENCY;
extern bool VERBY[10];

extern char t_altitude[7];
extern char t_grid6[7];
extern char t_callsign[7];
extern char t_grid6[7];

extern char _Band[3];  // string with 10, 12, 15, 17, 20 legal. null at end
extern char _tx_high[2];  // 0 is 4mA si5351. 1 is 8mA si5351

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
// state of the txcvr
enum tx_state_e {
    E_STATE_RX,
    E_STATE_TX
};

// state of the key line
enum key_state_e {
    E_KEY_UP,
    E_KEY_DOWN
};


// https://morsecode.world/international/timing.html
uint32_t dot_length_ms = 1000 * 60/(50 * 12); // 12 wpm default
uint32_t cw_ch_counter = 0;

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
uint32_t cw_keyer_speed(uint8_t wpm) {
    if (wpm < 5 || wpm > 20) {
        V1_print(F(EOL "cw_keyer_speed illegal wpm, using 12" EOL));
        wpm = 12;
    }
    // scale to wpm (10 wpm == 60mS dot length)
    // https://morsecode.world/international/timing.html
    // seconds per dit = 60/(50 * wpm)
    // millisecs per dit = 1000 * 60/(50 * wpm)
    uint32_t dot_length_ms = 1000 * 60/(50 * wpm);
    uint8_t target_wpm = 1000 * 60/(50 * dot_length_ms);
    V1_printf(EOL "cw_keyer_speed target_wpm %u dot_length_ms %lu" EOL,
        target_wpm, dot_length_ms);
    return dot_length_ms;
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
            vfo_turn_on_clk_out(CW_CLK_NUM, false);  // don't print
        }
        break;

        case E_KEY_UP: if (key_state != E_KEY_UP)  {
            // this has to be fast
            // V1_print(F("cw_key_state E_KEY_UP"));
            key_state = E_KEY_UP;
            vfo_turn_off_clk_out(CW_CLK_NUM, false);  // don't print
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
            sleep_ms(2000);
            // vfo_turn_on() does turn on the clk outputs!
            // does pll reset too
            uint32_t hf_freq = get_cw_freq(_Band);
            XMIT_FREQUENCY = hf_freq;
            // uses XMIT_FREQUENCY
            vfo_turn_on(CW_CLK_NUM);
            Watchdog.reset();
            V1_flush();
            sleep_ms(3000);
            V1_print(F("vfo_turn_off_clk_out()"));
            V1_flush();
            vfo_turn_off_clk_out(CW_CLK_NUM, true);  // print
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
    busy_wait_ms(dot_length_ms);  // key down for one dot period
    cw_key_state(E_KEY_UP);
    busy_wait_ms(dot_length_ms);  // wait for one dot period (space)
    // how much delay does this cause?
    V1_print(".");
    cw_ch_counter++;
}

//**************************************
void cw_send_dash() {
    cw_key_state(E_KEY_DOWN);
    // if(cw_ch_counter % SERIAL_LINE_WIDTH == 0) V1_println();
    busy_wait_ms(dot_length_ms * CW_DASH_LEN);  // key down for CW_DASH_LEN dot periods
    cw_key_state(E_KEY_UP);
    busy_wait_ms(dot_length_ms);  // wait for one dot period (space)
    // how much delay does this cause?
    V1_print("-");
    cw_ch_counter++;
}

//**************************************
void cw_send_letter_space() {
    busy_wait_ms(dot_length_ms * CW_LETTER_SPACE_LEN);  // wait for 4 dot periods
    // how much delay does this cause?
    V1_print(" ");
}

//**************************************
void cw_send_word_space() {
    busy_wait_ms(dot_length_ms * CW_WORD_SPACE_LEN);  // wait for 7 dot periods
    // how much delay does this cause?
    V1_print(" ");
    V1_print("  ");
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
    int i, j, n;
    int m_len = strlen(m);
    if (m_len == 0 || m_len > MORSE_CHARS_MAX) {
        V1_printf("ERROR: bad strlen(m) %d for morse message" EOL, m_len);
        return;
    }

    V1_printf("%s" EOL, m);
    for (i=0; i < m_len; i++) {
        Watchdog.reset();
        if(m[i] == ' ') {
            cw_send_word_space();
        } else {
            n = morse_lookup(m[i]);
            if (n == -1) {
                // char not found, ignore it (but report it on Serial)
                V1_print("Char in message not found in MorseTable <");
                V1_print(m[i]);
                V1_println(">");
            } else {
                // char found, so send it as dots and dashes
                for(j=1; j<7; j++) cw_send_morse_char(MorseCode[n].ch[j]);
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
    // uint8_t wpm = 12;
    uint8_t wpm = 5;
    dot_length_ms = cw_keyer_speed(wpm);
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
        cw_send(morse_msg);
        uint64_t actual = millis() - start_millis;
        uint64_t expected = 3 * 50 * dot_length_ms;
        V1_printf("duration (millis) actual %" PRIu64 " expected %" PRIu64 EOL, 
            actual, expected);
    } else {
        cw_send(morse_msg);
    }

    // time for PARIS should be 50 * dot_length_ms
    // turns gps back on!
    cw_tx_state(E_STATE_RX);
    cw_restore_drive_strength();

    Watchdog.reset();
    V1_print(F("cw_send_message END" EOL));
}
