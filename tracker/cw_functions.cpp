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

#include <stdint.h>

// NOTE! the clock disable mode doesn't work for keying the clk0/1 (ms5351m) ??
const bool USE_CLK_POWERDOWN_MODE = true;

// always integers
const uint8_t DEFAULT_WPM = 12;
uint8_t target_wpm;
uint32_t dit_ms;

const bool DO_WPM_CORRECTION = false;
// this was tested with default 18Mhz sys clk
// actual/target wpm for an unadjusted PARIS 3x test
// shuldn't need correction!
const float DIT_MS_MULTIPLIER = 1.0;

enum test_msg  {
    SEND_PARIS3X = 0,
    SEND_DIT,
    SEND_DIT5,
    SEND_DASH,
    SEND_DASH5,
    SEND_DITDASH,
    SEND_DASHDIT,
    SEND_DASHDIT4,
    SEND_LONGCHAR,
    SEND_BALLOON,
    // SEND_TEST_CYCLE must be last enum
    SEND_TEST_CYCLE
};
// const uint8_t test_to_do = SEND_BALLOON;
const uint8_t test_to_do = SEND_TEST_CYCLE;

//********************************
// we the full differential clk0/clk1 set with this
#define CW_CLK_NUM WSPR_TX_CLK_0_NUM

// https://morsecode.world/international/timing.html
#define CW_DAH_LEN 3
#define CW_LETTER_SPACE_LEN 3
#define CW_WORD_SPACE_LEN 7

// number of morse chars on Serial after which we newline
#define SERIAL_LINE_WIDTH 80

#define KEYING_DELAY sleep_ms
// #define KEYING_DELAY busy_wait_ms

//******************************************
// from hans
// https://groups.io/g/picoballoon/message/19324
// Yet ANOTHER way to do this (CW) is possible, and works without the PLL Reset.
// Register 3 Output Enable Control contains a set of bits, when set to 1,
// it disables the corresponding output.

// So I have discovered that I can key CW by setting these bits to
// 1 for key-up (no output) or 0 for key-down (has output).
// When the output is disabled, it still preserves the proper phase relationship.
// So you can key the CW transmitter WITHOUT those few milliseconds of mess 
// at key-down ..a millisecond or two of the outputs not being both on and
// and not with the correct phase relationship,
// then 1.46 milliseconds of delay while the PLL Reset does its stuff
// PLL reset duration may vary, based on other s5351a config. 

// There are TWO ways to key a clock output on/off:
// Use the "power down" bit in the CLK0 Control and/or CLK1 Control registers
// (16 and 17 respectively).
// This does NOT preserve the inverted (180-degree) phase relationship
// when you re-enable by setting the power down bit to 0;
// hence a PLL Reset is required,

// If you want differential antenna drive (U4B High Power mode).
// Use the Register 3 "Output Enable Control" documented in AN619 at page 17
// https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
// This DOES preserve the phase relationships so no PLL reset is required
// when using differential drive.

// OK, official.
// You DO need the PLL Reset, when using the PDN powerdown bit to turn on clk,
// to get the correct 180-degree phase difference,
// even when just using the inverted bit for phase difference.

// Tested using a little U4B BASIC program, and two-channel oscilloscope probes
// attached to each of Clk0 and Clk1 outputs.
// HP set to 1 to cause the inverted drive (high power) mode:
// withPLLReset.png: the standard U4B protection firmware version.
// There is a PLL reset (register 177) after switching on the clock outputs
// via their respective CLK0 Control and CLK1 Control regs: 16 and 17.
// Clk0 and Clk1 have the proper 180-degree phase relationship.

// After commenting out that line of code with the PLL reset,
// I see a random phase relationship between Clk0 and Clk1 on each keydown.
//******************************************

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog
#include "si5351_functions.h"
#include "print_functions.h"
#include "debug_functions.h"
#include "led_functions.h"
#include "gps_functions.h"
#include "cw_functions.h"

//********************************
extern uint32_t XMIT_FREQUENCY;
extern bool VERBY[10];
extern uint32_t PLL_SYS_MHZ;

extern char t_altitude[7];
extern char t_grid6[7];
extern char t_callsign[7];
extern char t_grid6[7];

extern char _Band[3];  // string with 10, 12, 15, 17, 20 legal. null at end
extern char _tx_high[2];  // 0 is 4mA si5351. 1 is 8mA si5351

//********************************
// stay in first 91 khz of each cw band for rbn?
// alright if the generated frequency is not exact.
// use whatever denom we have setup for wspr
uint32_t get_cw_freq(char *band) {
    uint32_t cw_freq;
    switch (atoi(band)) {
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
    {',', '-', '-', '.', '.', '-', '-'}
};
// orig
// {'(', '-', '.', '-', '.', '-', 0}, // unclear if correct
// {')', '.', '-', '.', '-', '.', 0} // unclear if correct
// disagrees
// https://blendertimer.com/web-tools/morse-code-translator



#define MORSE_CHARS_MAX 80
char cw_msg[MORSE_CHARS_MAX + 1];

//**************************************
int morse_lookup(char c) {
// returns the index of parameter 'c' in MorseCode array, or -1 if not found
    for (uint32_t i = 0; i < sizeof(MorseCode); i++) {
        if (c == MorseCode[i].ch[0]) return i;
    }
    return -1;
}

//**************************************
uint32_t char_cnt = 0;
uint32_t dit_cnt = 0;
// count the number of dit delays you need before the next dit or dah,
// and do a bulk delay then
uint8_t dit_delays_needed = 0;

void cw_keyer_speed(uint8_t wpm) {
    if (wpm < 5 || wpm > 20) {
        V1_print(F(EOL "cw_keyer_speed illegal wpm, using 12" EOL));
        wpm = 12;
    }
    // https://morsecode.world/international/timing.html
    // seconds per dit = 60/(50 * wpm)
    // millisecs per dit = 1000 * 60/(50 * wpm)

    // globals
    dit_ms = 1000 * 60/(50 * wpm);
    // always integers
    target_wpm = 1000 * 60/(50 * dit_ms);

    // set to false if creating a DIT_MS_MULTIPLIER with PARIS 3x test
    uint32_t dit_ms_uncorrected = dit_ms;
    if (DO_WPM_CORRECTION) {
        // now shorten the dit_ms based on real behavior with the code?
        // Reason for inaccuracy is unknown
        // FIX! this adjustment was done at 18Mhz
        // could be different at different PLL_SYS_MHZ
        if (PLL_SYS_MHZ != 18) {
            V1_printf("ERROR: wpm adjust from PLL_SYS_MHZ 18 Mhz, current PLL_SYS_MHZ %lu" EOL,
                PLL_SYS_MHZ);
        }
        dit_ms = (uint32_t)(DIT_MS_MULTIPLIER * (float)dit_ms);
    }
    V1_printf(EOL "cw_keyer_speed target_wpm %u dit_ms %lu dit_ms_uncorrected %lu" EOL,
        target_wpm, dit_ms, dit_ms_uncorrected);
}

//**************************************
key_state_e key_state = E_KEY_UP;
void cw_key_state(key_state_e k) {
    // 'cw_key_state' {E_KEY_DOWN, E_KEY_UP}
    // hm. not checking current state?
    // can't optimize with no-change to current state
    // affects timing?
    switch (k) {
        case E_KEY_DOWN:
            // this has to be fast
            if (USE_CLK_POWERDOWN_MODE) {
                si5351a_power_up_clk01();  // this does a pllb reset too
            } else {
                vfo_turn_on_clk_out(CW_CLK_NUM, false);  // don't print
                // tried other things
                // i2cWrite(3, 0xfc);  // just enable clk0/1
                // si5351a_reset_PLLB(false);  // don't print
            }
            // V1_print(F("cw_key_state E_KEY_DOWN"));
            key_state = E_KEY_DOWN;
            break;

        case E_KEY_UP:
            // this has to be fast
            if (USE_CLK_POWERDOWN_MODE) {
                si5351a_power_down_clk01();  // this does a pllb reset too
            } else {
                vfo_turn_off_clk_out(CW_CLK_NUM, false);  // don't print
                // tried other things..ms5351m just won't turn off with clock enable
                // i2cWrite(3, 0xff);  // disable clk0-7
                // si5351a_reset_PLLB(false);  // don't print
            }
            // V1_print(F("cw_key_state E_KEY_UP"));
            key_state = E_KEY_UP;
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
            // UPDATE: tracker.ino should turn gps on ??
            // don't do it here..to avoid while testing
            // GpsON(false); // no full cold reset
            tx_state = E_STATE_RX;
            V1_print(F(EOL ">Rx" EOL));
            V1_flush();
        }
        break;

        // should we check if already in the state?
        case E_STATE_TX: if (tx_state != E_STATE_TX) {
            V1_print(F(EOL "cw_tx_state E_STATE_TX" EOL));
            // make sure it GPS is off.
            GpsOFF(true);  // keep TinyGPS state
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
            char debugMsg[] = "";
            realPrintFlush(debugMsg, true);

            V1_print(EOL "Should not hear sdr CW tone now for 5 secs..RF should be off" EOL);
            realPrintFlush(debugMsg, true);
            if (USE_CLK_POWERDOWN_MODE) {
                si5351a_power_down_clk01();  // this does a pllb reset too
            } else {
                vfo_turn_off_clk_out(CW_CLK_NUM, false);  // don't print
                // tried other things
                // i2cWrite(9, 0xff); // no oeb pin enable control
                // si5351a_reset_PLLB(false);  // don't print
                // i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff);  // disable clk0-7
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
void drain_dit_delays() {
    switch (dit_delays_needed) {
        case 0: return;
        case 1:
        case CW_LETTER_SPACE_LEN:
        case CW_WORD_SPACE_LEN: break;
        default:
            V1_printf("ERROR: illegal drain_dit_delays %u" EOL, dit_delays_needed);
    }
    KEYING_DELAY(dit_delays_needed * dit_ms);
    dit_cnt += dit_delays_needed;
    dit_delays_needed = 0;
}

//**************************************
void cw_send_dit() {
    drain_dit_delays();
    // if(char_cnt % SERIAL_LINE_WIDTH == 0) V1_println();
    cw_key_state(E_KEY_DOWN);
    // key down for one dot period
    KEYING_DELAY(dit_ms);
    dit_cnt += 1;
    cw_key_state(E_KEY_UP);
    dit_delays_needed = 1;
    // how much delay does this cause?
    V1_print(".");
    char_cnt++;
}

//**************************************
void cw_send_dah() {
    drain_dit_delays();
    cw_key_state(E_KEY_DOWN);
    // if(char_cnt % SERIAL_LINE_WIDTH == 0) V1_println();
    // key down for CW_DAH_LEN dit periods
    KEYING_DELAY(dit_ms * CW_DAH_LEN);
    dit_cnt += CW_DAH_LEN;
    cw_key_state(E_KEY_UP);
    dit_delays_needed = 1;
    // how much delay does this cause?
    V1_print("-");
    char_cnt++;
}

//**************************************
void cw_send_ditdah(char c) {
    // ignore anything else, including 0s
    if (c != '.' && c != '-') return;
    // 'c' is a '.' or '-' char, so send it
    if (c == '.') cw_send_dit();
    else cw_send_dah();
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

    dit_cnt = 0;
    dit_delays_needed = 0;
    V1_printf("%s" EOL, m);
    for (int i = 0; i < m_len; i++) {
        Watchdog.reset();
        if (m[i] == ' ') {
            // how much delay does this cause?
            V1_print("  ");
            char_cnt += 2;
            dit_delays_needed = CW_WORD_SPACE_LEN;
        } else {
            int n = morse_lookup(m[i]);
            if (n == -1) {
                // char not found, ignore it (but report it on Serial)
                V1_printf("ERROR: i %d char %c not found in MorseTable" EOL, i, m[i]);
            } else {
                // char found, so send it as dits and dahs
                for (int j = 1; j < 7; j++) {
                    // early out for 0 terminator. guaranteed doesn't start with 0.
                    if (MorseCode[n].ch[j] == 0) break;
                    cw_send_ditdah(MorseCode[n].ch[j]);
                }
                V1_print(" ");
                char_cnt += 1;
                dit_delays_needed = CW_LETTER_SPACE_LEN;
            }
        }
    }
    // for the end of the message!
    drain_dit_delays();

    // EOL after all the dots and dashes..
    V1_print(EOL);
    V1_print(F("cw_send END" EOL));
}

//**************************************
void cw_init() {
    // assume vfo_init() has been done. si5351_functions.cpp
    tx_state  = E_STATE_RX;
    key_state = E_KEY_UP;
    char_cnt = 0;
    dit_cnt = 0;
    dit_delays_needed = 0;

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
    static uint8_t test_cycle_cnt = 0;
    dit_cnt = 0;
    dit_delays_needed = 0;

    Watchdog.reset();
    cw_keyer_speed(DEFAULT_WPM);
    uint8_t test_done;
    // SEND_TEST_CYCLE should be last enum
    if (test_to_do == SEND_TEST_CYCLE) 
        test_done = test_cycle_cnt % SEND_TEST_CYCLE;
    else 
        test_done = test_to_do;

    // cw_msg can be up to MORSE_CHARS_MAX  (80)
    // has to be uppercase letters
    switch (test_done) {
        case SEND_PARIS3X:
            snprintf(cw_msg, sizeof(cw_msg), "PARIS PARIS PARIS ");
            break;
        case SEND_DIT:
            snprintf(cw_msg, sizeof(cw_msg),
                "E EE EEE EEEE EEEEE  EEEEEE   /E / EEEEEEEEEEEEEEEEEE /?.,");
            break;
        case SEND_DIT5:
            snprintf(cw_msg, sizeof(cw_msg),
                "5 55 555 5555 55555  555555   /5 / 555555555555555555 /?.,");
            break;
        case SEND_DASH:
            snprintf(cw_msg, sizeof(cw_msg),
                "T TT TTT TTTT TTTTT  TTTTTT   /T / TTTTTTTTTTTTTTTTTT /?.,");
            break;
        case SEND_DASH5:
            snprintf(cw_msg, sizeof(cw_msg),
                "0 00 000 0000 00000  000000   /0 / 000000000000000000 /?.,");
            break;
        case SEND_DITDASH:
            snprintf(cw_msg, sizeof(cw_msg),
                "A AA AAA AAAA AAAAA  AAAAAA   /A / AAAAAAAAAAAAAAAAAA /?.,");
            break;
        case SEND_DASHDIT:
            snprintf(cw_msg, sizeof(cw_msg),
                "N NN NNN NNNN NNNNN  NNNNNN   /N / NNNNNNNNNNNNNNNNNN /?.,");
            break;
        case SEND_DASHDIT4:
            snprintf(cw_msg, sizeof(cw_msg),
                "C CC CCC CCCC CCCCC  CCCCCC   /C / CCCCCCCCCCCCCCCCCC /?.,");
            break;
        case SEND_LONGCHAR:
            snprintf(cw_msg, sizeof(cw_msg),
                "? ?? ??? ???? ?????  ??????   /? / ?????????????????? /?.,");
            break;
        default:  // SEND_BALLOON
            // if we didn't get a gps snapForTelemetry() the t_* will be blank
            if (t_callsign[0] == 0 || t_grid6[0] == 0 || t_altitude[0] == 0) {
                V1_print(F("No gps snapForTelemetry() before test? t_* is blank" EOL));
                V1_print(F("Wait for a successful wspr tx before switching to 'Z' test" EOL));
                V1_print(F("snapForTelemetry() will have happened then" EOL));
            }
            snprintf(cw_msg, sizeof(cw_msg), "CQ CQ CQ DE %s %s BALLOON %s %s %s %s K",
                t_callsign, t_callsign, t_grid6, t_grid6, t_altitude, t_altitude);
    }

    // cw_msg can't be empty here

    // More power scotty! best chance of someone spotting!
    cw_high_drive_strength();
    cw_tx_state(E_STATE_TX);

    //******************************************
    // The neat thing about "PARIS " is that it's a nice even 50 units long. (with word space
    // It translates to ".--. .- .-. .. .../" so there are:
    // 10 dits: 10 units;
    // 4 dahs: 12 units;
    // 9 intra-character spaces: 9 units;
    // 4 inter-character spaces: 12 units;
    // 1 word space: 7 units.
    // A grand total of 50 units.
    // we send 'PARIS ' 3x above

    V1_print(F(EOL "Should start to hear CW now.. RF should be toggling" EOL));
    uint64_t start_millis = millis();
    cw_send(cw_msg);
    uint32_t actual_duration = millis() - start_millis;

    //******************************************
    uint32_t expected_duration;
    uint32_t actual_dit_ms;
    // actually counted the dits in the message sent: dit_cnt
    if (test_done == SEND_PARIS3X) {
        expected_duration = (50 * 3) * dit_ms;
        actual_dit_ms = actual_duration / (50 * 3);
        if (dit_cnt != 150)
            V1_printf("ERROR: dit_cnt %lu should be 150 for SEND_PARIS3X" EOL,
                dit_cnt);
    } else {
        expected_duration = dit_cnt * dit_ms;
        // dit_cnt can't ever be zero? (check for empty send message before)
        actual_dit_ms = actual_duration / dit_cnt;
    }
    float new_dit_ms_multiplier = (float) dit_ms / (float) actual_dit_ms;
    float actual_wpm = 1000.0 * 60.0/(50.0 * (float)actual_dit_ms);

    //******************************************
    V1_printf("test %d dit_cnt %lu actual_duration %lu expected_duration %lu millis" EOL,
        test_done, dit_cnt, actual_duration, expected_duration);
    V1_printf("test %d dit_cnt %lu dit_ms %lu actual_dit_ms %lu millis" EOL,
        test_done, dit_cnt, dit_ms, actual_dit_ms);

    // actual_dit_ms can't ever be zero
    V1_printf("test %d wpm actual %.3f expected %u" EOL, 
        test_done, actual_wpm, target_wpm);

    // shouldn't need correction!
    if (DO_WPM_CORRECTION) {
        V1_printf("test %d DIT_MS_MULTIPLIER should be: %.4f" EOL,
            test_done, new_dit_ms_multiplier * DIT_MS_MULTIPLIER);
    } else {
        // best calculate value is when we didn't do correction on this run
        // (and USE_PARIS3X?)
        V1_printf("test %d DIT_MS_MULTIPLIER should be: %.4f" EOL , 
            test_done, new_dit_ms_multiplier);
    }
    //******************************************

    // time for PARIS should be 50 * dit_ms
    // turns gps back on!
    cw_tx_state(E_STATE_RX);
    V1_print(EOL "Shouldn't hear any CW now..RF should be off" EOL);
    KEYING_DELAY(5000);

    cw_restore_drive_strength();
    Watchdog.reset();
    V1_print(F("cw_send_message END" EOL));
    test_cycle_cnt += 1;
}
