// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include "u4b_functions.h"

//********************************
// the key output that is used to pwm rf ..162 symbols that are 4-FSK
extern uint8_t tx_buffer[255];  // is this bigger than WSPR_SYMBOL_COUNT?
extern uint32_t XMIT_FREQUENCY;

//********************************
// t_* (was: telemetry_buff) is a snapshot of consistent-in-time data used to
// generated encoded telemetry, spread out over a later time. (like 1 to 3 more tx)

// multiple wspr transmissions then can be combined to recreate the single
// consistent snapshot grid4 plus subsquare for grid6,
// in two different transmissions, is notable example.

// always positive int. clamped to 0, 360 ?
extern char t_course[4];  // 3 bytes + null term (like all here)

// always positive int. clamped to 0,999
extern char t_speed[4];   // 3 bytes

// always positive int. clamped to 0, 999999
extern char t_altitude[7];  // 6 bytes
// two letters, two digits, two letters
// base 18, base 18, base 10, base 10, base 24, base 24
// [A-R][A-R][0-9][0-9][A-X][A-X]
// I guess clamp to AA00AA if illegal? (easy to find errors?)

// always positive int. clamped to 0, 999
extern char t_tx_count_0[4];  // 3 bytes

//  positive or negative float. clamped to -999.9 to 999.9
extern char t_temp[7];  // 6 bytes

// examples
// 1 hPA = 100 PA
// 11 km (36,000 ft): 226 hPa
// 20 km (65,000 ft): 54.7 hPa
// 32 km (105,000 ft): 8.68 hPa
// always positive float. assume we read hPA. clamped to 0 to 999.999
// FIX! is it hPA?
extern char t_pressure[8];  // 7 bytes

// always positive float. clamped to 0, 99.99  volts
extern char t_voltage[6];  // 5 bytes

// always positive int. clamped to 0,99
extern char t_sat_count[3];  // 2 bytes
// two letters, two digits, two letters
// base 18, base 18, base 10, base 10, base 24, base 24
// [A-R][A-R][0-9][0-9][A-X][A-X]
// I guess clamp to AA00AA if illegal? (easy to find errors?)

// lon float clamped to 0 if out of range
// -180.0000000 to 180.0000000
// -179.9999999 to 179.9999999
extern char t_lat[13];  // 12 bytes
// lat float clamped to 0 if out of range
// -90.0000000 to 90.0000000
// -89.9999999 to 89.9999999
extern char t_lon[13];  // 12 bytes

// assume it's legal grid6?
// two letters, two digits, two letters
// base 18, base 18, base 10, base 10, base 24, base 24
// [A-R][A-R][0-9][0-9][A-X][A-X]
// I guess clamp to AA00AA if illegal? (easy to find errors?)
extern char t_grid[7];  // 6 bytes

// The above clamping is guaranteed by the assigments to t_*
// Further range clamping is done here, given restrictions of u4b-like telemetry
// but the above can be assumed to be always true.

extern char _Band[3];
extern char _id13[3];
extern char _U4B_chan[4];
extern char _lane[2];
extern char _clock_speed[4];

//*******************************
// use:
//  uint32_t XMIT_FREQUENCY = init_rf_freq()

// inputs: extern char[*] globals
// _U4B_chan
// _id13
// _Band
uint32_t init_rf_freq(void) {
    // base frequencies for different bands
    // 136000 474200 1836600 3568600 5364700 7038600 10138700
    // 14095600 18104600 21094600 24924600 28124600
    // 50293000 70091000 144489000
    // will support 20M, 17M, 15M, 12M, 10M

    enum BASE_FREQS {
        BF20M = 14095600UL,
        BF17M = 18104600UL,
        BF15M = 21094600UL,
        BF12M = 24924600UL,
        BF10M = 28124600UL
    };

    uint32_t BASE_FREQ_USED;
    switch (atoi(_Band)) {
        case 20: BASE_FREQ_USED = BF20M; break;
        case 17: BASE_FREQ_USED = BF17M; break;
        case 15: BASE_FREQ_USED = BF15M; break;
        case 12: BASE_FREQ_USED = BF12M; break;
        case 10: BASE_FREQ_USED = BF10M; break;
        // default to 20M in case of error cases
        default: BASE_FREQ_USED = BF20M;  
    }

    XMIT_FREQUENCY = BASE_FREQ_USED + 1400UL;
    // offset from base for start of passband. same for all bands
    // add offset based on lane ..same for every band
    switch (_lane[0]) {
            case '1':XMIT_FREQUENCY += 20UL;  break;
            case '2':XMIT_FREQUENCY += 60UL;  break;
            case '3':XMIT_FREQUENCY += 140UL; break;
            case '4':XMIT_FREQUENCY += 180UL; break;
            // in case invalid lane was read from EEPROM.
            // This is center passband?? (not a valid lane?)
            default: XMIT_FREQUENCY += 100UL;
        }

        printf("\nrf_freq_init _Band %s BASE_FREQ_USED %d XMIT_FREQUENCY %d _clock_speed %s\n",
            _Band, BASE_FREQ_USED, XMIT_FREQUENCY, _clock_speed);
        return XMIT_FREQUENCY;
}

/*
From Hans G0UPL  on 06/27/23 post #11140 (this is not documented elsewhere).
We had been using a table-driven mapping before Hans posted his algo.

The specification of U4B telemetry channels is as follows:

First callsign character:
Channels 0 - 199: '0'
Channels 200-399: '1'
Channels 400-599: 'Q'

Third callsign character:
(channel % 200) / 20

Frequency discrimination:
Frequency sector is
(channel % 20) / 5

That indicates into the array of transmit audio frequencies: {1420, 1460, 1540, 1580};
which are the target transmit frequencies, each in their 5 sectors.

The actual transmit frequency is the standard WSPR USB dial frequency +
the above mentioned audio frequency;

USB dial frequencies:
    {136000, 474200, 1836600, 3568600, 5364700, 7038600, 10138700,
    14095600, 18104600, 21094600, 24924600, 28124600,
    50293000, 70091000, 144489000};

Transmit slot:
The transmit slot (txSlot) is first calculated as (channel % 5).
Then the start time in minutes past the hour, repeated every 10 minutes, is:
    2 * ((txSlot + 2 * txBand) % 5);

txBand is:
0: 2200m
1: 630m
2: 160m
3: 80m
4: 60m
5: 40m
6: 30m
7: 20m
8: 17m
9: 15m
10: 12m
11: 10m
12: 6m
13: 4m
14: 2m
*/

void process_chan_num() {
    if ((atoi(_U4B_chan) >= 0) && (atoi(_U4B_chan) < 600)) {
        _id13[0]='1';
        // Channels 0 - 199: '0'
        // Channels 200-399: '1'
        // Channels 400-599: 'Q'
        if  (atoi(_U4B_chan) < 200) _id13[0]='0';
        if  (atoi(_U4B_chan) > 399) _id13[0]='Q';

        // (channel % 200) / 20
        int id3 = (atoi(_U4B_chan) % 200) / 20;
        _id13[1] = id3 + '0';

        // Frequency discrimination:
        // Frequency sector is
        // (channel % 20) / 5
        int lane = (atoi(_U4B_chan) % 20) / 5;
        _lane[0] = lane+'1';

        // The transmit slot (txSlot) is first calculated as (channel % 5).
        // Then the start time in minutes past the hour, repeated every 10 minutes
        // 2 * ((txSlot + 2 * txBand) % 5);
        int txSlot = atoi(_U4B_chan) % 5;
        int txBand;
        switch (atoi(_Band)) {
            case 20: txBand = 7;  break;  // 20m
            case 17: txBand = 8;  break;  // 17m
            case 15: txBand = 9;  break;  // 15m
            case 12: txBand = 10; break;  // 12m
            case 10: txBand = 11; break;  // 10m
            default: txBand = 7;  break;  // default to 20M in case of error cases
        }
        // will be char 0, 2, 4, 6 or 8 only
        _start_minute[0] = '0' + (2 * ((txSlot + (txBand*2)) % 5));
    }
}

//*******************************************
void u4b_encode_std() {
    // input: uses t_callsign t_grid6 t_power
    // output: modifies globals: hf_callsign, hf_grid4, hf_power

    // ..which is then encoded as 126 wspr symbols in tx_buffer,
    // and set out as RF with 4-FSK (each symbol has 4 values?)
    // normal telemetry
    if (DEVMODE && verbosity >= 3) printf("creating U4B telemetry 0\n");
    char grid4[5];
    strncpy(grid_3_0, t_grid6, 4);
    grid_3_0[4] = 0;

    // not used
    // char grid_5 = t_grid6[4]
    // char grid_6 = t_grid6[5]

    uint8_t grid5Val = grid5 - 'A';
    uint8_t grid6Val = grid6 - 'A';

    // modulo 20 for altitude. integer
    // decimal
    // this will be a floor divide: t_altitude is int
    // there are only 1068 encodings for altitude..clamp to max
    if (altitude >= (1068 * 20)) {
        altitude = (1068 * 20) - 20;  // 21340 meters?
    }
    uint16_t altitudeNum = t_altitude / 20);

    // convert inputs into a big number
    uint32_t val = 0;
    val *=   24; val += grid5Val;
    val *=   24; val += grid6Val;
    val *= 1068; val += altitudeNum;

    // extract into altered dynamic base
    uint8_t id6Val = val % 26; val = val / 26;
    uint8_t id5Val = val % 26; val = val / 26;
    uint8_t id4Val = val % 26; val = val / 26;
    uint8_t id2Val = val % 36; val = val / 36;

    // convert to encoded callsign
    // remember <space> is not used for encoding..legal for some wspr cases though?
    // alpha plus numeric (36)
    char id2 = EncodeBase36(id2Val);

    // easy to encode to base 26 (alpha only)
    char id4 = 'A' + id4Val;
    char id5 = 'A' + id5Val;
    char id6 = 'A' + id6Val;

    // string{ id13[0], id2, id13[1], id4, id5, id6 };
    callsignU4B[0] =  _id13[0];
    callsignU4B[1] =  id2;
    callsignU4B[2] =  _id13[1];
    callsignU4B[3] =  id4;
    callsignU4B[4] =  id5;
    callsignU4B[5] =  id6;
    callsignU4B[6] =  0;

    // these are stored as float strings (so is pressure)
    double tempC   = atof(t_temp);
    double voltage = atof(t_voltage)

    // handle possible illegal range (double wrap on tempC?).
    // or should it clamp at the bounds?
    int tempCNum      = (int) (tempC - -50) % 90;
    int voltageNum    = (int) round((((voltage * 100) - 300) / 5) + 20) % 40;

    // FIX! kl3cbr did encoding # of satelites into knots.
    // t_speed is integer
    // max t_speed could be 999 ?
    int speedKnotsNum = t_speed  / 2
    // range clamp t_speed (0-41 legal range).
    // clamp to max, not wrap. maybe from bad GNGGA field (wrong sat count?)
    if (speedKnotsNum > 41) speedKnotsNum = 41;

    //****************
    gpsValidNum = 1;
    // traquito site won't show the 6 char grid if this bit is off.
    // shift inputs into a big number
    val = 0;
    val *= 90; val += tempCNum;
    val *= 40; val += voltageNum;
    val *= 42; val += speedKnotsNum;
    val *=  2; val += gpsValidNum;
    // standard telemetry (1 for the 2nd U4B packet, 0 for "Extended TELEN")
    val *=  2; val += 1;

    // unshift big number into output radix values
    uint8_t powerVal = val % 19; val = val / 19;
    uint8_t g4Val    = val % 10; val = val / 10;
    uint8_t g3Val    = val % 10; val = val / 10;
    uint8_t g2Val    = val % 18; val = val / 18;
    uint8_t g1Val    = val % 18; val = val / 18;

    // map output radix to presentation
    char g1 = 'A' + g1Val;
    char g2 = 'A' + g2Val;
    char g3 = '0' + g3Val;
    char g4 = '0' + g4Val;

    char grid4[4];
    grid4[0] = g1;
    grid4[1] = g2;
    grid4[2] = g3;
    grid4[3] = g4;
    grid4[4] = 0;

    int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
    char power[3];
    // n is max number of bytes used. generated string is at most n-1 bytes
    // (leaves room for null term)
    // null term is appended after the generated string
    snprintf(power, sizeof(power), "%s", legalPower[powerVal]);

    // now encode into 162 symbols (4 value? 4-FSK) for tx_buffer
    wspr_encode(callsign, grid4, power, tx_buffer)
}

//********************************************************************
void u4b_encode_telen(uint32_t telen_val1, uint32_t telen_val2, for_telen2) {
    // creates callsign, grid4, power
    // wspr_encode() then creates tx_buffer() which is the 162 wspr symbols
    // used by pwm handler to send 4-FSK rf thru si5351
    uint32_t val = telen_val1;
    // extract into altered dynamic base
    uint8_t id6Val = val % 26; val = val / 26;
    uint8_t id5Val = val % 26; val = val / 26;
    uint8_t id4Val = val % 26; val = val / 26;
    uint8_t id2Val = val % 36; val = val / 36;

    // convert to encoded callsign
    telen_chars[0] = EncodeBase36(id2Val);
    telen_chars[1] = 'A' + id4Val;
    telen_chars[2] = 'A' + id5Val;
    telen_chars[3] = 'A' + id6Val;

    // (bitshift to the left twice to make room for gps bits at end)
    val = telen_val2 * 4;

    // unshift big number into output radix values
    uint8_t powerVal = val % 19; val = val / 19;
    uint8_t g4Val    = val % 10; val = val / 10;
    uint8_t g3Val    = val % 10; val = val / 10;
    uint8_t g2Val    = val % 18; val = val / 18;
    uint8_t g1Val    = val % 18; val = val / 18;

    // map output radix to presentation
    telen_chars[4] = 'A' + g1Val;
    telen_chars[5] = 'A' + g2Val;
    telen_chars[6] = '0' + g3Val;
    telen_chars[7] = '0' + g4Val;
    telen_chars[8] = 0;  // null term

    // identifies it as the 2nd extended TELEN packet.
    // this is the GPS-valid bit.
    // note for extended TELEN we did NOT set the gps-sat bit
    // this is the old GPS-valid bit
    if (for_telen2) powerVal = powerVal + 2;
    // telen1
    else powerVal = powerVal + 0;
    // the last bit, the old GPS-sat bit, is always 0 now.

    printf("val1 %d val2 %d goes to tx_buffer as callsign %s grid %s power %d)\n",
        telen_val1, telen_val2, callsign, grid4, power)

    char callsign[7];
    callsign[0] = _id13[0];
    callsign[1] = telen_chars[0];
    callsign[2] = _id13[1];
    callsign[3] = telen_chars[1];
    callsign[4] = telen_chars[2];
    callsign[5] = telen_chars[3];
    callsign[6] = 0;  // null term

    char grid4[5];
    grid4[0] = telen_chars[4];
    grid4[1] = telen_chars[5];
    grid4[2] = telen_chars[6];
    grid4[3] = telen_chars[7];
    grid4[4] = 0;  // null term

    int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
    // telen_power was passed by reference?
    char[3] power;
    // n is max number of bytes used. generated string is at most n-1 bytes
    // (space for null term)
    // null term is appended after the generated string
    snprintf(power, sizeof(power), "%s", legalPower[powerVal];

    // now encode into 162 symbols (4 value? 4-FSK) for tx_buffer
    wspr_encode(callsign, grid4, power, tx_buffer)
}
