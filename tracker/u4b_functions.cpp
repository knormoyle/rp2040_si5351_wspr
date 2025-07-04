// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

#include "u4b_functions.h"
#include "print_functions.h"

//********************************
extern bool VERBY[10];

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
extern char t_grid6[7];  // 6 bytes

// The above clamping is guaranteed by the assigments to t_*
// Further range clamping is done here, given restrictions of u4b-like telemetry
// but the above can be assumed to be always true.

//*******************************
char EncodeBase36(uint8_t val) {
    char retVal;
    if (val < 10) retVal = '0' + val;
    else retVal = 'A' + (val - 10);
    return retVal;
}

//*******************************
void init_rf_freq(uint32_t *xmit_freq, char *band, char *lane) {
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
        // avg rx spot: seeing +4 to +26 hz with this base freq
        // BF10M = 28124600UL,
        // subtract 15 to shift it down (also 3 hz offset for middle freq?)
        // FIX! default corrections for other bands?
        // wonder how this relates to load capacitance setting 
        BF10M = 28124600UL - 15UL,
        BF02M = 144489000UL, 
    };

    uint32_t base_freq_used;
    switch (atoi(band)) {
        case 20: base_freq_used = BF20M; break;
        case 17: base_freq_used = BF17M; break;
        case 15: base_freq_used = BF15M; break;
        case 12: base_freq_used = BF12M; break;
        case 10: base_freq_used = BF10M; break;
        case  2: base_freq_used = BF02M; break;
        // default to 20M in case of error cases
        default: base_freq_used = BF20M;
    }

    uint32_t xmit_freq_here = base_freq_used + 1400UL;
    // offset from base for start of passband. same for all bands
    // add offset based on lane ..same for every band
    switch (lane[0]) {
        case '1': xmit_freq_here += 20UL;  break;
        case '2': xmit_freq_here += 60UL;  break;
        case '3': xmit_freq_here += 140UL; break;
        case '4': xmit_freq_here += 180UL; break;
        // in case invalid lane was read from EEPROM.
        // This is center passband?? (not a valid lane?)
        default: xmit_freq_here += 100UL;
    }

    // printf uint32_t with %u
    V1_printf(EOL "rf_freq_init band %s base_freq_used %lu xmit_freq %lu " EOL,
        band, base_freq_used, xmit_freq_here);

    *xmit_freq = xmit_freq_here;
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
Frequency sector is: (isn't this the same as txSlot?)
    (channel % 20) / 5

That indicates into the array of transmit audio frequencies: 
Why is this only 4? should it be 5?
[1420, 1460, 1540, 1580]
which are the target transmit frequencies, each in their 5 sectors.

The actual transmit frequency is the standard WSPR USB dial frequency +
the above mentioned audio frequency.

USB dial frequencies:
    [136000, 474200, 1836600, 3568600, 5364700, 7038600, 10138700,
    14095600, 18104600, 21094600, 24924600, 28124600,
    50293000, 70091000, 144489000];

Transmit slot:
The transmit slot (txSlot) is first calculated as: 
    (channel % 5).

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

void process_chan_num(char *id13, char *start_minute, char *lane, char *band, char *u4b_chan) {
    int u4bChannel = atoi(u4b_chan);
    if (u4bChannel < 0 || u4bChannel > 599) {
        V1_printf("ERROR: bad u4bChannel %d ..using 599" EOL, u4bChannel);
        u4bChannel = 599;
    }

    id13[0]='1';
    // Channels 0 - 199: '0'
    // Channels 200-399: '1'
    // Channels 400-599: 'Q'
    if  (u4bChannel < 200) id13[0]='0';
    if  (u4bChannel > 399) id13[0]='Q';

    // (channel % 200) / 20
    int id3_here = u4bChannel % 200 / 20;
    id13[1] = id3_here + '0';
    id13[2] = 0;

    // Frequency discrimination:
    // Frequency sector is
    // (channel % 20) / 5
    int lane_here = (u4bChannel % 20) / 5;
    lane[0] = lane_here + '1';
    lane[1] = 0;

    // The transmit slot (txSlot) is first calculated as (channel % 5).
    // Then the start time in minutes past the hour, repeated every 10 minutes
    // 2 * ((txSlot + 2 * txBand) % 5);
    int txSlot = u4bChannel % 5;
    int txBand;
    switch (atoi(band)) {
        case 20: txBand = 7;  break;  // 20m
        case 17: txBand = 8;  break;  // 17m
        case 15: txBand = 9;  break;  // 15m
        case 12: txBand = 10; break;  // 12m
        case 10: txBand = 11; break;  // 10m
        case 2:  txBand = 14; break;  // 2m
        default: txBand = 7;  break;  // default to 20M in case of error cases
    }
    // will be char 0, 2, 4, 6 or 8 only
    start_minute[0] = '0' + (2 * ((txSlot + (txBand * 2)) % 5));
}

//*******************************************
// 3 parameters for return char arrays.
// then all variables (char arrays) used from the snapped telemetry:
// standard u4b telemetry
void u4b_encode_std(char *hf_callsign, char *hf_grid4, char *hf_power,
    char *t_grid6, char *t_altitude, char *t_temp, char *t_voltage, char *t_speed, 
    char *id13) {

    V1_printf(
        "u4b_encode_char START t_grid6 %s t_altitude %s t_temp %s 5_voltage %s t_speed %s" EOL,
        t_grid6, t_altitude, t_temp, t_voltage, t_speed);

    // ..which is then encoded as 126 wspr symbols in tx_buffer,
    // and set out as RF with 4-FSK (each symbol has 4 values?)

    if (t_grid6[4] < 'A' || t_grid6[4] > 'X') {
        V1_printf("ERROR: bad t_grid6[4] %s" EOL, t_grid6);
        t_grid6[1] = 'A';
    }
    if (t_grid6[5] < 'A' || t_grid6[5] > 'X') {
        V1_printf("ERROR: bad t_grid6[5] %s" EOL, t_grid6);
        t_grid6[5] = 'A';
    }
    uint8_t grid5Val = t_grid6[4] - 'A';
    uint8_t grid6Val = t_grid6[5] - 'A';

    // modulo 20 for altitude. integer
    // decimal
    int altitude = atoi(t_altitude);
    int altitudeVal = altitude / 20;
    // there are only 1068 encodings for altitude..clamp to max (or min)
    if (altitudeVal < 0) altitudeVal = 0;
    // this will be a floor divide: t_altitude is int
    else if (altitudeVal > 1067) altitudeVal = 1067;  // 21340 meters?

    // convert inputs into a big number
    uint32_t val = 0;
    val *=   24; val += grid5Val;
    val *=   24; val += grid6Val;
    val *= 1068; val += altitudeVal;

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

    char callsign[7];
    // string id13[0], id2, id13[1], id4, id5, id6
    callsign[0] =  id13[0];
    callsign[1] =  id2;
    callsign[2] =  id13[1];
    callsign[3] =  id4;
    callsign[4] =  id5;
    callsign[5] =  id6;
    callsign[6] =  0;

    // these are stored as float strings (so is pressure)
    double tempC   = atof(t_temp);
    double voltage = atof(t_voltage);

    // handle possible illegal range (double wrap on tempC?).
    // should it clamp at the bounds? Yes
    if (tempC < -50) tempC = -50;
    // only 0-89 encodings. (90)
    else if (tempC > (-50 + 89)) tempC = (-50 + 89);  // 39C max
    int tempCNum = (int) tempC + 50;

    // voltage: 20 to 39, 0 to 19 for 3.00 to 4.95V with a resolution of 0.05V
    /// 0 to 39 encodings
    if (voltage > 4.95) voltage = 4.95;
    else if (voltage < 3.00) voltage = 3.00;
    // should only 3 to 4.95
    int voltageNum = (int)(round ((voltage - 3.00) / .05) + 20) % 40;

    // FIX! kl3cbr did encoding # of satelites into knots.
    // t_speed is integer
    // max t_speed could be 999 ?
    int speed = atoi(t_speed);
    // this should round to nearest 2 since the telemety is 2 knot precision
    // we should have already rounded to nearest 2 when setting speed
    speed = 2 * ((speed + 1) / 2);

    // if we're sending solar elevation here, we forced it to be >=0
    // so we'll be sending half the solar elevation? 
    // will be able to get up to 82 deg angle reported?
    // with the 0-41 range?  (integer)

    int speedKnotsNum = speed / 2;
    // range clamp t_speed (0-41 legal range).
    // clamp to max, not wrap. maybe from bad GNGGA field (wrong sat count?)
    if (speedKnotsNum < 0) speedKnotsNum = 0;
    // since we're reporting 2x solarElevation ..we will clamp at 41 deg 
    if (speedKnotsNum > 41) speedKnotsNum = 41;

    //****************
    // traquito site won't show the 6 char grid if this bit is off.
    int gpsValidNum = 1;
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
    int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
    char power[3];
    snprintf(power, sizeof(power), "%2d", legalPower[powerVal]);

    uint8_t g4Val    = val % 10; val = val / 10;
    uint8_t g3Val    = val % 10; val = val / 10;
    uint8_t g2Val    = val % 18; val = val / 18;
    uint8_t g1Val    = val % 18; val = val / 18;

    // map output radix to presentation
    char grid4[5] = { 0 };
    grid4[0] = 'A' + g1Val;
    grid4[1] = 'A' + g2Val;
    grid4[2] = '0' + g3Val;
    grid4[3] = '0' + g4Val;

    // n is max number of bytes used. generated string is at most n-1 bytes
    // (leaves room for null term)
    // null term is appended after the generated string

    // ah, can't use sizeof. size is lost
    // BUG: reported by Rob KC3LBR
    // JTEncode walks thru chars 0-12. 
    // so we now have 14 char array with null term
    // left justify..i.e. pad with space for what JTEncode gets
    snprintf(hf_callsign, 14, "%-13s", callsign);
    snprintf(hf_grid4, 5, "%s", grid4);
    snprintf(hf_power, 3, "%s", power);

    V1_printf("u4b_encode_std created: hf_callsign %s hf_grid %s hf_power %s)" EOL,
        hf_callsign, hf_grid4, hf_power);
}

//********************************************************************
// first 3 parameters are all for return char arrays.
void u4b_encode_telen(char *hf_callsign, char *hf_grid4, char *hf_power,
    uint32_t t_telen_val1, uint32_t t_telen_val2, bool for_telen2, char *id13) {

    V1_printf("u4b_encode_telen START t_telen_val1 %lu t_telen_val2 %lu" EOL,
            t_telen_val1, t_telen_val2);

    //********************************************
    // creates callsign, grid4, power
    // wspr_encode() then creates tx_buffer() which is the 162 wspr symbols
    // used by pwm handler to send 4-FSK rf thru si5351
    uint32_t val = t_telen_val1;
    // extract into altered dynamic base
    uint8_t id6Val = val % 26; val = val / 26;
    uint8_t id5Val = val % 26; val = val / 26;
    uint8_t id4Val = val % 26; val = val / 26;
    uint8_t id2Val = val % 36; val = val / 36;

    char telen_chars[9];  // 6 + 3, callsign and grid

    // convert to encoded callsign
    telen_chars[0] = EncodeBase36(id2Val);
    telen_chars[1] = 'A' + id4Val;
    telen_chars[2] = 'A' + id5Val;
    telen_chars[3] = 'A' + id6Val;

    // unshift big number into output radix values
    uint8_t powerVal = val % 19; val = val / 19;
    int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
    char power[3];
    snprintf(power, sizeof(power), "%2d", legalPower[powerVal]);

    //********************************************
    // (bitshift to the left twice to make room for gps bits at end)
    val = t_telen_val2 * 4;

    uint8_t g4Val    = val % 10; val = val / 10;
    uint8_t g3Val    = val % 10; val = val / 10;
    uint8_t g2Val    = val % 18; val = val / 18;
    uint8_t g1Val    = val % 18; val = val / 18;

    // unshift big number into output radix values

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

    char callsign[7];
    callsign[0] = id13[0];
    callsign[1] = telen_chars[0];
    callsign[2] = id13[1];
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

    // can't use sizeof() .. size is lost
    // we shouldn't have to specify 6, 4, 2 for the %s ?
    snprintf(hf_callsign, 7, "%s", callsign);
    snprintf(hf_grid4, 5, "%s", grid4);
    snprintf(hf_power, 3, "%s", power);

    V1_printf("u4b_encode_telen END hf_callsign %s hf_grid4 %s hf_power %s" EOL,
        hf_callsign, hf_grid4, hf_power);
}
