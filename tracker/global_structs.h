
// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef GLOBAL_STRUCTS_H
#define GLOBAL_STRUCTS_H

// https://stackoverflow.com/questions/9749943/how-to-make-a-structure-extern-and-define-its-typedef

//***********************************************************
// config strings: all can be extern'ed by a function
// but there is also decode of them to another global that is used instead.
// see config_functions.cpp
// these get set via terminal, and then from NVRAM on boot
// init with all null

typedef struct ConfigStruct_ {
    // hmmm. can't declare these arrays 'volatile'
    char _callsign[7];
    char _verbose[2];   // 0 is used to disable all. 1 is all printing for now. 2:9 same
    char _ExtTelemetry[5];
    // FIX! why is this a compiler problem if volatile? an snprintf() fails
    // https://forum.arduino.cc/t/invalid-conversion-from-volatile-char-to-const-char-fpermissive/949522
    char _clock_speed[4];
    char _U4B_chan[4];  // 1 to 3 digits?
    // error: invalid convesion fro 'volatile char*' to 'char*'
    char _Band[3];      // string with 2, 10, 12, 15, 17, 20 legal. null at end
    char _Band_cw[3];   // string with 2, 10, 12, 15, 17, 20 legal. null at end
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
    char _tx_high[2];   // 0 is 4mA si5351. 1 is 8mA si5351
    char _testmode[2];  // currently: sweep telemetry

    // don't allow more than approx. 43 hz "correction" on a band. leave room for 6 chars
    char _correction[7];  // parts per billion -30000 to 30000. default 0
    // traquito: 500 correction does  ~7 hz lower on 20M (14095.600 base freq)
    // traquito: 500 correction does ~14 hz lower on 10M (28124.600 base freq)

    // test only: 1 means you don't wait for starting minute from _U4B_chan
    // does wait for any 2 minute alignment though
    char _go_when_rdy[2];
    char _factory_reset_done[2];
    char _use_sim65m[2];
    char _morse_also[2];   
    char _solar_tx_power[2];
    char _const_group[2];

    // decoded stuff from config strings: all can be extern'ed by a function
    // decodes from _Band _U4B_chan
    // 0 should never happen for XMIT_FREQUENCY
    char _id13[3];
    char _start_minute[2];
    char _lane[2];
} ConfigStruct;

//************************************************
// essentiallytt.* stuff is a telemetry data buffer/structure
// all can be extern'ed by a function
// init to 0 is just in case. Should always be set to something valid before use
// empty string is not valid
// not sure what will happen if used while empty.. I suppose it can print ok
// always positive. clamp to 0 I guess
typedef struct TeleStruct_ {
    char course[4];
    // always positive? 0-250 knots. clamp to 0 I guess
    char speed[4];
    // allow 60000 meters. plus 1 in case negative?
    char altitude[7];
    // 24 * 30 per hour;
    // reboot once per day? (starts at 0)
    char tx_count_0[4];
    char temp[7];
    char pressure[8];
    char temp_ext[8];
    char humidity[8];
    char voltage[6];
    char sat_count[3];
    // lat/lon precision: How much to store
    // https://stackoverflow.com/questions/1947481/how-many-significant-digits-should-i-store-in-my-database-for-a-gps-coordinate
    // 6 decimal places represent accuracy for ~ 10 cm
    // 7 decimal places for ~ 1 cm
    // The use of 6 digits should be enough. +/- is 1 more. decimal is one more. 0-180 is 3 more.
    // so 7 + 5 = 12 bytes should enough, with 1 more rounding digit?
    char lat[13];
    char lon[13];
    char callsign[7];
    char grid6[7];
    char power[3];
    // power clamped to 0 if not in this list of legal
    // legalPower = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]

    char solarElevation[5];
    char solarAzimuth[7];
    char solarDistance[8];

    char sats[3];
    char hdop[4];
    char gp_sats[3];
    char gb_sats[3];
    char gl_sats[3];
    char ga_sats[3];
    char gpsLockSecs[4];
    char gpsLockSecsMin[4];
    char gpsLockSecsMax[4];
    char gpsLockSecsAvg[4];

    int snap_cnt;

    int ExtTelemetry1_val1;
    int ExtTelemetry1_val2;
    int ExtTelemetry2_val1;
    int ExtTelemetry2_val2;
} TeleStruct;

#endif // GLOBAL_STRUCTS_H
