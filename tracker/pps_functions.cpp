// Project: https://github.com/knormoyle/rp2040_si5351_wspr // Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
// for the irq callback
// pico/stdlib.h ?
#include "stdlib.h"
#include "hardware/gpio.h"

#include "debug_functions.h"
#include "led_functions.h"
#include "print_functions.h"
#include "time_functions.h"
#include "gps_functions.h"
#include "time_functions.h"
#include "pps_functions.h"

// in libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// for setTime()
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

extern const int GPS_1PPS_PIN;
extern uint32_t PPS_rise_millis;
extern uint32_t PPS_rise_micros;
// valid after count is 10?
extern bool PPS_rise_valid;
extern uint32_t PPS_rise_cnt;

bool PPS_rise_active;
bool PPS_rise_count;

extern bool USE_SIM65M;
extern bool BALLOON_MODE;

// decode of verbose 0-9
extern bool VERBY[10];

// ATGM336H
// CFG-TMODE 0x06 0x06 0x05 configures PPS
// Read/Set timing mode

// CFG-TP 0x06 0x03
// offset
// Read/set time pulse parameters
// 0 interval (us)
// 4 width (us)
// 8 enable 
//     0 Turn off, 
//     1 enable, 
//     2 pulse is enabled, when no fix, update rate is maintained, 
//     3 enabled, no pulse when no fix
// 9 polar pulse polarity
//     0 rising
//     1 falling
// 10 timeRef
//     0 UTC time
//     1 Satellite time
// 11 timeSource
//     0 force single GPS timing
//     1 mandatory single BDS timing
//     2 mandatory single GLN timing
//     3 reserve
//     4 Mainly BDS, switch to other timing systems when BDS is unavailable
//     5 Mainly GPS, switch to other timing systems when GPS is unavailable
//     6 Mainly use GLN, switch to other timing systems when GLN is unavailable, 

// 12 userDelay

// FIX! I suppose we'll want to do writes to enable PPS
// SIM65M
// Packet Type:750 PAIR_PPS_SET_CONFIG
// Set the configuration of the local time in milliseconds and phase where the PPS should be placed
// default ??
// Example: $PAIR750,1,1345,555*13
// PPS_by_user "1",PPS output by user "0",PPS automatic output
// Local_ms Local receiver time tick. Range is from 0 to 2**32 - 1. If PSS is enabled, aligns to TOW
// Phase Time tick phase range is from 0 to 262143. If PSS is enabled, aligns to TOW
//
// Packet Type:752 PAIR_PPS_SET_CONFIG_CMD
// Configure the PPS settings
// default ?
// Example: $PAIR752,2,100*39
// Enable
// 0 Disable
// 1 After the first fix
// 2 3D fix only
// 3 2D/3D fix only
// 4 always
// PPSPulseWidth PPS Pulse Width. ms. Range 1-999

// Packet Type:755 PAIR_PPS_SET_TIMETAG
// Set enable/disable output time tag and time base
// Example: $PAIR755,1,1*3D
// default ?
// Enable 0, Disable 1, Enable
// Time_base
// 0 UTC
// 1 GPS
// 2 GLO
// 3 GAL
// 4 BDS
// 5 NavIC

// Packet Type:756 PAIR_PPS_GET_TIMETAG_CONFIG
// interesting has time offset data between GPS and GLO/GAL/BDS
// Now only support GPS time base??
// Example $PAIR756*3E  (why is this UTC rather than GPS??)
// Enable
// 0 disable
// 1 raw meas
// 2 raw meas + sv info + pvt(including time offset data between GPS and GLO/GAL/BDS)

//********************************************************
void PPS_countEnable(bool reset) {
    V1_print(F("PPS_countEnable START END" EOL));
    PPS_rise_active = true;
    if (reset) {
        // only start from scratch if we cold reset
        PPS_rise_cnt = 0;
        PPS_rise_valid = false;
    }
}
    
void PPS_countDisable(void) {
    V1_print(F("PPS_countDisable START END" EOL));
    PPS_rise_active = false;
    // don't change PPS_rise_cnt or PPS_rise_valid
    // we might have gotten enough PPS so PPS_rise_micros can be used
    // with modulo 1 sec, if PPS is now off
}

void setGpsPPSMode(void) {
    // FIX! if this is redone on every warm reset, PPS doesn't have much chance to stablize?
    // since we adjust skew always now, we want this even in BALLOON_MODE
    // what's the default? maybe we don't need this and PPS will be more stable?
    V1_println(F("setGpsPPSMode START"));
    if (false && USE_SIM65M) {
        // automatic. Local_ms and Phase are 0
        // Packet Type:750 PAIR_PPS_SET_CONFIG
        Serial2.print("$PAIR750,0,0,0*24" CR LF);
        Serial2.flush();
        nmeaBufferFastPoll(500, true);  // duration_millis, printIfFull
        // Packet Type:752 PAIR_PPS_SET_CONFIG_CMD
        // PPS after the first fix
        Serial2.print("$PAIR752,1*27" CR LF);
        Serial2.flush();
        nmeaBufferFastPoll(500, true);
        // Packet Type:755 PAIR_PPS_SET_TIMETAG
        // always based off GPS time base. Do they only support GPS time base?
        Serial2.print("$PAIR755,1,1*3D" CR LF);
        // response: UTC?
        // $PAIR001,755,0*3C

        // or should it be UTC
        // https://aviation.stackexchange.com/questions/90839/what-are-satellite-time-gps-time-and-utc-time
        // Serial2.print("$PAIR755,1,0*3C" CR LF);
        // GPS time:
        // Defined as equal to UTC at midnight on January 6th 1980 
        // when UTC was 19 seconds behind TAI.
        // 18 leap seconds were added to UTC since then.
        // GPS time is now 18 seconds ahead of UTC.

        Serial2.flush();
        nmeaBufferFastPoll(500, true);
        // 756 PAIR_PPS_GET_TIMETAG_CONFIG
        // 2 raw meas + sv info + pvt(including time offset data between GPS and GLO/GAL/BDS)
        Serial2.print("$PAIR756,2*20" CR LF);
        Serial2.flush();
        nmeaBufferFastPoll(2000, true);
        // normally we always get this response?
        // $PAIR001,756,0*3F

    }
    V1_println(F("setGpsPPSMode END"));
}

//********************************************************
void gpsPPS_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        // no modification or reporting if -1
        if (PPS_rise_active) {
            PPS_rise_cnt += 1;
            // keep it just 10, so the first wspr can have setTime that's got good skew measurement
            PPS_rise_valid = PPS_rise_cnt > 10;
            // just the first 30 after it's been reset
            if (PPS_rise_cnt < 30) {
                V1_printf("INFO: PPS_rise_cnt %lu" EOL, PPS_rise_cnt);
            }
        }
        
        uint32_t current_millis = millis();
        uint32_t current_micros = micros();
        static uint32_t printed = 0; // stop after printing 100
        if (PPS_rise_valid) {
            // no reporting if 0
            // 0 is legal value for the wraparound uint32_t on the PPS_rise_micros
            uint32_t elapsed_micros = current_micros - PPS_rise_micros;
            // should be 1e6 micros
            int elapsed_micros_error = 1000000 - ((int) elapsed_micros);
            if ((printed < 100) && abs(elapsed_micros_error) > 1) {
                printed += 1;
                V1_printf("INFO: PPS period micros %lu", elapsed_micros);
                V1_printf(" error %d ", elapsed_micros_error);
                printSystemDateTime();
                V1_print(F(EOL));
            }

            PPS_rise_millis = current_millis;
            PPS_rise_micros = current_micros;
        }
    // don't need to clear the interrupt for gpio interrupts
    }
}

// this is done for SIM65 or ATGM336H?
// FIX! does ATGM336 need commands to cause PPS?
void gpsPPS_init() {
    // only done once
    V1_println(F("gpsPPS_init START"));

    gpio_init(GPS_1PPS_PIN);
    gpio_set_dir(GPS_1PPS_PIN, GPIO_IN);
    gpio_pull_up(GPS_1PPS_PIN);
    gpio_set_irq_enabled_with_callback(GPS_1PPS_PIN, GPIO_IRQ_EDGE_RISE, true, &gpsPPS_callback);

    PPS_countEnable(true); 
    V1_println(F("gpsPPS_init END"));
}

//********************************************************
