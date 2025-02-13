// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
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
#include "pps_functions.h"

// in libraries: wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
// for setTime()
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

extern const int GPS_1PPS_PIN;
extern uint32_t PPS_rise_millis;
extern bool USE_SIM65M;

// decode of verbose 0-9
extern bool VERBY[10];

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
void setGpsPPSMode(void) {
    V1_println(F("setGpsPPSMode START"));
    if (USE_SIM65M) {
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
        // always based off GPS time base
        Serial2.print("$PAIR755,1,1*3D" CR LF);
        Serial2.flush();
        nmeaBufferFastPoll(500, true);
        // 756 PAIR_PPS_GET_TIMETAG_CONFIG
        // 2 raw meas + sv info + pvt(including time offset data between GPS and GLO/GAL/BDS)
        Serial2.print("$PAIR756,2,1*3D" CR LF);
        Serial2.flush();
        nmeaBufferFastPoll(2000, true);
    }
    V1_println(F("setGpsPPSMode END"));
}

//********************************************************
void gpsPPS_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        PPS_rise_millis = millis();
    }
    // don't need to clear the interrupt for gpio interrupts
}

void gpsPPS_init() {
    gpio_init(GPS_1PPS_PIN);
    gpio_set_dir(GPS_1PPS_PIN, GPIO_IN);
    gpio_pull_up(GPS_1PPS_PIN);
    gpio_set_irq_enabled_with_callback(GPS_1PPS_PIN, GPIO_IRQ_EDGE_RISE, true, &gpsPPS_callback);
}

//********************************************************
