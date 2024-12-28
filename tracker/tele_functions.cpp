// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <stdlib.h>

#include "config_functions.h"
#include "print_functions.h"
#include "bmp_functions.h"
#include "mh_functions.h"
#include "adc_functions.h"
#include "tele_functions.h"

#include <TinyGPS++.h>  // https://github.com/mikalhart/TinyGPSPlus

extern uint64_t GpsTimeToLastFix;  // milliseconds

extern char t_course[4];      // 3 bytes
extern char t_speed[4];       // 3 bytes
extern char t_altitude[7];    // 6 bytes
extern char t_tx_count_0[4];  // 3 bytes
extern char t_temp[7];        // 6 bytes
extern char t_pressure[8];    // 7 bytes
extern char t_temp_ext[8];    // 7 bytes
extern char t_humidity[8];    // 7 bytes
extern char t_voltage[6];     // 5 bytes
extern char t_sat_count[3];   // 2 bytes
extern char t_hdop[4];        // 3 bytes

extern int t_TELEN1_val1;
extern int t_TELEN1_val2;
extern int t_TELEN2_val1;
extern int t_TELEN2_val2;

extern bool TESTMODE;

// lat/lon precision: How much to store
// https://stackoverflow.com/questions/1947481/how-many-significant-digits-should-i-store-in-my-database-for-a-gps-coordinate
// 6 decimal places represent accuracy for ~ 10 cm
// 7 decimal places for ~ 1 cm
// The use of 6 digits should be enough. +/- is 1 more. decimal is one more. 0-180 is 3 more.
// so 7 + 5 = 12 bytes should enough, with 1 more rounding digit?
extern char t_lat[12];        // 12 bytes
extern char t_lon[12];        // 12 bytes
extern char t_grid6[7];       // 6 bytes
extern char t_callsign[7];    // 6 bytes
extern char t_power[3];       // 2 bytes

// clamped to 0 if not in this list of legal
// legalPower = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]
extern char t_power[3];       // 2 bytes

extern int TELEN1_val1;
extern int TELEN1_val2;
extern int TELEN2_val1;
extern int TELEN2_val2;
extern char _TELEN_config[5];

extern char _tx_high[2];      // 1 byte
extern char _callsign[7];     // 6 bytes

extern TinyGPSPlus gps;
extern int tx_cnt_0;
// decode of _verbose 0-9
extern bool VERBY[10];

//****************************************************

int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
int legalPowerSize = 19;

void snapForTelemetry(void) {
    V1_println(F("snapForTelemetry START"));
    if (TESTMODE) {
        V1_println(F("snapForTelemetry TESTMODE detected"));
        telemetrySweepAllForTest();
        // continue sweep inc'ing the static data (and wrapping)
        return;
    }
    // FIX! didn't we already check this?
    // FIX! why does isUpdated() get us past here?
    // don't qualify here. all of the qualification should be in tracker.ino?

    // if (!gps.location.isValid()) return;
    // else if (gps.location.age() >= 1000 && !gps.location.isUpdated()) return;
    // else if (gps.satellites.value() <= 3) return;

    int course = gps.course.isValid() ? gps.course.deg() : 0;
    if (course < 0)   course = 0;
    if (course > 360) course = 360;
    snprintf(t_course, sizeof(t_course), "%3d", course);

    int speed = gps.speed.isValid() ? gps.speed.knots() : 0;
    if (speed < 0)   speed = 0;
    if (speed > 999) speed = 999;
    snprintf(t_speed, sizeof(t_speed), "%3d", speed);

    // fixing negative altitude values
    int altitude = (int) gps.altitude.meters();
    if (altitude < 0) altitude = 0;
    if (altitude > 999999) altitude = 999999;
    snprintf(t_altitude, sizeof(t_altitude), "%6d", altitude);

    //*********************************
    // FIX! remove. this result is bogus. does it not work for temp? does temp have a /3 divider
    const float conversionFactor_a = 3.3f / (1 << 12);  // read temperature
    int adc_val_a = 0;

    // FIX! does this not apply to pi pico?
    // Reads the value from the specified analog pin.
    // The input range can be changed using analogReference(),
    // While the resolution can be changed analogReadResolution().
    // takes 100usec

    // pin is the analog input pint
    adc_val_a += analogRead(4);
    adc_val_a += analogRead(4);
    adc_val_a += analogRead(4);

    float adc_a = (conversionFactor_a * (float) adc_val_a) / 3;
    float tempC_a = 27.0f - (adc_a - 0.706f) / 0.001721f;
    // does it need a divide by 3?

    //*********************************
    // okay
    float vref = 3.3f;
    float tempC_b = 0.0;
    tempC_b += analogReadTemp(vref);
    tempC_b += analogReadTemp(vref);
    tempC_b += analogReadTemp(vref);
    tempC_b = tempC_b / 3.0;
    // https://github.com/NuclearPhoenixx/Arduino-Pico-Analog-Correction

    // library PicoAnalogCorrection
    // linear calibration and arithmetic mean of an analog pin

    // okay (it's getting analogReadTemp() from readTemp())
    // readTemp does 30 analogReadTemp() and averages
    float tempC_c = readTemp();

    // examples:
    // readTemp END tempC_a 82 tempC_b 25
    // readTemp END tempC_a 82 tempC_b 24
    //  tempC_a 437 tempC_b 24 tempC_c 24

    V1_printf("tempC_a %.f tempC_b %.f tempC_c %.f" EOL,
        tempC_a, tempC_b, tempC_c);
    // tempC_b is best
    float tempC = tempC_b;

    // turn floats into strings
    // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string);

    if (tempC < -999.9) tempC = -999.9;
    if (tempC > 999.9) tempC = 999.9;
    snprintf(t_temp, sizeof(t_temp), "%6.1f", tempC);

    //*********************************
    // examples
    // 1 hPA = 100 PA
    // 11 km (36,000 ft): 226 hPa
    // 20 km (65,000 ft): 54.7 hPa
    // 32 km (105,000 ft): 8.68 hPa
    // FIX! do we read hPA

    float pressure = bmp_read_pressure();
    if (pressure < 0) pressure = 0;
    if (pressure > 999.99) pressure = 999.99;
    snprintf(t_pressure, sizeof(t_pressure), "%7.2f", pressure);

    float temp_ext = bmp_read_temperature();
    if (temp_ext < 0) temp_ext = 0;
    if (temp_ext > 999.99) temp_ext = 999.99;
    snprintf(t_temp_ext, sizeof(t_temp_ext), "%7.2f", temp_ext);

    float humidity = bmp_read_humidity();
    if (humidity < 0) humidity = 0;
    if (humidity > 999.99) humidity = 999.99;
    snprintf(t_humidity, sizeof(t_humidity), "%7.2f", humidity);

    float voltage = readVoltage();
    if (voltage < 0) voltage = 0;
    if (voltage > 99.99) voltage = 99.99;
    snprintf(t_voltage, sizeof(t_voltage), "%5.2f", voltage);

    // FIX! could use this for some TELEN telemetry?
    int hdop = gps.hdop.isValid() ? (int) gps.hdop.value() : 0;
    if (hdop < 0) hdop = 0;  // hundredths. <100 is very good
    if (hdop > 999) hdop = 999;  // can get >999 from the gps, but we don't tx it (usually?)
    snprintf(t_hdop, sizeof(t_hdop), "%3d", hdop);

    // FIX! could use this for some TELEN telemetry?
    int sat_count = gps.satellites.isValid() ? (int) gps.satellites.value() : 0;
    if (sat_count < 0) sat_count = 0;
    if (sat_count > 99) sat_count = 99;
    snprintf(t_sat_count, sizeof(t_sat_count), "%2d", sat_count);

    double lat = gps.location.lat();
    // FIX is both 90 and -90 legal for maidenhead translate?
    if (lat < -90) lat = -90;
    if (lat > 90) lat = 90;
    // 12 bytes max with - and . counted
    snprintf(t_lat, sizeof(t_lat), "%.7f", lat);

    double lon = gps.location.lng();
    // FIX is both 180 and -180 legal for maidenhead translate?
    if (lon < -180) lon = -180;
    if (lon > 180) lon = 180;
    snprintf(t_lon, sizeof(t_lon), "%.7f", lon);

    //*********************************
    char grid6[7] = { 0 };
    // get_mh_6 modifies grid6 here with the grid6 ptr

    // Note passing char array to function:
    // foo(char* s) and foo(char s[]) are exactly equivalent to one another.
    // In both cases, you pass the array with its name:
    // char array[4];
    // foo(array); // regardless of whether foo accepts a char* or a char[]

    // the gps.location.lat/lng are double
    get_mh_6(grid6, gps.location.lat(), gps.location.lng());

    // two letters, two digits, two letters
    // base 18, base 18, base 10, base 10, base 24, base 24
    // [A-R][A-R][0-9][0-9][A-X][A-X]
    // I guess clamp to AA00AA if illegal? (will be easy to find errors in website reports?)
    bool bad_grid = false;
    if (grid6[0] < 'A' || grid6[0] > 'R') bad_grid = true;
    if (grid6[1] < 'A' || grid6[1] > 'R') bad_grid = true;
    if (grid6[2] < '0' || grid6[2] > '9') bad_grid = true;
    if (grid6[3] < '0' || grid6[3] > '9') bad_grid = true;
    if (grid6[4] < 'A' || grid6[4] > 'X') bad_grid = true;
    if (grid6[5] < 'A' || grid6[5] > 'X') bad_grid = true;
    if (bad_grid)
        // can't use sizeof(grid6) here because it's a pointer
        snprintf(grid6, 7, "%6s", "AA00AA");

    snprintf(t_grid6, sizeof(t_grid6), "%6s", grid6);

    //*********************************
    // snap callsign just for consistency with everything else
    // we should never tx callsign or telemetry if we didn't get a fix
    // so okay if t_callsign is blank until we get a fix?
    snprintf(t_callsign, sizeof(t_callsign), "%s", _callsign);

    //*********************************
    int power_int;
    if (_tx_high[0] == '1') power_int = 7;  // legal
    else power_int = 3;  // legal

    // we clamp to a legalPower when we snapForTelemetry()
    // basically we look at _tx_high[0] to decide our power level that will be used for rf
    // we could use values that are unique for this tracker,
    // for easy differentiation from u4b/traquito!!
    // like 3 and 7!

    // validity check the power. for 'same as everything else' checking
    bool found = false;
    for (int i = 0; i < legalPowerSize; i++) {
        if (legalPower[i] == power_int) {
            found = true;
            break;
        }
    }
    if (!found) power_int = 0;
    snprintf(t_power, sizeof(t_power), "%2d", power_int);

    //*********************************
    int tx_cnt_0_val = tx_cnt_0;
    if (tx_cnt_0 < 0) tx_cnt_0_val = 0;
    // do we need to count more than 99 in a day?
    if (tx_cnt_0 > 99) tx_cnt_0_val = 99;
    // we have room for 999
    snprintf(t_tx_count_0, sizeof(t_tx_count_0), "%3d", tx_cnt_0_val);

    //*********************************
    // snap for consistency with everything else (all at one instant in time)
    t_TELEN1_val1 = TELEN1_val1;
    t_TELEN1_val2 = TELEN1_val2;
    t_TELEN2_val1 = TELEN2_val1;
    t_TELEN2_val2 = TELEN2_val2;

    //*********************************
    V1_printf("t_************" EOL);
    V1_printf("t_tx_count_0 %3s " EOL, t_tx_count_0);
    V1_printf("t_callsign %6s" EOL, t_callsign);
    V1_printf("t_grid6 %6s" EOL, t_grid6);
    V1_printf("t_power %2s" EOL, t_power);
    V1_printf("t_sat_count %2s " EOL, t_sat_count);
    V1_printf("t_lat %12s " EOL, t_lat);
    V1_printf("t_lon %12s " EOL, t_lon);
    V1_printf("t_altitude %6s " EOL, t_altitude);
    V1_printf("t_voltage %5s " EOL, t_voltage);
    V1_printf("t_temp %6s" EOL, t_temp);
    V1_printf("t_course %3s " EOL, t_course);
    V1_printf("t_speed %3s " EOL, t_speed);
    V1_printf("t_temp_ext %7s" EOL, t_temp);
    V1_printf("t_pressure %7s " EOL, t_pressure);
    V1_printf("t_TELEN1_val1 %d " EOL, t_TELEN1_val1);
    V1_printf("t_TELEN1_val2 %d " EOL, t_TELEN1_val2);
    V1_printf("t_TELEN2_val1 %d " EOL, t_TELEN2_val1);
    V1_printf("t_TELEN2_val2 %d " EOL, t_TELEN2_val2);
    V1_printf("t_************" EOL);

    V1_println(F("snapForTelemetry END"));
}

//****************************************************
// FIX! are these assigned by anything yet? No?
static float onewire_values[10] = { 0 };

void process_TELEN_data(void) {
    V1_println(F("process_TELEN_data START"));
    // minutes_since_GPS_acquistion (should this be last time to fix);
    // we don't send stuff out if we don't get gps acquistion.
    // so minutes since fix doesn't really matter?

    // 3.3 * 1000. the 3.3 is from vref,
    // the 1000 is to convert to mV.
    // the 12 bit shift is because thats resolution of ADC
    const float conversionFactor = 3300.0f / (1 << 12);

    int telen_values[4] = { 0 };
    uint32_t timeSinceBoot_secs = millis() / 1000UL;  // seconds
    for (int i=0; i < 4; i++) {
        switch (_TELEN_config[i]) {
            case '-':  break;  // do nothing, telen chan is disabled
            case '0':
                telen_values[i] = round((float)analogRead(0) * conversionFactor);
                break;
            case '1':
                telen_values[i] = round((float)analogRead(1) * conversionFactor);
                break;
            case '2':
                telen_values[i] = round((float)analogRead(2) * conversionFactor);
                break;
            case '3':
                // ADC3 is hardwired to Battery via 3:1 voltage divider: make the conversion here
                telen_values[i] = round((float)analogRead(3) * conversionFactor * 3.0f);
                break;
            case '4':
                telen_values[i] = timeSinceBoot_secs;  // seconds since running
                break;
            case '5':
                telen_values[i] = GpsTimeToLastFix;  // FIX! is always time to fix, now?
                break;
            case '6':
                telen_values[i] = tx_cnt_0;
                break;
            case '7':
                telen_values[i] = atoi(t_sat_count);
                break;
            case '8':
                telen_values[i] = atoi(t_hdop);  // hundredths
                break;
            case '9': { ; }
                // FIX! what are these onewire_values?
                if (onewire_values[_TELEN_config[i]-'6'] > 0)
                    telen_values[i] = onewire_values[_TELEN_config[i] -'6'] * 100;
                else
                    telen_values[i] = 20000 + (-1 * onewire_values[_TELEN_config[i] - '6']) * 100;
                break;
        }
    }
    // will get sent as TELEN #1 (extended Telemetry) (a third packet in the U4B protocol)
    TELEN1_val1 = telen_values[0];
    // max values are 630k and 153k for val and val2
    TELEN1_val2 = telen_values[1];
    // will get sent as TELEN #2 (extended Telemetry) (a 4th packet in the U4B protocol)
    TELEN2_val1 = telen_values[2];
    // max values are 630k and 153k for val and val2
    TELEN2_val2 = telen_values[3];
    V1_println(F("process_TELEN_data END"));
}
//****************************************************
// isFloat tells you the string is float not string
// all strings are null terminated
void doTelemetrySweepInteger(char *t_string, uint8_t t_length, int t_min, int t_max, int t_inc) {
    if (t_string[0] == 0) {  // empty
        snprintf(t_string, t_length, "%d", t_min);
    }
    int value = atoi(t_string);
    value = value + t_inc;
    // wrap
    if (value > t_max) value = t_min;

    // snprintf: If the resulting string would be longer than n-1 characters,
    // the remaining characters are discarded and not stored,
    // but counted for the value returned by the function.
    // A terminating null character is automatically appended after the content written.
    snprintf(t_string, t_length, "%d", value);
}

//****************************************************
void doTelemetrySweepFloat(char *t_string,
    uint8_t t_length, float t_min, float t_max, float t_inc) {

    float value;
    if (t_string[0] == 0) {  // empty
        value = t_min;
    } else {
        value = atof(t_string);
        value = value + t_inc;
        // wrap
        if (value > t_max) value = t_min;
    }

    // snprintf: If the resulting string would be longer than n-1 characters,
    // the remaining characters are discarded and not stored,
    // but counted for the value returned by the function.
    // A terminating null character is automatically appended after the content written.
    if (t_length == 7)
        snprintf(t_string, t_length, "%.2f", value);
    else if (t_length == 6)
        snprintf(t_string, t_length, "%.1f", value);
    else
        snprintf(t_string, t_length, "%.2f", value);
}

//****************************************************
void telemetrySweepAllForTest(void) {
    V1_println(F("telemetrySweepAllForTest START"));

    // Force these to static values. used during normal callsign tx
    snprintf(t_grid6, sizeof(t_grid6), "%s", "AA00AA");
    snprintf(t_callsign, sizeof(t_callsign), "%s", "TES999");
    snprintf(t_power, sizeof(t_power), "%d", 0);

    // the t_* stuff are ascii chars
    // walk them thru their range. wrap at boundary
    doTelemetrySweepInteger(t_course, 3, 0, 361, 1);            // 3 bytes (not counting null term)
    doTelemetrySweepInteger(t_speed, 3, 0, 300, 1);             // 3 bytes
    doTelemetrySweepInteger(t_altitude, 6, 0, 99999, 1);        // 6 bytes
    doTelemetrySweepInteger(t_tx_count_0, 3, 0, 999, 1);        // 3 bytes
    doTelemetrySweepFloat(t_temp, 6, -100.1, 200.4, 1.0);       // 6 bytes (float)
    doTelemetrySweepFloat(t_pressure, 7, -20.10, 100.10, 1.0);  // 7 bytes (float)
    doTelemetrySweepFloat(t_temp_ext, 7, -50.12, 200.45, 1.0);  // 7 bytes (float)
    doTelemetrySweepFloat(t_humidity, 7, -50.12, 200.45, 1.0);  // 7 bytes (float)
    doTelemetrySweepFloat(t_voltage, 5, 0,  6.00, 6.0);         // 5 bytes (float)
    doTelemetrySweepInteger(t_sat_count, 2, 0, 99, 1);          // 2 bytes
    doTelemetrySweepInteger(t_hdop, 3, 0, 999, 1);              // 3 bytes

    // not sure of what the range is (max for the integer allowed?)
    static char telen1_str1[25] = { 0 };
    static char telen1_str2[25] = { 0 };
    static char telen2_str1[25] = { 0 };
    static char telen2_str2[25] = { 0 };

    doTelemetrySweepInteger(telen1_str1, 24, 0, 999, 1);  // 3 bytes
    doTelemetrySweepInteger(telen1_str2, 24, 0, 999, 1);  // 3 bytes
    doTelemetrySweepInteger(telen2_str1, 24, 0, 999, 1);  // 3 bytes
    doTelemetrySweepInteger(telen2_str2, 24, 0, 999, 1);  // 3 bytes

    t_TELEN1_val1 = atoi(telen1_str1);  // int
    t_TELEN1_val2 = atoi(telen1_str2);  // int
    t_TELEN2_val1 = atoi(telen2_str1);  // int
    t_TELEN2_val2 = atoi(telen2_str2);  // int

    V1_printf("TESTMODE t_course: %s" EOL, t_course);
    V1_printf("TESTMODE t_speed: %s" EOL, t_speed);
    V1_printf("TESTMODE t_altitude: %s" EOL, t_altitude);
    V1_printf("TESTMODE t_tx_count_0: %s" EOL, t_tx_count_0);
    V1_printf("TESTMODE t_temp: %s" EOL, t_temp);
    V1_printf("TESTMODE t_pressure: %s" EOL, t_pressure);
    V1_printf("TESTMODE t_temp_ext: %s" EOL, t_temp_ext);
    V1_printf("TESTMODE t_humidity: %s" EOL, t_humidity);
    V1_printf("TESTMODE t_voltage: %s" EOL, t_voltage);
    V1_printf("TESTMODE t_sat_count: %s" EOL, t_sat_count);
    V1_printf("TESTMODE t_hdop: %s" EOL, t_hdop);

    V1_printf("TESTMODE t_TELEN1_val1: %x" EOL, t_TELEN1_val1);
    V1_printf("TESTMODE t_TELEN1_val2: %x" EOL, t_TELEN1_val2);
    V1_printf("TESTMODE t_TELEN2_val1: %x" EOL, t_TELEN2_val1);
    V1_printf("TESTMODE t_TELEN2_val2: %x" EOL, t_TELEN2_val2);

    V1_println(F("telemetrySweepAllForTest END"));
}
//****************************************************
