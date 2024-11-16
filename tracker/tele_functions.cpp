// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <stdlib.h>
#include "config_functions.h"
#include "bmp_functions.h"
#include "mh_functions.h"
#include "adc_functions.h"
#include "defines.h"

#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus

extern const int BattPin;
extern uint64_t GpsTimeToLastFix; // milliseconds

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
extern bool DEVMODE;
extern int tx_cnt_0;

//****************************************************

int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
int legalPowerSize = 19;

//****************************************************
void snapForTelemetry(void) {
    if (DEVMODE) Serial.println(F("snapForTelemetry START"));
    // FIX! didn't we already check this?
    // FIX! why does isUpdated() get us past here?
    if (!gps.location.isValid()) return;
    else if (gps.location.age() >= 1000 && !gps.location.isUpdated()) return;
    else if (gps.satellites.value() <= 3) return;

    int course = gps.course.isValid() ? gps.course.deg() : 0;
    if (course < 0)   course = 0;
    if (course > 360) course = 360;
    snprintf(t_course, sizeof(t_course), "%3d", course);

    int speed = gps.speed.isValid() ? gps.speed.knots() : 0;
    if (speed < 0)   speed = 0;
    if (speed > 999) speed = 999;
    snprintf(t_speed, sizeof(t_speed), "%3d", speed);

    // fixing negative altitude values causing display bug on aprs.fi
    int altitude = (int) gps.altitude.meters();
    if (altitude < 0) altitude = 0;
    if (altitude > 999999) altitude = 999999;
    snprintf(t_altitude, sizeof(t_altitude), "%6d", altitude);

    const float conversionFactor = 3.3f / (1 << 12); //read temperature
    int adc_val = 0;
    // FIX! is this the right gpio for temp
    adc_val += analogRead(4);
    adc_val += analogRead(4);
    adc_val += analogRead(4);
    float adc = (adc_val * conversionFactor) / 3;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    // turn floats into strings
    // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string);
    if (tempC < -999.9) tempC = -999.9;
    if (tempC > 999.9) tempC = 999.9;
    snprintf(t_temp, sizeof(t_temp), "%6.1f", tempC);

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

    // get_mh_6 returns a pointer
    char *grid6;  // null term
    // FIX! are lat/lon double
    grid6 = get_mh_6(gps.location.lat(), gps.location.lng());
    // two letters, two digits, two letters
    // base 18, base 18, base 10, base 10, base 24, base 24
    // [A-R][A-R][0-9][0-9][A-X][A-X]
    // I guess clamp to AA00AA if illegal? (easy to find errors?)
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
    // just for consistency with everything else
    snprintf(t_callsign, sizeof(t_callsign), "%6s", _callsign);

    // string literals are null terminated
    char power[3] = "3";
    if (_tx_high[0] == '1')
        snprintf(power, sizeof(power), "%1s", "5");
    // we clamp to a legalPower when we snapForTelemetry()
    // basically we look at _tx_high[0] to decide our power level that will be used for rf
    // we could use values that are unique for this tracker,
    // for easy differentiation from u4b/traquito!!
    // like 3 and 7!

    // validity check the power. for 'same as everything else' checking
    bool found = false;
    int power_int = atoi(power);
    for (int i = 0; i < legalPowerSize; i++) {
        if (legalPower[i] == power_int) {
            found = true;
            break;
        }
    }
    if (!found) power_int = 0;
    snprintf(t_power, sizeof(t_power), "%2d", power_int);

    int tx_cnt_0_val = tx_cnt_0;
    if (tx_cnt_0 < 0) tx_cnt_0_val = 0;
    if (tx_cnt_0 > 99) tx_cnt_0_val = 99;
    snprintf(t_tx_count_0, sizeof(t_tx_count_0), "%3d", tx_cnt_0_val);

    if (DEVMODE) {
        Serial.printf("t_************" EOL);
        Serial.printf("t_tx_count_0 %3s " EOL, t_tx_count_0);
        Serial.printf("t_callsign %6s" EOL, t_callsign);
        Serial.printf("t_grid6 %6s" EOL, t_grid6);
        Serial.printf("t_power %2s" EOL, t_power);
        Serial.printf("t_sat_count %2s " EOL, t_sat_count);
        Serial.printf("t_lat %12s " EOL, t_lat);
        Serial.printf("t_lon %12s " EOL, t_lon);
        Serial.printf("t_altitude %6s " EOL, t_altitude);
        Serial.printf("t_voltage %5s " EOL, t_voltage);
        Serial.printf("t_temp %6s" EOL, t_temp);
        Serial.printf("t_course %3s " EOL, t_course);
        Serial.printf("t_speed %3s " EOL, t_speed);
        Serial.printf("t_temp_ext %7s" EOL, t_temp);
        Serial.printf("t_pressure %7s " EOL, t_pressure);
        Serial.printf("t_************" EOL);
    }
    if (DEVMODE) Serial.println(F("snapForTelemetry END"));
}


//****************************************************
static float onewire_values[10] = { 0 };

void process_TELEN_data(void) {
    if (DEVMODE) Serial.println(F("process_TELEN_data START"));
    // FIX! where do these come from
    // minutes_since_boot
    // minutes_since_GPS_acquistion (should this be last time to fix);
    // we don't send stuff out if we don't get gps acquistion. so minutes since fix doesn't really matter?
    // 3.3 * 1000. the 3.3 is from vref,
    // the 1000 is to convert to mV.
    // the 12 bit shift is because thats resolution of ADC
    const float conversionFactor = 3300.0f / (1 << 12);

    int telen_values[4] = { 0 };
    uint32_t timeSinceBoot_secs = millis() / 1000UL;  // seconds
    for (int i=0; i < 4;i++) {
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
                telen_values[i] = timeSinceBoot_secs; break;  // seconds since running
                break;
            case '5': telen_values[i] = GpsTimeToLastFix; break;
            case '6': { ; }
            case '7': { ; }
            case '8': { ; }
            case '9': { ; }
                if (onewire_values[_TELEN_config[i]-'6'] > 0)
                    telen_values[i] = onewire_values[_TELEN_config[i] -'6'] * 100;
                else
                    telen_values[i] = 20000 + (-1 * onewire_values[_TELEN_config[i] - '6']) * 100;
                break;
        }
    }
    // onewire_values
    // will get sent as TELEN #1 (extended Telemetry) (a third packet in the U4B protocol)
    TELEN1_val1 = telen_values[0];
    // max values are 630k and 153k for val and val2
    TELEN1_val2 = telen_values[1];
    // will get sent as TELEN #2 (extended Telemetry) (a 4th packet in the U4B protocol)
    TELEN2_val1 = telen_values[2];
    // max values are 630k and 153k for val and val2
    TELEN2_val2 = telen_values[3];
    if (DEVMODE) Serial.println(F("process_TELEN_data END"));
}
