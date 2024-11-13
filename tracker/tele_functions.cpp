// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>

#include <stdlib.h>

extern char t_course[4];      // 3 bytes
extern char t_speed[4];       // 3 bytes
extern char t_altitude[7];    // 6 bytes
extern char t_tx_count_0[4];  // 3 bytes
extern char t_temp[7];        // 6 bytes
extern char t_pressure[8];    // 7 bytes
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

// clamped to 0 if not in this list of legal
// legalPower = [0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60]
extern char t_power[3];       // 2 bytes

extern int TELEN1_val1;
extern int TELEN1_val2;
extern int TELEN2_val1;
extern int TELEN2_val2;

#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus


extern TinyGPSPlus gps;
extern bool DEVMODE;

int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
int legalPowerSize = 19;
void snapTelemetry() {
    // FIX! didn't we already check this?
    // FIX! why does isUpdated() get us past here?
    if (!gps.location.isValid()) return
    else if (gps.location.age() >= 1000 && !gps.location.isUpdated()) return
    else if (gps.satellites.value() <= 3) return

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

    // FIX! get temp from rp2040??
    float tempC = 0
    // turn floats into strings
    // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
    if (tempC < -999.9) tempC = -999.9;
    if (tempC > 999.9) tempC = 999.9;
    snprintf(t_temp, sizeof(t_temp), "%6.1f", tempC)

    // examples
    // 1 hPA = 100 PA
    // 11 km (36,000 ft): 226 hPa
    // 20 km (65,000 ft): 54.7 hPa
    // 32 km (105,000 ft): 8.68 hPa
    // FIX! do we read hPA
    float pressure = bmp_read_pressure();
    if (pressure < 0) pressure = 0;
    if (pressure > 999.999) pressure = 999.999;
    snprintf(t_pressure, sizeof(t_pressure), "%7.2f", pressure)

    float voltage = battRead();
    if (voltage < 0) voltage = 0;
    if (voltage > 99.99) voltage = 99.99;
    snprintf(t_voltage, sizeof(t_voltage), "%5.2f", voltage)

    sat_count = gps.satellites.isValid() ? (int) gps.satellites.value() : 0;
    if (sat_count < 0) sat_count = 0;
    if (sat_count > 99) sat_count = 99;
    snprintf(t_sat, sizeof(t_sat)_count, "%2d", sat_count)

    double lat = gps.location.lat();
    // FIX is both 90 and -90 legal for maidenhead translate?
    if (lat < -90) lat = -90;
    if (lat > 90) lat = 90;
    // 12 bytes max with - and . counted
    snprintf(t_lat, sizeof(t_lat), "%12.7f", lat)

    double lon = gps.location.lon()
    // FIX is both 180 and -180 legal for maidenhead translate?
    if (lon < -180 lon = -180;
    if (lon > 180 lon = 180;
    snprintf(t_lon, sizeof(t_lon), "%12.7f", lon)

    char grid6[7];  // null term
    // FIX! are lat/lon double
    grid6 = get_mh_6(gps.location.lat(), gps.location.lon()
    // two letters, two digits, two letters
    // base 18, base 18, base 10, base 10, base 24, base 24
    // [A-R][A-R][0-9][0-9][A-X][A-X]
    // I guess clamp to AA00AA if illegal? (easy to find errors?)
    bool bad_grid = false;
    if (grid6[0] < "A" || grid6[0] > "R"]) bad_grid = true;
    if (grid6[1] < "A" || grid6[1] > "R"]) bad_grid = true;
    if (grid6[2] < "0" || grid6[2] > "9"]) bad_grid = true;
    if (grid6[3] < "0" || grid6[3] > "9"]) bad_grid = true;
    if (grid6[4] < "A" || grid6[4] > "X"]) bad_grid = true;
    if (grid6[5] < "A" || grid6[5] > "X"]) bad_grid = true;
    if (bad_grid) grid6 = "AA00AA";
    snprintf(t_grid6, sizeof(t_grid6), "%6s", grid6)

    // string literals are null terminated
    char power[3] = "3";
    if (_tx_high[0] == '1') power = "5";
    // we clamp to a legalPower when we snapTelemetry()
    // basically we look at _tx_high[0] to decide our power level that will be used for rf
    // we could use values that are unique for this tracker,
    // for easy differentiation from u4b/traquito!!
    // like 3 and 7!

    // validity check the power. for 'same as everything else' checking
    bool found = false;
    int power_int = atoi(power);
    for (int i = 0; i < size; i++) {
        if (legalPower[i] == power_int) {
            found = true;
            break;
        }
    }
    if (!found) power = "0";

    snprintf(t_power, sizeof(t_power), "%2d", power);

    if (DEVMODE) {
        printf("t_* ");
        printf("course %3d ", t_course);
        printf("speed %3d ", t_speed);
        printf("altitude %6d ", t_altitude);
        printf("tx_count_0 %3d ", t_tx_count_0);
        printf("temp %6.1f ", t_temp);
        printf("pressure %7.3f ", t_pressure);
        printf("voltage %2.2f ", t_voltage);
        printf("sat_count %2d ", t_sat_count);
        printf("lat %12.7 ", t_lat);
        printf("lon %12.7 ", t_lon);
        printf("grid6 %6s", t_grid6);
        printf("power %2d\n", t_power):
    }
}

//****************************************************
float readVoltage() {
    int adc_val = 0;
    adc_val += analogRead(BattPin);
    adc_val += analogRead(BattPin);
    adc_val += analogRead(BattPin);

    // The Raspberry Pi Pico's analog to digital converter (ADC)
    // can measure voltages between 0 and 3.3 volts.
    // The ADC uses a 3.3V reference voltage,
    // and a read operation returns a number between 0 and 4095.
    // The ADC's resolution is 3.3/4096, or roughly 0.8 millivolts.
    // is the precision set to 4096? (12 not 16 bits resolution)
    // 4096/3.3 = 1241
    // 1241 / 3 = 413.66

    // FIX! this doesn't seem right. should I just multiply by the conversion factor
    // he's got some special 1/3 voltage divider for VBUS to BATT_V
    // you leave it open than the ADC converter voltage reference is the 3.3V .
    // In reality it is the voltage of the pin 3V3
    // - ( ~150uA * 200) which is roughly a 30mv drop. (0.8mv * 30 = 24 steps)

    // this must be a calibrated linear equation? only need to calibrate between 2.8v and 5v?
    float solar_voltage = ((float)adc_val / 3.0f - 27.0f) / 412.0f;

    // there is a 200 ohm resistor between 3V3 and ADC_AVDD
    // we did 3 reads above ..averaging?
    // so don't need the 3x because of onboard voltage divider
    // pico-WSPRer does this (no use of ADC_AVDD) ?
    // const float conversionFactor = 3.3f / (1 << 12);
    // float solar_voltage = 3 * (float)adc_read() * conversionFactor;

    // if (solar_voltage < 0.0f) solar_voltage = 0.0f;
    // if (solar_voltage > 9.9f) solar_voltage = 9.9f;
    return solar_voltage;
}

static float onewire_values[10] = { 0 };

//****************************************************
void process_TELEN_data(void) {
    // FIX! where do these come from
    // minutes_since_boot
    // minutes_since_GPS_acquistion (should this be last time to fix);
    // we don't send stuff out if we don't get gps acquistion. so minutes since fix doesn't really matter?
    // 3.3 * 1000. the 3.3 is from vref,
    // the 1000 is to convert to mV.
    // the 12 bit shift is because thats resolution of ADC
    const float conversionFactor = 3300.0f / (1 << 12);

    for (int i=0; i < 4;i++) {
        switch (_TELEN_config[i]) {
            case '-':  break;  // do nothing, telen chan is disabled
            case '0':
                adc_select_input(0);
                telen_values[i] = round((float)adc_read() * conversionFactor);
                break;
            case '1':
                adc_select_input(1);
                telen_values[i] = round((float)adc_read() * conversionFactor);
                break;
            case '2':
                adc_select_input(2);
                telen_values[i] = round((float)adc_read() * conversionFactor);
                break;
            case '3':
                adc_select_input(3);
                // ADC3 is hardwired to Battery via 3:1 voltage divider: make the conversion here
                telen_values[i] = round((float)adc_read() * conversionFactor * 3.0f);
                break;
            case '4': telen_values[i] = minutes_since_boot; break;
            case '5': telen_values[i] = minutes_since_GPS_aquisition break;
            case '6':
            case '7':
            case '8':
            case '9':
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
}
