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
#include "global_structs.h"

// to calc solar elevations
// first one I did
#include "solar_functions.h"
// fast algo
#include "solar2_functions.h"
// most accurate algo
#include "solar4_functions.h"
// suncal
// #include "solar5_functions.h"

bool SPEED_IS_SOLAR_ELEVATION_MODE = true;

#include <TinyGPS++.h>  // https://github.com/mikalhart/TinyGPSPlus
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

extern uint64_t GpsTimeToLastFix;  // milliseconds

extern bool BALLOON_MODE;
extern bool TESTMODE;

extern bool GpsInvalidAll;
extern uint64_t GpsTimeToLastFix;  // milliseconds
extern uint64_t GpsTimeToLastFixMin;
extern uint64_t GpsTimeToLastFixMax;
extern uint64_t GpsTimeToLastFixAvg;

extern TinyGPSCustom gp_sats;
extern TinyGPSCustom gb_sats;
extern TinyGPSCustom gl_sats;
extern TinyGPSCustom ga_sats;
// FIX! do we only get GNGSA ?? ?? don't need?
extern TinyGPSCustom gp_pdop;
extern TinyGPSCustom gp_hdop;
extern TinyGPSCustom gp_vdop;


extern int TELEN1_val1;
extern int TELEN1_val2;
extern int TELEN2_val1;
extern int TELEN2_val2;

// FIX! update this based on solar elevation
extern uint8_t SOLAR_SI5351_TX_POWER;

extern TeleStruct tt;
extern ConfigStruct cc;
extern TinyGPSPlus gps;

extern int tx_cnt_0;
// decode of _verbose 0-9
extern bool VERBY[10];

//****************************************************
int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60};
int legalPowerSize = 19;

uint64_t clamp_uint64_t(uint64_t v, uint64_t minv, uint64_t maxv) {
    return min(maxv, max(minv, v));
}
uint32_t clamp_uint32_t(uint32_t v, uint32_t minv, uint32_t maxv) {
    return min(maxv, max(minv, v));
}
uint8_t clamp_uint8_t(uint8_t v, uint8_t minv, uint8_t maxv) {
    return min(maxv, max(minv, v));
}
int clamp_int(int v, int minv, int maxv) {
    return min(maxv, max(minv, v));
}
float clamp_float(float v, float minv, float maxv) {
    return min(maxv, max(minv, v));
}
double clamp_double(double v, double minv, double maxv) {
    return min(maxv, max(minv, v));
}

//****************************************************
// https://www.google.com/search?q=c+modify+elements+of+struct+passed+to+function&oq=c+modify+elements+of+struct+passed+to+function&gs_lcrp=EgZjaHJvbWUyBggAEEUYOdIBCDg1NTlqMGo3qAIAsAIA&sourceid=chrome&ie=UTF-8
void snapForTelemetry() {
    V1_println(F("snapForTelemetry START"));
    Watchdog.reset();
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
    course = clamp_int(course, 0, 360);
    snprintf(tt.course, sizeof(tt.course), "%d", course);

    //****************************************************
    // returns degrees, not radians. I guess it's decimal integer here
    double solarElevation; // can this be negative?. decimal, integer accuracy
    double solarAzimuth; // can this be negative?. decimal, integer accuracy
    double solarDistance; // can this be negative? maybe error case. kilometers.

    // always call, so we get prints we can use for debug, even if not used.
    // just in case this might blow up in flight (fp error?)
    if (!BALLOON_MODE) {
        // first one I did
        // https://github.com/KenWillmott/SolarPosition/blob/master/SolarPosition.cpp

        // not working? don't use for now
        // calcSolarElevation(&solarElevation, &solarAzimuth, &solarDistance);
        // V1_printf("Elevation solarElevation %.7f solarAzimuth %.7f solarDistance %.1f" EOL,
        //     solarElevation, solarAzimuth, solarDistance);

        // suncalc.js translation to cpp
        // Not working? need to debug
        V1_print(F(EOL));
        if (false) {
            // not working?
            // calcSolarElevation5(&solarElevation, &solarAzimuth, &solarDistance);
            // V1_printf("Elevation5 solarElevation %.7f solarAzimuth %.7f solarDistance %.1f" EOL,
            //     solarElevation, solarAzimuth, solarDistance);
        }

        // fast algo
        calcSolarElevation2(&solarElevation, &solarAzimuth, &solarDistance);
        V1_printf("Elevation2 solarElevation %.7f solarAzimuth %.7f solarDistance %.1f" EOL,
            solarElevation, solarAzimuth, solarDistance);
    }
    // accurate algo
    calcSolarElevation4(&solarElevation, &solarAzimuth, &solarDistance);

    V1_printf("Elevation4 solarElevation %.7f solarAzimuth %.7f solarDistance %.1f" EOL,
        solarElevation, solarAzimuth, solarDistance);
    V1_print(F(EOL));


    solarElevationCalcs(solarElevation);

    solarElevation = clamp_double(solarElevation, -90.0, 90.0);
    solarAzimuth = clamp_double(solarAzimuth, -360, 360);
    solarDistance = clamp_double(solarDistance, 0, 180000);

    // -90.0 to 90.0?
    snprintf(tt.solarElevation, sizeof(tt.solarElevation), "%.1f", solarElevation);
    // -180.0 to 180.0
    snprintf(tt.solarAzimuth, sizeof(tt.solarAzimuth), "%.1f", solarAzimuth);
    // 145 to 150 (km) ?
    snprintf(tt.solarDistance, sizeof(tt.solarDistance), "%.0f", solarDistance);

    int speed;
    if (SPEED_IS_SOLAR_ELEVATION_MODE)
        speed = solarElevation; // 0 if bad?
    else
        speed = gps.speed.isValid() ? gps.speed.knots() : 0;

    //****************************************************

    // FIX! I guess if SPEED_IS_SOLAR_ELEVATION_MODE
    // this means any negative solar elevation angles will clamp at 0
    // only an issue if someone is doing vertical solar arrays?
    speed = clamp_int(speed, 0, 999);
    snprintf(tt.speed, sizeof(tt.speed), "%d", speed);

    // fixing negative altitude values
    int altitude = (int) gps.altitude.meters();
    altitude = clamp_int(altitude, 0, 999999);
    snprintf(tt.altitude, sizeof(tt.altitude), "%d", altitude);

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
    // dtostrf(floatt.value, min_width, num_digits_after_decimal, where_to_store_string);

    tempC = clamp_float(tempC, -999.9, 999.9);
    snprintf(tt.temp, sizeof(tt.temp), "%.1f", tempC);

    //*********************************
    // examples
    // 1 hPA = 100 PA
    // 11 km (36,000 ft): 226 hPa
    // 20 km (65,000 ft): 54.7 hPa
    // 32 km (105,000 ft): 8.68 hPa
    // FIX! do we read hPA

    float pressure = bmp_read_pressure();
    pressure = clamp_float(pressure, -999.9, 999.9);
    snprintf(tt.pressure, sizeof(tt.pressure), "%.2f", pressure);

    float temp_ext = bmp_read_temperature();
    temp_ext = clamp_float(temp_ext, -999.9, 999.9);
    snprintf(tt.temp_ext, sizeof(tt.temp_ext), "%.2f", temp_ext);

    float humidity = bmp_read_humidity();
    humidity = clamp_float(humidity, -999.9, 999.9);
    snprintf(tt.humidity, sizeof(tt.humidity), "%.2f", humidity);

    float voltage = readVoltage();
    voltage = clamp_float(voltage, 0, 99.99);
    snprintf(tt.voltage, sizeof(tt.voltage), "%.2f", voltage);

    // FIX! could use this for some TELEN telemetry?
    int hdop = gps.hdop.isValid() ? (int) gps.hdop.value() : 0;
    // hundredths. <100 is very good
    // can get >999 from the gps, but we don't tx it (usually?)
    hdop = clamp_int(hdop, 0, 999);
    snprintf(tt.hdop, sizeof(tt.hdop), "%d", hdop);

    // FIX! could use this for some TELEN telemetry?
    int sat_count = gps.satellites.isValid() ? (int) gps.satellites.value() : 0;
    sat_count = clamp_int(sat_count, 0, 99);
    snprintf(tt.sat_count, sizeof(tt.sat_count), "%d", sat_count);

    double lat = gps.location.lat();
    // FIX! is both 90 and -90 legal for maidenhead translate?
    lat = clamp_double(lat, -90.0000000, 90.0000000);
    // 12 bytes max with - and . counted
    snprintf(tt.lat, sizeof(tt.lat), "%.7f", lat);

    double lon = gps.location.lng();
    // FIX! is both 180 and -180 legal for maidenhead translate?
    lon = clamp_double(lon, -180.0000000, 180.0000000);
    snprintf(tt.lon, sizeof(tt.lon), "%.7f", lon);

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

    snprintf(tt.grid6, sizeof(tt.grid6), "%s", grid6);

    //*********************************
    // snap callsign just for consistency with everything else
    // we should never tx callsign or telemetry if we didn't get a fix
    // so okay if tt.callsign is blank until we get a fix?
    snprintf(tt.callsign, sizeof(tt.callsign), "%s", cc._callsign);

    //*********************************
    int power_int;
    if (cc._solar_tx_power[0] == '1') {
        switch (SOLAR_SI5351_TX_POWER)  {
            case 0: power_int = 3; break;
            case 1: power_int = 7; break;
            case 2: power_int = 10; break;
            case 3: power_int = 13; break;
            default: power_int = 17; // so we'll know about illegal case?
        }
    } else {
        if (cc._tx_high[0] == '1') power_int = 13;
        else power_int = 10;
    }

    // we clamp to a legalPower when we snapForTelemetry()
    // basically we look at cc._tx_high[0] to decide our power level that will be used for rf
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
    snprintf(tt.power, sizeof(tt.power), "%d", power_int);

    //*********************************
    int tx_cnt_0_val = tx_cnt_0;
    if (tx_cnt_0 < 0) tx_cnt_0_val = 0;
    // do we need to count more than 99 in a day?
    tx_cnt_0_val = clamp_int(tx_cnt_0_val, 0, 999);
    // we have room for 999
    snprintf(tt.tx_count_0, sizeof(tt.tx_count_0), "%d", tx_cnt_0_val);

    //*********************************
    // snap for consistency with everything else (all at one instant in time)
    tt.TELEN1_val1 = TELEN1_val1;
    tt.TELEN1_val2 = TELEN1_val2;
    tt.TELEN2_val1 = TELEN2_val1;
    tt.TELEN2_val2 = TELEN2_val2;

    bool validA = gps.satellites.isValid() && !GpsInvalidAll;
    // bool validB = gps.hdop.isValid() && !GpsInvalidAll;
    bool validB_gp = gp_sats.isValid() && !GpsInvalidAll;
    bool validB_gb = gb_sats.isValid() && !GpsInvalidAll;
    bool validB_gl = gl_sats.isValid() && !GpsInvalidAll;
    bool validB_ga = ga_sats.isValid() && !GpsInvalidAll;
    // bool validC = gps.location.isValid() && !GpsInvalidAll;
    // bool validD = gps.altitude.isValid() && !GpsInvalidAll;
    uint8_t s;
    // FIX! not range checking these?
    if (validA) s = gps.satellites.value();
    else s = 0;
    s = clamp_uint8_t(s, 0, 99);
    snprintf(tt.sats, sizeof(tt.sats), "%u", s);

    // what if it's blank?
    if (validB_gp) s = atoi(gp_sats.value());
    else s = 0;
    s = clamp_uint8_t(s, 0, 99);
    snprintf(tt.gp_sats, sizeof(tt.gp_sats), "%u", s);

    if (validB_gb) s = atoi(gb_sats.value());
    else s = 0;
    s = clamp_uint8_t(s, 0, 99);
    snprintf(tt.gb_sats, sizeof(tt.gb_sats), "%u", s);

    if (validB_gl) s = atoi(gl_sats.value());
    else s = 0;
    s = clamp_uint8_t(s, 0, 99);
    snprintf(tt.gl_sats, sizeof(tt.gl_sats), "%u", s);

    if (validB_ga) s = atoi(ga_sats.value());
    else s = 0;
    s = clamp_uint8_t(s, 0, 99);
    snprintf(tt.ga_sats, sizeof(tt.ga_sats), "%u", s);
    // FIX! add to all the prints

    // milliseconds
    s = GpsTimeToLastFix / 1000;
    s = clamp_uint64_t(s, 0, 999);
    snprintf(tt.gpsLockSecs,    sizeof(tt.gpsLockSecs),    "%u", s);
    s = GpsTimeToLastFixMin / 1000;
    s = clamp_uint64_t(s, 0, 999);
    snprintf(tt.gpsLockSecsMin, sizeof(tt.gpsLockSecsMin), "%u", s);
    s = GpsTimeToLastFixMax / 1000;
    s = clamp_uint64_t(s, 0, 999);
    snprintf(tt.gpsLockSecsMax, sizeof(tt.gpsLockSecsMax), "%u", s);
    s = GpsTimeToLastFixAvg / 1000;
    s = clamp_uint64_t(s, 0, 999);
    snprintf(tt.gpsLockSecsAvg, sizeof(tt.gpsLockSecsAvg), "%u", s);

    //*********************************
    V1_printf("************" EOL);
    V1_printf("tt.tx_count_0 %s " EOL, tt.tx_count_0);
    V1_printf("tt.callsign %s" EOL, tt.callsign);
    V1_printf("tt.grid6 %s" EOL, tt.grid6);
    V1_printf("tt.power %s" EOL, tt.power);
    V1_printf("tt.sat_count %s " EOL, tt.sat_count);
    V1_printf("tt.lat %s " EOL, tt.lat);
    V1_printf("tt.lon %s " EOL, tt.lon);
    V1_printf("tt.altitude %s " EOL, tt.altitude);
    V1_printf("tt.voltage %s " EOL, tt.voltage);
    V1_printf("tt.temp %s" EOL, tt.temp);
    V1_printf("tt.course %s " EOL, tt.course);
    V1_printf("tt.speed %s " EOL, tt.speed);

    V1_printf("tt.solarElevation %s " EOL, tt.solarElevation);
    V1_printf("tt.solarAzimuth %s " EOL, tt.solarAzimuth);

    V1_printf("tt.temp_ext %s" EOL, tt.temp);
    V1_printf("tt.pressure %s " EOL, tt.pressure);

    // V1_printf("tt.TELEN1_val1 %d " EOL, tt.TELEN1_val1);
    // V1_printf("tt.TELEN1_val2 %d " EOL, tt.TELEN1_val2);
    // V1_printf("tt.TELEN2_val1 %d " EOL, tt.TELEN2_val1);
    // V1_printf("tt.TELEN2_val2 %d " EOL, tt.TELEN2_val2);

    V1_printf("tt.gp_sats %s " EOL, tt.gp_sats);
    V1_printf("tt.gb_sats %s " EOL, tt.gb_sats);
    V1_printf("tt.gl_sats %s " EOL, tt.gl_sats);
    V1_printf("tt.ga_sats %s " EOL, tt.ga_sats);
    V1_printf("tt.gpsLockSecs %s " EOL, tt.gpsLockSecs);
    V1_printf("tt.gpsLockSecsMin %s " EOL, tt.gpsLockSecsMin);
    V1_printf("tt.gpsLockSecsMax %s " EOL, tt.gpsLockSecsMax);
    V1_printf("tt.gpsLockSecsAvg %s " EOL, tt.gpsLockSecsAvg);
    V1_printf("************" EOL);

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
        switch (cc._TELEN_config[i]) {
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
                telen_values[i] = atoi(tt.sat_count);
                break;
            case '8':
                telen_values[i] = atoi(tt.hdop);  // hundredths
                break;
            case '9': { ; }
                // FIX! what are these onewire_values?
                if (onewire_values[cc._TELEN_config[i]-'6'] > 0)
                    telen_values[i] = onewire_values[cc._TELEN_config[i] -'6'] * 100;
                else
                    telen_values[i] = 20000 + (-1 * onewire_values[cc._TELEN_config[i] - '6']) * 100;
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
    snprintf(tt.grid6, sizeof(tt.grid6), "%s", "AA00AA");
    snprintf(tt.callsign, sizeof(tt.callsign), "%s", "TES999");
    snprintf(tt.power, sizeof(tt.power), "%d", 0);

    // the tt.* stuff are ascii chars
    // walk them thru their range. wrap at boundary
    // https://stackoverflow.com/questions/17928123/how-to-pass-a-struct-member-as-a-pointer-in-a-function
    doTelemetrySweepInteger(tt.course, 3, 0, 361, 1);            // 3 bytes (not counting null term)
    doTelemetrySweepInteger(tt.speed, 3, 0, 300, 1);             // 3 bytes
    doTelemetrySweepInteger(tt.altitude, 6, 0, 99999, 1);        // 6 bytes
    doTelemetrySweepInteger(tt.tx_count_0, 3, 0, 999, 1);       // 3 bytes
    doTelemetrySweepFloat(tt.temp, 6, -100.1, 200.4, 1.0);       // 6 bytes (float)
    doTelemetrySweepFloat(tt.pressure, 7, -20.10, 100.10, 1.0);  // 7 bytes (float)
    doTelemetrySweepFloat(tt.temp_ext, 7, -50.12, 200.45, 1.0);  // 7 bytes (float)
    doTelemetrySweepFloat(tt.humidity, 7, -50.12, 200.45, 1.0);  // 7 bytes (float)
    doTelemetrySweepFloat(tt.voltage, 5, 0,  6.00, 6.0);         // 5 bytes (float)
    doTelemetrySweepInteger(tt.sat_count, 2, 0, 99, 1);          // 2 bytes
    doTelemetrySweepInteger(tt.hdop, 3, 0, 999, 1);              // 3 bytes

    // not sure of what the range is (max for the integer allowed?)
    static char telen1_str1[25] = { 0 };
    static char telen1_str2[25] = { 0 };
    static char telen2_str1[25] = { 0 };
    static char telen2_str2[25] = { 0 };

    doTelemetrySweepInteger(telen1_str1, 24, 0, 999, 1);  // 3 bytes
    doTelemetrySweepInteger(telen1_str2, 24, 0, 999, 1);  // 3 bytes
    doTelemetrySweepInteger(telen2_str1, 24, 0, 999, 1);  // 3 bytes
    doTelemetrySweepInteger(telen2_str2, 24, 0, 999, 1);  // 3 bytes

    tt.TELEN1_val1 = atoi(telen1_str1);  // int
    tt.TELEN1_val2 = atoi(telen1_str2);  // int
    tt.TELEN2_val1 = atoi(telen2_str1);  // int
    tt.TELEN2_val2 = atoi(telen2_str2);  // int

    V1_printf("TESTMODE tt.course: %s" EOL, tt.course);
    V1_printf("TESTMODE tt.speed: %s" EOL, tt.speed);
    V1_printf("TESTMODE tt.altitude: %s" EOL, tt.altitude);
    V1_printf("TESTMODE tt.tx_count_0: %s" EOL, tt.tx_count_0);
    V1_printf("TESTMODE tt.temp: %s" EOL, tt.temp);
    V1_printf("TESTMODE tt.pressure: %s" EOL, tt.pressure);
    V1_printf("TESTMODE tt.temp_ext: %s" EOL, tt.temp_ext);
    V1_printf("TESTMODE tt.humidity: %s" EOL, tt.humidity);
    V1_printf("TESTMODE tt.voltage: %s" EOL, tt.voltage);
    V1_printf("TESTMODE tt.sat_count: %s" EOL, tt.sat_count);
    V1_printf("TESTMODE tt.hdop: %s" EOL, tt.hdop);

    V1_printf("TESTMODE tt.TELEN1_val1: %x" EOL, tt.TELEN1_val1);
    V1_printf("TESTMODE tt.TELEN1_val2: %x" EOL, tt.TELEN1_val2);
    V1_printf("TESTMODE tt.TELEN2_val1: %x" EOL, tt.TELEN2_val1);
    V1_printf("TESTMODE tt.TELEN2_val2: %x" EOL, tt.TELEN2_val2);

    V1_println(F("telemetrySweepAllForTest END"));
}

//****************************************************
void solarElevationCalcs(double solarElevation) {
    V1_print(F("solarElevationCalcs START"));
    // Do nothing if the gps data isn't good
    // what about GpsInvalidAll from tracker.ino???

    // just need a good 2d fix?
    bool fix_valid_all =
        gps.time.isValid() &&
        (gps.date.year() >= 2024 && gps.date.year() <= 2034) &&
        gps.satellites.isValid() && (gps.satellites.value() >= 3) &&
        gps.location.isValid();

    if (!fix_valid_all) return;

    // figure how solar peak during the day
    // keep a static for the last value. make it rounded to integer. Compare to current.
    // keep the max found. When the current is less than the max, the first time,
    // say "solarPeakPos" is detected. I guess solarRising/solarSetting
    // the transition from solarRising to solarSetting would be the time for solarPeakPos
    // maybe save the time of the solar max also

    // we wouldn't get here (snapForTelemetry())
    // unless we had qualified the fix as a good 3d fix.
    // so don't need to qualify the gps info
    static int8_t solarElevationInt_prev = 127;
    static int8_t solarElevationIntMax  = 127;
    static int8_t solarElevationIntMin  = 127;

    static double solarElevation_earliest = 127;
    static uint64_t epochTime_earliest = 0;

    bool initialCondition = solarElevation_earliest == 127;

    static bool solarRising_prev = false;
    static bool solarSetting_prev = false;
    static bool solarPeakPos_sticky = false;
    static bool solarPeakNeg_sticky = false;

    uint64_t epochTime = getEpochTime();
    uint64_t flightSecs = 0;
    if (!initialCondition) {
        flightSecs = epochTime - epochTime_earliest;
    }
    V1_printf("flightSecs %" PRIu64 EOL, flightSecs);

    if (initialCondition) {
        solarElevation_earliest = solarElevation;
        V1_printf("GOOD: new solarElevation_earliest: %.1f epochTime %" PRIu64 EOL,
            solarElevation_earliest, epochTime);
    }
    // nearest for pos and neg numbers
    int8_t solarElevationInt = (int8_t) round(solarElevation);

    if (initialCondition || (solarElevationInt < solarElevationIntMin)) {
        solarElevationIntMin = solarElevationInt;
        V1_printf("GOOD: new solarElevationIntMin %d: epochTime %" PRIu64 EOL,
            solarElevationIntMin, epochTime);
    }

    if (initialCondition || (solarElevationInt > solarElevationIntMax)) {
        solarElevationIntMax = solarElevationInt;
        V1_printf("GOOD: new solarElevationIntMax %d: epochTime %" PRIu64 EOL,
            solarElevationIntMax, epochTime);
    }

    // could adjust the tx power output (strength) depending
    // on solar elevation? Makes it dependent on correct gps lat/lon/utc calcs
    // for ground level power testing, wouldn't want to enable dynamic power..
    // separate config bit
    // < 10 deg 2mA
    // 11 to 20 deg 4mA
    // > 20 8mA
    // FIX! update this based on solar elevation
    // extern uint8_t SOLAR_SI5351_TX_POWER;

    uint8_t tx_power = 3;
    if (solarElevationInt < 10) {
        tx_power = 0;
    } else if (solarElevationInt < 20) {
        tx_power = 1;
    } else {
        tx_power = 3;
    }

    if (tx_power != SOLAR_SI5351_TX_POWER) {
        SOLAR_SI5351_TX_POWER = tx_power;
        V1_printf("GOOD: new SOLAR_SI5351_TX_POWER %u solarElevationInt %d epochTime %" PRIu64 EOL,
            SOLAR_SI5351_TX_POWER, solarElevationInt, epochTime);
    }

    // FIX! could send a peak speed number (82 knots) at the solarPeakPos, to mark it
    // in the telemetry!
    bool solarPeakPos = false;
    bool solarPeakNeg = false;

    bool solarRising = false;
    bool solarSetting = false;
    bool solarSame = false;

    if (initialCondition) {
        solarPeakPos = false;
        solarPeakNeg = false;
        solarRising = true;
        solarSetting = false;
        solarSame = false;
    } else {
        // there must be someplace where we hit the int transition
        // solarSame = solarElevationInt == solarElevationInt_prev;
        solarRising  = solarElevationInt > solarElevationInt_prev;
        solarSetting = solarElevationInt < solarElevationInt_prev;
        solarSame = solarElevationInt == solarElevationInt_prev;
        // should just happen once!
        solarPeakPos = solarSetting && solarRising_prev;
        solarPeakNeg = solarRising && solarSetting_prev;
        V1_printf("solarRising %u solarSetting %u solarSame %u %" PRIu64 EOL,
            solarRising, solarSetting, solarSame, epochTime);

        if (solarPeakPos) {
            // should only have one of these?
            V1_printf("GOOD: solarPeakPos discovered:"
                " solarElevation %.1f epochTime %" PRIu64 EOL,
                solarElevation, epochTime);
        }
        if (solarPeakPos && solarPeakPos_sticky) {
            // should only have one of these?
            V1_printf("ERROR: multiple solarPeakPos discovered/set:"
                " solarElevation %.1f epochTime %" PRIu64 EOL,
                solarElevation, epochTime);
        }
        if (solarPeakNeg) {
            // should only have one of these?
            V1_printf("GOOD: solarPeakNeg discovered:"
                " solarElevation %.1f epochTime %" PRIu64 EOL,
                solarElevation, epochTime);
        }
        if (solarPeakNeg && solarPeakNeg_sticky) {
            // should only have one of these?
            V1_printf("ERROR: multiple solarPeakNeg discovered/set:"
                " solarElevation %.1f epochTime %" PRIu64 EOL,
                solarElevation, epochTime);
        }
    }

    solarElevationInt_prev = solarElevationInt;
    solarRising_prev = solarRising;
    solarSetting_prev = solarSetting;
    // sticky!
    solarPeakPos_sticky |= solarPeakPos;
    solarPeakNeg_sticky |= solarPeakNeg;

    V1_print(F("solarElevationCalcs END"));
}
