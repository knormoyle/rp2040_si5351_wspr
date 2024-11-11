
// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

// Incorporates work by: Roman Piksaykin R2BDY. Thank you.
// https://github.com/RPiks/pico-WSPR-tx

// always positive. clamp to 0 I guess
extern char t_course[4];     // 3 bytes, starts at 0
// always positive? 0-250 knots. clamp to 0 I guess
extern char t_speed[4];      // 3 bytes, starts at 4
// 60000 meters. plus 1 in case negative?
extern char t_altitude[7];   // 6 bytes, starts at 10 .. guaranteed positive
// 24 * 30 per hour = 720 per day if every two minutes 
// reboot once per day? (starts at 0)
extern char t_tx_count_0[4]; // 3 bytes starts at 17 ...stored tx_count_0 + 1
extern char t_temp[7];       // 6 bytes starts at 24 (float)
extern char t_pressure[8];   // 7 bytes starts at 31(float) Pa
extern char t_voltage[6];    // 5 bytes starts at 43
extern char t_sat_count[3];  // 2 bytes starts at 49

// lat/lon precision: How much to store
// https://stackoverflow.com/questions/1947481/how-many-significant-digits-should-i-store-in-my-database-for-a-gps-coordinate
// 6 decimal places represent accuracy for ~ 10 cm
// 7 decimal places for ~ 1 cm
// The use of 6 digits should be enough. +/- is 1 more. decimal is one more. 0-180 is 3 more. 
// so 7 + 5 = 12 bytes should enough, with 1 more rounding digit?
extern char t_lat[12];       // 12 bytes starts at 52
extern char t_lon[12];       // 12 bytes starts at 64
extern char t_grid6[7];      // 6 bytes starts at 64

void updateTelemetryBuff() {
    // FIX! why does isUpdated() get us past here?
    if (! (gps.location.isValid() && (gps.location.age() < 1000 || gps.location.isUpdated())) ) {
        return
    }
    if (! (gps.satellites.isValid() && gps.satellites.value() > 3)) {
        return
    }

    int deg = gps.course.isValid() gps.course.deg() : 0;
    if (deg < 0)   deg = 0;
    if (deg > 999) deg = 999;
    sprintf(t_deg, "%3d", deg);

    int speed = gps.speed.isValid() gps.speed.knots() : 0;
    if (speed < 0)   deg = 0;
    if (speed > 999) deg = 999;
    sprintf(t_speed, "%3d", speed);

    //fixing negative altitude values causing display bug on aprs.fi
    float altitude = gps.altitude.feet();
    // negative
    if (altitude<0) altitude = 0;

    // int will floor the float
    sprintf(t_altitude, "%6d", (int) altitude);

    // FIX! get temp from rp2040??
    float tempC = 0;
    // turn floats into strings
    // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
    if (tempC < 0) tempC = 0;
    // FIX! okay to just have integer temp
    sprintf(t_temp, "%3d", tempC)

    float pressure = bmp_read_pressure() / 100.0; //Pa to hPa
    sprintf(t_pressure, "%7.2f", pressure)
    sprintf(t_voltage, "%5.2f", readBatt())
    
    sat_count = gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
    sprint(t_sat_count, "%2d", sat_count)

    float lat = gps.location.lat();
    // 12 bytes max with - and . counted
    sprint(t_lat, "%12.7f", lat)
    float lon = gps.location.lon()
    sprint(t_lon, "%12.7f", lon)

    char grid6[7]; // null term
    // FIX! are lat/lon double
    grid6 = get_mh_6(gps.location.lat(), gps.location.lon()
    sprint(t_grid6, "%6s", grid6)
    
    if (DEVMODE) Serial.println(telemetry_buff);

}

//****************************************************
float readBatt() {
    int adc_val = 0;
    adc_val = analogRead(BattPin);
    adc_val += analogRead(BattPin);
    adc_val += analogRead(BattPin);

    // The Raspberry Pi Pico's analog to digital converter (ADC) can measure voltages between 0 and 3.3 volts.
    // The ADC uses a 3.3V reference voltage,
    // and a read operation returns a number between 0 and 4095.
    // The ADC's resolution is 3.3/4096, or roughly 0.8 millivolts.
    // is the precision set to 4096? (12 not 16 bits resolution)
    // 4096/3.3 = 1241
    // 1241 / 3 = 413.66

    // FIX! this doesn't seem right. should I just multiply by the conversion factor
    // he's got some special 1/3 voltage divider for VBUS to BATT_V
    // you leave it open than the ADC converter voltage reference is the 3.3V .
    // In reality it is the voltage of the pin 3V3 - ( ~150uA * 200) which is roughly a 30mv drop. (0.8mv * 30 = 24 steps)

    // this must be a calibrated linear equation? only need to calibrate between 2.8v and 5v?
    float solar_voltage = ((float)adc_val / 3.0f - 27.0f) / 412.0f;
    // there is a 200 ohm resistor between 3V3 and ADC_AVDD
    // we did 3 reads above ..averaging? so don't need the 3x because of onboard voltage divider
    // pico-WSPRer does this (no use of ADC_AVDD) ?
    // const float conversionFactor = 3.3f / (1 << 12);
    // float solar_voltage = 3 * (float)adc_read() * conversionFactor;

    // if (solar_voltage < 0.0f) solar_voltage = 0.0f;
    // if (solar_voltage > 9.9f) solar_voltage = 9.9f;
    return solar_voltage;

}
    

