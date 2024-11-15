// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <stdlib.h>
#include "defines.h"

extern const int BattPin;
extern bool DEVMODE;

//****************************************************
void adc_init() {
    if (DEVMODE) Serial.println(F("adc_init START"));
    // FIX! why was this commented out?
    // FIX! move this into an adc_init() in adc_functions.cpp? and move readBatt in there?
    pinMode(BattPin, INPUT);
    // using the analog reference low solves the analog measurement errors.
    analogReadResolution(12);
    if (DEVMODE) Serial.println(F("adc_init END"));

}
//****************************************************
float readVoltage(void) {
    if (DEVMODE) Serial.println(F("readVoltage START"));
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
    if (DEVMODE) Serial.printf("solar_voltage %.f adc_val %d" EOL, solar_voltage, adc_val);

    if (DEVMODE) Serial.println(F("readVoltage END"));
    return solar_voltage;
}

