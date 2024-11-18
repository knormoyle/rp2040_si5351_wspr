// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <stdlib.h>
#include "defines.h"
#include "adc_functions.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// per https://github.com/DeimosHall/RP2040_CPU_Temperature/blob/main/src/CPU.cpp

extern const int BattPin;
// decode of _verbose 0-9
extern bool VERBY[10];

//****************************************************
void adc_INIT() {
    if (VERBY[0]) Serial.println(F("adc_init START"));
    // FIX! why was this commented out?
    // FIX! move this into an adc_init() in adc_functions.cpp? and move readBatt in there?
    pinMode(BattPin, INPUT);
    // using the analog reference low solves the analog measurement errors.
    analogReadResolution(12);

    adc_init();
    adc_set_temp_sensor_enabled(true);

    if (VERBY[0]) Serial.println(F("adc_init END"));
}

float readTemp(void) {
    if (VERBY[0]) Serial.println(F("readTemp START"));
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float CONVERSION_FACTOR = 3.3f / (1 << 12);

    float adc_voltage;
    // Select ADC input 4 for internal temperature sensor
    adc_select_input(4);
    delay(100); // milliseconds

    adc_voltage =  adc_read();
    adc_voltage += adc_read();
    adc_voltage += adc_read();

    adc_voltage = ((float) adc_voltage / 3) * CONVERSION_FACTOR;
    // formula found on page 71 (section 4.1.1. hardware_adc) of
    // the Raspberry Pi Pico C/C++ SDK documentation
    // bogus
    float tempC_a = 27 - (adc_voltage - 0.706) / 0.001721;

    // Another way
    float tempC_b = analogReadTemp();
    tempC_b += analogReadTemp();
    tempC_b += analogReadTemp();
    tempC_b = tempC_b / 3;

    // readTemp END tempC_a 82 tempC_b 25
    // readTemp END tempC_a 82 tempC_b 24
    //  tempC_a 437 tempC_b 24 tempC_c 24

    if (VERBY[0]) Serial.printf("readTemp END tempC_a %.f tempC_b %.f" EOL, tempC_a, tempC_b);
    return tempC_b;
}


//****************************************************
float readVoltage(void) {
    if (VERBY[0]) Serial.println(F("readVoltage START"));

    // this should'nt be needed?

    // adc_select_input(BattPin);
    // delay(100); // milliseconds

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
    float voltage = ((float)adc_val / 3.0f - 27.0f) / 412.0f;

    // there is a 200 ohm resistor between 3V3 and ADC_AVDD
    // we did 3 reads above ..averaging?
    // so don't need the 3x because of onboard voltage divider
    // pico-WSPRer does this (no use of ADC_AVDD) ?

    // const float conversionFactor = 3.3f / (1 << 12);
    // float voltage = 3 * (float)adc_read() * conversionFactor;
    // if (voltage < 0.0f) voltage = 0.0f;
    // if (voltage > 9.9f) voltage = 9.9f;
    if (VERBY[0]) Serial.printf("voltage %.f adc_val %d" EOL, voltage, adc_val);
    if (VERBY[0]) Serial.printf("readVoltage END voltage %.f" EOL, voltage);
    return voltage;
}

