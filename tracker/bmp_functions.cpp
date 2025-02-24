// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

//******************************************
// adafruit has a bmp280 library
// here's another one that might be good 
// might want to report raw and temp-compensated results?
// https://github.com/ebrezadev/BMP280-Barometric-Pressure-and-Temperature-Sensor-C-Driver


//******************************************
// alternative bosch 8 pin sensors (same footprint?)
// 3x3mm not recommended for new designs. 8 pin lga
// mouser has 9289 digikey has 15534
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bme680-an014.pdf

// https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/

// 2x2.5mm 8 pin LGA
// https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

// pressure oversampling 1 to 16 (16 is ultra high resolution)
// for weather monitoring (recommended) (loest power)
// mode forced
// oversampling ultra low power
// osrs_p x1
// osrs_t x1
// IIR off
// Ioo (uA) 0.14
// ODR 1/60 hz
// RMS Noise 26.4

// The BMP280 offers three power modes: sleep mode, forced mode and normal mode. These can be
// selected using the mode[1:0] bits in control register 0xF4.
//
// 00 Sleep mode
// 01 and 10 Forced mode
// 11 Normal mode
//
// Sleep mode is set by default after power on reset. In sleep mode, no measurements are performed
// and power consumption (IDDSM) is at a minimum. All registers are accessible; Chip-ID and
// compensation coefficients can be read.
//
// In forced mode, a single measurement is performed according to selected measurement and filter
// options. When the measurement is finished, the sensor returns to sleep mode and the measurement
// results can be obtained from the data registers. For a next measurement, forced mode needs to be
// selected again. This is similar to BMP180 operation. Forced mode is recommended for applications
// which require low sampling rate or host-based synchronization.
//
// 3.11.3 Compensation formula
// Please note that it is strongly advised to use the API available from Bosch Sensortec to perform
// readout and compensation. If this is not wanted, the code below can be applied at the user’s risk. Both
// pressure and temperature values are expected to be received in 20 bit format, positive, stored in a 32
// bit signed integer.
//
// The variable t_fine (signed 32 bit) carries a fine resolution temperature value over to the pressure
// compensation formula and could be implemented as a global variable.
//
// The data type “BMP280_S32_t” should define a 32 bit signed integer variable type and can usually be
// defined as “long signed int”.
//
// The data type “BMP280_U32_t” should define a 32 bit unsigned integer variable type and can usually
// be defined as “long unsigned int”.
//
// For best possible calculation accuracy, 64 bit integer support is needed. If this is not possible on your
// platform, please see appendix 8.2 for a 32 bit alternative.
//
// The data type “BMP280_S64_t” should define a 64 bit signed integer variable type, which on most
// supporting platforms can be defined as “long long signed int”. The revision of the code is rev.1.1.

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value

/*
// this is for 64 bit systems? they have appendix code for 32 bit systems
BMP280_S32_t t_fine;
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
    BMP280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) – ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) – ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) – ((BMP280_S32_t)dig_T1))) >> 12) * ((BMP280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}
*/

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
/*
BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P) {
    BMP280_S64_t var1, var2, p;
    var1 = ((BMP280_S64_t)t_fine) – 128000;
    var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
    var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
    if (var1 == 0) { return 0; } // avoid exception caused by division by zero

    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
    return (BMP280_U32_t)p;
}
*/

// calculating pressure and temperature
// page 22 of datasheet

// The following figure shows the detailed algorithm for pressure and temperature measurement.
// This algorithm is available to customers as reference C source code (“BMP28x_ API”)

// FIX! do we use Wire1 for this i2c ?
// do we need a Wire1.begin() in tracker.ino
#include "bmp_functions.h"
#include "print_functions.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>  // https://github.com/adafruit/Adafruit_BMP280_Library
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

extern const int BMP_I2C_SDA_PIN;
extern const int BMP_I2C_SCL_PIN;
extern const int BMP_I2C_SCL_HZ;
#define BMP_I2C_INSTANCE i2c1

// extern Adafruit_BMP085 bmp;
extern Adafruit_BMP280 bmp;

// decode of _verbose 0-9
extern bool VERBY[10];
extern Adafruit_BMP280 bmp; 
extern bool BMP_FOUND;

void bmp_init(void) {
    Watchdog.reset();
    V1_println(F("bmp_init START"));
    // always on?

    // init I2C for BMP
    // using i2c0 so we can just use the default Wire?
    // but we have to to set pins to 2 and 3

    // will this be Wire? 
    // Wire1 should already be setup
    if (false) {
        i2c_init(BMP_I2C_INSTANCE, BMP_I2C_SCL_HZ);
        gpio_set_pulls(BMP_I2C_SDA_PIN, false, false);
        gpio_set_pulls(BMP_I2C_SCL_PIN, false, false);
        gpio_set_function(BMP_I2C_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(BMP_I2C_SCL_PIN, GPIO_FUNC_I2C);
    } else {
        Wire1.setSDA(BMP_I2C_SDA_PIN);
        Wire1.setSCL(BMP_I2C_SCL_PIN);
        Wire1.setClock(BMP_I2C_SCL_HZ);
        Wire1.begin();
    }
    sleep_ms(1000);

    // should be 0x76 or 0x77
    BMP_FOUND = false;
    if (bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
        BMP_FOUND = true;
    } else {
        V1_println(F("WARN: Could not find a valid BMP280 sensor using 0x76 on Wire1"));
    }

    if (BMP_FOUND) {
        V1_printf("bmp280 sensorID() %x" EOL, bmp.sensorID());
        V1_println(F("bmp280 default settings from datasheet. Forced"));

        // do we need to set this again to get another measurement?
        bmp_forced_mode();
    }
    
    V1_println(F("bmp_init END"));
}


//*********************
// FIX! compensation code?
void bmp_forced_mode() {
    V1_println(F("bmp_forced_mode START"));
    // do we need to set this again every time we want a measurement?
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, // Operating Mode
        Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
        Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
        Adafruit_BMP280::FILTER_X16,      // Filtering
        Adafruit_BMP280::STANDBY_MS_500); // Standby time
    sleep_ms(200);
    V1_println(F("bmp_forced_mode END"));
}

float bmp_read_temperature(void) {
    V1_println(F("bmp_read_temperature START"));
    float temperature = 0.0;
    if (BMP_FOUND) temperature = bmp.readTemperature(); // *C
    V1_printf("bmp_read_temperature END %.4f" EOL, temperature);
    return temperature;
}

float bmp_read_pressure(void) {
    V1_println(F("bmp_read_pressure START"));
    float pressure = 0.0;
    if (BMP_FOUND) pressure = bmp.readPressure(); // Pa
    V1_printf("bmp_read_pressure END %.4f" EOL, pressure);
    return pressure;
}

float bmp_read_altitude(void) {
    V1_println(F("bmp_read_altitude START"));
    float altitude = 0.0;
    if (BMP_FOUND) altitude = bmp.readAltitude(); // M
    V1_printf("bmp_read_altitude END %.4f" EOL, altitude);
    return altitude;
}

