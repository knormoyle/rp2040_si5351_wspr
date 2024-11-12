// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.


// Bosch has stepped up their game with their new BME280 sensor, an environmental sensor with temperature, barometric pressure and humidity! This sensor is great for all sorts of indoor environmental sensing and can even be used in both I2C and SPI!
// 
// This precision sensor from Bosch is the best low-cost sensing solution for measuring humidity with ±3% accuracy, barometric pressure with ±1 hPa absolute accuraccy, and temperature with ±1.0°C accuracy. Because pressure changes with altitude, and the pressure measurements are so good, you can also use it as an altimeter with  ±1 meter or better accuracy!
// 
// The BME280 is the next-generation of sensors from Bosch, and is the upgrade to the BMP085/BMP180/BMP183 - with a low altitude noise of 0.25m and the same fast conversion time. It has the same specifications, but can use either I2C or SPI. For simple easy wiring, go with I2C. If you want to connect a bunch of sensors without worrying about I2C address collisions, go with SPI.

// https://www.adafruit.com/product/2652
// https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
// 2.5x2.5mm 8 pin lga?



// alternative bosch 8 pin sensors (same footprint?)
// 3x3mm not recommended for new designs. 8 pin lga
// mouser has 9289 digikey has 15534
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme680-ds001.pdf
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bme680-an014.pdf




// Like the BME280 & BMP280, this precision sensor from Bosch can measure humidity with ±3% accuracy, barometric pressure with ±1 hPa absolute accuracy, and temperature with ±1.0°C accuracy. Because pressure changes with altitude, and the pressure measurements are so good, you can also use it as an altimeter with  ±1 meter or better accuracy!

// The BME680 is the first gas sensor that integrates high-linearity and high-accuracy gas, pressure, humidity and temperature sensors.
// Personal air quality tracker
// Air quality mapping
// Air quality inside cars & public transport
// Enhanced context awareness
// Accurate step & calorie tracker
// Quick GPS-fix & improved navigation
// Indicator of too high / low humidity
// Air quality & well-being indicator
// Sleep / recovery tracker
// Weather trend
// Stair counter
// Floor level detection

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
// This algorithm is available to customers as reference C source code (“BMP28x_ API”) from Bosch
// Sensortec and via its sales and distribution partners.


#define BMP280_I2C_INSTANCE i2c1
#include "bmp_functions.h"

extern const int BMP280_I2C1_SDA_PIN = 2;
extern const int BMP280_I2C1_SCL_PIN = 3;
// extern Adafruit_BMP085 bmp;
extern Adafruit_BMP280 bmp;

// This precision sensor from Bosch is the best low-cost sensing solution
// for measuring barometric pressure and temperature.

const int BMP280_I2C1_SCL_HZ = (1000 * 1000);

void bmp_init(void) {
    // always on? these pins don't exist
    // gpio_init(BMP280_VDD_ON_N_PIN)

    // gpio_put(BMP280_VDD_ON_N_PIN, 0);

    // init I2C1 for BMP
    i2c_init(BMP280_I2C_INSTANCE, BMP280_I2C1_SCL_HZ);

    gpio_set_pulls(BMP280_I2C1_SDA_PIN, false, false);
    gpio_set_pulls(BMP280_I2C1_SCL_PIN, false, false);

    gpio_set_function(BMP280_I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BMP280_I2C1_SCL_PIN, GPIO_FUNC_I2C);
}

#include <Wire.h>
 
// Default settings from datasheet
bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500);

 
//*********************
void i2c_scan(void) {
  uint8_t error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}

//*********************
float bmp_read_temperature(void) {
    return bmp.readTemperature();
}

float bmp_read_pressure(void) {
    return bmp.readPressure();
}


