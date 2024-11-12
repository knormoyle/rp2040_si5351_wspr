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


