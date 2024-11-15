// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.
#ifndef BMP_FUNCTIONS_H
#define BMP_FUNCTIONS_H
#include <stdint.h>
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_library

// what about the tiny gps library? already included in the .ino?

void bmp_init(void);
void i2c_scan(void);
float bmp_read_temperature(void);
float bmp_read_pressure(void);
float bmp_read_humidity(void);

// FIX! add temp and pressure reads?

#endif
