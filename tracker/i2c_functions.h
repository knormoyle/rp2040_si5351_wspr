// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef I2C_FUNCTIONS_H
#define I2C_FUNCTIONS_H
#include <stdint.h>

//************************************************
void i2c_scan();
int I2cWriteTest(uint8_t reg, uint8_t val);
int i2cWrReadTestTest(uint8_t reg, uint8_t *val);
void i2c_scan_both();

void i2c_scan_with_Wire_setup();
void i2c_scan_with_Wire(void);
#include <Adafruit_I2CDevice.h>  // https://github.com/adafruit/Adafruit_BusIO
void scan_Wire(unsigned int SDA, unsigned int SCL, TwoWire &Wire, bool usingWire1);

#endif
