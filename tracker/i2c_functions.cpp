// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.


// https://wellys.com/posts/rp2040_arduino_i2c/
// from https://github.com/lkoepsel/I2C/blob/main/Arduino/Pico/I2C_Scanner/I2C_Scanner.ino

#include "i2c_functions.h"

// I2C_Scanner - scans all Wire and Wire interfaces on Pico for devices
// Specific pins must be set for both interfaces

#include <Wire.h>
#include "defines.h"

// extern char _verbose[2];
// decode of _devmode
extern bool DEVMODE;
// decode of verbose 0-9
extern bool VERBY[10];

//************************************************
// this assumes Wire is created beforehand?
void i2c_scan(void) {
    if (VERBY[0]) Serial.println(F("i2c_scan START"));
    uint8_t error, address;
    int nDevices;
    Serial.println("Scanning...");
    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) Serial.println("No I2C devices found\n");
    else Serial.println("done\n");
    if (VERBY[0]) Serial.println(F("i2c_scan END"));
}
//************************************************

byte error, address;
unsigned int nDevices;
unsigned int wireN = 0;

// Pi Pico SDA/SCL Pins
const unsigned int SDA_00 = 0;
const unsigned int SCL_00 = 1;
const unsigned int SDA_10 = 2;
const unsigned int SCL_10 = 3;
const unsigned int SDA_01 = 4;
const unsigned int SCL_01 = 5;
const unsigned int SDA_11 = 6;
const unsigned int SCL_11 = 7;
const unsigned int SDA_02 = 8;
const unsigned int SCL_02 = 9;
const unsigned int SDA_12 = 10;
const unsigned int SCL_12 = 11;
const unsigned int SDA_03 = 12;
const unsigned int SCL_03 = 13;
const unsigned int SDA_13 = 14;
const unsigned int SCL_13 = 15;
const unsigned int SDA_04 = 16;
const unsigned int SCL_04 = 17;
const unsigned int SDA_14 = 18;
const unsigned int SCL_14 = 19;
const unsigned int SDA_05 = 20;
const unsigned int SCL_05 = 21;
const unsigned int SDA_15 = 26;
const unsigned int SCL_15 = 27;

// scan the Wire interface for devices
void scan_Wire(unsigned int SDA, unsigned int SCL, TwoWire &Wire);

void i2c_scanner_setup() {
    if (VERBY[0]) Serial.println(F("i2c_scanner_setup START"));

    if (VERBY[0]) Serial.println(F("Just Wire"));

    scan_Wire(SDA_00, SCL_00, Wire);
    scan_Wire(SDA_01, SCL_01, Wire);
    scan_Wire(SDA_02, SCL_02, Wire);
    scan_Wire(SDA_03, SCL_03, Wire);
    scan_Wire(SDA_04, SCL_04, Wire);
    scan_Wire(SDA_05, SCL_05, Wire);

    if (VERBY[0]) Serial.println(F("Just Wire1"));
    scan_Wire(SDA_10, SCL_10, Wire1);
    scan_Wire(SDA_11, SCL_11, Wire1);
    scan_Wire(SDA_12, SCL_12, Wire1);
    scan_Wire(SDA_13, SCL_13, Wire1);
    scan_Wire(SDA_14, SCL_14, Wire1);
    scan_Wire(SDA_15, SCL_15, Wire1);

    Serial.println("All Scans Successful!\n");
    if (VERBY[0]) Serial.println(F("i2c_scanner_setup END"));
}

// scan the Wire interfaces for devices
void scan_Wire(unsigned int SDA, unsigned int SCL, TwoWire &Wire)
{
    if (VERBY[0]) Serial.printf("scan_WIRE %u %u START" EOL, SDA, SCL );
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
    Wire.begin();

    Serial.print("Wire");
    Serial.print(wireN % 2);
    Serial.print(" SDA: ");
    Serial.print(SDA);
    Serial.print(" SCL: ");
    Serial.println(SCL);

    nDevices = 0;
    for(address = 1; address < 127; address++ ) {

        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address<16) Serial.print("0");
            Serial.println(address,HEX);
            nDevices++;
        } else if (error==4) {
            Serial.print("Unknown error at address 0x");
            if (address<16) { Serial.print("0"); }
            Serial.println(address,HEX);
        }    
    }
    if (nDevices == 0) { Serial.println("No I2C devices found" EOL);
    } else { Serial.println("Scan Complete" EOL); }
    Wire.end();

    wireN++;

    if (VERBY[0]) Serial.printf("scan_WIRE %u %u END" EOL, SDA, SCL );
}

// https://deepbluembedded.com/arduino-i2c-tutorial-examples/
