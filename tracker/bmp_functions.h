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

#ifndef BMP_FUNCTIONS_H
#define BMP_FUNCTIONS_H
#include <stdint.h>
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_library

// what about the tiny gps library? already included in the .ino?

void bmp_init(void);
float bmp_read_temperature(void);
float bmp_read_pressure(void);

// FIX! add temp and pressure reads?

#endif
