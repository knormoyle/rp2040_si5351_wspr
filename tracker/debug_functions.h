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

#ifndef DEBUG_FUNCTIONS_H
#define DEBUG_FUNCTIONS_H
#include <stdint.h>

#include <TinyGPS++.h>
// no buffering
void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
void printStr(const char *str, int len);
void printFloat(float val, bool valid, int len, int prec);
void printInt(unsigned long val, bool valid, int len);

// buffering that emptys with DoLogPrint()
void StampPrintf(const char* pformat, ...);
void DoLogPrint();

#endif