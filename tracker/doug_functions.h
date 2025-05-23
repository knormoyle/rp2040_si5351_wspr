// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef DOUG_FUNCTIONS_H
#define DOUG_FUNCTIONS_H
#include <stdint.h>

void encodeBasicTele(char *hf_callsign, char *hf_grid4, char *hf_power, const char *id13, const char *grid56, uint32_t altitudeMeters, int8_t temperatureCelsius, double voltageVolts, uint8_t speedKnots, bool gpsIsValid); 
void define_codecGpsMsg(void);
void encode_codecGpsMsg(char *hf_callsign, char *hf_grid4, char *hf_power, uint8_t slot);

void define_codecBmpMsg(void);
void encode_codecBmpMsg(char *hf_callsign, char *hf_grid4, char *hf_power, uint8_t slot);

#endif

