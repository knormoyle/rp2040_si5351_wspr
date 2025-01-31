// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef TELE_FUNCTIONS_H
#define TELE_FUNCTIONS_H
#include <stdint.h>

void snapForTelemetry(void);
float readVoltage(void);
void process_TELEN_data(void);
void doTelemetrySweepInteger(char *t_string, uint8_t t_length, int t_min, int t_max, int t_inc);
void doTelemetrySweepFloat (char *t_string, uint8_t t_sizeof, float t_min, float t_max, float t_inc);
void telemetrySweepAllForTest(void);
void solarElevationCalcs(double solarElevation);

#endif
