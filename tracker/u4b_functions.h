// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef U4B_FUNCTIONS_H
#define U4B_FUNCTIONS_H
#include <stdint.h>

uint32_t init_rf_freq(char *band, char *lane);
void process_chan_num(char *id13, char *start_minute, char *lane, char *band, char *u4b_chan);
void u4b_encode_telen(char *hf_callsign, char *hf_grid4, char *hf_power, uint32_t telen_val1, uint32_t telen_val2, bool for_telen2, char *id13);
void u4b_encode_std( char *hf_callsign, char *hf_grid4, char *hf_power, char *t_grid6, char *t_altitude, char *t_temp, char *t_voltage, char *t_speed, char *id13);

#endif
