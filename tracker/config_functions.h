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

#ifndef CONFIG_FUNCTIONS_H
#define CONFIG_FUNCTIONS_H
#include <stdint.h>

// any of this needed
// #include <stdio.h>
// #include <string.h>
// #include <ctype.h>
// #include <defines.h>
// #include "pico/stdlib.h"

void user_interface(void);
void get_user_input(const char *prompt, char *input_variable, int max_length);
void display_intro(void);
void show_values(void);
void printFLASH(const uint8_t *buf, size_t len);
void read_FLASH(void);
void write_FLASH(void);
void show_TELEN_msg(void);

int check_data_validity_and_set_defaults(void);
void convertToUpperCase(char *str);

#endif
