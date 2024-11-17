// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef CONFIG_FUNCTIONS_H
#define CONFIG_FUNCTIONS_H
#include <stdint.h>
#include "defines.h"

// any of this needed
// #include <stdio.h>
// #include <string.h>
// #include <ctype.h>
// #include <defines.h>
// #include "pico/stdlib.h"

void forceHACK(void);
void decodeVERBY(void);
int read_FLASH(void);
int check_data_validity_and_set_defaults(void);

void user_interface(void);
void get_user_input(const char *prompt, char *input_variable, int max_length);
void display_intro(void);
void show_values(void);

void printFLASH(const uint8_t *buf, size_t len);
void write_FLASH(void);
void show_TELEN_msg(void);

void convertToUpperCase(char *str);

#endif
