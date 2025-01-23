// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef CONFIG_FUNCTIONS_H
#define CONFIG_FUNCTIONS_H
#include <stdint.h>


//***********************************************************
void forceHACK(void);
void decodeVERBY(void);
int read_FLASH(void);
void printFLASH(const uint8_t *buf, size_t len);
void write_FLASH(void);
int check_data_validity_and_set_defaults(void);
void makeSureClockIsGood(void);

void user_interface(void);
void config_intro(void);
void display_intro(void);
void show_values(void);
void show_commands(void);
void show_TELEN_msg(void);
void get_user_input(const char *prompt, char *input_variable, int max_length);

void convertToUpperCase(char *str);
void do_someTest(void);
void do_gpsResetTest();
void doFactoryReset();

#endif
