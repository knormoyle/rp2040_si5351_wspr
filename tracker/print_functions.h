// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef PRINT_FUNCTIONS_H
#define PRINT_FUNCTIONS_H
#include <stdint.h>

//***********************************************
void yPrint(const char* pformat, ...);
void yPrintf(const char* pformat, ...);
void yPrintln(const char* pformat, ...);

// FIX! oddball. elsewhere?
#define kHz 1000U

// hmm. why is \n not sufficient?
#define EOL "\r\n"
// ascii 13
#define CR "\r"
// ascii 10
#define LF "\n"

// #define EOL "\n"

//***********************************************
// this is another compile option
#define USE_SPECIAL_ASCII 0

// ANSI escape codes for color
// arduino ide serial monitor won't interpret these..don't send them
#if USE_SPECIAL_ASCII == 1
#define RED "\x1b[91m"
#define BRIGHT "\x1b[97m"
#define NORMAL "\x1b[37m"
#define GREEN "\x1b[32m"
#define YELLOW "\x1b[33m"
#define BLUE "\x1b[34m"
#define RESET "\x1b[0m"
#define CLEAR_SCREEN "\x1b[2J"
#define CURSOR_HOME "\x1b[H"
#define UNDERLINE_ON "\033[4m"
#define UNDERLINE_OFF "\033[24m"
#define BOLD_ON "\033[1m"
#define BOLD_OFF "\033[0m"
#else
#define RED ""
#define BRIGHT ""
#define NORMAL ""
#define GREEN ""
#define YELLOW ""
#define BLUE ""
#define RESET ""
#define CLEAR_SCREEN ""
#define CURSOR_HOME ""
#define UNDERLINE_ON ""
#define UNDERLINE_OFF ""
#define BOLD_ON ""
#define BOLD_OFF ""
#endif

#endif
