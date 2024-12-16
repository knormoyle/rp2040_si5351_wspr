// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef PRINT_FUNCTIONS_H
#define PRINT_FUNCTIONS_H
#include <stdint.h>

//***********************************************
// these aren't used
void yPrint(const char* pformat, ...);
void yPrintf(const char* pformat, ...);
void yPrintln(const char* pformat, ...);

// FIX! oddball. elsewhere?
#define kHz 1000U

// hmm. why is \n not sufficient?
// escape the double quotes, since we want them
#define EOL "\"\r\n\""
// ascii 13
#define CR "\"\r\""
// ascii 10
#define LF "\"\n\""


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


#define V0_println if (VERBY[0]) Serial.println
#define V0_printf if (VERBY[0]) Serial.printf
#define V0_print if (VERBY[0]) Serial.print
#define V0_flush if (VERBY[0]) Serial.flush
#define V0_chars_available VERBY[0] && Serial.available


#define V1_println if (VERBY[1]) Serial.println
#define V1_printf if (VERBY[1]) Serial.printf
#define V1_print if (VERBY[1]) Serial.print
#define V1_flush if (VERBY[1]) Serial.flush

#define V2_println if (VERBY[2]) Serial.println
#define V2_printf if (VERBY[2]) Serial.printf
#define V2_print if (VERBY[2]) Serial.print
#define V2_flush if (VERBY[2]) Serial.flush

#define V3_println if (VERBY[3]) Serial.println
#define V3_printf if (VERBY[3]) Serial.printf
#define V3_print if (VERBY[3]) Serial.print
#define V3_flush if (VERBY[3]) Serial.flush

#define V4_println if (VERBY[4]) Serial.println
#define V4_printf if (VERBY[4]) Serial.printf
#define V4_print if (VERBY[4]) Serial.print
#define V4_flush if (VERBY[4]) Serial.flush

#endif

