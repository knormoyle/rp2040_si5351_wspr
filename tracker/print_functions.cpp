// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// va_start stdarg https://www.tutorialspoint.com/c_standard_library/c_macro_va_start.htm

include "print_functions.h"

// https://en.wikipedia.org/wiki/Variadic_macro_in_the_C_preprocessor

// Without variadic macros, writing wrappers to printf is not directly possible. 
// The standard workaround is to use the stdargs functionality of C/C++, and have the function call printf instead.

//***********************************************
// could make FLY_WITH_NO_USBSERIAL a sticky configuration
// to avoid reloading separate firmware
// set it right before you cut off the usb connector?
// Probably better to have separate firmware compiled

// va_start is in stdarg 
// https://www.tutorialspoint.com/c_standard_library/c_macro_va_start.htm

// The C stdarg library va_start() macro enables access to the variable arguments following the named argument parmN. It is used to initialize a 'va_list' variable, which is then used to retrieve the additional arguments passed to the function.

// The va_start should be called with an instance 
// to a valid va_list (variable list) object ap 
// before any calls to va_arg (variable argument).

// This macro is useful for creating a variadic function, allowing us to create a function that can take variable number of arguments. It contains at least one fixed argument followed by an ellipsis (...).

// should only be undefined or defined to be 0 or 1 by outside forces
#ifndef FLY_WITH_NO_USBSERIAL
#define FLY_WITH_NO_USBSERIAL 0
#endif

// NOTE: we could expand this to have timestamps in front of each print,
// if desirable
void yPrint(const char* pformat, ...) {
#if FLY_WITH_NO_USBSERIAL == 1
    ;
#else
    va_list argptr;
    va_start(argptr, pformat);
    // there shouldn't be any comma-separated args..otherwise Serial.print will fail)
    // Serial.print(F(pformat, argptr));
    Serial.print(argptr);
    va_end(argptr);
#endif
    return;
}
void yPrintf(const char* pformat, ...) {
#if FLY_WITH_NO_USBSERIAL == 1
    va_list argptr;
    va_start(argptr, pformat);
    Serial.printf(pformat, argptr);
    va_end(argptr);
#else
    ;
#endif
    return
}
void yPrintln(const char* pformat, ...) {
#if FLY_WITH_NO_USBSERIAL == 1
    va_list argptr;
    va_start(argptr, pformat);
    // there shouldn't be any comma-separated args..otherwise Serial.print will fail)
    Serial.println(pformat);
    va_end(argptr);
#else
    ;
#endif
    return
}

