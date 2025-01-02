// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef DEBUG_FUNCTIONS_H
#define DEBUG_FUNCTIONS_H
#include <stdint.h>

#include <TinyGPS++.h>
// no buffering
void printStr(const char *str, int len);
void printFloat(float val, bool valid, int len, int prec);
void printInt(uint64_t val, bool valid, int len);

// buffering that emptys with DoLogPrint()
void StampPrintf(const char* pformat, ...);
void DoLogPrint();
// report all interesting clock freqs
void measureMyFreqs(void);

#endif
