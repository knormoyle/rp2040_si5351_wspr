/*
 * debug_functions.h
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z) 
*/  

#ifndef DEBUG_FUNCTIONS_H
#define DEBUG_FUNCTIONS_H
#include <stdint.h>

#include <TinyGPS++.h>
void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
void printStr(const char *str, int len);
void printFloat(float val, bool valid, int len, int prec);
void printInt(unsigned long val, bool valid, int len);

#endif
