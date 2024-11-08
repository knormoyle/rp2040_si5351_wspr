/* 
 * bmp_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#ifndef BMP_FUNCTIONS_H
#define BMP_FUNCTIONS_H
#include <stdint.h>
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_library

// what about the tiny gps library? already included in the .ino?

void bmp_init(void);
float bmp_read_temperature(void);
float bmp_read_pressure(void);

// FIX! add temp and pressure reads?

#endif
