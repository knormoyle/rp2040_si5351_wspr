/* 
 * gps_functions.cpp
 * Created: 11.7.2024
 * initial: Kazuhisa “Kazu” Terasaki (AG6NS)
 * Kevin Normoyle (AD6Z)
*/

#ifndef GPS_FUNCTIONS_H
#define GPS_FUNCTIONS_H
#include <stdint.h>

// why was this static void before
void updateGpsData(int ms);
void setGPS_DynamicModel6();
void sendUBX(uint8_t *MSG, uint8_t len);
void gpsDebug();
void GridLocator(char *dst, float latt, float lon);
void GpsON();
void GpsOFF();

#endif
