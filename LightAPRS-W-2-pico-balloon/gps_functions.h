#ifndef GPS_FUNCTIONS_H
#define GPS_FUNCTIONS_H

// why was this static void before
void updateGpsData(int ms);
void setGPS_DynamicModel6();
void sendUBX(uint8_t *MSG, uint8_t len);
void gpsDebug();
void GridLocator(char *dst, float latt, float lon)

