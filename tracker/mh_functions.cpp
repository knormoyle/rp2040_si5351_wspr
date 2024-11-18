// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include "mh_functions.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

char letterize(int x) {
    // KC3LBR 07/23/24 alternate/redundant fix 
    // (the increased resolution may preclude a need)
    // this clamps the returned characters at 'X' or lower.
    // Original code sometimes returned a invalid Y for 5th or 6 char, 
    // because of no bounds check.
    if (x < 24) return (char) x + 65;
    else return (char) 23 + 65;
}

// call with double for more precision
// force size to be 6
// always return char[7] (null term)
// will always return upper case

//  tinyGPS gives you doubles for lat long, so use doubles here
// only place we return a pointer to a static char array ! (locator)
void get_mh_6(char *locator, double lat, double lon) {

    double LON_F[] = {20, 2.0, 0.0833330, 0.008333, 0.0003472083333333333};
    double LAT_F[] = {10, 1.0, 0.0416665, 0.004166, 0.0001735833333333333};
    int i;

    lon += 180;
    lat += 90;
    int size = 6;
    for (i = 0; i < size/2; i++){
        if (i % 2 == 1) {
            locator[i*2]   = (char) (lon/LON_F[i] + '0');
            locator[i*2+1] = (char) (lat/LAT_F[i] + '0');
        } else {
            locator[i*2]   = letterize((int) (lon/LON_F[i]));
            locator[i*2+1] = letterize((int) (lat/LAT_F[i]));
        }
        lon = fmod(lon, LON_F[i]);
        lat = fmod(lat, LAT_F[i]);
    }
    locator[6] = 0; // null term
}
