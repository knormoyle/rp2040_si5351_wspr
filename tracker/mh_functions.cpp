// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

// Incorporates work by: Roman Piksaykin R2BDY. Thank you.
// https://github.com/RPiks/pico-WSPR-tx

// Incorporates work by: Mateusz Salwach SP6
// https://github.com/sp6q/maidenhead

#include "mh_functions.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

char letterize(int x) {
    // KC3LBR 07/23/24   
    // an alternate/redundant fix to the one below, 
    // this clamps the returned characters at 'X' or lower. 
    // The original code sometimes returned a Y for 5th or 6 char, which is invalid
    if (x<24) return (char) x + 65;
    else return (char) 23 + 65; 

}

// call with double for more precision
// force size to be 6
// always return char[7] (null term)
// will always return upper case

//  tinyGPS returns doubles for lat long
char* get_mh_6(double lat, double lon) {
    static char locator[7];
    double LON_F[]={20,2.0,0.083333,0.008333,0.0003472083333333333};
    double LAT_F[]={10,1.0,0.0416665,0.004166,0.0001735833333333333};
    int i;
    lon += 180;
    lat += 90;

    size = 6;
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
    locator[6] = 0;
    return locator;
}


