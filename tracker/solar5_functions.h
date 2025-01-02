// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php 
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef SOLAR5_FUNCTIONS_H
#define SOLAR5_FUNCTIONS_H

void solarZenithAndAzimuthAngle5(double *sza, double *saa, double longitude, double latitude, time_t timeStamp);
void calcSolarElevation5(double *solarElevation, double *solarAzimuth, double *solarDistance);


#endif  // SOLAR5_FUNCTIONS_H
