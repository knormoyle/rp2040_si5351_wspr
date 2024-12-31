
// forked from SolarAngles.cpp

// https://github.com/david-salac/Fast-SZA-and-SAA-computation/tree/master
// https://github.com/david-salac/Fast-SZA-and-SAA-computation/blob/master/CPP_version/SolarAngles.cpp
// Fast algorithm for the computation of the SZA and SAA
// Author: David Salac https://github.com/david-salac/
// The implementation of the fast algorithm for computation of the Solar Zenith Angle (aka SZA)
// and Solar Azimut Angle (aka SAA) based on the logic proposed by Roberto Grena in 2012
// https://doi.org/10.1016/j.solener.2012.01.024.
// The precision of the algorithm is 0.3 degrees for the SZA and 0.5 degrees f
// or the SAA (mean-average error).

//****************************************
inline double deg2rad(double valInDegrees) {
    return M_PI * valInDegrees / 180.0;
}

inline double rad2deg(double valInDegrees) {
    return 180.0 * valInDegrees / M_PI;
}
//****************************************
// Solar zenith and azimuth angle (SZA, SAA) computation
// longitude longitude in degrees
// latitude latitude in degrees
// time_stamp time in UTC
// return SZA, SAA
void solarZenithAndAzimuthAngle2(double *sza, double *saa, double longitude, double latitude, time_t timeStamp) {
    // timeStamp is epoch time?
    // The C library gmtime() function of type struct uses the value pointed by timer to fill 
    // a structure(tm) with the values that represent the corresponding time, 
    // expressed in coordinated universal time (UTC) or GMT timezone.
    auto timeConv =gmtime(&timeStamp);

    // Time vectors:
    double year = timeConv->tm_year + 1900.0;
    double month = timeConv->tm_mon + 1.0;
    double day = timeConv->tm_mday;
    double dayHours = 
        double(timeConv->tm_hour) + 
        double(timeConv->tm_min) / 60.0 + 
        double(timeConv->tm_sec) / 3600.0;

    // Time transformation:
    double yearVal = floor(365.25 * (year - 2000.0));
    double monthVal = floor(30.6001 * (month + 1.0));
    double yearValPc = floor(0.01 * year);

    double timeVec = yearVal + monthVal - yearValPc + day + 
        (0.0416667 * dayHours) - 21958.0;

    // Transform latitude/longitude to radians
    double latRad = deg2rad(latitude);
    double lonRad = deg2rad(longitude);

    // Reasonable estimates for pressure and temperature
    double pressure = 1.0;  // unit: atmosphere
    double temperature = 20.0;  // unit: degree of celsius

    // Computation
    double dT = 96.4 + 0.567 * (year - 2061.0);
    double te = timeVec + 1.1574e-5 * dT;
    double wte = 0.0172019715 * te;

    double lambdaDecl = 
        -1.388803 + 
        (1.720279216e-2 * te) + 
        (3.3366e-2 * sin(wte - 0.06172)) + 
        (3.53e-4 * sin((2.0 * wte) - 0.1163));

    double epsilon = 4.089567e-1 - (6.19e-9 * te);

    double sl = sin(lambdaDecl);
    double cl = cos(lambdaDecl);
    double se = sin(epsilon);

    double ce = sqrt(1.0 - se * se);

    double rAsc = atan2(sl * ce, cl);
    // Get rid of negative values in rAsc
    if (rAsc < 0) {
        rAsc += 2.0 * M_PI;
    }

    double hAng = fmod(
        (1.7528311 + (6.300388099 * timeVec) + lonRad - rAsc + M_PI), 
        (2.0 * M_PI)) - M_PI;

    // Get rid of negative values in hAng
    if (hAng < 0) {
        hAng += 2.0 * M_PI;
    }

    double sp = sin(latRad);
    double cp = sqrt(1.0 - (sp * sp));
    double sd = sl * se;
    double cd = sqrt(1.0 - (sd * sd));
    double sH = sin(hAng);
    double cH = cos(hAng);

    double se0 = sp * sd + cp * cd * cH;
    double ep = asin(se0) - (4.26e-5 * sqrt(1.0 - (se0 * se0)));

    double dE = (0.08422 * pressure) / ((273.0 + temperature) * tan(ep + 0.003138 / (ep + 0.08919)));

    double sza_here = rad2deg(M_PI_2 - ep - dE);
    double saa_here = rad2deg(M_PI + atan2(sH, (cH * sp) - (sd * cp / cd)));

    *sza = sza_here;
    *saa = saa_here;
}
