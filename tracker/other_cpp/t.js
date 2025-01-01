// Fast algorithm for the computation of the SZA and SAA
// Author: David Salac https://github.com/david-salac/
// The implementation of the fast algorithm for computation of the Solar Zenith Angle (aka SZA)
// and Solar Azimut Angle (aka SAA) based on the logic proposed by Roberto Grena in 2012
// https://doi.org/10.1016/j.solener.2012.01.024.
// The precision of the algorithm is 0.3 degrees for the SZA and 0.5 degrees for the SAA (mean-average error).

//****************************************************
// gen'ed with
// https://app.codeconvert.ai/code-converter?inputLang=C%2B%2B&outputLang=JavaScript
// Notes
// 
// Major changes made in the translation:
// 
// Changed the function to return an object with sza and saa properties instead of using pointer parameters
// Replaced C++ time_t and gmtime with JavaScript Date object
// Converted C++ math constants and functions to JavaScript equivalents:
// M_PI → Math.PI
// M_PI_2 → Math.PI / 2
// Mathematical functions like sin, cos, etc. are now using Math. prefix
// Changed floor() to Math.floor()
// Changed modulo operation fmod() to JavaScript's % operator
// Used const instead of double for variable declarations where values don't change
// Replaced pointer dereferencing with object property access
// Converted C++ time struct members to JavaScript Date methods (getUTCHours, getUTCMinutes, etc.)
// Important note about timestamp:
// 
// The function expects the timestamp in Unix epoch seconds
// JavaScript Date constructor expects milliseconds, so we multiply the input by 1000
// The algorithm's mathematical logic remains exactly the same, 
// only the implementation details have been adapted to JavaScript conventions and capabilities.
//****************************************************

function deg2rad(valInDegrees) {
    return Math.PI * valInDegrees / 180.0;
}

function rad2deg(valInRadians) {
    return 180.0 * valInRadians / Math.PI;
}

// Solar zenith and azimuth angle (SZA, SAA) computation
// longitude longitude in degrees
// latitude latitude in degrees
// time_stamp time in UTC
// return SZA, SAA
function solarZenithAndAzimuthAngle2(longitude, latitude, timeStamp) {
    const timeConv = new Date(timeStamp * 1000); // Convert epoch time to milliseconds

    // Time vectors:
    const year = timeConv.getUTCFullYear();
    const month = timeConv.getUTCMonth() + 1; // Months are zero-based
    const day = timeConv.getUTCDate();
    const dayHours = 
        timeConv.getUTCHours() + 
        timeConv.getUTCMinutes() / 60.0 + 
        timeConv.getUTCSeconds() / 3600.0;

    // Time transformation:
    const yearVal = Math.floor(365.25 * (year - 2000.0));
    const monthVal = Math.floor(30.6001 * (month + 1.0));
    const yearValPc = Math.floor(0.01 * year);

    const timeVec = yearVal + monthVal - yearValPc + day + 
        (0.0416667 * dayHours) - 21958.0;

    // Transform latitude/longitude to radians
    const latRad = deg2rad(latitude);
    const lonRad = deg2rad(longitude);

    // Reasonable estimates for pressure and temperature
    const pressure = 1.0;  // unit: atmosphere
    const temperature = 20.0;  // unit: degree of celsius

    // Computation
    const dT = 96.4 + 0.567 * (year - 2061.0);
    const te = timeVec + 1.1574e-5 * dT;
    const wte = 0.0172019715 * te;

    const lambdaDecl = 
        -1.388803 + 
        (1.720279216e-2 * te) + 
        (3.3366e-2 * Math.sin(wte - 0.06172)) + 
        (3.53e-4 * Math.sin((2.0 * wte) - 0.1163));

    const epsilon = 4.089567e-1 - (6.19e-9 * te);

    const sl = Math.sin(lambdaDecl);
    const cl = Math.cos(lambdaDecl);
    const se = Math.sin(epsilon);

    const ce = Math.sqrt(1.0 - se * se);

    let rAsc = Math.atan2(sl * ce, cl);
    // Get rid of negative values in rAsc
    if (rAsc < 0) {
        rAsc += 2.0 * Math.PI;
    }

    let hAng = ((1.7528311 + (6.300388099 * timeVec) + lonRad - rAsc + Math.PI) % (2.0 * Math.PI)) - Math.PI;

    // Get rid of negative values in hAng
    if (hAng < 0) {
        hAng += 2.0 * Math.PI;
    }

    const sp = Math.sin(latRad);
    const cp = Math.sqrt(1.0 - (sp * sp));
    const sd = sl * se;
    const cd = Math.sqrt(1.0 - (sd * sd));
    const sH = Math.sin(hAng);
    const cH = Math.cos(hAng);

    const se0 = sp * sd + cp * cd * cH;
    const ep = Math.asin(se0) - (4.26e-5 * Math.sqrt(1.0 - (se0 * se0)));

    const dE = (0.08422 * pressure) / ((273.0 + temperature) * Math.tan(ep + 0.003138 / (ep + 0.08919)));

    const sza_here = rad2deg(Math.PI / 2 - ep - dE);
    const saa_here = rad2deg(Math.PI + Math.atan2(sH, (cH * sp) - (sd * cp / cd)));

    return { sza: sza_here, saa: saa_here };
}

