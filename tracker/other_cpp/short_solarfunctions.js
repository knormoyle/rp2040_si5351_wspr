// forked from:
/*
 * solarfunctions: precise solar calculations
 * Copyright (c) 2024 Christian Menne
 * https://github.com/chrmenne/solarfunctions
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

const VERSION = 1.0;
const INVALID_VALUE = 999999999.999999;
const PI = Math.PI;

// Various constants used in the formulae
const MEAN_RADIUS_VECTOR_CORRECTION = 1.000001018;
const SOLAR_ANGULAR_DIAMETER_RAD = 0.0145380805;
const SECONDS_PER_DAY = 86400.0;
const MINUTES_PER_DAY = 1440.0;
const SECONDS_PER_HOUR = 3600.0;
const MINUTES_PER_HOUR = 60.0;
const SECONDS_PER_MINUTE = 60.0;
const MINUTES_PER_DEGREE_LONGITUDE = 4.0;
const DEGREES_LONGITUDE_PER_HOUR = 15.0;
const UNIX_EPOCH_JD = 2440587.5;
const Y2K_JULIAN_DATE = 2451545.0;
const SOLAR_ANOMALY_AT_EPOCH = 357.5291092;
const SOLAR_ANOMALY_RATE = 35999.05029;
const SOLAR_ANOMALY_CORRECTION = -0.0001537;
const MEAN_LONGITUDE_SUN_AT_EPOCH = 280.46646;
const MEAN_LONGITUDE_SUN_RATE = 36000.76983;
const MEAN_LONGITUDE_SUN_CORRECTION = 0.0003032;
const APPARENT_LONGITUDE_CORRECTION_1 = 0.00569;
const APPARENT_LONGITUDE_CORRECTION_2 = 0.00478;
const OBLIQUITY_CORRECTION_FACTOR = 0.00256;
const MEAN_OBLIQUITY_AT_EPOCH = 23.439292;
const OBLIQUITY_RATE1 = 0.013004167;
const OBLIQUITY_RATE2 = 0.00000016389;
const OBLIQUITY_RATE3 = 0.0000005036;
const OMEGA_BASE_VALUE = 125.04;
const OMEGA_RATE = -1934.136;
const ECCENTRICITY_BASE = 0.016708634;
const ECCENTRICITY_COEFF1 = 0.000042037;
const ECCENTRICITY_COEFF2 = 0.0000001267;
const SOLAR_EQ_CENTER_COEFF1 = 1.914602;
const SOLAR_EQ_CENTER_COEFF2 = 0.004817;
const SOLAR_EQ_CENTER_COEFF3 = 0.000014;
const SOLAR_EQ_CENTER_COEFF4 = 0.019993;
const SOLAR_EQ_CENTER_COEFF5 = 0.000101;
const SOLAR_EQ_CENTER_COEFF6 = 0.000289;

function degToRad(degrees) {
    return degrees * PI / 180;
}

function radToDeg(radians) {
    return radians * 180 / PI;
}

function normalizeDegrees180(angle) {
    let degrees = normalizeDegrees360(angle);
    if (degrees > 180.0) {
        degrees -= 360.0;
    }
    return degrees;
}

function normalizeDegrees360(angle) {
    return ((angle % 360) + 360) % 360;
}

function normalizeRadiansPi(angle) {
    let radians = normalizeRadians2Pi(angle);
    if (radians > PI) {
        radians -= 2 * PI;
    }
    return radians;
}

function normalizeRadians2Pi(angle) {
    return ((angle % (2 * PI)) + 2 * PI) % (2 * PI);
}

function calculateJulianDate(timestamp_utc) {
    return timestamp_utc / SECONDS_PER_DAY + UNIX_EPOCH_JD;
}

function calculateJulianCenturyNumber(timestamp_utc) {
    return (calculateJulianDate(timestamp_utc) - Y2K_JULIAN_DATE) / 36525.0;
}

function calculateEccentricity(julianCenturyNumber) {
    return ECCENTRICITY_BASE - julianCenturyNumber * (ECCENTRICITY_COEFF1 + ECCENTRICITY_COEFF2 * julianCenturyNumber);
}

function calculateObliquity(julianCenturyNumber) {
    return MEAN_OBLIQUITY_AT_EPOCH - julianCenturyNumber * (OBLIQUITY_RATE1 + julianCenturyNumber * (OBLIQUITY_RATE2 - OBLIQUITY_RATE3 * julianCenturyNumber));
}

function calculateCorrectedObliquity(julianCenturyNumber) {
    if (typeof julianCenturyNumber === 'number') {
        return calculateCorrectedObliquityHelper(calculateObliquity(julianCenturyNumber), calculateOmega(julianCenturyNumber));
    }
    return calculateCorrectedObliquityHelper(arguments[0], arguments[1]);
}

function calculateCorrectedObliquityHelper(obliquity, omega) {
    return obliquity + OBLIQUITY_CORRECTION_FACTOR * Math.cos(degToRad(omega));
}

function calculateOmega(julianCenturyNumber) {
    return normalizeDegrees360(OMEGA_BASE_VALUE + OMEGA_RATE * julianCenturyNumber);
}

function calculateSunMeanAnomaly(julianCenturyNumber) {
    const M = SOLAR_ANOMALY_AT_EPOCH + julianCenturyNumber * (SOLAR_ANOMALY_RATE - SOLAR_ANOMALY_CORRECTION * julianCenturyNumber);
    return normalizeDegrees360(M);
}

function calculateSolarEquationOfCenter(julianCenturyNumber, meanAnomaly) {
    return (SOLAR_EQ_CENTER_COEFF1
        - julianCenturyNumber * (SOLAR_EQ_CENTER_COEFF2 + SOLAR_EQ_CENTER_COEFF3 * julianCenturyNumber)) * Math.sin(degToRad(meanAnomaly))
        + (SOLAR_EQ_CENTER_COEFF4 - SOLAR_EQ_CENTER_COEFF5 * julianCenturyNumber) * Math.sin(degToRad(2 * meanAnomaly))
        + SOLAR_EQ_CENTER_COEFF6 * Math.sin(degToRad(3 * meanAnomaly));
}

function calculateSunMeanLongitude(julianCenturyNumber) {
    const meanLongitude = MEAN_LONGITUDE_SUN_AT_EPOCH + julianCenturyNumber * (MEAN_LONGITUDE_SUN_RATE + julianCenturyNumber * MEAN_LONGITUDE_SUN_CORRECTION);
    return normalizeDegrees360(meanLongitude);
}

function calculateSunTrueLongitude(julianCenturyNumber) {
    return calculateSunMeanLongitude(julianCenturyNumber) + calculateSolarEquationOfCenter(julianCenturyNumber, calculateSunMeanAnomaly(julianCenturyNumber));
}

function calculateSunApparentLongitude(trueLongitude, julianCenturyNumber) {
    return trueLongitude - APPARENT_LONGITUDE_CORRECTION_1 - APPARENT_LONGITUDE_CORRECTION_2 * Math.sin(degToRad(calculateOmega(julianCenturyNumber)));
}

function calculateDeclination(julianCenturyNumber) {
    const trueLongitude = calculateSunTrueLongitude(julianCenturyNumber);
    const lambda = calculateSunApparentLongitude(trueLongitude, julianCenturyNumber);
    const epsilon = calculateCorrectedObliquity(julianCenturyNumber);
    return radToDeg(Math.asin(Math.sin(degToRad(epsilon)) * Math.sin(degToRad(lambda))));
}

function calculateEquationOfTime(julianCenturyNumber) {
    const L0 = calculateSunMeanLongitude(julianCenturyNumber);
    const L0_rad = degToRad(L0);
    const M = calculateSunMeanAnomaly(julianCenturyNumber);
    const e = calculateEccentricity(julianCenturyNumber);
    const epsilon_corr = calculateCorrectedObliquity(julianCenturyNumber);
    const var_y = Math.pow(Math.tan(degToRad(epsilon_corr / 2)), 2);
    const M_rad = degToRad(M);
    return 4 * radToDeg(var_y * Math.sin(2 * L0_rad) - 2 * e * Math.sin(M_rad) + 4 * e * var_y * Math.sin(M_rad) * Math.cos(2 * L0_rad) - 0.5 * var_y * var_y * Math.sin(4 * L0_rad) - 1.25 * e * e * Math.sin(2 * M_rad));
}

function calculateLocalSolarTime(timestamp_utc, equationOfTime, longitude) {
    const secondsOfTheDay = timestamp_utc % SECONDS_PER_DAY;
    const totalMinutes_utc = secondsOfTheDay / SECONDS_PER_MINUTE;
    const timeOffset = longitude * MINUTES_PER_DEGREE_LONGITUDE;
    let localSolarTime = totalMinutes_utc + timeOffset + equationOfTime;
    if (localSolarTime < 0) {
        localSolarTime += MINUTES_PER_DAY;
    } else if (localSolarTime >= MINUTES_PER_DAY) {
        localSolarTime -= MINUTES_PER_DAY;
    }
    return localSolarTime / SECONDS_PER_MINUTE;
}

function calculateHourAngle(localSolarTime) {
    return normalizeDegrees180((localSolarTime - 12) * 15.0);
}

function calculateSolarElevation(hourAngle, declination, latitude) {
    const declination_rad = degToRad(declination);
    const latitude_rad = degToRad(latitude);
    const hourAngle_rad = degToRad(hourAngle);
    const elevation_rad = Math.asin(Math.sin(latitude_rad) * Math.sin(declination_rad) + 
                                  Math.cos(latitude_rad) * Math.cos(declination_rad) * Math.cos(hourAngle_rad));
    return radToDeg(elevation_rad);
}

function calculateSolarAzimuth(hourAngle, declination, elevation, latitude) {
    if (Math.abs(latitude) >= 90) {
        if (elevation > 0) {
            return latitude > 0 ? 180 : 0;
        }
        return INVALID_VALUE;
    }
    
    const latitude_rad = degToRad(latitude);
    const declination_rad = degToRad(declination);
    const zenith_rad = degToRad(calculateSolarZenithAngle(elevation));
    let cosAzimuth = (Math.sin(latitude_rad) * Math.cos(zenith_rad) - Math.sin(declination_rad)) / 
                     (Math.cos(latitude_rad) * Math.sin(zenith_rad));
    
    cosAzimuth = Math.max(-1.0, Math.min(1.0, cosAzimuth));
    let azimuth = radToDeg(Math.acos(cosAzimuth));
    
    if (hourAngle > 0) {
        azimuth = (azimuth + 180.0) % 360.0;
    } else {
        azimuth = (540.0 - azimuth) % 360.0;
    }
    
    return azimuth;
}

function calculateSolarZenithAngle(trueelevation) {
    return 90 - trueelevation;
}

function calculateCorrectedSolarElevation(elevation) {
    return elevation + calculateApproximateAtmosphericRefraction(elevation);
}

function calculateApproximateAtmosphericRefraction(elevation) {
    const elevation_rad = degToRad(elevation);
    let refraction = 0.0;
    
    if (elevation > 85.0) {
        refraction = 0.0;
    } else if (elevation > 5.0) {
        refraction = 58.1 / Math.tan(elevation_rad) - 
                    0.07 / Math.pow(Math.tan(elevation_rad), 3) + 
                    0.000086 / Math.pow(Math.tan(elevation_rad), 5);
    } else if (elevation > -0.575) {
        refraction = 1735.0 + elevation * (-518.2 + elevation * (103.4 + elevation * (-12.79 + elevation * 0.711)));
    } else {
        refraction = -20.772 / Math.tan(elevation_rad);
    }
    
    return refraction / 3600.0;
}
