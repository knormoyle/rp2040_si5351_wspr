/*
 * solarfunctions: precise solar calculations
 * Copyright (c) 2024 Christian Menne
 * https://github.com/chrmenne/solarfunctions
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SOLARFUNCTIONS_H
#define SOLARFUNCTIONS_H

#define VERSION 1.0

#define INVALID_VALUE 999999999.999999d

// SUPPORT FUNCTIONS 

/**
 * @brief Converts an angle from degrees to radians.
 *
 * This function takes an angle in degrees and returns its equivalent 
 * in radians. Useful for calculations requiring angular measurements 
 * in radians, which are standard for trigonometric functions in C++.
 *
 * @param degrees The angle in degrees.
 * @return The equivalent angle in radians.
 */
double degToRad(double degrees);

/**
 * @brief Converts an angle from radians to degrees.
 *
 * This function takes an angle in radians and returns its equivalent 
 * in degrees. Useful for converting angular measurements into degrees, 
 * often for more interpretable output or for use in calculations 
 * that require degree-based values.
 *
 * @param radians The angle in radians.
 * @return The equivalent angle in degrees.
 */
double radToDeg(double radians);

/**
 * @brief Normalizes an angle in degrees to a value within the [-180, 180) range.
 *
 * This function normalizes an angle given in degrees, ensuring it falls 
 * within a standard range. It restricts the angle to values between 
 * -180 (including) and 180 (excluding) degrees. This is commonly used for 
 * representing angles symmetrically around zero.
 *
 * @param angle The angle in degrees to be normalized.
 * @return The normalized angle in degrees within the [-180, 180) range.
 */ 
double normalizeDegrees180(double angle);

/**
 * @brief Normalizes an angle in degrees to a value within the [0, 360) range.
 *
 * This function normalizes an angle given in degrees, ensuring it falls 
 * within a standard range. It restricts the angle to values between 
 * 0 (including) and 360 (excluding) degrees.
 *
 * @param angle The angle in degrees to be normalized.
 * @return The normalized angle in degrees within the [0, 360) range.
 */ 
double normalizeDegrees360(double angle);

/**
 * @brief Normalizes an angle in radians to a value within the [-π, π) range.
 *
 * This function normalizes an angle provided in radians, ensuring it falls 
 * within a standard range. It restricts the angle to values between 
 * -π (including) and π (excluding). This is commonly used for 
 * representing angles symmetrically around zero.
 *
 * @param angle The angle in radians to be normalized.
 * @return The normalized angle in radians within the [-π, π) range.
 */
double normalizeRadiansPi(double angle);

/**
 * @brief Normalizes an angle in radians to a value within the [0, 2π) range.
 *
 * This function normalizes an angle provided in radians, ensuring it falls 
 * within a standard range. It restricts the angle to values between 
 * 0 (including) and 2π (excluding). 
 *
 * @param angle The angle in radians to be normalized.
 * @return The normalized angle in radians within the [0, 2π) range.
 */
double normalizeRadians2Pi(double angle);

// DATE AND TIME CONVERSIONS

/**
 * @brief The Julian Date based on a given UTC timestamp.
 *
 * This function returns the Julian Date, which is a continuous count of days 
 * from noon (12:00 UTC) on January 1, 4713 BC (proleptic Julian calendar), 
 * based on a given UTC timestamp.
 *
 * @param timestamp_utc The UTC timestamp in seconds since the Unix epoch (January 1, 1970, 00:00 UTC).
 * @return The Julian Date as a double-precision floating-point number, representing days 
 *         since January 1, 4713 BC at 12:00 UTC.
 */
double calculateJulianDate(long timestamp_utc);

/**
 * @brief The Julian Century from a UTC timestamp.
 *
 * Converts a given UTC timestamp to the Julian Century, which represents the number of 
 * Julian centuries (100 Julian years of 365.25 days) since January 1, 2000, at 12:00 UTC. 
 * This value is widely used in astronomical calculations.
 *
 * @param timestamp_utc The UTC timestamp in seconds since the Unix epoch (January 1, 1970, 00:00 UTC).
 * @return The Julian Century as a double-precision floating-point number, representing centuries 
 *         since January 1, 2000, at 12:00 UTC.
 */
double calculateJulianCenturyNumber(long timestamp_utc);

// EARTH'S ORBIT AND MOVEMENT

/**
 * @brief The eccentricity of Earth's orbit for a given Julian Century.
 *
 * Eccentricity is the measure of deviation of Earth's orbit from a perfect circle. 
 * This function calculates the eccentricity at a given Julian Century (julianCenturyNumber) from 
 * January 1, 2000, at 12:00 UTC.
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The eccentricity of Earth's orbit as a dimensionless double.
 */
double calculateEccentricity(double julianCenturyNumber);

/**
 * @brief The mean obliquity of the ecliptic for a given Julian Century.
 *
 * The obliquity of the ecliptic is the angle between Earth's equatorial plane and the orbital 
 * plane, influencing seasonal changes. This function returns the mean obliquity for a given 
 * Julian Century (julianCenturyNumber).
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The mean obliquity of the ecliptic in degrees.
 */
double calculateObliquity(double julianCenturyNumber);

/**
 * @brief The corrected obliquity directly from Julian Century.
 *
 * This function computes the corrected obliquity, combining the mean obliquity calculation 
 * and nutation correction from the Julian Century (julianCenturyNumber).
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The corrected obliquity in degrees.
 */
double calculateCorrectedObliquity(double julianCenturyNumber);

/**
 * @brief The mean obliquity corrected for nutation, based on obliquity and omega.
 *
 * This function adjusts the mean obliquity by adding a small correction factor based on the 
 * nutation angle omega, returning the corrected obliquity.
 *
 * @param obliquity The mean obliquity of the ecliptic in degrees.
 * @param omega     The longitude of the ascending lunar node in degrees.
 * @return The corrected obliquity in degrees.
 */
double calculateCorrectedObliquity(double obliquity, double omega);

/**
 * @brief The longitude of the ascending lunar node, Omega, for a given Julian Century.
 *
 * Omega is the angle indicating the position of the ascending lunar node, which is crucial 
 * for adjusting obliquity and for nutation-related calculations.
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The nutation angle Omega in degrees.
 */
double calculateOmega(double julianCenturyNumber);

/**
 * @brief The nutation in longitude.
 *
 * Nutation is a small oscillation in Earth's orientation affecting apparent celestial positions. 
 * This function calculates nutation based on the Julian Century (julianCenturyNumber) and returns the angle in degrees.
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The nutation in longitude in degrees.
 */
double calculateNutation(double julianCenturyNumber);

/**
 * @brief The Sun's radius vector, the distance between the Sun and Earth in astronomical units (AU),
 *        for a given eccentricity and true anomaly.
 *
 * The radius vector calculation accounts for Earth's orbital eccentricity and the true anomaly,
 * representing the Sun-Earth distance as it varies with orbital position. Useful in determining
 * the apparent size and intensity of sunlight.
 *
 * @param eccentricity Earth's orbital eccentricity.
 * @param trueAnomaly  The true anomaly of the Sun in degrees, indicating its position along its orbit.
 * @return Sun-Earth radius vector in astronomical units (AU).
 */
double calculateSunRadiusVector(double eccentricity, double trueAnomaly);

// SOLAR ANGLES AND ANOMALY

/**
 * @brief The Sun's mean anomaly for a given Julian Century.
 *
 * The mean anomaly is the angle between the position of an orbiting body (the Sun) and the 
 * closest point of approach (perihelion) in an idealized circular orbit. This function calculates 
 * the Sun's mean anomaly based on the Julian Century (julianCenturyNumber).
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The Sun's mean anomaly in degrees.
 */
double calculateSunMeanAnomaly(double julianCenturyNumber);

/**
 * @brief The Sun's true anomaly for a given Julian Century, including the effect of the equation of center.
 *
 * The true anomaly represents the angle between the Sun’s current position and perihelion,
 * adjusted for orbital eccentricity to reflect the actual elliptical path. This function calculates 
 * the Sun's true anomaly based on the Julian Century (julianCenturyNumber).
 *
 * @param julianCenturyNumber The Julian Century since January 1, 2000, at 12:00 UTC.
 * @return The Sun's true anomaly in degrees.
 */
double calculateSunTrueAnomaly(double julianCenturyNumber);

/**
 * @brief The Solar Equation of Center, an orbital correction for the Sun's true position on a given Julian Century.
 *
 * The Solar Equation of Center accounts for the Sun’s orbit eccentricity, correcting the Sun's mean 
 * anomaly to determine its true anomaly. This function adjusts the mean anomaly (meanAnomaly) based 
 * on the Julian Century (julianCenturyNumber).
 *
 * @param julianCenturyNumber          The Julian Century from January 1, 2000, at 12:00 UTC.
 * @param meanAnomaly The Sun's mean anomaly in degrees.
 * @return The Solar Equation of Center in degrees.
 */
double calculateSolarEquationOfCenter(double julianCenturyNumber, double meanAnomaly);

/**
 * @brief The mean longitude of the Sun for a given Julian Century.
 *
 * The mean longitude is the average angle that the Sun makes with respect to the vernal equinox,
 * corrected to a circular orbit. It increases over time due to Earth's orbit around the Sun.
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The Sun's mean longitude in degrees.
 */
double calculateSunMeanLongitude(double julianCenturyNumber);

/**
 * @brief The true longitude of the Sun for a given Julian Century.
 *
 * The true longitude incorporates corrections from the Solar Equation of Center to account for
 * the Sun's elliptical orbit, providing a more accurate measure of the Sun's position.
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The Sun's true longitude in degrees.
 */
double calculateSunTrueLongitude(double julianCenturyNumber);

/**
 * @brief The apparent longitude of the Sun for a given Julian Century.
 *
 * The apparent longitude includes a small correction for the Sun's position due to the Earth's
 * orbital eccentricity and atmospheric effects, yielding the Sun's visible position in the sky.
 *
 * @param trueLongitude The Sun's true longitude in degrees.
 * @param julianCenturyNumber            The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The Sun's apparent longitude in degrees.
 */
double calculateSunApparentLongitude(double trueLongitude, double julianCenturyNumber);

/**
 * @brief The Sun's right ascension for a given Julian Century.
 *
 * The right ascension represents the angle of the Sun measured along the celestial equator from the 
 * vernal equinox, taking into account the Sun's position (lambda) and the obliquity of the ecliptic.
 *
 * @param lambda  The Sun's apparent longitude in degrees.
 * @param epsilon The obliquity of the ecliptic in degrees.
 * @return The Sun's right ascension in degrees.
 */
double calculateRightAscension(double lambda, double epsilon);

/**
 * @brief The Sun's declination for a given Julian Century.
 *
 * The declination represents the Sun's angular distance north or south of the celestial equator.
 * It varies throughout the year as the Earth orbits the Sun. This function calculates the declination
 * based on the Julian Century (julianCenturyNumber).
 *
 * @param julianCenturyNumber The Julian Century from January 1, 2000, at 12:00 UTC.
 * @return The Sun's declination in degrees.
 */
double calculateDeclination(double julianCenturyNumber);

// SOLAR TIME

/**
 * @brief The Equation of Time (EOT), which accounts for the difference between solar time and clock time for a given Julian Century.
 *
 * The EOT corrects for the eccentricity of Earth's orbit and axial tilt, providing the offset between apparent 
 * solar time (sundial time) and mean solar time (clock time).
 *
 * @param julianCenturyNumber Julian Century, representing the number of centuries since J2000.0.
 * @return Equation of Time in minutes, where positive values indicate a "fast" Sun and negative values a "slow" Sun.
 */
double calculateEquationOfTime(double julianCenturyNumber);

/**
 * @brief The local solar time based on UTC time, equation of time, and longitude.
 *
 * Local solar time is derived from UTC time, adjusted by the equation of time and the longitude of the observer's
 * location. This value is the solar hour angle, which relates to the Sun’s position in the sky. Basically, this 
 * is the time a sundial would show.
 *
 * @param timestamp_utc  The current time in UTC, expressed in hours (e.g., 13.5 for 13:30).
 * @param equationOfTime The equation of time in minutes, accounting for the difference between solar time 
 *                       and standard time.
 * @param longitude      The observer's longitude in degrees (east is positive, west is negative).
 * @return The local solar time in hours.
 */
double calculateLocalSolarTime(long timedouble_utc, double equationOfTime, double longitude);

/**
 * @brief The local solar noon, the time when the Sun reaches its highest point in the sky.
 *
 * This function determines local solar noon based on the observer's longitude, 
 * the equation of time, and the specified UTC offset. Local solar noon represents 
 * the midpoint of the solar day when the Sun crosses the observer's meridian.
 *
 * @param longitude       Observer’s longitude in degrees (positive for east, negative for west).
 * @param equationOfTime  Equation of time in minutes, which corrects for the apparent solar motion due to Earth's tilt and orbit.
 * @param timezone_offset Offset from UTC in hours, including any adjustments for daylight saving time (e.g., +2 for CEST in summer, +1 for CET in winter).
 *                        Positive offsets represent locations east of UTC (e.g., UTC+1 for Central Europe),
 *                        while negative offsets represent locations west of UTC (e.g., UTC-5 for Eastern Standard Time).
 * @return Local solar noon in hours (decimal format).
 */
double calculateLocalSolarNoon(double timezone_offset, double equationOfTime, double longitude);

/**
 * @brief The time of sunrise or sunset based on the solar noon and the hour angle.
 *
 * This function adds or subtracts the hour angle from solar noon to yield the time of sunrise 
 * or sunset in hours.
 *
 * @param solarNoon    Time of solar noon in hours.
 * @param hourAngle_h0 Hour angle in hours, calculated for either sunrise or sunset.
 * @return Time of sunrise or sunset in hours (decimal format). 
 *         INVALID_VALUE is returned for polar days/nights with no sunset/sunrise.
 */
double calculateSunriseSunsetTime(double solarNoon, double hourAngle_h0);

/**
 * @brief The total duration of daylight in minutes, based on the hour angle for sunrise or sunset.
 *
 * This function doubles the sunrise hour angle and converts it to minutes to give the total duration of 
 * daylight in a day.
 *
 * @param sunriseSunsetHourAngle The hour angle for sunrise (or sunset, either will work as the day is "symmetric") in degrees.
 * @return Total duration of daylight in minutes.
 */
double calculateDaylightMinutes(double sunriseSunsetHourAngle);

// SOLAR POSITION

/**
 * @brief The hour angle of the Sun at a given local solar time.
 * 
 * The hour angle represents the position of the Sun relative to the observer's meridian, 
 * with a value of 0° indicating solar noon (when the Sun is at its highest point in the sky).
 * Positive values represent times after solar noon, while negative values represent times
 * before solar noon.
 *
 * The hour angle is calculated based on the difference between local solar time and 12:00 (solar noon),
 * with each hour corresponding to a 15° shift. The result is normalized to the range [-180°, 180°].
 *
 * @param localSolarTime The local solar time in hours, where 12 represents solar noon.
 * @return The hour angle in degrees, normalized to the range [-180°, 180°].
 */
double calculateHourAngle(double localSolarTime);

/**
 * @brief The hour angle at sunrise, which is the angular distance between the sun's 
 *        position at sunrise and solar noon, based on the specified latitude and solar declination.
 *
 * This function computes the hour angle, h0, at sunrise (in degrees) for a given latitude and 
 * declination. The hour angle represents the time it takes for the sun to move across the sky
 * from the horizon at sunrise to solar noon, where it reaches its highest point.
 * 
 * The hour angle at sunrise is always negative.
 *
 * @param declination  The solar declination in degrees, representing the angle between the sun's 
 *                     rays and the Earth's equatorial plane.
 * @param latitude 	   The observer's latitude in degrees.
 * @return The hour angle at sunrise in degrees. This value is constrained within [-180°, 0°] 
 *         to handle edge cases like polar days and polar nights.
 *
 * @note The returned angle will be positive for standard day-night cycles. In polar regions,
 *       where the sun does not set or rise for extended periods, the hour angle is capped at 
 *       -180° or 0° to indicate continuous daylight or darkness.
 */
double calculateSunriseHourAngle(double declination, double latitude);

/**
 * @brief The hour angle at sunset, based on the observer's latitude and the solar declination.
 *
 * The hour angle at sunset (or sunrise) represents the angular distance from solar noon at which 
 * the Sun appears at the horizon. This value helps determine sunrise and sunset times.
 *
 * @param declination Solar declination in degrees (north positive, south negative).
 * @param latitude    Observer’s latitude in degrees (north positive, south negative).
 * @return Hour angle for sunset (or sunrise) in degrees.
 */
double calculateSunsetHourAngle(double declination, double latitude);

/**
 * @brief The solar elevation angle, the angle between the Sun and the observer’s horizon.
 *
 * Solar elevation indicates how high the Sun is above the horizon and is calculated using the hour 
 * angle, solar declination, and observer’s latitude.
 *
 * @param hourAngle    The hour angle of the Sun in degrees (positive in the afternoon, negative in the morning).
 * @param declination  Solar declination in degrees (north positive, south negative).
 * @param latitude     Observer’s latitude in degrees (north positive, south negative).
 * @return Solar elevation angle in degrees.
 */
double calculateSolarElevation(double hourAngle, double declination, double latitude);

/**
 * @brief Applies an atmospheric correction to the solar elevation angle to account for refraction.
 *
 * This correction adjusts the calculated solar elevation by a standard refraction value, making 
 * the observed elevation more accurate for typical atmospheric conditions near the horizon.
 *
 * @param elevation  Solar elevation angle in degrees, before applying refraction.
 * @return Corrected solar elevation angle in degrees.
 */
double calculateCorrectedSolarElevation(double elevation);

/**
 * @brief Applies a specific atmospheric refraction correction to the solar elevation angle.
 *
 * This function allows for a custom refraction adjustment, making it useful for precise 
 * applications or non-standard atmospheric conditions.
 *
 * @param elevation  Solar elevation angle in degrees, before applying refraction.
 * @param refraction Atmospheric refraction in degrees to apply.
 * @return Corrected solar elevation angle in degrees.
 */
double calculateCorrectedSolarElevation(double elevation, double refraction);

/**
 * @brief The solar azimuth angle, the compass direction from which the sunlight is coming.
 *
 * The azimuth angle is measured clockwise from true north, allowing for orientation relative to the 
 * observer’s location and time of day. It requires the observer's latitude, solar declination, hour 
 * angle, and solar elevation angle.
 *
 * @param hourAngle   Hour angle of the Sun in degrees (positive in the afternoon, negative in the morning).
 * @param declination Solar declination in degrees.
 * @param elevation   Solar elevation angle in degrees.
 * @param latitude    Observer’s latitude in degrees (north positive, south negative).
 * @return Solar azimuth angle in degrees, measured clockwise from true north.
           Returns 180 for latitute 90° (North Pole) and 0 for latitude -90° (South Pole), if the sun is visible. 
 */
double calculateSolarAzimuth(double hourAngle, double declination, double elevation, double latitude);

/**
 * @brief The solar zenith angle, the angle between the Sun and the observer’s zenith (directly overhead).
 *
 * The zenith angle complements the solar elevation angle, giving the position of the Sun relative 
 * to a vertical line through the observer.
 *
 * @param elevation Solar elevation angle in degrees.
 * @return Solar zenith angle in degrees.
 */
double calculateSolarZenithAngle(double elevation);

/**
 * @brief The Sun's radius vector, the distance between the Sun and Earth in astronomical units (AU),
 *        for a given Julian Century.
 *
 * The radius vector accounts for Earth’s orbital eccentricity, and it is used to determine the 
 * apparent size and intensity of sunlight.
 *
 * @param julianCenturyNumber Julian Century, representing the number of centuries since J2000.0.
 * @return Sun-Earth radius vector in astronomical units (AU).
 */
double calculateSunRadiusVector(double julianCenturyNumber);

/**
 * @brief The approximate atmospheric refraction correction, which adjusts the Sun's apparent position near the horizon.
 *
 * This correction accounts for the bending of sunlight through Earth's atmosphere, affecting the observed solar elevation.
 * It is particularly important for solar calculations near sunrise and sunset.
 *
 * @param elevation The observed solar elevation angle in degrees.
 * @return Atmospheric refraction in degrees to be added to the true solar elevation angle.
 */
double calculateApproximateAtmosphericRefraction(double elevation);

#endif // SOLARFUNCTIONS_H
