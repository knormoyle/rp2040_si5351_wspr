/*
 (c) 2011-2015, Vladimir Agafonkin
 SunCalc is a C++ library for calculating sun/moon position and light phases.
 https://github.com/mourner/suncalc
*/

#include <cmath>
#include <vector>
#include <map>
#include <chrono>
#include <ctime>
// kbn 1/2/25 added
#include <string>

class SunCalc {
private:
    // Constants
    // there is a #define for PI in ArduinoCore-API/api/Common.h already
    // static constexpr double PI = M_PI;
    static constexpr double rad = PI / 180.0;
    static constexpr double dayMs = 1000.0 * 60.0 * 60.0 * 24.0;
    static constexpr int J1970 = 2440588;
    static constexpr int J2000 = 2451545;
    static constexpr double J0 = 0.0009;
    static constexpr double e = rad * 23.4397; // obliquity of the Earth

    struct TimeConfig {
        double angle;
        std::string riseName;
        std::string setName;
    };

    static std::vector<TimeConfig> times;

    // Helper functions
    static double toJulian(const std::chrono::system_clock::time_point& date) {
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
            date.time_since_epoch()).count();
        return milliseconds / dayMs - 0.5 + J1970;
    }

    static std::chrono::system_clock::time_point fromJulian(double j) {
        auto milliseconds = std::chrono::milliseconds(
            static_cast<long long>((j + 0.5 - J1970) * dayMs));
        return std::chrono::system_clock::time_point(milliseconds);
    }

    static double toDays(const std::chrono::system_clock::time_point& date) {
        return toJulian(date) - J2000;
    }

    static double rightAscension(double l, double b) {
        return atan2(sin(l) * cos(e) - tan(b) * sin(e), cos(l));
    }

    static double declination(double l, double b) {
        return asin(sin(b) * cos(e) + cos(b) * sin(e) * sin(l));
    }

    static double azimuth(double H, double phi, double dec) {
        return atan2(sin(H), cos(H) * sin(phi) - tan(dec) * cos(phi));
    }

    static double altitude(double H, double phi, double dec) {
        return asin(sin(phi) * sin(dec) + cos(phi) * cos(dec) * cos(H));
    }

    static double siderealTime(double d, double lw) {
        return rad * (280.16 + 360.9856235 * d) - lw;
    }

    static double astroRefraction(double h) {
        if (h < 0)
            h = 0;
        return 0.0002967 / tan(h + 0.00312536 / (h + 0.08901179));
    }

    static double solarMeanAnomaly(double d) {
        return rad * (357.5291 + 0.98560028 * d);
    }

    static double eclipticLongitude(double M) {
        double C = rad * (1.9148 * sin(M) + 0.02 * sin(2 * M) + 0.0003 * sin(3 * M));
        double P = rad * 102.9372;
        return M + C + P + PI;
    }

    struct SunCoords {
        double dec;
        double ra;
    };

    static SunCoords sunCoords(double d) {
        double M = solarMeanAnomaly(d);
        double L = eclipticLongitude(M);
        return {declination(L, 0), rightAscension(L, 0)};
    }

public:
    struct Position {
        double azimuth;
        double altitude;
    };

    static Position getPosition(double *elev, double *azim, const std::chrono::system_clock::time_point& date, 
        double lat, double lng) {

        double lw = rad * -lng;
        double phi = rad * lat;
        double d = toDays(date);

        auto c = sunCoords(d);
        double H = siderealTime(d, lw) - c.ra;

        double azimuth_here = azimuth(H, phi, c.dec);
        double altitude_here = altitude(H, phi, c.dec);
        *azim = azimuth_here;
        *elev = altitude_here;

        return {azimuth(H, phi, c.dec), altitude(H, phi, c.dec)};
    }

    // Additional methods would go here (getMoonPosition, getMoonIllumination, etc.)
    // They would follow the same pattern of conversion from JavaScript
};

// Initialize the static times vector
std::vector<SunCalc::TimeConfig> SunCalc::times = {
    {-0.833, "sunrise", "sunset"},
    {-0.3, "sunriseEnd", "sunsetStart"},
    {-6, "dawn", "dusk"},
    {-12, "nauticalDawn", "nauticalDusk"},
    {-18, "nightEnd", "night"},
    {6, "goldenHourEnd", "goldenHour"}
};
