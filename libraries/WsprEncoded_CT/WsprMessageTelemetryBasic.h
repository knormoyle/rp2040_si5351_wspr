#ifndef WSPR_MESSAGE_TELEMETRY_BASIC_H_
#define WSPR_MESSAGE_TELEMETRY_BASIC_H_

#include <cstdint>
#include <cstring>
#include <math.h>

#include "WsprMessageTelemetryCommon.h"

class WsprMessageTelemetryBasic : public WsprMessageTelemetryCommon {
public:
    WsprMessageTelemetryBasic() {
        Reset();
    }

    void Reset() {
        WsprMessageTelemetryCommon::Reset();
        grid56_[0] = 'A';
        grid56_[1] = 'A';
        grid56_[2] = '\0';
        altitude_meters_     = 0;
        temperature_celsius_ = 0;
        voltage_volts_       = 3.3;
        speed_knots_         = 0;
        gps_is_valid_        = false;
    }

    /////////////////////////////////////////////////////////////////
    // Telemetry Setter / Getter Interface
    /////////////////////////////////////////////////////////////////

    // 'A' through 'X' for each char.
    bool SetGrid56(const char* grid56) {
        bool ret_val = false;
        if (grid56) {
            if (strlen(grid56) == 2) {
                ret_val = true;
                char grid5 = grid56[0];
                char grid6 = grid56[1];
                if      (grid5 < 'A') { grid5 = 'A'; ret_val = false; }
                else if (grid5 > 'X') { grid5 = 'X'; ret_val = false; }
                if      (grid6 < 'A') { grid6 = 'A'; ret_val = false; }
                else if (grid6 > 'X') { grid6 = 'X'; ret_val = false; }
                grid56_[0] = grid5;
                grid56_[1] = grid6;
            }
        }
        return ret_val;
    }

    // A through X for each char.
    const char* GetGrid56() const {
        return grid56_;
    }

    // 0 through 21,340, steps of 20.
    bool SetAltitudeMeters(int32_t altitude_meters) {
        bool ret_val = true;
        if      (altitude_meters < 0)     { altitude_meters = 0;     ret_val = false; }
        else if (altitude_meters > 21340) { altitude_meters = 21340; ret_val = false; }
        altitude_meters_ = altitude_meters;
        return ret_val;
    }

    // 0 through 21,340, steps of 20.
    uint16_t GetAltitudeMeters() const {
        return altitude_meters_;
    }

    // -50 through 39.
    bool SetTemperatureCelsius(int32_t temperature_celsius) {
        bool ret_val = true;
        if      (temperature_celsius < -50) { temperature_celsius = -50; ret_val = false; }
        else if (temperature_celsius >  39) { temperature_celsius =  39; ret_val = false; }
        temperature_celsius_ = temperature_celsius;
        return ret_val;
    }

    // -50 through 39.
    int8_t GetTemperatureCelsius() const {
        return temperature_celsius_;
    }

    // 3.0v through 4.95v, steps of 0.05v.
    bool SetVoltageVolts(double voltage_volts) {
        bool ret_val = true;
        if      (voltage_volts < 3.00) { voltage_volts = 3.00; ret_val = false; }
        else if (voltage_volts > 4.95) { voltage_volts = 4.95; ret_val = false; }
        voltage_volts_ = voltage_volts;
        return ret_val;
    }

    // 3.0v through 4.95v, steps of 0.05v.
    double GetVoltageVolts() const {
        return voltage_volts_;
    }

    // 0 through 82, steps of 2.
    bool SetSpeedKnots(int32_t speed_knots) {
        bool ret_val = true;
        if      (speed_knots <  0) { speed_knots =  0; ret_val = false; }
        else if (speed_knots > 82) { speed_knots = 82; ret_val = false; }
        speed_knots_ = speed_knots;
        return ret_val;
    }

    // 0 through 82, steps of 2.
    uint8_t GetSpeedKnots() const {
        return speed_knots_;
    }

    bool SetGpsIsValid(bool gps_valid) {
        gps_is_valid_ = gps_valid;
        return true;
    }

    bool GetGpsIsValid() const {
        return gps_is_valid_;
    }

    /////////////////////////////////////////////////////////////////
    // Encode / Decode Interface
    /////////////////////////////////////////////////////////////////

    void Encode() {
        EncodeCallsign();
        EncodeGridPower();
    }

    bool Decode() {
        bool ret_val = true;
        ret_val &= DecodeU4BCall();
        ret_val &= DecodeU4BGridPower();
        return ret_val;
    }

private:
    /////////////////////////////////////////////////////////////////
    // Encode
    /////////////////////////////////////////////////////////////////

    void EncodeCallsign() {
        // pick apart inputs
        char grid5 = grid56_[0];
        char grid6 = grid56_[1];
        // convert inputs into components of a big number
        uint8_t grid5_val = grid5 - 'A';
        uint8_t grid6_val = grid6 - 'A';
        uint16_t alt_frac_m = static_cast<uint16_t>(
            round(static_cast<double>(altitude_meters_) / 20));
        // convert inputs into a big number
        uint32_t val = 0;
        val *=   24; val += grid5_val;
        val *=   24; val += grid6_val;
        val *= 1068; val += alt_frac_m;
        // extract
        uint8_t id6_val = val % 26; val = val / 26;
        uint8_t id5_val = val % 26; val = val / 26;
        uint8_t id4_val = val % 26; val = val / 26;
        uint8_t id2_val = val % 36; val = val / 36;
        // convert to encoded form
        char id2 = WsprMessageTelemetryCommon::EncodeBase36(id2_val);
        char id4 = 'A' + id4_val;
        char id5 = 'A' + id5_val;
        char id6 = 'A' + id6_val;
        // store callsign
        static const uint8_t kCallsignLen = 6;
        char buf[kCallsignLen + 1] = { 0 };
        buf[0] = GetId13()[0];
        buf[1] = id2;
        buf[2] = GetId13()[1];
        buf[3] = id4;
        buf[4] = id5;
        buf[5] = id6;
        WsprMessageRegularType1::SetCallsign(buf);
    }

    void EncodeGridPower() {
        // map input presentations onto input radix (numbers within their
        // stated range of possibilities)
        uint8_t temp_c_num      = temperature_celsius_ - -50;
        uint8_t voltage_num     =
            (static_cast<uint8_t>(
                 round(((voltage_volts_ * 100) - 300) / 5)) + 20) % 40;
        uint8_t speed_knots_num =
            static_cast<uint8_t>(round(static_cast<double>(speed_knots_) / 2.0));
        uint8_t gps_valid_num   = gps_is_valid_ ? 1 : 0;
        // convert inputs into a big number
        uint32_t val = 0;
        val *= 90; val += temp_c_num;
        val *= 40; val += voltage_num;
        val *= 42; val += speed_knots_num;
        val *=  2; val += gps_valid_num;
        val *=  2; val += 1;                // basic telemetry
        // extract
        uint8_t power_val = val % 19; val = val / 19;
        uint8_t g4_val    = val % 10; val = val / 10;
        uint8_t g3_val    = val % 10; val = val / 10;
        uint8_t g2_val    = val % 18; val = val / 18;
        uint8_t g1_val    = val % 18; val = val / 18;
        // convert to encoded form
        char g1 = 'A' + g1_val;
        char g2 = 'A' + g2_val;
        char g3 = '0' + g3_val;
        char g4 = '0' + g4_val;
        // store grid
        static const uint8_t kGrid4Len = 4;
        char buf[kGrid4Len + 1] = { 0 };
        buf[0] = g1;
        buf[1] = g2;
        buf[2] = g3;
        buf[3] = g4;
        WsprMessageRegularType1::SetGrid4(buf);
        // store power
        uint8_t power_dbm = Wspr::GetPowerDbmList()[power_val];
        WsprMessageRegularType1::SetPowerDbm(power_dbm);
    }

    /////////////////////////////////////////////////////////////////
    // Decode
    /////////////////////////////////////////////////////////////////

    bool DecodeU4BCall() {
        bool ret_val = true;
        static const uint8_t kCallsignDecodeLen = 6;
        const char* call = WsprMessageRegularType1::GetCallsign();
        if (strlen(call) == kCallsignDecodeLen) {
            // break call down
            uint8_t id2 = call[1];
            uint8_t id4 = call[3];
            uint8_t id5 = call[4];
            uint8_t id6 = call[5];
            // convert to values which are offset from 'A'
            uint8_t id2_val = WsprMessageTelemetryCommon::DecodeBase36(id2);
            uint8_t id4_val = id4 - 'A';
            uint8_t id5_val = id5 - 'A';
            uint8_t id6_val = id6 - 'A';
            // integer value to use to decode
            uint32_t val = 0;
            // combine values into single integer
            val *= 36; val += id2_val;
            val *= 26; val += id4_val;   // spaces aren't used, so 26 not 27
            val *= 26; val += id5_val;   // spaces aren't used, so 26 not 27
            val *= 26; val += id6_val;   // spaces aren't used, so 26 not 27
            // extract values
            uint16_t alt_frac_m = val % 1068; val /= 1068;
            uint8_t  grid6_val  = val %   24; val /=   24;
            uint8_t  grid5_val  = val %   24; val /=   24;
            // store grid56
            char grid5 = grid5_val + 'A';
            char grid6 = grid6_val + 'A';
            char grid56[3] = { 0 };
            grid56[0] = grid5;
            grid56[1] = grid6;
            ret_val &= SetGrid56(grid56);
            // store altitude_meters
            uint16_t altitude_meters = alt_frac_m * 20;
            ret_val &= SetAltitudeMeters(altitude_meters);
        } else {
            ret_val = false;
        }
        return ret_val;
    }

    bool DecodeU4BGridPower() {
        bool ret_val = true;
        const char* grid4 = WsprMessageRegularType1::GetGrid4();
        uint8_t g1_val = grid4[0] - 'A';
        uint8_t g2_val = grid4[1] - 'A';
        uint8_t g3_val = grid4[2] - '0';
        uint8_t g4_val = grid4[3] - '0';
        uint8_t power_val = WsprMessageTelemetryCommon::DecodePowerDbmToNum(
            WsprMessageRegularType1::GetPowerDbm());
        uint32_t val = 0;
        val *= 18; val += g1_val;
        val *= 18; val += g2_val;
        val *= 10; val += g3_val;
        val *= 10; val += g4_val;
        val *= 19; val += power_val;
        uint8_t telemetry_id    = val %  2; val /=  2;
        uint8_t bit2            = val %  2; val /=  2;
        uint8_t speed_knots_num = val % 42; val /= 42;
        uint8_t voltage_num     = val % 40; val /= 40;
        uint8_t temp_c_num      = val % 90; val /= 90;
        // can only decode basic telemetry
        if (telemetry_id == 1) {
            int8_t  temp_c      = -50 + static_cast<int8_t>(temp_c_num);
            double  voltage     = 3.0 + (((voltage_num + 20) % 40) * 0.05);
            uint8_t speed_knots = speed_knots_num * 2;
            bool    gps_valid   = bit2;
            ret_val &= SetTemperatureCelsius(temp_c);
            ret_val &= SetVoltageVolts(voltage);
            ret_val &= SetSpeedKnots(speed_knots);
            ret_val &= SetGpsIsValid(gps_valid);
        } else {
            ret_val = false;
        }
        return ret_val;
    }

private:
    char     grid56_[3];
    uint16_t altitude_meters_;
    int8_t   temperature_celsius_;
    double   voltage_volts_;
    uint8_t  speed_knots_;
    bool     gps_is_valid_;
};

#endif  // WSPR_MESSAGE_TELEMETRY_BASIC_H_
