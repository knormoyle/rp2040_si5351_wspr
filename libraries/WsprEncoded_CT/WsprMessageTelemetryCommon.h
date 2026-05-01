#ifndef WSPR_MESSAGE_TELEMETRY_COMMON_H
#define WSPR_MESSAGE_TELEMETRY_COMMON_H

#include <cstdint>
#include <cstring>
#include "WsprMessageRegularType1.h"

class WsprMessageTelemetryCommon : public WsprMessageRegularType1 {
public:
    WsprMessageTelemetryCommon() {
        Reset();
    }

    void Reset() {
        WsprMessageRegularType1::Reset();
        id13_[0] = '0';
        id13_[1] = '0';
        id13_[2] = '\0';
    }

    // 00 through 09, 10 through 19, Q0 through Q9.
    bool SetId13(const char* id13) {
        bool ret_val = false;
        if (id13 && strlen(id13) == 2) {
            char id1 = id13[0];
            char id3 = id13[1];
            if ((id1 == '0' || id1 == '1' || id1 == 'Q') &&
                ('0' <= id3 && id3 <= '9')) {
                id13_[0] = id1;
                id13_[1] = id3;
                ret_val = true;
            }
        }
        return ret_val;
    }

    // 00 through 09, 10 through 19, Q0 through Q9.
    const char* GetId13() const {
        return id13_;
    }

protected:
    /////////////////////////////////////////////////////////////////
    // Encode/Decode utilities
    /////////////////////////////////////////////////////////////////
    static char EncodeBase36(uint8_t val) {
        char ret_val;
        if (val < 10) {
            ret_val = '0' + val;
        } else {
            ret_val = 'A' + (val - 10);
        }
        return ret_val;
    }

    static uint8_t DecodeBase36(char c) {
        uint8_t ret_val = 0;
        uint8_t c_val = c;
        uint8_t a_val    = 'A';
        uint8_t z_val    = 'Z';
        uint8_t zero_val = '0';
        if (a_val <= c_val && c_val <= z_val) {
            ret_val = 10 + (c_val - a_val);
        } else {
            ret_val = c_val - zero_val;
        }
        return ret_val;
    }

    static uint8_t DecodePowerDbmToNum(uint8_t power_dbm) {
        uint8_t ret_val = 0;
        const uint8_t* power_dbm_list = Wspr::GetPowerDbmList();
        uint8_t power_dbm_count = Wspr::GetPowerDbmCount();
        for (uint8_t i = 0; i < power_dbm_count; ++i) {
            if (power_dbm == power_dbm_list[i]) {
                ret_val = i;
            }
        }
        return ret_val;
    }

private:
    char id13_[3];
};

#endif  // WSPR_MESSAGE_TELEMETRY_COMMON_H
