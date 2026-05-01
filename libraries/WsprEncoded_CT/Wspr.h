#ifndef WSPR_H
#define WSPR_H

#include <cstdint>
#include <cstring>

class Wspr {
public:
    struct BandData {
        const char* band;
        uint32_t freq;
    };

    static const BandData* GetBandDataList() {
        static const BandData kBandDataList[kBandCount] = {
            { "2190m",        136000 },
            { "630m",         474200 },
            { "160m",        1836600 },
            { "80m",         3568600 },
            { "60m",         5287200 },
            { "40m",         7038600 },
            { "30m",        10138700 },
            { "20m",        14095600 },
            { "17m",        18104600 },
            { "15m",        21094600 },
            { "12m",        24924600 },
            { "10m",        28124600 },
            { "6m",         50293000 },
            { "4m",         70091000 },
            { "2m",        144489000 },
            { "70cm",      432300000 },
            { "23cm",     1296500000 },
        };
        return kBandDataList;
    }

    static uint8_t GetBandDataCount() {
        return kBandCount;
    }

    static uint32_t GetDialFreqFromBandStr(const char* band_str) {
        band_str = GetDefaultBandIfNotValid(band_str);
        uint32_t dial_freq = 0;
        const BandData* list = GetBandDataList();
        for (uint8_t i = 0; i < kBandCount; ++i) {
            if (strcmp(list[i].band, band_str) == 0) {
                dial_freq = list[i].freq;
                break;
            }
        }
        return dial_freq;
    }

    static const char* GetDefaultBandIfNotValid(const char* band_str) {
        const char* ret_val = "20m";
        if (band_str) {
            const BandData* list = GetBandDataList();
            for (uint8_t i = 0; i < kBandCount; ++i) {
                if (strcmp(list[i].band, band_str) == 0) {
                    ret_val = list[i].band;
                    break;
                }
            }
        }
        return ret_val;
    }

    static const uint8_t* GetPowerDbmList() {
        static const uint8_t kPowerDbmList[kPowerDbmCount] = {
             0,  3,  7,
            10, 13, 17,
            20, 23, 27,
            30, 33, 37,
            40, 43, 47,
            50, 53, 57,
            60
        };
        return kPowerDbmList;
    }

    static uint8_t GetPowerDbmCount() {
        return kPowerDbmCount;
    }

    static bool PowerDbmInSet(uint8_t power_dbm_in) {
        bool ret_val = false;
        const uint8_t* list = GetPowerDbmList();
        for (uint8_t i = 0; i < kPowerDbmCount; ++i) {
            if (power_dbm_in == list[i]) {
                ret_val = true;
                break;
            }
        }
        return ret_val;
    }

private:
    static const uint8_t kBandCount = 17;
    static const uint8_t kPowerDbmCount = 19;
};

#endif  // WSPR_H
