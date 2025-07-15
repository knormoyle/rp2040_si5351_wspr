#pragma once

#include <array>
#include <cstdint>
#include <cstring>


class Wspr
{
public:

    struct BandData
    {
        const char *band;
        uint32_t freq;
    };

    static const std::array<BandData, 17> &GetBandDataList()
    {
        static const std::array<BandData, 17> bandDataList = {
            BandData{ "2190m",        136000 },
            BandData{ "630m",         474200 },
            BandData{ "160m",        1836600 },
            BandData{ "80m",         3568600 },
            BandData{ "60m",         5287200 },
            BandData{ "40m",         7038600 },
            BandData{ "30m",        10138700 },
            BandData{ "20m",        14095600 },
            BandData{ "17m",        18104600 },
            BandData{ "15m",        21094600 },
            BandData{ "12m",        24924600 },
            BandData{ "10m",        28124600 },
            BandData{ "6m",         50293000 },
            BandData{ "4m",         70091000 },
            BandData{ "2m",        144489000 },
            BandData{ "70cm",      432300000 },
            BandData{ "23cm",     1296500000 },
        };

        return bandDataList;
    }

    static uint32_t GetDialFreqFromBandStr(const char *bandStr)
    {
        bandStr = GetDefaultBandIfNotValid(bandStr);

        uint32_t dialFreq = 0;
        for (const auto &bandData : GetBandDataList())
        {
            if (strcmp(bandData.band, bandStr) == 0)
            {
                dialFreq = bandData.freq;

                break;
            }
        }

        return dialFreq;
    }

    static const char *GetDefaultBandIfNotValid(const char *bandStr)
    {
        const char *retVal = "20m";

        if (bandStr)
        {
            for (const auto &bandData : GetBandDataList())
            {
                if (strcmp(bandData.band, bandStr) == 0)
                {
                    retVal = bandData.band;

                    break;
                }
            }
        }

        return retVal;
    }

    static std::array<uint8_t, 19> &GetPowerDbmList()
    {
        static std::array<uint8_t, 19> powerDbmList = {
             0,  3,  7,
            10, 13, 17,
            20, 23, 27,
            30, 33, 37,
            40, 43, 47,
            50, 53, 57,
            60
        };

        return powerDbmList;
    }

    static bool PowerDbmInSet(uint8_t powerDbmIn)
    {
        bool retVal = false;

        for (uint8_t powerDbm : GetPowerDbmList())
        {
            if (powerDbmIn == powerDbm)
            {
                retVal = true;

                break;
            }
        }

        return retVal;
    }
};