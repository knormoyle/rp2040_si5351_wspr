#pragma once

#include <array>
#include <cstdint>

#include "Wspr.h"
#include "WsprUtl.h"


class WsprChannelMap
{
public:

    static uint16_t GetDefaultChannelIfNotValid(uint16_t channel)
    {
        uint16_t retVal = 0;

        if (channel <= 599)
        {
            retVal = channel;
        }

        return retVal;
    }

    // minute list, some bands are defined as rotation from 20m
    static std::array<uint8_t, 5> GetMinuteListForBand(const char *band)
    {
        band = Wspr::GetDefaultBandIfNotValid(band);

        // get index into list (guaranteed to be found)
        uint8_t idx = 0;
        for (const auto &bandData : Wspr::GetBandDataList())
        {
            if (bandData.band == band)
            {
                break;
            }

            ++idx;
        }

        // rotation is modded place within this list
        std::array<uint8_t, 5> rotationList = { 4, 2, 0, 3, 1 };
        uint8_t rotation = rotationList[idx % 5];

        std::array<uint8_t, 5> minuteList = { 8, 0, 2, 4, 6 };
        minuteList = WsprUtl::Rotate(minuteList, rotation);

        return minuteList;
    }

    struct ChannelDetails
    {
        const char   *band     = "20m";
        uint16_t      channel  = 0;
        char          id1      = '0';
        char          id3      = '0';
        char          id13[3] = { '0', '0', '\0' };
        uint8_t       min      = 8;
        uint8_t       lane     = 1;
        uint32_t      freq     = 14097020ULL;
        uint32_t      freqDial = 14095600ULL;
    };

    static ChannelDetails GetChannelDetails(const char *band, uint16_t channel)
    {
        band    = Wspr::GetDefaultBandIfNotValid(band);
        channel = GetDefaultChannelIfNotValid(channel);

        std::array<char, 3>  id1List = { '0', '1', 'Q' };
        std::array<char, 10> id3List = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };

        uint32_t dialFreq = Wspr::GetDialFreqFromBandStr(band);

        uint32_t freqTxLow = dialFreq + 1500 - 100;
        uint32_t freqTxHigh = dialFreq + 1500 + 100;
        uint32_t freqTxWindow = freqTxHigh - freqTxLow;

        uint8_t freqBandCount = 5;
        uint8_t bandSizeHz = freqTxWindow / freqBandCount;

        std::array<uint8_t, 4> freqBandList = { 1, 2, 4, 5 };    // skip middle band 3, but really label as 1,2,3,4

        std::array<uint8_t, 5> minuteList = GetMinuteListForBand(band);

        uint8_t rowCount = 0;
        for (auto freqBand : freqBandList)
        {
            // figure out the frequency
            uint8_t freqBandLow    = (freqBand - 1) * bandSizeHz;
            uint8_t freqBandHigh   = freqBandLow + bandSizeHz;
            uint8_t freqBandCenter = (freqBandHigh + freqBandLow) / 2;

            uint8_t rowsPerCol = freqBandCount * freqBandList.size();

            for (auto minute : minuteList)
            {
                uint8_t freqBandLabel = freqBand;
                if (freqBandLabel >= 4) { freqBandLabel = freqBandLabel - 1; }
                
                for (char id1 : id1List)
                {
                    uint8_t  colCount = 0;
                    uint16_t id1Offset = 0;
                    if (id1 == '1') { id1Offset = 200; }
                    if (id1 == 'Q') { id1Offset = 400; }

                    for (char id3 : id3List)
                    {
                        uint16_t channelCalc = id1Offset + (colCount * rowsPerCol) + rowCount;

                        if (channelCalc == channel)
                        {
                            ChannelDetails cd;
                            cd.band       = band;
                            cd.channel    = channelCalc;
                            cd.id1        = id1;
                            cd.id3        = id3;
                            cd.id13[0]    = id1;
                            cd.id13[1]    = id3;
                            cd.min        = minute;
                            cd.lane       = freqBandLabel;
                            cd.freq       = freqTxLow + freqBandCenter;
                            cd.freqDial   = dialFreq;

                            return cd;
                        }

                        ++colCount;
                    }
                }

                ++rowCount;
            }
        }

        return {};
    }
};


