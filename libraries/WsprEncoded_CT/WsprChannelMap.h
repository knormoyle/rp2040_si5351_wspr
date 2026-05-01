#ifndef WSPR_CHANNEL_MAP_H
#define WSPR_CHANNEL_MAP_H

#include <cstdint>

#include "Wspr.h"
#include "WsprUtl.h"

class WsprChannelMap {
public:
    static uint16_t GetDefaultChannelIfNotValid(uint16_t channel) {
        uint16_t ret_val = 0;
        if (channel <= 599) {
            ret_val = channel;
        }
        return ret_val;
    }

    // Minute list. Some bands are defined as a rotation from 20m.
    //
    // Fills `minute_list_out` with 5 entries.
    static void GetMinuteListForBand(const char* band,
                                     uint8_t minute_list_out[5]) {
        band = Wspr::GetDefaultBandIfNotValid(band);
        // get index into list (guaranteed to be found)
        uint8_t idx = 0;
        const Wspr::BandData* band_list = Wspr::GetBandDataList();
        uint8_t band_count = Wspr::GetBandDataCount();
        for (uint8_t i = 0; i < band_count; ++i) {
            if (band_list[i].band == band) {
                break;
            }
            ++idx;
        }
        // rotation is modded place within this list
        const uint8_t kRotationList[5] = { 4, 2, 0, 3, 1 };
        uint8_t rotation = kRotationList[idx % 5];
        minute_list_out[0] = 8;
        minute_list_out[1] = 0;
        minute_list_out[2] = 2;
        minute_list_out[3] = 4;
        minute_list_out[4] = 6;
        WsprUtl::Rotate5(minute_list_out, rotation);
    }

    struct ChannelDetails {
        const char* band      = "20m";
        uint16_t    channel   = 0;
        char        id1       = '0';
        char        id3       = '0';
        char        id13[3]   = { '0', '0', '\0' };
        uint8_t     min       = 8;
        uint8_t     lane      = 1;
        uint32_t    freq      = 14097020UL;
        uint32_t    freq_dial = 14095600UL;
    };

    static ChannelDetails GetChannelDetails(const char* band,
                                            uint16_t channel) {
        band    = Wspr::GetDefaultBandIfNotValid(band);
        channel = GetDefaultChannelIfNotValid(channel);
        const char kId1List[3] = { '0', '1', 'Q' };
        const char kId3List[10] = {
            '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
        };
        uint32_t dial_freq = Wspr::GetDialFreqFromBandStr(band);
        uint32_t freq_tx_low    = dial_freq + 1500 - 100;
        uint32_t freq_tx_high   = dial_freq + 1500 + 100;
        uint32_t freq_tx_window = freq_tx_high - freq_tx_low;
        const uint8_t kFreqBandCount = 5;
        uint8_t band_size_hz = freq_tx_window / kFreqBandCount;
        // skip middle band 3, but really label as 1,2,3,4
        const uint8_t kFreqBandList[4] = { 1, 2, 4, 5 };
        const uint8_t kFreqBandListLen = 4;
        uint8_t minute_list[5];
        GetMinuteListForBand(band, minute_list);
        uint8_t row_count = 0;
        for (uint8_t fb_idx = 0; fb_idx < kFreqBandListLen; ++fb_idx) {
            uint8_t freq_band = kFreqBandList[fb_idx];
            // figure out the frequency
            uint8_t freq_band_low    = (freq_band - 1) * band_size_hz;
            uint8_t freq_band_high   = freq_band_low + band_size_hz;
            uint8_t freq_band_center = (freq_band_high + freq_band_low) / 2;
            uint8_t rows_per_col = kFreqBandCount * kFreqBandListLen;
            for (uint8_t m_idx = 0; m_idx < 5; ++m_idx) {
                uint8_t minute = minute_list[m_idx];
                uint8_t freq_band_label = freq_band;
                if (freq_band_label >= 4) {
                    freq_band_label = freq_band_label - 1;
                }
                for (uint8_t i1 = 0; i1 < 3; ++i1) {
                    char id1 = kId1List[i1];
                    uint8_t  col_count = 0;
                    uint16_t id1_offset = 0;
                    if (id1 == '1') { id1_offset = 200; }
                    if (id1 == 'Q') { id1_offset = 400; }
                    for (uint8_t i3 = 0; i3 < 10; ++i3) {
                        char id3 = kId3List[i3];
                        uint16_t channel_calc =
                            id1_offset + (col_count * rows_per_col) +
                            row_count;
                        if (channel_calc == channel) {
                            ChannelDetails cd;
                            cd.band      = band;
                            cd.channel   = channel_calc;
                            cd.id1       = id1;
                            cd.id3       = id3;
                            cd.id13[0]   = id1;
                            cd.id13[1]   = id3;
                            cd.id13[2]   = '\0';
                            cd.min       = minute;
                            cd.lane      = freq_band_label;
                            cd.freq      = freq_tx_low + freq_band_center;
                            cd.freq_dial = dial_freq;
                            return cd;
                        }
                        ++col_count;
                    }
                }
                ++row_count;
            }
        }
        return ChannelDetails();
    }
};

#endif  // WSPR_CHANNEL_MAP_H
