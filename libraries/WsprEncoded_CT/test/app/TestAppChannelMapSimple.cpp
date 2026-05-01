#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main()
{
    // Configure band and channel
    const char *band    = "20m";
    uint16_t    channel = 123;

    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

    // Examine the details
    cout << "Channel Details for band " << band << ", channel " << channel << endl;
    cout << "id13: " << cd.id13      << endl;
    cout << "min : " << (int)cd.min  << endl;
    cout << "freq: " << cd.freq      << endl;

    return 0;
}