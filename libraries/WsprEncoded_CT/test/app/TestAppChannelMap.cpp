#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main(int argc, char *argv[])
{
    int retVal = 0;

    if (argc != 3)
    {
        cout << "Usage: " << argv[0] << " <band> <channel>" << endl;

        retVal = 1;
    }
    else
    {
        // Configure band and channel
        const char *band    = argv[1];
        uint16_t    channel = atoi(argv[2]);

        // Get channel details
        WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

        // Examine the details
        cout << "Channel Details for band " << band << ", channel " << channel << endl;
        cout << "id13: " << cd.id13      << endl;
        cout << "min : " << (int)cd.min  << endl;
        cout << "freq: " << cd.freq      << endl;
    }

    return retVal;
}