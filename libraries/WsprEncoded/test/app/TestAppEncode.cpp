#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main(int argc, char *argv[])
{
    int retVal = 0;

    if (argc != 9)
    {
        cout << "Usage: " << argv[0] << " <band> <channel> <grid56> <altM> <tempC> <volts> <knots> <gpsValid>" << endl;

        retVal = 1;
    }
    else
    {
        // Create Basic Telemetry object
        WsprMessageTelemetryBasic tb;

        // Set telemetry fields
        tb.SetGrid56(argv[3]);
        tb.SetAltitudeMeters(atoi(argv[4]));
        tb.SetTemperatureCelsius(atoi(argv[5]));
        tb.SetVoltageVolts(atof(argv[6]));
        tb.SetSpeedKnots(atoi(argv[7]));
        tb.SetGpsIsValid(atoi(argv[8]));

        // Configure band and channel
        const char *band    = argv[1];
        uint16_t    channel = atoi(argv[2]);

        // Get channel details
        WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

        // Encode the data
        tb.SetId13(cd.id13);
        tb.Encode();

        // Extract the encoded WSPR fields
        cout << "Encoded data" << endl;
        cout << "------------" << endl;
        cout << "Callsign: "<< tb.GetCallsign() << endl;
        cout << "Grid4   : "<< tb.GetGrid4()    << endl;
        cout << "PowerDbm: "<< (int)tb.GetPowerDbm() << endl;
    }

    return retVal;
}