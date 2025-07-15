#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main()
{
    // Create Basic Telemetry object
    WsprMessageTelemetryBasic tb;

    // Set telemetry fields
    tb.SetGrid56("XS");
    tb.SetAltitudeMeters(12360);
    tb.SetTemperatureCelsius(-28);
    tb.SetVoltageVolts(3.35);
    tb.SetSpeedKnots(72);
    tb.SetGpsIsValid(true);

    // Configure band and channel
    const char *band    = "20m";
    uint16_t    channel = 123;

    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

    // Encode the data
    tb.SetId13(cd.id13);
    tb.Encode();

    // Extract the encoded WSPR fields
    cout << "Encoded data" << endl;
    cout << "------------" << endl;
    cout << "Callsign: "<< tb.GetCallsign()      << endl;
    cout << "Grid4   : "<< tb.GetGrid4()         << endl;
    cout << "PowerDbm: "<< (int)tb.GetPowerDbm() << endl;

    return 0;
}