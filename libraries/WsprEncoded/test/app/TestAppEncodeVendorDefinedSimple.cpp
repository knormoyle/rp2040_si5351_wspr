#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main()
{
    // Create User-Defined Telemetry object for the number of
    // fields you want, maximum of 29 1-bit fields possible.
    WsprMessageTelemetryExtendedVendorDefined<5> codec;

    /////////////////////////////////////////////////////////////////
    // Define telemetry fields
    /////////////////////////////////////////////////////////////////

    // Define tracker-specific telemetry, meant for the tracker
    // author to better understand the operation of the tracker
    // itself, rather than, say, a user deciding telemetry to see.
    codec.DefineField("UptimeMinutes",     0, 3600, 1);
    codec.DefineField("RebootsTodayCount", 0,  100, 1);

    /////////////////////////////////////////////////////////////////
    // Set fields (based on GPS data sourced elsewhere)
    /////////////////////////////////////////////////////////////////

    codec.Set("UptimeMinutes",     95);
    codec.Set("RebootsTodayCount",  2);


    /////////////////////////////////////////////////////////////////
    // Look up channel details for use in encoding
    /////////////////////////////////////////////////////////////////

    // Configure band and channel
    const char *band    = "20m";
    uint16_t    channel = 123;

    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);


    /////////////////////////////////////////////////////////////////
    // Take note of which transmission slot you will send in
    // for use in encoding
    /////////////////////////////////////////////////////////////////

    // this is just an example, you will need to know this based on
    // the behavior of your tracker
    uint8_t slot = 2;


    /////////////////////////////////////////////////////////////////
    // Encode the data in preparation to transmit
    /////////////////////////////////////////////////////////////////

    codec.SetId13(cd.id13);   // "06"
    codec.SetHdrSlot(slot);
    codec.Encode();


    /////////////////////////////////////////////////////////////////
    // Extract the now-encoded WSPR message fields
    /////////////////////////////////////////////////////////////////

    const char *callsign = codec.GetCallsign();   // "006AAH"
    const char *grid4    = codec.GetGrid4();      // "KK13"
    int         powerDbm = codec.GetPowerDbm();   // 30

    cout << "Encoded data"          << endl;
    cout << "------------"          << endl;
    cout << "Callsign: "<< callsign << endl;
    cout << "Grid4   : "<< grid4    << endl;
    cout << "PowerDbm: "<< powerDbm << endl;

    return 0;
}