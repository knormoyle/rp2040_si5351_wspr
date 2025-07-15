#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main()
{
    // Create User-Defined Telemetry object for the number of
    // fields you want, maximum of 29 1-bit fields possible.
    WsprMessageTelemetryExtendedUserDefined<5> codecGpsMsg;

    /////////////////////////////////////////////////////////////////
    // Define telemetry fields
    /////////////////////////////////////////////////////////////////

    // Define counts of GPS satellites for each constellation type.
    // Values will be clamped between 0 - 128 inclusive.
    // Resolution will be in increments of 4.
    codecGpsMsg.DefineField("SatCountUSA",    0, 128, 4);
    codecGpsMsg.DefineField("SatCountChina",  0, 128, 4);
    codecGpsMsg.DefineField("SatCountRussia", 0, 128, 4);

    // Define a metric for GPS lock times, in seconds.
    // Values will be clamped between 0 - 30 inclusive.
    // Resolution will be in increments of 0.5.
    codecGpsMsg.DefineField("LockTimeSecs",    0, 30, 0.5);
    codecGpsMsg.DefineField("LockTimeSecsAvg", 0, 30, 0.5);


    /////////////////////////////////////////////////////////////////
    // Set fields (based on GPS data sourced elsewhere)
    /////////////////////////////////////////////////////////////////

    codecGpsMsg.Set("SatCountUSA",    12);
    codecGpsMsg.Set("SatCountChina",  10);      // rounded to 12   into encoded data on Encode()
    codecGpsMsg.Set("SatCountRussia",  0);

    codecGpsMsg.Set("LockTimeSecs",    10.74);  // rounded to 10.5 into encoded data on Encode()
    codecGpsMsg.Set("LockTimeSecsAvg", 12.76);  // rounded to 13   into encoded data on Encode()


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

    codecGpsMsg.SetId13(cd.id13);   // "06"
    codecGpsMsg.SetHdrSlot(slot);
    codecGpsMsg.Encode();


    /////////////////////////////////////////////////////////////////
    // Extract the now-encoded WSPR message fields
    /////////////////////////////////////////////////////////////////

    const char *callsign = codecGpsMsg.GetCallsign();   // "036KVF"
    const char *grid4    = codecGpsMsg.GetGrid4();      // "PP73"
    int         powerDbm = codecGpsMsg.GetPowerDbm();   // 30

    cout << "Encoded data"          << endl;
    cout << "------------"          << endl;
    cout << "Callsign: "<< callsign << endl;
    cout << "Grid4   : "<< grid4    << endl;
    cout << "PowerDbm: "<< powerDbm << endl;

    return 0;
}