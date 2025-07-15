#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main()
{
    // Create Vendor-Defined Telemetry object for the number of
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
    // Get encoded WSPR message fields (sourced elsewhere)
    /////////////////////////////////////////////////////////////////

    codec.SetCallsign("006AAH");
    codec.SetGrid4("KK13");
    codec.SetPowerDbm(30);


    /////////////////////////////////////////////////////////////////
    // Decode
    /////////////////////////////////////////////////////////////////

    codec.Decode();


    /////////////////////////////////////////////////////////////////
    // Extract the now-decoded WSPR message fields
    /////////////////////////////////////////////////////////////////

    int uptimeMinutes     = codec.Get("UptimeMinutes");
    int rebootsTodayCount = codec.Get("RebootsTodayCount");

    cout << "Encoded data"                             << endl;
    cout << "------------"                             << endl;
    cout << "uptimeMinutes    : " << uptimeMinutes     << endl; // 95
    cout << "rebootsTodayCount: " << rebootsTodayCount << endl; //  2

    return 0;
}