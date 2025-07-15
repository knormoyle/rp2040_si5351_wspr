#include <cstdint>
#include <iostream>
using namespace std;

#include "WsprEncoded.h"


int main()
{
    // Create Basic Telemetry object
    WsprMessageTelemetryBasic tb;

    // Set encoded fields
    tb.SetCallsign("0Y6RLQ");
    tb.SetGrid4("EI27");
    tb.SetPowerDbm(33);

    // Decode the data
    tb.Decode();

    // Extract the decoded telemetry
    cout << "Decoded data" << endl;
    cout << "------------" << endl;
    cout << "Grid56            : "<< tb.GetGrid56()                  << endl;
    cout << "AltitudeMeters    : "<< tb.GetAltitudeMeters()          << endl;
    cout << "TemperatureCelsius: "<< (int)tb.GetTemperatureCelsius() << endl;
    cout << "VoltageVolts      : "<< tb.GetVoltageVolts()            << endl;
    cout << "SpeedKnots        : "<< (int)tb.GetSpeedKnots()         << endl;
    cout << "GpsIsValid        : "<< tb.GetGpsIsValid()              << endl;

    return 0;
}