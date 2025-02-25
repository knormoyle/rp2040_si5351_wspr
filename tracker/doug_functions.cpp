// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#include <Arduino.h>
#include <Adafruit_SleepyDog.h>
#include "led_functions.h"
#include "debug_functions.h"
#include "print_functions.h"
#include "doug_functions.h"
#include "WsprEncoded.h"
#include "global_structs.h"

extern TeleStruct tt;
extern ConfigStruct cc;
extern bool VERBY[10];

WsprMessageTelemetryBasic msg;

// FIX! currently not using
void encodeBasicTele(char *hf_callsign, char *hf_grid4, char *hf_power,
    const char *id13, const char *grid56,
    uint32_t altitudeMeters, int8_t temperatureCelsius,
    double voltageVolts, uint8_t speedKnots, bool gpsIsValid) {

    V1_print(F("encodeBasicTele START" EOL));

    // Set the telemetry fields
    msg.SetGrid56(grid56);
    msg.SetAltitudeMeters(altitudeMeters);
    msg.SetTemperatureCelsius(temperatureCelsius);
    msg.SetVoltageVolts(voltageVolts);
    msg.SetSpeedKnots(speedKnots);
    msg.SetGpsIsValid(gpsIsValid);
    msg.SetId13(id13);

    V1_printf("id13 %s %s" EOL, id13, msg.GetId13());
    V1_printf("grid56 %s %s" EOL, grid56, msg.GetGrid56());
    V1_printf("altM %lu %d" EOL, altitudeMeters, msg.GetAltitudeMeters());
    V1_printf("tempC %u %u" EOL, temperatureCelsius, msg.GetTemperatureCelsius());
    V1_printf("voltage %.2f %.2f" EOL, voltageVolts, msg.GetVoltageVolts());
    V1_printf("speed %u %u" EOL, speedKnots, msg.GetSpeedKnots());
    V1_printf("gpsValid %u %u" EOL, gpsIsValid, msg.GetGpsIsValid());

    msg.Encode();

    const char *GetCallsign = msg.GetCallsign();
    const char *GetGrid4 = msg.GetGrid4();
    uint8_t GetPowerDbm = msg.GetPowerDbm();

    V1_printf("callsign: %s" EOL, GetCallsign);
    V1_printf("grid4: %s" EOL, GetGrid4);
    V1_printf("pwrDbm: %u" EOL, GetPowerDbm);

    // ah, can't use sizeof. size is lost
    // BUG: reported by Rob KC3LBR
    // JTEncode walks thru chars 0-12. 
    // so we now have 14 char array with null term
    // left justify..i.e. pad with space for what JTEncode gets
    snprintf(hf_callsign, 14, "%-13s", GetCallsign);
    snprintf(hf_grid4, 5, "%s", GetGrid4);

    // shouldn't be bigger than 60
    if (GetPowerDbm > 60) GetPowerDbm = 60;
    snprintf(hf_power, 3, "%u", GetPowerDbm);

    V1_print(F("encodeBasicTele END" EOL));
}

//***************************************************************
WsprMessageTelemetryExtendedUserDefined<4> codecGpsMsg;
void define_codecGpsMsg() {
    V1_print(F("define_codecGpsMsg() START" EOL));

    // Create User-Defined Telemetry object for the number of
    // fields you want, maximum of 29 1-bit fields possible.

    // Define telemetry fields

    // Define counts of GPS satellites for each constellation type.
    // Values will be clamped between 0 - 100 inclusive.
    // Resolution will be in increments of 1.
    bool accepted;
    accepted = codecGpsMsg.DefineField("SatCountUSA", 0, 40, 1);
    if (!accepted) {
        V1_println(F("ERROR: codecGpsMsg.DefineField('SatCountUSA', 0, 40, 1) not accepted"));
    }
    accepted = codecGpsMsg.DefineField("SatCountChina", 0, 40, 1);
    if (!accepted) {
        V1_println(F("ERROR: codecGpsMsg.DefineField('SatCountChina', 0, 40, 1) not accepted"));
    }
    accepted = codecGpsMsg.DefineField("SatCountRussia", 0, 40, 1);
    if (!accepted) {
        V1_println(F("ERROR: codecGpsMsg.DefineField('SatCountRussia', 0, 40, 1) not accepted"));
    }

    // Define a metric for GPS lock times, in seconds.
    // Values will be clamped between 0 - 180 inclusive.
    // Resolution will be in increments of 1.
    accepted = codecGpsMsg.DefineField("LockTimeSecs", 0, 600, 10);
    if (!accepted) {
        V1_println(F("ERROR: codecGpsMsg.DefineField('LockTimeSecs', 0, 600, 10) not accepted"));
    }


    // for use in the traquito website
    /* JSON
{ "name": "SatUSA",       "unit": "Count",   "lowValue": 0, "highValue": 40,  "stepSize": 1 },
{ "name": "SatChina",     "unit": "Count",   "lowValue": 0, "highValue": 40,  "stepSize": 1 },
{ "name": "SatRussia",    "unit": "Count",   "lowValue": 0, "highValue": 40,  "stepSize": 1 },
{ "name": "LockTimeSecs",  "unit": "Seconds", "lowValue": 0, "highValue": 600, "stepSize": 10},
    */
    /*
    */

    // Returns true if field is accepted
    // Returns false if field is rejected
    //
    // A field will be rejected due to:
    // - The template-specified number of fields have already been configured
    // - The field name is a nullptr
    // - The field already exists
    // - lowValue, highValue, or stepSize is too precise (more than 3 decimal places of precision)
    // - lowValue >= highValue
    // - stepSize <= 0
    // - The stepSize does not evenly divide the range between lowValue and highValue
    // - The field size exceeds the sum total capacity of 29.180 bits along with other fields
    // or by itself

    // how to form url
    // https://traquito.github.io/copilot/dashboard/#overview

    V1_print(F("define_codecGpsMsg() END" EOL));
}

//***************************************************************
// FIX! have to add parameters to encode or will it grab from global TinyGps state?
void encode_codecGpsMsg(char *hf_callsign, char *hf_grid4, char *hf_power, uint8_t slot) {
    V1_print(F("encode_codecGpsMsg START" EOL));
    switch (slot) {
        case 0: {;}
        case 1: {;}
        case 2: {;}
        case 3: {;}
        case 4: break;
        default:
            // count 0,1,2,3,4 here
            V1_printf("ERROR: encode_codecGpsMsg illegal slot %u ..using 2" EOL, slot);
            slot = 2;
    }

    // const char *band = "20m";
    // uint16_t channel = 123;
    char band[4];
    // append 'm'
    snprintf(band, sizeof(band), "%sm", cc._Band);
    uint16_t channel = atoi(cc._U4B_chan);

    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

    // Encode the data in preparation to transmit
    bool accepted = true;
    codecGpsMsg.SetId13(cd.id13);
    // Note which transmission slot you will send in
    codecGpsMsg.SetHdrSlot(slot);

    accepted &= codecGpsMsg.Set("SatCountUSA", atoi(tt.gp_sats));
    accepted &= codecGpsMsg.Set("SatCountChina", atoi(tt.gb_sats));
    accepted &= codecGpsMsg.Set("SatCountRussia", atoi(tt.gl_sats));
    accepted &= codecGpsMsg.Set("LockTimeSecs", atoi(tt.gpsLockSecs));

    codecGpsMsg.Encode();
    if (!accepted) {
        V1_println("Something didn't get accepted in encode_codecGpsMsg(). missing defines?");
    }

    const char *GetCallsign = codecGpsMsg.GetCallsign();
    const char *GetGrid4 = codecGpsMsg.GetGrid4();
    uint8_t GetPowerDbm = (uint8_t) codecGpsMsg.GetPowerDbm();

    V1_print(F("WsprEncode encoded data:" EOL));
    V1_printf("GetCallsign: %s" EOL, GetCallsign);
    V1_printf("GetGrid4: %s" EOL, GetGrid4);
    V1_printf("GetPowerDbm: %u" EOL, GetPowerDbm);

    // ah, can't use sizeof. size is lost
    // BUG: reported by Rob KC3LBR
    // JTEncode walks thru chars 0-12. 
    // so we now have 14 char array with null term
    // left justify..i.e. pad with space for what JTEncode gets
    snprintf(hf_callsign, 14, "%-13s", GetCallsign);
    snprintf(hf_grid4, 5, "%s", GetGrid4);

    // https://stackoverflow.com/questions/77869711/returning-a-char-pointer-when-the-argument-is-a-constant-char-pointer

    // wants to be char array instead of uint8_t?
    // shouldn't be bigger than 60
    if (GetPowerDbm > 60) GetPowerDbm = 60;
    snprintf(hf_power, 3, "%u", GetPowerDbm);

    V1_print(F("encode_codecGpsMsg END" EOL));
}

//***************************************************************
WsprMessageTelemetryExtendedUserDefined<3> codecBmpMsg;
void define_codecBmpMsg() {
    V1_print(F("define_codecBmpMsg() START" EOL));
    bool accepted;
    accepted = codecBmpMsg.DefineField("Pressure", 0.0, 80000.0, 5);
    if (!accepted) {
        V1_println(F("ERROR: codecBmpMsg.DefineField('Pressure', 0.0, 80000.0, 5) not accepted"));
    }

    accepted = codecBmpMsg.DefineField("Temperature", -60, 100, 2.5);
    if (!accepted) {
        V1_println(F("ERROR: codecBmpMsg.DefineField('Temperature', -60, 100, 2.5) not accepted"));
    }

    accepted = codecBmpMsg.DefineField("Altitude", 0, 55000, 100);
    if (!accepted) {
        V1_println(F("ERROR: codecBmpMsg.DefineField('Altitude', 0, 55000, 100) not accepted"));
    }

// good expected values here
// https://www.mide.com/air-pressure-at-altitude-calculator

// size fields at sandbox https://traquito.github.io/pro/codec
// programmed channel 391. comes out on 384
/* JSON
{ "name": "Pressure",    "unit": "Pa", "lowValue": 0,   "highValue": 80000, "stepSize": 5 },
{ "name": "Temperature", "unit": "C",  "lowValue": -60, "highValue": 100,    "stepSize": 2.5 },
{ "name": "Altitude",    "unit": "M",  "lowValue": 0,   "highValue": 55000,  "stepSize": 100 },
*/

    // how to form url
    // https://traquito.github.io/copilot/dashboard/#overview

    V1_print(F("define_codecBmpMsg() END" EOL));
}

//***************************************************************
// FIX! have to add parameters to encode or will it grab from global TinyBmp state?
void encode_codecBmpMsg(char *hf_callsign, char *hf_grid4, char *hf_power, uint8_t slot) {
    V1_print(F("encode_codecBmpMsg START" EOL));
    switch (slot) {
        case 0: {;}
        case 1: {;}
        case 2: {;}
        case 3: {;}
        case 4: break;
        default:
            // count 0,1,2,3,4 here
            V1_printf("ERROR: encode_codecBmpMsg illegal slot %u ..using 3" EOL, slot);
            slot = 3;
    }

    // const char *band = "20m";
    // uint16_t channel = 123;
    char band[4];
    // append 'm'
    snprintf(band, sizeof(band), "%sm", cc._Band);
    uint16_t channel = atoi(cc._U4B_chan);

    // Get channel details
    WsprChannelMap::ChannelDetails cd = WsprChannelMap::GetChannelDetails(band, channel);

    bool accepted = true;
    // Encode the data in preparation to transmit
    codecBmpMsg.SetId13(cd.id13);
    // Note which transmission slot you will send in
    codecBmpMsg.SetHdrSlot(slot);

    accepted &= codecBmpMsg.Set("Pressure", atof(tt.bmp_pressure));
    V1_printf("atof(tt.bmp_pressure %f" EOL, atof(tt.bmp_pressure));
    accepted &= codecBmpMsg.Set("Temperature", atof(tt.bmp_temperature));
    V1_printf("atof(tt.bmp_temperature %f" EOL, atof(tt.bmp_temperature));
    accepted &= codecBmpMsg.Set("Altitude", atof(tt.bmp_altitude));
    V1_printf("atof(tt.bmp_altitude %f" EOL, atof(tt.bmp_altitude));

    codecBmpMsg.Encode();
    if (!accepted) {
        V1_println("Something didn't get accepted in encode_codecBmpMsg(). missing defines?");
    }

    const char *GetCallsign = codecBmpMsg.GetCallsign();
    const char *GetGrid4 = codecBmpMsg.GetGrid4();
    uint8_t GetPowerDbm = (uint8_t) codecBmpMsg.GetPowerDbm();

    V1_print(F("WsprEncode encoded data:" EOL));
    V1_printf("GetCallsign: %s" EOL, GetCallsign);
    V1_printf("GetGrid4: %s" EOL, GetGrid4);
    V1_printf("GetPowerDbm: %u" EOL, GetPowerDbm);

    // ah, can't use sizeof. size is lost
    // BUG: reported by Rob KC3LBR
    // JTEncode walks thru chars 0-12. 
    // so we now have 14 char array with null term
    // left justify..i.e. pad with space for what JTEncode gets
    snprintf(hf_callsign, 14, "%-13s", GetCallsign);
    snprintf(hf_grid4, 5, "%s", GetGrid4);

    // https://stackoverflow.com/questions/77869711/returning-a-char-pointer-when-the-argument-is-a-constant-char-pointer

    // wants to be char array instead of uint8_t?
    // shouldn't be bigger than 60
    if (GetPowerDbm > 60) GetPowerDbm = 60;
    snprintf(hf_power, 3, "%u", GetPowerDbm);

    V1_print(F("encode_codecBmpMsg END" EOL));
}
