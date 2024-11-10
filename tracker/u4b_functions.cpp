// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

// Incorporates work by: Roman Piksaykin R2BDY. Thank you.
// https://github.com/RPiks/pico-WSPR-tx

#include <stdint.h>

// to use:
//  uint32_t XMIT_FREQUENCY = init_rf_freq()

// FIX!
// externs for char[*]
// _U4B_chan
// _id13
// _Band

//*******************************
uint32_t init_rf_freq(void)
{
    // base frequencies for different bands
    // the pico should be able to do up to 10M band with appropriate clock frequency? (250Mhz? or ??)
    // 136000 474200 1836600 3568600 5364700 7038600 10138700 
    // 14095600 18104600 21094600 24924600 28124600 
    // 50293000 70091000 144489000
    // will support 20M, 17M, 15M, 12M, 10M

    enum BASE_FREQS {
        BF20M=14095600UL,
        BF17M=18104600UL,
        BF15M=21094600UL,
        BF12M=24924600UL,
        BF10M=28124600UL
    };

    uint32_t BASE_FREQ_USED;
    switch(atoi(_Band))
    {
        case 20: BASE_FREQ_USED=BF20M; break;
        case 17: BASE_FREQ_USED=BF17M; break;
        case 15: BASE_FREQ_USED=BF15M; break;
        case 12: BASE_FREQ_USED=BF12M; break;
        case 10: BASE_FREQ_USED=BF10M; break;
        default: BASE_FREQ_USED=BF20M; // default to 20M in case of error cases
    }

    XMIT_FREQUENCY=BASE_FREQ_USED + 1400UL; // offset from base for start of passband. same for all bands

    // add offset based on lane ..same for every band
    switch(_lane[0])                                     
        {
            case '1':XMIT_FREQUENCY+=20UL;  break;
            case '2':XMIT_FREQUENCY+=60UL;  break;
            case '3':XMIT_FREQUENCY+=140UL; break;
            case '4':XMIT_FREQUENCY+=180UL; break;
            // in case invalid lane was read from EEPROM. This is center passband?? (not a valid lane?)
            default: XMIT_FREQUENCY+=100UL; 
        }    

        printf("\nrf_freq_init _Band %s BASE_FREQ_USED %d XMIT_FREQUENCY %d _Klock_speed %s\n", 
            _Band, BASE_FREQ_USED, XMIT_FREQUENCY, _Klock_speed);
        return XMIT_FREQUENCY;
}

/*
From Hans G0UPL  on 06/27/23 post #11140 (this is not documented elsewhere). 
We had been using a table-driven mapping before Hans posted his algo.
 
The specification of U4B telemetry channels is as follows:
 
First callsign character:
Channels 0 - 199: '0'
Channels 200-399: '1'
Channels 400-599: 'Q'
 
Third callsign character:
(channel % 200) / 20
 
Frequency discrimination:
Frequency sector is
(channel % 20) / 5
 
That indicates into the array of transmit audio frequencies: {1420, 1460, 1540, 1580};
which are the target transmit frequencies, each in their 5 sectors.

The actual transmit frequency is the standard WSPR USB dial frequency + 
the above mentioned audio frequency; 

USB dial frequencies:
{136000, 474200, 1836600, 3568600, 5364700, 7038600, 10138700, 
14095600, 18104600, 21094600, 24924600, 28124600, 
50293000, 70091000, 144489000};
 
Transmit slot:
The transmit slot (txSlot) is first calculated as (channel % 5).
Then the start time in minutes past the hour, repeated every 10 minutes, is given by:
2 * ((txSlot + 2 * txBand) % 5);
 
txBand is:
0: 2200m
1: 630m
2: 160m
3: 80m
4: 60m
5: 40m
6: 30m
7: 20m
8: 17m
9: 15m
10: 12m
11: 10m
12: 6m
13: 4m
14: 2m
*/
 
include "u4b_functions.h"

void process_chan_num()
{
    if ( (atoi(_U4B_chan)>=0) && (atoi(_U4B_chan)<600)) 
    {
        _id13[0]='1';
        // Channels 0 - 199: '0'
        // Channels 200-399: '1'
        // Channels 400-599: 'Q'
        if  (atoi(_U4B_chan)<200) _id13[0]='0';
        if  (atoi(_U4B_chan)>399) _id13[0]='Q';

        // (channel % 200) / 20
        int id3 = (atoi(_U4B_chan) % 200) / 20;
        _id13[1]=id3+'0';
        
        // Frequency discrimination:
        // Frequency sector is
        // (channel % 20) / 5
        int lane = (atoi(_U4B_chan) % 20) / 5;
        _lane[0]=lane+'1';

        // The transmit slot (txSlot) is first calculated as (channel % 5).
        // Then the start time in minutes past the hour, repeated every 10 minutes, is given by:
        // 2 * ((txSlot + 2 * txBand) % 5);
        int txSlot = atoi(_U4B_chan) % 5;
        int txBand;
        switch(atoi(_Band))
        {
            case 20: txBand = 7;  break; // 20m
            case 17: txBand = 8;  break; // 17m
            case 15: txBand = 9;  break; // 15m
            case 12: txBand = 10; break; // 12m
            case 10: txBand = 11; break; // 10m
            default: txBand = 7;  break; // default to 20M in case of error cases
        }
        _start_minute[0] = '0' + (2 * ((txSlot + (txBand*2)) % 5));
    }
}


void u4b_encode() 
{
if 
    _u8_txpower =10; //hardcoded at 10dbM when doing u4b MSG 1
    if (verbosity>=3) printf("creating U4B packet 1\n");
    char _4_char_version_of_locator[5];
    //only take first 4 chars of locator
    strncpy(_4_char_version_of_locator,_pu8_locator, 4);  _4_char_version_of_locator[4]=0;  //add null terminator
    wspr_encode(_pu8_callsign, _4_char_version_of_locator, _u8_txpower, _pu8_outbuf, verbosity);  

    //record the values of grid chars 5 and 6 now, but they won't be used until packet type 2 is created
    grid5 = _pu8_locator[4];  
    grid6 = _pu8_locator[5];
    // save the value for later when used in 2nd packet
    altitude_snapshot=_altitude;     
    at_least_one_first_packet_sent=1;
}
    if (verbosity>=3) printf("creating U4B packet 2 \n");
    char CallsignU4B[7];
    char Grid_U4B[7];
    uint8_t  power_U4B;

    // record the values of grid chars 5 and 6 now, but they won't be used until packet type 2 is created
    // grid5 = _pu8_locator[4];  
    // grid6 = _pu8_locator[5];
    // altitude_snapshot=_pGPStime->_altitude;
            
/* inputs:  _pu8_locator (6 char grid)
            _temp_in_Celsius
            _id13
            _voltage
*/    
        // pick apart inputs
      // char grid5 = _pu8_locator[4];  values of grid 5 and 6 were already set previously when packet 1 was created
      //char grid6 = _pu8_locator[5];
      // convert inputs into components of a big number
        uint8_t grid5Val = grid5 - 'A';
        uint8_t grid6Val = grid6 - 'A';
        uint16_t altFracM =  round((double)altitude_snapshot/ 20);     

     // convert inputs into a big number
        uint32_t val = 0;
        val *=   24; val += grid5Val;
        val *=   24; val += grid6Val;
        val *= 1068; val += altFracM;
                // extract into altered dynamic base
        uint8_t id6Val = val % 26; val = val / 26;
        uint8_t id5Val = val % 26; val = val / 26;
        uint8_t id4Val = val % 26; val = val / 26;
        uint8_t id2Val = val % 36; val = val / 36;
        // convert to encoded CallsignU4B
        char id2 = EncodeBase36(id2Val);
        char id4 = 'A' + id4Val;
        char id5 = 'A' + id5Val;
        char id6 = 'A' + id6Val;

        //string{ id13[0], id2, id13[1], id4, id5, id6 };
        CallsignU4B[0] =  _txSched.id13[0];   
        CallsignU4B[1] =  id2;
        CallsignU4B[2] =  _txSched.id13[1];
        CallsignU4B[3] =  id4;
        CallsignU4B[4] =  id5;
        CallsignU4B[5] =  id6;
        CallsignU4B[6] =  0;

        /* inputs:  
            _pu8_locator (6 char grid)
            _temp_in_Celsius
            _id13
            _voltage
        */    
        /* outputs :    
                char CallsignU4B[6]; 
                char Grid_U4B[7]; 
                uint8_t  power_U4B;
                */
        // parse input presentations
        double tempC   = _txSched.temp_in_Celsius;
        double voltage = _txSched.voltage;
        // map input presentations onto input radix (numbers within their stated range of possibilities)

        //**************
        // handle possible illegal range (double wrap on tempC?). or should it clamp at the bounds?
        // uint8_t tempCNum      = tempC - -50;
        uint8_t tempCNum      = ((uint8_t) tempC - -50) % 90;
        uint8_t voltageNum    = ((uint8_t)round(((voltage * 100) - 300) / 5) + 20) % 40;
        // FIX! encoding # of satelites into knots
        uint8_t speedKnotsNum = _pTX->_p_oscillator->_pGPStime->_time_data.sat_count;   
        // handle possible illegal (0-41 legal range). 
        // clamp to max, not wrap. maybe from bad GNGGA field (wrong sat count?)
        if (speedKnotsNum > 41) speedKnotsNum = 41;
        //****************
        
        // kevin old code since this isn't 0 or 1 
        // it should really check zero. don't want to say valid if dead reckoning fix? (6)
        uint8_t gpsValidNum   = _pTX->_p_oscillator->_pGPStime->_time_data._u8_is_solution_active;
        gpsValidNum=1; //changed sept 27 2024. because the traquito site won't show the 6 char grid if this bit is even momentarily off. Anyway, redundant cause sat count is sent as knots
        // shift inputs into a big number
        val = 0;
        val *= 90; val += tempCNum;
        val *= 40; val += voltageNum;
        val *= 42; val += speedKnotsNum;
        val *=  2; val += gpsValidNum;
        // standard telemetry (1 for the 2nd U4B packet, 0 for "Extended TELEN")
        val *=  2; val += 1;          

        // unshift big number into output radix values
        uint8_t powerVal = val % 19; val = val / 19;
        uint8_t g4Val    = val % 10; val = val / 10;
        uint8_t g3Val    = val % 10; val = val / 10;
        uint8_t g2Val    = val % 18; val = val / 18;
        uint8_t g1Val    = val % 18; val = val / 18;
        // map output radix to presentation
        char g1 = 'A' + g1Val;
        char g2 = 'A' + g2Val;
        char g3 = '0' + g3Val;
        char g4 = '0' + g4Val;
     
        Grid_U4B[0] = g1; // = string{ g1, g2, g3, g4 };
        Grid_U4B[1] = g2;
        Grid_U4B[2] = g3;
        Grid_U4B[3] = g4;
        Grid_U4B[4] = 0;

    power_U4B=valid_dbm[powerVal];

    wspr_encode(CallsignU4B, Grid_U4B, power_U4B, _pu8_outbuf,_txSched.verbosity); 
   }
