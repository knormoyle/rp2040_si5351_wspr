// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// Uses 4 space indent, OTBS (One True Brace) indentation style
// although single statement blocks can exclude braces.
// https://en.wikipedia.org/wiki/Indentation_style
// vim users can use this in .vimrc to good effect:
// set tabstop=4 softtabstop=0 expandtab shiftwidth=4 smarttab

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "pico/stdio.h"
#include <stdlib.h>
#include <string.h>

// HACK to stay in balloon mode for debug.  Normally should be false.
bool FORCE_BALLOON_MODE = false;

// These are in arduino-pio core
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries
// #include <SPI.h>
// #include <Wire.h>

// THe Raspberry Pi Pico SDK is included with the arduino-pico core,
// and all functions in the core are available inside the standard link libraries.
// When you use SDK functions, the core and libraries are not aware of any changes
// to the Pico you perform.

// Be wary of multicore and use of libraries. arduino-pico is not thread-safe.

// The Arduino-Pico core implements a software-based Serial-over-USB port
// using the USB ACM-CDC model to support a wide variety of operating systems.

// Serial is the USB serial port, and while Serial.begin()
// does allow specifying a baud rate, it is ignored since it is USB-based.

// This USB Serial port is responsible for resetting the RP2040 during the IDE upload
// process, following the Arduino standard of 1200bps = reset to bootloader.

// RP2040 has two hardware-based UARTS with configurable pin selection.
// Serial1 is UART0, and Serial2 is UART1.

// The RP2040 chip has 2 cores that can run independently of each other,
// sharing peripherals and memory with each other.
// Arduino code will normally execute only on core 0,
// with the 2nd core sitting idle in a low power state.

// setup1() and loop1() functions allow use of the second core.
// Anything called from within the setup1() or loop1() routines
// will execute on the second core.
// https://arduino-pico.readthedocs.io/en/latest/multicore.html

// Using the 2nd core to handle keyboard interrupts and entry to config coe
// core0 will stop balloon processing at an appropriate boundary which
// may be longer than optimal for quick keyboard response.
// changes to the config state may have atomicity issues, but the ballon code will
// always be rebooted when done, as if to start from scratch after config changes

// some stuff on deep sleep (I don't do)
// https://github.com/matthias-bs/arduino-pico-sleep

//**************************
// flash config
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

//**************************
// libraries/TinyGpsPlus
#include <TinyGPS++.h>  // https://github.com/mikalhart/TinyGPSPlus
// gets the head 1.1-beta? not released version
// wget https://github.com/mikalhart/TinyGPSPlus/archive/refs/heads/master.zip
// TinyGPSPlus is a Arduino library for parsing NMEA data streams provided
// by GPS modules.

// libraries/Adafruit_SleepyDog
// wget https://github.com/adafruit/Adafruit_SleepyDog/archive/refs/heads/master.zip
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// libraries/Adafruit_BMP280_Library
// wget https://github.com/adafruit/Adafruit_BMP280_Library/archive/refs/heads/master.zip
#include <Adafruit_BMP280.h>  // https://github.com/adafruit/Adafruit_BMP280_Library

// libraries/Adafruit_Sensor
// wget https://github.com/adafruit/Adafruit_Sensor/archive/refs/heads/master.zip
// https://github.com/adafruit/Adafruit_sensor
// added Adafruit_BusIO to our repo 11_7_24 (libraries)
// in libraries: wget https://github.com/adafruit/Adafruit_BusIO/archive/refs/heads/master.zip
// #include <Adafruit_I2CDevice.h>  // https://github.com/adafruit/Adafruit_BusIO

// libraries/JTEncode
#include <JTEncode.h>  // https://github.com/etherkit/JTEncode (JT65/JT9/JT4/FT8/WSPR/FSQ Encoder Library)
// wget https://github.com/etherkit/JTEncode/archive/refs/heads/master.zip

// libraries/Time
// wget https://github.com/PaulStoffregen/Time/archive/refs/heads/master.zip
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time

// libraries/arduinomorse
// wget https://github.com/knormoyle/arduinomorse/archive/refs/heads/master.zip
// might be interesting for getting better led messaging. not used yet.
// this has non-blocking capability.
// I could use my morse cw and have it toggle an led? but it would be blocking
#include <morse.h> // https://github.com/markfickett/arduinomorse

// libraries/Arduino-MemoryFree
// wget https://github.com/maniacbug/MemoryFree/archive/refs/heads/master.zip
#include <MemoryFree.h>  // https://github.com/maniacbug/MemoryFree

// https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm/
// #include "hardware/pwm.h"
// https://arduino-pico.readthedocs.io/en/latest/rp2040.html

// The RP2040 has two hardware-based UARTS with configurable pin selection.
// Serial1 is UART0, and Serial2 is UART1.
// For applications where an IRQ driven serial port is not appropriate,
// use setPollingMode(true) before calling begin()
//     Serial1.setPollingMode(true);

// extern so it links okay?
extern const int Si5351Pwr = 4;
extern const int BattPin = A3;

//******************************* CONFIG **********************************
// FIX! are these used now? remnants of APRS messaging
char comment[] = "tracker 1.0";
char telemetry_buff[100] = { 0 };  // telemetry buffer
uint16_t Tx_0_cnt = 0;  // increase +1 after every callsign tx
uint16_t Tx_1_cnt = 0;  // increase +1 after every telemetry tx
uint16_t Tx_2_cnt = 0;  // increase +1 after every telen1 tx
uint16_t Tx_3_cnt = 0;  // increase +1 after every telen2 tx

// The maximum number of binary channel symbols in a WSPR message is 162.
// This is calculated by adding the constraint length (K) of 32 to the
// total number of bits in a standard message (50), and then multiplying by 2.
// background
// https://hackaday.io/project/166875-careless-wspr/log/167301-encoding-wsprs
// http://www.g4jnt.com/Coding/WSPR_Coding_Process.pdf
// python code for playing around with wspr encoding
// https://github.com/robertostling/wspr-tools

//*********************************
// PWM stuff
uint32_t PWM_DIV;
uint32_t PWM_WRAP_CNT;
// this can get modified for 18 Mhz operation (to 1?)
uint32_t INTERRUPTS_PER_SYMBOL = 8;

// below we always loop thru the entire hf_tx_buffer?
uint8_t hf_tx_buffer[162] = { 0 };  // is this bigger than WSPR_SYMBOL_COUNT?

// FIX! why is this volatile? Because it's set by the ISR for PWM interrupts?
// alternatives discussed here. Interesting there's also a potential issue
// for cross-core global references, like this
// https://forums.raspberrypi.com/viewtopic.php?t=312685
volatile bool PROCEED = false;

//*********************************
// if we need to ignore TinyGps++ state for a while, because
// we turned off the Gps, and then TinyGps++ won't change
// state until we give it a full broadcast's worth of sentences (1 sec of broadcast)
uint32_t GpsInvalidAllCnt = 0;
bool GpsInvalidAll = false;

// when should do a gps cold reset? when not getting a new fix we can use
uint32_t GpsWatchdogCnt = 0;

// gps_functions.cpp refers to this
TinyGPSPlus gps;

#include "print_functions.h"
#include "debug_functions.h"
#include "config_functions.h"
#include "tele_functions.h"
#include "mh_functions.h"
#include "adc_functions.h"
#include "wspr_functions.h"
#include "si5351_functions.h"
#include "u4b_functions.h"
#include "bmp_functions.h"
#include "led_functions.h"
#include "keyboard_functions.h"
#include "gps_functions.h"
#include "cw_functions.h"
#include "sweep_functions.h"

//*********************************
// extern so it links okay
extern const int BMP280_I2C1_SDA_PIN = 2;
extern const int BMP280_I2C1_SCL_PIN = 3;
Adafruit_BMP280 bmp;

JTEncode jtencode;

//*********************************
// all extern consts can be externed by a function
// the linker will will fail on these global constants unless labelled
// extern here also (like in the other files that reference this)
extern const int STATUS_LED_PIN = 25;
// these are the short blinks or long blinks, depending on value
extern const int LED_STATUS_NO_GPS = 1;        // 1 short
extern const int LED_STATUS_GPS_TIME = 2;      // 2 short
extern const int LED_STATUS_GPS_FIX = 3;       // 3 short
extern const int LED_STATUS_TX_WSPR = 4;       // 4 short
extern const int LED_STATUS_TX_TELEMETRY = 5;  // 1 long
extern const int LED_STATUS_TX_TELEN1 = 6;     // 2 long
extern const int LED_STATUS_TX_TELEN2 = 7;     // 3 long
extern const int LED_STATUS_TX_CW = 8;         // 4 long
extern const int LED_STATUS_REBOOT_NO_SERIAL = 8;  // 5 long
extern const int LED_STATUS_USER_CONFIG = 9;   // 6 long

//*********************************
// some stuff on using namespace
// https://forum.arduino.cc/t/using-a-constant-defined-in-the-header-file/380178

// flash background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

//*********************************
// all extern consts can be externed for reference in another file (like gps_functions.cpp)
// extern is needed here or the linker doesn't find it.
// see https://forum.arduino.cc/t/linker-problems-with-extern-const-struct/647136/2
extern const int GpsPwr = 16;  // output ..this cuts VCC, leaves VBAT. assert low

// not used..GpsPwr is used.
// const int GPS_VCC_ON_N_PIN=16;

extern const int GPS_NRESET_PIN = 5;
// FIX! for now, we don't toggle the on pin..always keep it on, just toggle power
extern const int GPS_ON_PIN = 6;

// FIX! where is this used (calibration maybe). I suppose could calibrate
// the rp2040 clock? maybe the si5351 clk2 output if we wanted?
extern const int GPS_1PPS_PIN = 17;   // input

extern const int GPS_UART1_TX_PIN = 8;
extern const int GPS_UART1_RX_PIN = 9;

// Serial2 talks to gps.
// can't really make the hardware uart fifo size bigger
extern const int SERIAL2_FIFO_SIZE = 32;
// earlephilhower says the hw serial units use the hardware rx fifo
// so only 32?

// default? (for usb serial)
// separate for rx. we don't want to be blocked
// also don't want large delay on flush!
// leave it as default
#define SERIAL_TX_BUFFER_SIZE 256


//*********************************************
// extern const int SIM65M_BAUD_RATE = 4800;
// what we want to change it too
extern const int SIM65M_BAUD_RATE = 9600;
// extern const int SIM65M_BAUD_RATE = 19200;
// extern const int SIM65M_BAUD_RATE = 38400;
// extern const int SIM65M_BAUD_RATE = 57600;
// default at power up
// extern const int SIM65M_BAUD_RATE = 115200;
//*********************************************

// legal choices? 9600 (default ATGM336H) 19200 38400
// is in receive fifo 12-bit wide default 32 deep ??
// https://arduino-pico.readthedocs.io/en/latest/serial.html

// hmm sometimes have to restart Arduino IDE when it gets stuck in
// bad state after changing baud rates
// is my full cold reset not working to gps and vbat is remembering the baud?
// so I can't talk to him without a full power off (vcc and vbat?)
// AG6NS board can't turn off VBAT under progam control

// Update: we use the gps resetn pin now, for full cold reset to revert
// baud to 9600 ALWAYS at init (then set desired baud rate
// created GpsWarmReset() and GpsColdReset()
// GpsWarmReset is the normal "just power" on/off management that keeps vbat on

// see gps_functions.cpp and problems with resetting to 9600 from higher bauds
// too dangerous to use anything higher than 9600..could get stuck
// with out vcc plus vbat power cycle.. I can't control vbat when on usb power

// with the GLONASS sats
// old numbers: we reduced the broadcast?
// There are 821 chars in a burst, handling takes 892 milliseconds per burst
// try increasing the baud rate to see if duration takes less
// works going from 9600 to 19200
// reduced broadcast works now, with 3 constellations
// 670 chars with 396 ms duration. going back to 9600

// lets try 4800. is power less at power on?
// hmm something was not working with gps warm reset with 4800. back to 9600

// seem to leave the polling for NMEA data too soon at 4800..miss some chars
// extern const int ATGM336H_BAUD_RATE = 4800;
// works down to 50 Mhz fine? what's the lowest SYS_PLL_MHZ that works?
extern const int ATGM336H_BAUD_RATE = 9600;
// extern const int ATGM336H_BAUD_RATE = 19200;

// can't seem to recover to 9600 after trying 38400? full cold reset not working?
// need to unplug usb to get back to 9600

// extern const int ATGM336H_BAUD_RATE = 38400;

// this is too fast. I can't keep up with incoming data
// GPS burst: duration is 165 millisecs for 646 chars. 3954 baud effective
// extern const int ATGM336H_BAUD_RATE = 57600;
// don't use. rx buffer overruns
// extern const int ATGM336H_BAUD_RATE = 115200;

//*********************************
// was 4, but not getting enough precision. see si5351_functions.cpp
// 7 will give 1/128ths precision after the decimal (for symbol frequency),
// as opposed to 1/16ths
// was 1/7/25
// extern const int PLL_CALC_SHIFT = 7;

// double precision fp mantissa only has 52 bits of precision
// scaled integer arith has 64 bits of precision because we have 64-bit integers to use.
// So scaled integer math can have more precision if you can cover the dynamic range of values 
// needed.

// The biggest numbers are the PLL freq. 900 Mhz at most.

// And if you want the possibility of 1e-6 precision for the wspr shifts, 
// that's 10e6 more range needed (which is 20 bits!!)
// How many bits needed altogether:
// log2(900e6 * 10e6) = 52.99 bits needed.
// So actually, it's pushing the limits of what you can do with double precision fp (which only 
// has 52 bits of precision)

// Sure the fact that the si5351a divisor chops some low order bits out, in the output freq you
// get, means you can reduce the precision needed by those lost bits. 
// But with a small divisor like 19, that's just 4 or 5 bits less precision needed.

// The integer-scaled arith is optimally a left shift/right shift, because that's powers of two, 
// so less likely to misunderstand where bits get lost

// If I shift everything left by 15 bits when I do the integer-scaled arith to figure
// out the real I need to get a fraction for. 
// so 900e6 * 2**15 = 2.95e13 and that needs log2(2.95e13) = 44.74 bits.

// I should be able to shift by 20 bits to get the 1e-6 precision noted above?

// Once I get that (scaled) remainder, I cast it as a double fp, then divide by pow(2, 15), 
// to get back to a real I want to feed the Farey algo (which is between 0 and 1) to get fraction (num/denom). 

// That gives me perfect accuracy for getting a Farey result that has perfect 
// absolute accuracy and perfect symbol shift accuracy.

// Once you get to the 0-1 real for Farey, then you you have plenty of bits of precision if you use double fp.
// Since you're no longer dealing with the full 900e6 of the pll.

// 1/7/25 This works now!
// extern const int PLL_CALC_SHIFT = 15;
// didn't work?
// extern const int PLL_CALC_SHIFT = 20;
// didn't work?
// extern const int PLL_CALC_SHIFT = 18;
extern const int PLL_CALC_SHIFT = 16;

// this is the target PLL freq when making muliplier/divider initial calculations
// could change this per band?
// the implied mul/div for 5 bands is covered by denom choices from spreadsheet for 700000000
// uint64_t PLL_FREQ_TARGET = 900000000;
// uint64_t PLL_FREQ_TARGET = 700000000;
// uint64_t PLL_FREQ_TARGET = 600000000;

// 15 (min multiplier) * 26Mhz = 390 Mhz
// the other (not used PLL) will run at this freq in default config?
// so what about targetting that? (will it error on the multiplier?)
// check that code (set to min 15 if too small in si5351_functions.cpp
// so we should be able

//***************************
// 390Mhz pll test:
// here's results from the magic spreadsheet showing denom values for 10/12/15/17/20M
// with mult/divisor selected to get pll in the 390Mhz region.
// 10M doesn't have a good value (uses max value: means there was no perfect value)
//
// It shows that I can't get optimal denominator on 10M
// ..so the target pll freq is too low.
// But there were numerator-step-1 values that worked for 12/15/17/20M
// I don't show actual pll freq in spreadsheet,
// because I generate the numerator elsewhere (in tracker code)
//
// no green cells on 10M (and the 12M value is small)
// so I won't use 390Mhz target because I want the perfect symbol shift on 10M too
//***************************
// uint64_t PLL_FREQ_TARGET = 390000000;

// 16 * 26 = 416.. so maybe that will work
// not good
// uint64_t PLL_FREQ_TARGET = 416000000;
// this is good
// in use 1/6/24 for both Fary and num-shift methods
// uint64_t PLL_FREQ_TARGET = 500000000;

// won't work well for num-shift method. may work for Farey
uint64_t PLL_FREQ_TARGET = 400000000;


// why am I getting it shift to 10hz wide audio signal on sdruno?
// oh, that's the symbol shifting at 1.46 secs or so
// uint64_t PLL_FREQ_TARGET = 600000000;
// try 900
// uint64_t PLL_FREQ_TARGET = 900000000;

// anything else will use PLL_DENOM_MAX
// double check the values if the algo for div/mul in si5351_functios.cpp changes relative
// to reaction to PLL_FREQ_TARGET

extern const int VFO_VDD_ON_N_PIN = 4;
// are these really on Wire1
extern const int VFO_I2C0_SDA_PIN = 12;
extern const int VFO_I2C0_SCL_PIN = 13;

// extern const int VFO_I2C0_SCL_HZ = (1000 * 1000);
// maybe go lower frequency?

// FIX! is this wrong with lower frequency sys clk??
extern const int VFO_I2C0_SCL_HZ = (100 * 1000);
extern const int BMP_I2C1_SCL_HZ = (100 * 1000);
// FIX! used in i2c_functions for test of both i2c0 and i2c1
// pullup resistors are different on each i2c bus on the pcb
extern const int PICO_I2C_CLK_HZ = (100 * 1000);

extern const int BMP_I2C1_SDA_PIN = 2;
extern const int BMP_I2C1_SCL_PIN = 3;

// FIX! are the pcb pullups less aggressive on BMP i2c?
// maybe have to stay slower on speed?


// The I2C address for the MS5351M is the same as the Si5351A-B-GT/GTR, which is 0x60
extern const int SI5351A_I2C_ADDR = 0x60;

//**********************************
// IMPORTANT: GLOBALS and MULTICORE ACCESS

// https://forums.raspberrypi.com/viewtopic.php?t=347326
// Yes. In the jargon of C, variable x has static storage duration.
// That means only one instance of storage is allocated for x and
// it has the same lifetime as the execution of the program.
// In other words, both cores see the same x.
//
// One must be very careful when accessing a variable from both cores.
// Among other things the C compiler is allowed to pretend that there is only one core,
// and optimize away accesses of x that it thinks are redundant.
//
// You might find making the variable "volatile" helps.
// "Volatile" is not technically the correct way to address access to a variable
// from multiple cores but it works on RP2040
// since the config stuff is read in core0, then used in core1, it should be volatile
//**********************************

// essentially t_* stuff is a telemetry data buffer/structure
// all can be extern'ed by a function
// init to 0 is just in case. Should always be set to something valid before use
// empty string is not valid
// not sure what will happen if used while empty.. I suppose it can print ok
// always positive. clamp to 0 I guess
char t_course[4] = { 0 };      // 3 bytes + null term (like all here
// always positive? 0-250 knots. clamp to 0 I guess
char t_speed[4] = { 0 };       // 3 bytes
// allow 60000 meters. plus 1 in case negative?
char t_altitude[7] = { 0 };    // 6 bytes
// 24 * 30 per hour = 720 per day if every two minutes
// reboot once per day? (starts at 0)
char t_tx_count_0[4] = { 0 };  // 3 bytes
char t_temp[7] = { 0 };        // 6 bytes
char t_pressure[8] = { 0 };    // 7 bytes
char t_temp_ext[8] = { 0 };    // 7 bytes
char t_humidity[8] = { 0 };    // 7 bytes
char t_voltage[6] = { 0 };     // 5 bytes
char t_sat_count[3] = { 0 };   // 2 bytes
// lat/lon precision: How much to store
// https://stackoverflow.com/questions/1947481/how-many-significant-digits-should-i-store-in-my-database-for-a-gps-coordinate
// 6 decimal places represent accuracy for ~ 10 cm
// 7 decimal places for ~ 1 cm
// The use of 6 digits should be enough. +/- is 1 more.
// decimal is one more. 0-180 is 3 more.
// so 7 + 5 = 12 bytes should enough, with 1 more rounding digit?
char t_lat[13] = { 0 };            // 12 bytes
char t_lon[13] = { 0 };            // 12 bytes
char t_callsign[7] = { 0 };
char t_grid6[7] = { 0 };           // 6 bytes
char t_power[3] = { 0 };           // 2 bytes
char t_hdop[4] = { 0 };            // 3 bytes;
char t_solarElevation[5] = { 0 };  // 3 bytes -90 to 90?
char t_solarAzimuth[7] = { 0 };    // 4 bytes -180 to 180?
char t_solarDistance[8] = { 0 };   // 3 bytes 145 to 150 km ?

int t_snap_cnt = 0;

//***********************************************************
// config strings: all can be extern'ed by a function
// but there is also decode of them to another global that is used instead.
// see config_functions.cpp
// these get set via terminal, and then from NVRAM on boot
// init with all null

// hmmm. can't declare these arrays 'volatile'
char _callsign[7] = { 0 };
char _suffix[2] = { 0 };
char _verbose[2] = { 0 };
char _TELEN_config[5] = { 0 };
// FIX! why is this a compiler problem if volatile? an snprintf() fails
// https://forum.arduino.cc/t/invalid-conversion-from-volatile-char-to-const-char-fpermissive/949522
char _clock_speed[4] = { 0 };
char _U4B_chan[4] = { 0 };
// FIX! why is this a compiler problem if volatile? parameter to function fails
// error: invalid convesion fro 'volatile char*' to 'char*'
char _Band[3] = { 0 };  // string with 10, 12, 15, 17, 20 legal. null at end
char _tx_high[2] = { 0 };  // 0 is 4mA si5351. 1 is 8mA si5351
char _testmode[2] = { 0 };
char _correction[7] = { 0 };
char _go_when_rdy[2] = { 0 };
char _factory_reset_done[2] = { 0 };
char _use_sim65m[2] = { 0 };
char _morse_also[2] = { 0 };

// decoded stuff from config strings: all can be extern'ed by a function
// decodes from _Band _U4B_chan
// 0 should never happen for XMIT_FREQUENCY
char _id13[3] = { 0 };
char _start_minute[2] = { 0 };
char _lane[2] = { 0 };

// decode of _clock_speed
// extern const uint32_t DEFAULT_PLL_SYS_MHZ = 125;
// this seems to work fine even with VERBY[3] doing a little more output during wspr
// be nice if I could get Serial2 working with pll_sys off,
// so I could do 12Mhz with crystal osc.
extern const uint32_t DEFAULT_PLL_SYS_MHZ = 18;
uint32_t PLL_SYS_MHZ = 0;  // should never try to use it while it's 0

//*****************************
// this gets correction if any in setup()
uint32_t SI5351_TCXO_FREQ = 26000000;
// changed for band/u4b channel
uint32_t XMIT_FREQUENCY;
// optimized?
uint32_t PLL_DENOM_OPTIMIZE = 1048575;

bool USE_FAREY_WITH_PLL_REMAINDER = true;
bool TEST_FAREY_WITH_PLL_REMAINDER = true;
// bool USE_FAREY_WITH_PLL_REMAINDER = false;
// /bool TEST_FAREY_WITH_PLL_REMAINDER = false;

//*****************************
bool BALLOON_MODE = true;
bool CORE1_PROCEED = false;
// decode of _testmode
bool TESTMODE = false;
// decode of _verbose 0-9
bool VERBY[10] = { false };
bool USE_SIM65M = false;

//*****************************
// t_power is clamped to string versions of these. use 0 if illegal
// int legalPower[] = {0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60}
// will differentiate from u4b (10) and traquito (13) by
// using 3 or 7 in normal callsign tx 3 for low power, 7 for high power
//*********************************
absolute_time_t GpsStartTime = 0;
uint64_t loopCnt = 0;

// the global IGNORE_KEYBOARD_CHARS is used to guarantee no interrupting of core1
// while we've messed with clocks during the gps agressive power on control
// it should always be re-enabled after 30 secs.
// Worst case to recover: unplug power and plug in again
bool IGNORE_KEYBOARD_CHARS = false;
bool IGNORE_KEYBOARD_CHARS_last = false;

//***********************************************************
// FIX! should this be non-zero? 0 disables any if/else behavior
// associated with the voltage read (rp2040 adc)
// Maybe all a don't care now with the voltage monitor that causes reset.
float GpsMinVolt = 0.0;   // min Volts for GPS to wake up.
float BattMin = 0.0;      // min Volts to wake up.
float WsprBattMin = 0.0;  // min Volts for HF (WSPR) radio module to transmit (TX) ~10 mW
// GPS is always on if the voltage exceeds this value to protect solar caps from overcharge
float HighVolt = 9.9;

//***********************************************************
// To allocate a separate 8K stack for core 1,
// resulting in 8K stacks being available for both cores,
bool core1_separate_stack = true;

//***********************************************************
// https://arduino-pico.readthedocs.io/en/latest/multicore.html
// are the V1_print* functions not threadsafe?
// Generally Serial is not thread-safe.
// simple V1_print() might be thread safe
// but not V1_println or especially not Serial.printf()
// everything is shared/accessible between two cores, but little is thread safe?

// so we can see the setup() read_FLASH() results later
int read_FLASH_result1 = 0;
int read_FLASH_result2 = 0;

//***********************************************************
void setup() {
    // https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html
    // https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html#ga2ad3247806ca16dec03e655eaec1775f
    // Initialize a core such that the other core can lock it out during flash_safe_execute.
    // see the dire need at the link above (around flash access)
    // flash_safe_execute_core_init();

    // FIX! what if we start with usb connected, boot, do some printing or keyboard
    // and then reboot and unplug usb. Or unplug usb and leave running
    // will it reboot when watch dog timer detects hang on Serial?
    // no it will reboot because of power going away and coming back?
    // what things are thread safe?
    // https://forums.raspberrypi.com/viewtopic.php?t=370841
    // sleep_us (Depends on Children, Unsolved due to "sleep_until")

    // absolute_time_diff_us (Depends on Children, Looks SMP Safe/Thread Safe)
    // get_absolute_time (Depends on Children, Looks SMP Safe/Thread Safe)

    // to_ms_since_boot (absolute_time_t t)
    // Convert a timestamp into a number of milliseconds since boot.

    // to_us_since_boot (Looks SMP Safe/Thread Safe)

    // static bool time_reached (absolute_time_t t)
    // Check if the specified timestamp has been reached.

    // time_reached (Depends on Children, Looks SMP Safe/Thread Safe)
    // busy_wait_until (Depends on Children, Looks SMP Safe/Thread Safe)

    // FIX! in case user is frantically trying to get to the config menu
    // to avoid setting clock speed or ??
    // if anything was found by incomingByte above, go to the config menu
    // (potentially a balloon weird case would timeout)
    BALLOON_MODE = false;
    decodeVERBY();

    Watchdog.enable(30000);
    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    while (!Serial) {
        // hmmm.. is Watchdog threadsafe? maybe watch out for that!
        Watchdog.reset();
        updateStatusLED();
        // No race here with anyone looking at it
        // millis since running this program
        // wait for 15 seconds before deciding to switch to balloon mode..
        // no serial
        if (millis() > 15000) break;
        sleep_ms(1000);
    }
    uint32_t sieValue = get_sie_status();
    // https://stackoverflow.com/questions/43028865/how-to-print-hex-from-uint32-t
    V1_printf("SETUP() after finding Serial.* sieValue %" PRIx32 EOL, sieValue);
    Watchdog.reset();
    updateStatusLED();
    // FIX! do we detect Serial if we never open putty, and data + power is connected on USB?
    // eventually we'll time out in config menu and reboot
    // detecting usb is connected
    // read the nvram and decode VERBY and TESTMODE. This will control printing
    read_FLASH_result1 = read_FLASH();
    if (read_FLASH_result1 == -1) {
        // oneliner
        V1_print(F("SETUP() WARN: first read_FLASH_result1 -1 ..retrying"));
        V1_println(F("redo. ..errors were fixed to default"));
        read_FLASH_result2 = read_FLASH();
    }
    Watchdog.reset();
    if (read_FLASH_result2 == -1) {
        V1_println(F("SETUP() ERROR: retry read_FLASH_result2 -1 , ignore"));
    }
    show_values();

    // Get the SIE_STATUS to see if we're connected or what?
    // this is what I see when I'm using the putty window
    // SIE_STATUS:0x40050009
    bool usbConnected = Serial && get_sie_connected();
    V0_printf("SETUP() usbConnected %u" EOL, usbConnected);

    // FIX! is 'Serial" sufficient? it's not formed putty window not opened?
    // FORCE_BALLLON_MODE is test mode:
    // guarantees balloon mode for debug when plugged into USB power
    if (FORCE_BALLOON_MODE | !usbConnected) {
        V0_print(F(EOL "SETUP() set BALLOON_MODE true" EOL));
        BALLOON_MODE = true;
        decodeVERBY();
        // BALLOON_MODE forces all false, so no point in printing here?
        // Watch out! Once enabled, the RP2040's Watchdog Timer can NOT be disabled.
        // Watchdog.disable();
        // just make it very long?
        Watchdog.enable(60000);
        // Watchdog.reset();
        // Serial on core1 is only used for printing (no keyboard input)
        // so okay to manage that with VERBY
        CORE1_PROCEED = true;
    } else {
        BALLOON_MODE = false;
        decodeVERBY();
        V0_print(F(EOL "SETUP() ..Found usb serial. set BALLOON_MODE false" EOL));
        Watchdog.reset();
        // hmm IGNORE_KEYBOARD_CHARS is not factored into this.
        // should always be false at this point?
        // char incomingByte = drainSerialTo_CRorNL(1000);
        // only drain for 10ms.. in case user hits <enter>
        char incomingByte = drainSerialTo_CRorNL(10);
        // CR or LF to interrupt?
        if (incomingByte == 10 || incomingByte == 13) {
            // do this branching BEFORE setting clock speed in case of bad clock speed setting!
            V0_print(F(EOL "LEAVING SETUP() TO GO TO user_interface() (1)" EOL EOL));
            user_interface();
            // won't return here, since all exits from user_interface reboot
        } else {
            V0_print(F(EOL "Hit <enter> to go to config mode."));
            V0_print(F(" Either before gps cold reset, or wait until after" EOL));
            V0_print(F("otherwise it's running (1)" EOL EOL));
            // we drained but it wasn't CR or LF at the end
            // core1 takes over Watchdog.reset() at this point
            CORE1_PROCEED = true;
        }
    }
    // FIX! shouldn't do Watchdog.reset() from here on in, unless core1 is stopped?
    // from here on, if this code hangs, we just don't get keyboard input
    // but that's not an issue if BALLOON_MODE
    if (VERBY[1]) {
        V1_print(F("setup() freeMem()" EOL));
        freeMem();
    }
    V1_print(F(EOL "LEAVING SETUP() (2)" EOL EOL));
    V0_print(F(EOL "Hit <enter> to go to config mode. otherwise it's running (2)" EOL EOL));
}
//*********************************************************
// Don't use the low level pi pico interprocessor stuff
// https://github.com/raspberrypi/pico-examples/tree/master/multicore/multicore_fifo_irqs
// use arduino-pico core interprocessor stuff

// run all balloon stuff on core1
// core 0 just looks for keyboard interrupts. Can interrupt core1 to prevent
// So can't resume core1 after interrupting it. (Serial.print isn't going to resume)
// Can only reboot which is the normal behavior afte messing with config state.

// So core0 can stop core1 at any time. Do stuff with config, then reboot.
// That should be safe.

//**************************************
// Here is our very basic way to not worry about thread-safe
// for keyboard usb serial -> user configuration changes

// rp2040.idleOtherCore()
// Sends a message to stop the other core
// (i.e. when called from core 0 it pauses core 1, and vice versa).
// Waits for the other core to acknowledge before returning.

// The other core will have its interrupts disabled and be busy-waiting in an RAM-based routine,
// so flash and other peripherals can be accessed.

// NOTE idle core 0 too long, and the USB port can become frozen.
// Because core 0 manages the USB and needs to service IRQs
// in a timely manner (which it can’t do when idled).
// So we never idle core 0 !!

//**********************************************************
// FIX! what if did all the updateStatusLED from this core?
// the other core could update the count as a writer and we would just be a reader
// we could strip all the updateStatusLED() out of the other core
// I guess we don't have a watch dog timer just for this core
// but that's okay..worst case we lose keyboard response and if we do led here
// we could lose led update if we hang here.
// so when we loop here, we also do updateStatusLED()
// have to remove all of it from the other core.
// NO! we'd have to wake up more than once a sec. Leave it on the other core
// which is active anyhow

//**************************************
absolute_time_t loop_us_start = 0;
// FIX! is there a watchdog for this core?
bool core1_idled = false;
void loop() {
    if (BALLOON_MODE) {
        // just sleep. hmm do we have any interrupts to deal with
        // hmm can we lose usb printing if we delay too long here?
        // sleep_ms(100000); // 100 secs
        // try just 30 secs
        // sleep_ms(100000);
        sleep_ms(30000);
        // hmm could just return, loop will be called again
        // return;
    }

    // FIX! musing: should never get any usb/serial input while balloon is flying
    // Could disable this if we knew we're flying.
    // How? equivalent to cutting off USB connector.
    // Shouldn't be necessary?
    // if garbage serial arrived, and we went to config mode, eventually we'd timeout/reboot

    bool usbConnected = get_sie_connected();
    while (!BALLOON_MODE && usbConnected && !IGNORE_KEYBOARD_CHARS) {
        // detect the transition of 1 -> 0 on IGNORE_KEYBOARD_CHARS (by core1)  (gps cold reset)
        // and drain all garbage chars in serial. Should only happen around gps cold reset
        // and it's low power nonsense: turning usb off/on
        if (IGNORE_KEYBOARD_CHARS_last & !IGNORE_KEYBOARD_CHARS) {
            // drain it of everything..garbage during gps cold reset
            // should have a limited number of ?? chars
            while (Serial.available()) Serial.read();
        }
        // don't use watchog reset..not thread safe?
        if (core1_idled) {
            // hmm..don't do any fetch from nvram with F()
            V1_print(F(EOL "loop() LOOPING WITH core1_idled()" EOL EOL));
            sleep_ms(1000);
        } else {
            // Serial.print(F(EOL "loop() LOOPING QUICKLY WITH !core1_idled()" EOL EOL));
            // with a 1 sec sleep..most of the time we're sleeping this core?
            // we couldn't sleep that long if we were updating leds? or ??
            sleep_ms(1000);
        }

        // This core can handle modifying config state, not the other core
        // so the other core can just be timely in stopping normal balloon work
        // wen it gets rp2040.idleOtherCore()
        bool usbConnected = get_sie_connected();
        // FIX! is 'charsAvailable" sufficient?
        int charsAvailable = (int) Serial.available();
        if (usbConnected && !IGNORE_KEYBOARD_CHARS && charsAvailable) {
            // CR or LF to interrupt?
            // odd chars while plugged into USB power with data, but no serial window?
            char incomingByte = drainSerialTo_CRorNL(1000);
            // random CR or LF if using usb plug with data and no serial window?
            // hopefully not!
            if (incomingByte == 10 || incomingByte == 13) {
                V0_print(F(EOL "CR or LF detected: Core 0 WILL TRY TO TAKE OVER" EOL EOL));
                Watchdog.enable(30000);
                rp2040.idleOtherCore();
                core1_idled = true;
                V0_print(F(EOL "Core 1 IDLED" EOL EOL));
                // we own the led's and the watch dog interface now
                Watchdog.reset();
                // FIX! this is a just-in-case we had temporarily slowed the clock to 18Mhz
                // during the first gps cold reset, and keyboard interrupted that?
                // this will make the clock right again
                initPicoClock(PLL_SYS_MHZ);
                initStatusLED();
                setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
                updateStatusLED();
                V0_print(F(EOL "Core 0 TOOK OVER AFTER SUCCESSFULLY IDLING Core 1" EOL EOL));
                V0_print(F(EOL "Core 0 IS CURRENTLY DOING NOTHING" EOL EOL));
                V0_println(F("tracker.ino: (A) Going to user_interface() from loop()"));
                user_interface();
                // won't return here, since all exits from user_interface reboot
                // so will never resume the other core if we idled it?
                // rp2040.resumeOtherCore();
            }
        }
        IGNORE_KEYBOARD_CHARS_last = IGNORE_KEYBOARD_CHARS;
    }
}
//***********************************************************
void setup1() {
    // CORE1_PROCEED should start false, so no need to wait?
    sleep_ms(1000);
    // https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html
    // https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html#ga2ad3247806ca16dec03e655eaec1775f
    // Initialize a core such that the other core can lock it out during flash_safe_execute.
    // see the dire need at the link above (around flash access)
    // flash_safe_execute_core_init();

    while (!CORE1_PROCEED) {
        // no printing inside this..potentially BALLOON_MODE/VERBY not setup yet?
        // updateStatusLED();
        // debug: just keep it turning it on until we get a CORE1_PROCEED?
        // hmm. FIX! the setup() will be messing with LED blinking at same time?
        // do we care? (thread-safety?)
        turnOnLED(true);
        sleep_ms(10);
    }
    // take over watchdog and LED from core0
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    Watchdog.enable(30000);
    Watchdog.reset();
    V1_println(F("setup1() START"));

    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

    //*************************************
    Watchdog.reset();
    Watchdog.enable(30000);
    Watchdog.reset();

    V0_flush();
    blockingLongBlinkLED(8);

    adc_INIT();
    Watchdog.reset();

    // was 12/26/24
    // no reason to turn si5351a on during the setup init?
    // but need it on during the repeated inits
    // just get rid of the vfo_init in setup!
    // vfo_init();

    //**********************
    // Some notes in case we ever use Wire
    // https://arduino-pico.readthedocs.io/en/latest/wire.html
    // are the si5351 pins okay (ISC0) for Wire? (ISC1) pins are for Wire1?

    // https://github.com/earlephilhower/arduino-pico/blob/master/docs/pins.rst
    // doc had
    // bool setSDA(pin_size_t VFO_I2C0_SDA_PIN);
    // bool setSCL(pin_size_t VFO_I2C0_SCL_PIN);
    // now says
    // Wire.setSDA(pin_size_t VFO_I2C0_SDA_PIN);
    // Wire.setSCL(pin_size_t VFO_I2C0_SCL_PIN);
    // per May 2022 forum post

    //    Wire.setSDA(VFO_I2C0_SDA_PIN);  // 12
    //    Wire.setSCL(VFO_I2C0_SCL_PIN);  // 13
    //    Wire.begin();

    // I don't use Wire, so this shouldn't matter
    // https://docs.arduino.cc/language-reference/en/functions/communication/wire/
    // default pins for Wire  are SDA=4 SCL=5 (not right for our ISC1? or ??)
    // default pins for Wire1 are SDA=26 SCL=27 ..wants to be our ISC0

    // our ISC0 for the Si5351 is SDA 12, SCL 13))
    // our ISC1 for the BMP    is SDA 2,  SCL 3))

    Watchdog.reset();

    // also turns on and checks for output
    // does a full gps cold reset now?
    // 12/7/24. the GpsINIT covers GpsON() now?
    GpsINIT();

    // usb power means vbat is always on. so a hot reset!
    // we already did a cold reset in the GpsINIT() ..don't do it again!

    if (!BALLOON_MODE) {
        if (!Serial) {
            // V1_println("Why can't we see Serial from setup1()..rebooting");
            setStatusLEDBlinkCount(LED_STATUS_REBOOT_NO_SERIAL);
            // we're going to have to reboot..even balloon needs Serial created?
            // If serial data output buf is full, we just overflow it (on balloon)
            Watchdog.enable(10000);  // milliseconds
            while (true) {
                // maybe just force it off?
                // (after always on above)?
                // show we're gonna reboot with long 1 sec on, 1 sec off
                blockingLongBlinkLED(5);
                // updateStatusLED();
            }
        }
    }

    // back to blinking
    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

    Watchdog.reset();
    // sets minute/lane/id from chan number.
    // FIX! is it redundant at this point?..remove?
    XMIT_FREQUENCY = init_rf_freq(_Band, _lane);
    Watchdog.reset();
    // FIX! do we really have to read flash again. No..I don't think so!
    // keeps the read_FLASH in core1() always? no worries about "safe" access to flash
    // (interrupts and fetch out of nvram?)
    V0_printf("prior read_FLASH() results: read_FLASH_result1: %d read_FLASH_result2: %d" EOL,
        read_FLASH_result1, read_FLASH_result2);
    show_values();

    // This gets undone if we have kazu slow clocks in gps_functions.cpp (during cold reset)
    initPicoClock(PLL_SYS_MHZ);
    // figure out tcxo correction once. here.
    // Remember we reboot after any config change ..i.e. correction..
    // so don't have to worry about the propagation when _correction changes
    SI5351_TCXO_FREQ = doCorrection(SI5351_TCXO_FREQ);

    //***************
    Watchdog.reset();
    bmp_init();

    // FIX! we're not detecting presence of bmp280 correctly?
    if (!bmp.begin()) {
        // i2c_scan();
        V1_println(F("Could not find a valid BMP280 sensor"));
    } else {
        // Default settings from datasheet.. should we do forced sample
        // like weather station recommendations (rather than free running)
        bmp.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::SAMPLING_X16,
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_500);
    }

    //***************
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    updateStatusLED();
    Watchdog.reset();

    //***************
    // have the last one be the current PLL_SYS_MHZ,
    // so we could just use PMW_DIV, PWM_WRAP_CNT to set below
    // FIX! I didn't test all the different frequencies. But 18 is better with 1
    // 125/133 was probably tested with 8. some fuzziness okay cause
    // PWM_DIV/PWM_WRAP_CNT have some range to vary
    if (PLL_SYS_MHZ <= 18) INTERRUPTS_PER_SYMBOL = 1;
    else if (PLL_SYS_MHZ <= 33) INTERRUPTS_PER_SYMBOL = 2;
    else if (PLL_SYS_MHZ <= 66) INTERRUPTS_PER_SYMBOL = 4;
    else if (PLL_SYS_MHZ <= 133) INTERRUPTS_PER_SYMBOL = 8;
    else INTERRUPTS_PER_SYMBOL = 16;

    // calculate for different PLL_SYS_MHZ
    // just to see if we get a good PWM div/wrap cnt for different freqs.
    if (false) {
        calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, 18);
        calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, 60);
        calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, 100);
        calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, 125);
        calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, 133);
    }

    calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);
    V1_printf("calcPwmDivAndWrap() using");
    V1_printf(" PLL_SYS_MHZ %lu PWM_DIV %lu PWM_WRAP_CNT %lu INTERRUPTS_PER_SYMBOL %lu " EOL,
        PLL_SYS_MHZ, PWM_DIV, PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL);

    //***************
    // varies by band PLL_FREQ_TARGET
    if (false && VERBY[1]) {
        set_PLL_DENOM_OPTIMIZE(_Band);
        // FIX! is this needed? do we look in the cache or install it during sweep?
        vfo_calc_cache_flush();
        si5351a_calc_sweep();
        vfo_calc_cache_flush();
    }
    if (false && VERBY[1]) {
        vfo_calc_cache_flush();
        // FIX! is this needed? do we look in the cache or install it during sweep?
        si5351a_calc_sweep_band();
        vfo_calc_cache_flush();
    }
    if (false && VERBY[1]) {
        vfo_calc_cache_flush();
        // FIX! is this needed? do we look in the cache or install it during this search?
        si5351a_denom_optimize_search();
        vfo_calc_cache_flush();
    }

    //***************
    // restore to know fixed values per band
    set_PLL_DENOM_OPTIMIZE(_Band);
    // do this to sweep the symbols for the u4b channel in use and fill the cache for Farey results?
    // FIX! should we calc the 4 symbols?
    double symbolShiftError;
    double symbolAbsoluteError;
    uint32_t pll_num;
    // this walks thru the 4 symbols we're going to use
    si5351a_calc_optimize(&symbolShiftError, &symbolAbsoluteError, &pll_num, true);
    if (USE_FAREY_WITH_PLL_REMAINDER) {
        V1_printf("Using Farey? symbolAbsoluteError %.8f symbolShiftError %.8f" EOL,
        symbolAbsoluteError, symbolShiftError);
    } else {
        V1_print(F("SEED for num-shift algo:"));
        V1_printf(" PLL_DENOM_OPTIMIZE %lu pll_num %lu", PLL_DENOM_OPTIMIZE, pll_num);
        V1_printf(" symbolAbsoluteError %.8f symbolShiftError %.8f" EOL,
        symbolAbsoluteError, symbolShiftError);
    }

    // check 4 digits of precision
    int sse = (int)10000 * symbolShiftError;
    if (sse != 0) {
        V1_printf("WARN:FINAL symbolShiftError != 0 to 4 digits of precision. sse %d" EOL, sse);
    }

    //***************
    V1_println(F(EOL "setup1() END"));
    // show we're done with setup1() with long 2 sec on, 2 sec off
    // the other core won't be messing with led's at this time
    // unless it goes to user config?
    Watchdog.reset();
    blockingLongBlinkLED(4);
    // back to non-blocking blinking
    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    if (VERBY[1]) {
        V1_print(F("setup1() freeMem()" EOL));
        freeMem();
    }
}

//*************************************************************************
uint64_t GpsTimeToLastFix = 0;    // milliseconds

// these are used as globals
// FIX! right now, where are they set?
int TELEN1_val1 = 0;
int TELEN1_val2 = 0;
int TELEN2_val1 = 0;
int TELEN2_val2 = 0;

// FIX! we should probably snap them for consistency in snapForTelemetry!
int t_TELEN1_val1 = 0;
int t_TELEN1_val2 = 0;
int t_TELEN2_val1 = 0;
int t_TELEN2_val2 = 0;

int tx_cnt_0 = 0;
int tx_cnt_1 = 0;
int tx_cnt_2 = 0;
int tx_cnt_3 = 0;
int tx_cnt_4 = 0;

//*************************************************************************
// GPS NMEA bursts: The thinking behind how we deal with it
// LightAPRS only looked for GPS NMEA data when it needed a fix. Not all the time.

// TinyGPS++ is not running as a separate task. Only does work when we call it.
// we call it with every new char, and to get data it creates from a history
// of NMEA sentences in those chars.

// NMEA sentence come in bursts at 1Hz ..
// And the pi pico only has 32 byte receive bufffer on the UART that talks to ATGM336H-51

// Maybe not aligned to a second, but the burst is less than one full second of data.
// and burst intervals are at 1 sec. Data is not spread out over the full second.

// Order of each NMEA sentence is not random either. Order stays the same for each burst.
// Just a interesting note

// we can handle each char at about 300 usec avg.
// Obviously a char processing time is more when TinyGPS++ sees a char is a "end of NMEA sentence"
// (32 deep rx fifo can absorb some chars if we're delayed.
// Always want the rx fifo to be almost empty,
// to allow that little bit buffering if there's any backup.

// If characters arrived at max 9600 baud, thats 1 char per .1 ms or 100 us.
// So we're too slow to handle that.
// but the effective baud rate out of ATGM336H-51 is maybe 900 chars/sec
// at 9660 baud Serial2

// so that's around 1.1ms allowed time per char.

// Plenty..probably even if more processingsometimes by TinyGPS++.
// We can even allow faster baud rate for increased effective chars/sec

// we don't have to locally buffer chars to allow for backpressure from TinyGPS++
// (we could create a local fifo, to effectively absorb more than the 32 deep uart rx fifo)
// If we created a secondary RX buffer to hold 600 plus chars (the entire burst of multiple
// NMEA sentences, per second, for the default enabled sentences (US and Baidu satellites)
// Then if we could absorb/empty it at least every second, we'd never lose anything.

// Less processing/power if we only absorb GPS data when we need it.
// Allows us to sleep when  we don't need a fix update.

// Key that TinyGPS++ absorbs our char send at bounded delays (per NMEA sentence).
// end of NMEA sentence has longer delay.
// Apparently the CR LF is needed as a 'boundary' post checksum?
// FIX! do we need both CR and LF or is one enough? Shouldn't matter.

//*********************************************************************************
// Now about sleepSeconds() for next beacon (HF or VHF).
// https://arduiniana.org/libraries/tinygps/

// If the $GPRMC sentence reports a validity of “V” (void) instead of “A” (active),
// or if the $GPGGA sentence reports fix type “0” (no fix) then those sentences are discarded.

// TinyGPS::GPS_INVALID_AGE is the value when you never got a valid fix.


// FIX! should this be exactly a minute, so all the other things
// bump by seconds will end up being in the same offset in a minute, the next minute?
uint16_t  BEACON_WAIT = 60;  // secs
// seconds sleep if super capacitors/batteries are below BattMin
uint16_t  BATT_WAIT = 1;  // secs

// GPS_LOCATION_AGE_MAX should a bit greater than GPS_WAIT_FOR_NMEA_BURST_MAX
// We could live with data that is more 'stale' but theoretically no older than this?
// how stale can it get? we might be waiting for a new hot fix?
// and the old one is in there and usable

// maybe make an old fix good for up to 5 secs?
// if we did a quick gps power off/on we won't get new
// one for maybe 5 secs. but maybe that says we don't want anything older than 1-2 secs?
// but who knows how tinyGPS++ creates the .age ??
// maybe it needs to be hot fix max time? (allow 5 secs?)
// worse would be if it only updates
// age to 0 when the new fix changes any of location/altitude etc?

// at 180mph we could move 3 horizontal miles in 1 minute?
// hmm. what about a descending balloon?
// fixes could be 1 minute old? that's kind of like a cold fix time

// Definitely do this. having a 70 sec old fix allowed, is good for lower power
// but then we could take a long time to get a cold fix going? doing force cold reset
// warm fix should be fast in GpsON()

// FIX! if we have a good fix, and good age, should we turn gps off
// and only turn it on again when the age is bad?
// was getting fix_age violations around 299824 millisecs max with 700000 here
// 5 minutes? could be due to the go_when_rdy testing...back to back, no room for gps
// const uint32_t GPS_LOCATION_AGE_MAX = 70000;
// lets go 5 minutes! can travel 10 miles in that time though? 2 subsquares?
// we'll be getting long back to back when we do "Extended Telemetry".
// so maybe big max age is needed?
const uint32_t GPS_LOCATION_AGE_MAX = 30000;

// FIX! since we break out of the sleepSeconds when gps data starts
// with Serial2.available(), we could make this bigger?
// needs to be at least 1 sec (a little more) since it
// wants to grab a full burst, and we don't know where we are in the repeating
// burst behavior when we start (idle or in the middle of a burst?)
const int GPS_WAIT_FOR_NMEA_BURST_MAX = 1100;

//*************************************************************************
void loop1() {
    // used to ignore TinyGps++ state for couple of iterations of GPS burst 
    // gathering after turning Gps off then on.
    if (GpsIsOn() && GpsInvalidAllCnt > 0) GpsInvalidAllCnt--;
    GpsInvalidAll = GpsInvalidAllCnt > 0;

    // FIX! updated to have a global static int global GpsInvalidateAll
    // I count that down during loop1() iterations and ignore all gps
    // until it's zero. TinyGPS++ state should have transitioned by then
    // based on new NMEA sentences. (should transition cleanly within 2 secs?)
    // if we were ready to go but not aligned, this is a computed delay into the next
    // minute to align better (not so good if we're only tx starting every 10 minutes
    // but good for test mode _go_when_ready that disables
    // channel starting minute requirement.

    int SMART_WAIT;

    // FIX! should change baud back to 9600 (lower power?) only gettin 1800 baud during 300-400ms
    // when looking for data for slight >1 sec time period (all broadcasts are at 1 sec intervals)
    // they all go in a burst together? so all within one sec, and actually a tighter burst.
    loopCnt++;
    Watchdog.reset();
    V1_printf(EOL "loopCnt %" PRIu64 " loop1() START" EOL, loopCnt);

    // copied from loop_us_end while in the loop (at bottom)
    if (loop_us_start == 0) loop_us_start = get_absolute_time();
    updateStatusLED();

    // always make sure tx is off when we're expecting to be doing Gps fix.
    // does nothing if already off
    // FIX! why are we turning off. Is this just a double-check in case of bugs?
    vfo_turn_off();

    // Gps may already be on
    if (!GpsIsOn()) {
        GpsTimeToLastFix = 0;
    }
    GpsON(false);  // no full cold reset

    // is our solar/battery good?
    float solarVoltage;
    solarVoltage = readVoltage();
    if ( solarVoltage <= BattMin || solarVoltage <= GpsMinVolt ) {
        sleepSeconds(BATT_WAIT);
    } else {
        V1_printf("loop1() good solarVoltage %.f" EOL, solarVoltage);
        // FIX! this can set time and unload NMEA sentences?
        // unload for 2 secs to make sure we get 1 sec broadcasts?
        // actually just need a little over 1 sec.

        // Also: what about corruption if buffer overrun?)
        // NMEA checksum will handle that? okay if we have overrun?
        updateGpsDataAndTime(GPS_WAIT_FOR_NMEA_BURST_MAX);
        gpsDebug();

        // this just handles led for time/fix and gps reboot check/execution
        uint32_t fix_age = gps.location.age();
        bool fix_valid = gps.location.isValid() && !GpsInvalidAll;
        // isUpdated() indicates whether the object’s value has been updated
        // (not necessarily changed) since the last time you queried it
        bool fix_updated = gps.location.isUpdated();
        uint32_t fix_sat_cnt = gps.satellites.value();

        // all the data should be valid to consider it a good fix.
        // this doesn't need qualification on whether we got a good date/time
        // since we check that first, before we do any looking for a 3d fix
        bool fix_valid_all = !GpsInvalidAll &&
            (gps.date.year() >= 2024 && gps.date.year() <= 2034) &&
            gps.satellites.isValid() && (gps.satellites.value() >= 3) &&
            gps.hdop.isValid() &&
            gps.altitude.isValid() &&
            gps.location.isValid() &&
            gps.speed.isValid() &&
            gps.course.isValid();

        // fix_valid_all includes sat cnt >= 3 now.
        // so we won't update time until we get a valid 3d fix?
        // no..should we keep it looser for time?
        // to update time
        // fix_valid is a subset (just 2d location) of fix_valid_all.
        if ( fix_valid && (fix_age < GPS_LOCATION_AGE_MAX) ) {
            setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
        } else {
            // FIX! at what rate is this incremented? ..once per loop iteration (time varies)
            GpsWatchdogCnt++;
            V1_printf("loop1() GpsWatchdogCnt++ %lu" EOL, GpsWatchdogCnt);

            // why doesn't this year check get included in determining valid gps fix?
            // if gps time is valid, we constantly (each NMEA burst grab)
            // update RP2040 time from gps time in gps_functions.cpp updateGpsDataAndTime()
            // so don't here. Only update LED state here, though
            // it is common for gps chips to send out 1/1/2080 dates when invalid
            // I see 2080-01-01 2080-01-07 in SIM65M. 2000-00-00 ? in ATGM336 
            // (is month/day wrong?)
            // On 1/2/25 ~19:00 I saw this: 2080-01-05 23:59:50
            // although time sees okay? (utc time)
            // this is a check for validity
            if (gps.date.year() >= 2024 && gps.date.year() <= 2034)
                setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
            else
                setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

            // FIX! this is loop iterations? could be 60 * 30 secs per loop (30 minutes)
            if (GpsWatchdogCnt > 60) {
                // here's a case, where TinyGps++ said gps valid, but the altitude was wrong
                // Note HDOP was very large, and sat count was 0.
                // Fix Age is in milliseconds

                // Here's an interesting tidbit about deciding whether a "fix" is good,
                // using TinyGPS++ arduino libray

                // I noticed in some code I'm using that they qualify the idea of a "fix" with
                // whether the satellite count is 3.
                // I wonder if I should also quality it with the relevant isValid() bit
                // for the altitude also, or maybe all of the isValid() bits below..
                // that might be best..i.e. could even include course and speed valid,
                //
                // Interestingly, I saw that there are cases where you would think
                // you have a fix, but HDOP is high and satellite count is 0.
                // So either qualifying the notion of "fix" as to having 3 or more sats,
                // or a reasonable non-zero altitude or a reasonable HDOP, seems necessary.
                // Requiring sat count > 3 seems like the right tradeoff.

                // Background
                // TinyGps++ has these methods that return info,
                // that reflects the accumulated info from the NMEA data
                // you've sent it (as individual chars, which it interprets as sentences)
                //
                // The long print at the bottom uses these methods,
                // qualified by the relevant isValid() ..note therre are different ones.
                // gps.date
                // gps.time
                // gps.satellites.value() qualified by gps.satellites.isValid()
                // gps.hdop.value() qualified by gps.hdop.isValid()
                // gps.altitude.meters() qualified by gps.altitude.isValid()
                // gps.location.lat() qualified by gps.location.isValid()
                // gps.location.lng() qualified by gps.location.isValid()
                // gps.location.age() qualified by gps.location.isValid()
                //
                // // heading
                // gps.course.deg() qualified by gps.course.isValid()
                // gps.course.value() qualified by gps.course.isValid()
                // gps.speed.kmph() qualified by gps.speed.isValid()
                //
                // gps.charsProcessed()
                // gps.sentencesWithFix()
                // gps.failedChecksum()

                V1_println(F("ERROR: loop1() GpsWatchdogCnt > 60 ..gps full cold reset"));
                // FIX! have to send cold gps reset, in case ephemeris is corrupted?
                // since vbat is always there.. otherwise this is a warm reset?
                GpsOFF(false);  // don't keep TinyGPS state

                // note that GpsOFF() has public access to TinyGPS++ now and
                // update: it uses some hacked-in public methods to flush, now
                Watchdog.reset();
                sleep_ms(1000);
                // also do gps cold reset.
                GpsON(true);
                GpsWatchdogCnt = 0;
                setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
                // https://forum.arduino.cc/t/possible-to-continue-the-main-loop/95541/5
                return;
            }
        }
        updateStatusLED();

        //*********************
        // some detail on TinyGPS. precision?
        // https://sites.google.com/site/wayneholder/self-driving-rc-car/getting-the-most-from-gps
        // fix_age will be 4294967295 if not valid
        V1_print(F(EOL));
        V1_printf("fix_valid_all %u" EOL, fix_valid_all);
        V1_printf("fix_valid %u" EOL, fix_valid);
        V1_printf("fix_age %lu millisecs" EOL, fix_age);
        V1_printf("fix_sat_cnt %lu" EOL, fix_sat_cnt);
        V1_printf("fix_updated %u" EOL, fix_updated);
        V1_print(F(EOL));

        Watchdog.reset();
        if ( !(fix_valid_all && (fix_age <= GPS_LOCATION_AGE_MAX)) ) {
            if (fix_valid_all) {
                V1_printf("loopCnt %" PRIu64, loopCnt);
                V1_printf(" WARN: GPS issue: valid but fix_age %lu millisecs" EOL, fix_age);
            } else {
                // invalid gps should have this fix_age
                V1_printf("loopCnt %" PRIu64 " WARN: invalid GPS fix.", loopCnt);
                if (fix_age != 4294967295) {
                    V1_printf(" Why unexpected fix_age %lu ? (millis)", fix_age);
                }
                V1_print(F(EOL));
            }
            // Be sure vfo is off (rf noise?), and flush TinyGPS++ state. Then make sure gps is on.
            vfo_turn_off();
            invalidateTinyGpsState();
            GpsON(false);  // no gps cold reset
            sleepSeconds(BEACON_WAIT);

        } else if (fix_sat_cnt <= 3) {  // implied also 'not the first if clause' .. i.e good fix
            // FIX! should we have separate led count for 2d fix and 3d fix?
            V1_printf("loopCnt %" PRIu64 "WARN: GPS fix issue: only %lu sats ..2d fix only" EOL,
                loopCnt, fix_sat_cnt);
            // Be sure vfo is off (rf noise?), and flush TinyGPS++ state. Then make sure gps is on.
            vfo_turn_off();
            invalidateTinyGpsState();
            GpsON(false);  // no gps cold reset
            sleepSeconds(BEACON_WAIT);

        } else {
            V1_printf("loopCnt %" PRIu64 " Good recent 3d fix" EOL, loopCnt);
            // GpsWatchdog doesn't get cleared unti we got a 3d fix here!
            // so we can get a gps cold reset if we never get here!
            GpsWatchdogCnt = 0;
            // snapForTelemetry (all the t_* state) right before we do all the WSPRing
            // we can update the telemetry buffer any minute we're not tx'ing

            // we know we have a good 3d fix at this point
            // we don't check the valid bits again? they could have changed
            // at any time (async) ..so can't really be an atomic grab anyhow?
            // keep the snap close to the valid checks above
            snapForTelemetry();

            // GpsStartTime is reset every time we turn the gps on
            // cleared every time we turn it off (don't care)
            // Should this is also cleared when we turn gps off? no?
            // GpsStartTime is set by gps_functions.cpp

            // FIX! odd case. Did the GPS get turned off, but TinyGPS++ says it still has valid fix?
            // until I figure out why, set GpsTimeToLastFix to 0 for this case
            if (GpsStartTime == 0) {
                GpsTimeToLastFix = 0;
            } else {
                GpsTimeToLastFix = (
                    absolute_time_diff_us(GpsStartTime, get_absolute_time()) ) / 1000ULL;
            }

            V1_printf("loopCnt %" PRIu64 " first Gps Fix, after off->on! "
                PRIu64 " GpsTimeToLastFix %" PRIu64 EOL, loopCnt, GpsTimeToLastFix);

            // sets all the t_* strings above
            // voltage is captured when we write the buff? So it's before GPS is turned off?
            // should we look at the live voltage instead?
            V1_printf("loopCnt %" PRIu64, loopCnt);
            V1_printf(" After snapTelemetry() timeStatus(): %u minute: %u second: %u" EOL,
                timeStatus(), minute(), second());

            // FIX! it should depend on the channel starting minute - 1 (modulo 10)
            // preparations for HF starts one minute before TX time
            // at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57.

            // FIX! why not look at seconds here?
            // it will stall until lined up on secs below
            // align to somewhere in the minute before the callsign starting minute

            // so we can start the vfo 20 seconds before needed
            // if we're in the minute before sending..just live with gps fix
            // because it might take a minute to have another one?
            Watchdog.reset();
            // make sure readVoltage always returns postive # (> 0)
            // readVoltage can return 0
            float voltageBeforeWSPR = readVoltage();
            if (voltageBeforeWSPR >= WsprBattMin) {
                if (!alignMinute(-1)) {
                    // oneliner
                    V1_printf(EOL "loopCnt %" PRIu64 " OKAY: wspr no send.", loopCnt);
                    V1_printf(" because minute() %d second: %d *alignMinute(-1) %u*" EOL,
                        minute(), second(), alignMinute(-1));
                    // we fall thru and can get another gps fix or just try again.
                    // sleep because we don't want to cycle endlessly waiting to align
                    SMART_WAIT = (60 - second() + 20);
                    // oneliner
                    V1_printf("loopCnt %" PRIu64, loopCnt);
                    V1_print(F(" case 0: sleepSeconds() 20 secs into the next minute"));
                    V1_printf(" with SMART_WAIT %d" EOL, SMART_WAIT);
                    sleepSeconds(SMART_WAIT);
                } else {
                    V1_printf("loopCnt %" PRIu64, loopCnt);
                    V1_printf("wspr good alignMinute(-1) and voltageBeforeWSPR %.f" EOL,
                        voltageBeforeWSPR);
                    if (second() > 40) {
                        // to late..don't try to send
                        // minute() second() come from Time.h as ints
                        // oneliner
                        V1_printf(EOL "loopCnt %" PRIu64, loopCnt);
                        V1_print(F(" WARN: wspr no send, past 40 secs in pre-minute:"));
                        V1_printf(" minute: %d *second: %d* alignMinute(-1) %u" EOL,
                            minute(), second(), alignMinute(-1));
                        SMART_WAIT = (60 - second() + 20);
                        // oneliner
                        V1_printf("loopCnt %" PRIu64, loopCnt);
                        V1_print(F(" case 1: sleepSeconds() 20 secs into the next minute"));
                        V1_printf(" with SMART_WAIT %d" EOL, SMART_WAIT);
                        sleepSeconds(SMART_WAIT);
                    } else {
                        // oneliner
                        V1_printf("loopCnt %" PRIu64, loopCnt);
                        V1_print(F(" wspr: wait until 20 secs before starting minute"));
                        V1_printf(" now: minute: %d *second: %d*" EOL, minute(), second());
                        while (second() < 40) {
                            Watchdog.reset();
                            delay(10);  // 10 millis
                            updateStatusLED();
                        }
                        V1_printf("loopCnt %" PRIu64, loopCnt);
                        V1_print(F(" wspr: 20 secs until starting minute."));
                        V1_print(F(" wspr: vfo turn on for warmup."));
                        V1_printf(" now: minute: %d second: %d" EOL, minute(), second());

                        // will call this with less than or equal to 20 secs to go
                        uint32_t hf_freq = XMIT_FREQUENCY;
                        int res = alignAndDoAllSequentialTx(hf_freq);
                        if (res == -1) {
                            // FIX! gps should be off at this point and not firing data? or ?
                            SMART_WAIT = (60 - second() + 20);
                            // oneliner
                            V1_printf("loopCnt %" PRIu64, loopCnt);
                            V1_print(F(" alignAndDoAllSequentialTx() res -1 alignment?"));
                            V1_printf(" sleep with SMART_WAIT %d" EOL, SMART_WAIT);
                            sleepSeconds(SMART_WAIT);
                        }
                    }
                }
            } else {
                // this line will print a lot if we're failing because of this?
                // but we have the min at 0.0 for now
                V1_printf(EOL "loopCnt %" PRIu64, loopCnt);
                V1_printf(" WARN: no send: voltageBeforeWSPR %.f WsprBattMin %.f" EOL,
                    voltageBeforeWSPR, WsprBattMin);
                sleepSeconds(BATT_WAIT);
            }
        }
    }

    //*****************************************************************
    // should have 5 entries after the first looping. problem if less than that?
    // can print an error here if not 5
    uint8_t VCC_valid_cnt = vfo_calc_cache_print_and_check();
    // should be 4 if just sending wspr. 5 if sending cw too
    // won't get any until first wspr tx goes out
    if (tx_cnt_0 > 0)  {
        if (_morse_also[0] == '1') {
            if (VCC_valid_cnt != 5)
                V1_printf("WARN: loop1() VCC_valid_cnt %u != 5" EOL, VCC_valid_cnt);
        } else {
            if (VCC_valid_cnt != 4)
                V1_printf("ERROR: loop1() VCC_valid_cnt %u != 4" EOL, VCC_valid_cnt);
        }
    }
    //*****************************************************************

    absolute_time_t loop_us_end = get_absolute_time();
    int64_t loop_us_elapsed = absolute_time_diff_us(loop_us_start, loop_us_end);
    // floor divide to get milliseconds
    int64_t loop_ms_elapsed = loop_us_elapsed / 1000ULL;

    // maybe show GpsWatchdogCnt also? how old are they
    // FIX! StampPrintf doesn't seem to be printing correctly with this format string?
    // oh I had the wrong formats for the variables. V1_printf exposed that. StampPrintf didn't.
    // use V1_printf()
    V1_println(F(EOL));
    V1_printf(
        "loopCnt %" PRIu64 " "
        "t_tx_count_0: %s "
        "t_callsign: %s "
        "t_temp: %s "
        "t_voltage: %s "
        "t_altitude: %s "
        "t_grid6: %s "
        "t_power: %s "
        "t_sat_count: %s "
        "GpsTimeToLastFix %" PRIu64 " "
        "GpsWatchdogCnt %lu" EOL,
        loopCnt, t_tx_count_0, t_callsign, t_temp, t_voltage,
        t_altitude, t_grid6, t_power, t_sat_count,
        GpsTimeToLastFix, GpsWatchdogCnt);

    V1_println(F(EOL));
    V1_printf(
        "loopCnt %" PRIu64 " "
        "t_tx_count_0: %s "
        "_Band %s "
        "loop_ms_elapsed: %" PRIu64 " millisecs "
        "loop_us_start: %llu microsecs "
        "loop_us_end: %llu microsecs" EOL,
        loopCnt, t_tx_count_0, _Band, loop_ms_elapsed, loop_us_start, loop_us_end);

    updateStatusLED();

    // next start is this end
    loop_us_start = loop_us_end;

    // FIX! does this cause a reboot?
    if (VERBY[1]) {
        V1_print(F("loop1() freeMem()" EOL));
        freeMem();
    }

    // whenever we have spin loops we need to updateStatusLED()
    V1_printf("loopCnt %" PRIu64 " loop1() END" EOL, loopCnt);
}

//*******************************************************
int alignAndDoAllSequentialTx(uint32_t hf_freq) {
    V1_println(F(EOL "alignAndDoAllSequentialTX START"));
    V1_printf(" now: minute: %d second: %d" EOL, minute(), second());

    // if we called this too early, just return so we don't wait 10 minutes here
    // it should loop around in loop1() ..after how much of a wait? smartWait?
    // (calculated delay above?)
    // at most wait up to a minute if called the minute before start time)
    if (!alignMinute(-1)) {
        V1_print(F("FAIL: alignAndDoAllSequentialTX END early out: alignment wrong!"));
        V1_printf(" now: minute: %d second: %d" EOL, minute(), second());
        return -1;
    }

    while (second() < 40)  {
        Watchdog.reset();
        sleep_ms(50);  //
        // we could end up waiting for 40 secs
        updateStatusLED();
    }

    // don't want gps power and tx power together
    GpsOFF(true);  // keep TinyGPS state

    // start the vfo 20 seconds before needed
    // if off beforehand, it will have no clocks running
    // we could turn it off, then on, to guarantee always starting from reset state?
    vfo_turn_off();
    // new: 1/6/2025 was 2000 
    sleep_ms(1000);

    // FIX! does this include a full init at the rp2040?
    // vfo_turn_on() doesn't turn on the clk outputs!
    vfo_turn_on(WSPR_TX_CLK_0_NUM);  // clk0/1 both affected

    //**************************
    // New 1/6/24
    // Quickly cycle thru all 4 symbols to see the cache with the Farey algo results
    // to avoid adding latency due to the iterations when the symbol is first used and cache
    // End with symbol 0
    absolute_time_t start_usecs_1 = get_absolute_time();

    // will print programming if false, since we have time
    // how many in cache to start?
    uint8_t VCC_init_valid_cnt = vfo_calc_cache_print_and_check();
    startSymbolFreq(hf_freq, 3, false);  // symbol 3, change more than just pll_num
    startSymbolFreq(hf_freq, 2, true);   // symbol 2, should just be num and denom change?
    startSymbolFreq(hf_freq, 1, true);   // symbol 1, should just be num and denom change?
    startSymbolFreq(hf_freq, 0, true);   // symbol 0, should just be num and denom change?

    absolute_time_t end_usecs_1 = get_absolute_time();
    int64_t elapsed_usecs_1 = absolute_time_diff_us(start_usecs_1, end_usecs_1);
    float elapsed_millisecs_1 = (float)elapsed_usecs_1 / 1000.0;
    V1_printf("VCC cache initially had %u valid entries\n" EOL, VCC_init_valid_cnt);
    V1_printf("Time to calc, or cache lookup, 4 Farey algo symbol freq si5351 reg values: %.4f millisecs" EOL, 
        elapsed_millisecs_1);

    //**************************
    setStatusLEDBlinkCount(LED_STATUS_TX_WSPR);

    // GPS will stay off for all
    char hf_callsign[7] = {0};
    snprintf(hf_callsign, sizeof(hf_callsign), "%s", t_callsign);
    // same declared size, so could just strncpy
    // strncpy(hf_callsign, t_callsign, 6);

    double lat_double = atof(t_lat);
    double lon_double = atof(t_lon);

    // get_mh_6 users the first arg as pointer to a char array for the return data
    char hf_grid6[7] = { 0 };
    char hf_grid4[5] = "AA00";
    get_mh_6(hf_grid6, lat_double, lon_double);
    // just the first 4 chars
    for (int i = 0; i < 4; i++) hf_grid4[i] = hf_grid6[i];
    hf_grid4[4] = 0;

    char hf_power[3] = { 0 };
    snprintf(hf_power, sizeof(hf_power), "%s", t_power);
    Watchdog.reset();

    // will sync up to the right minute and second == 0
    int txNum = 0;
    V1_print(F(EOL));
    V1_printf("WSPR txNum %d Prepared.." EOL, txNum);
    V1_printf("hf_callsign %-6s" EOL, hf_callsign);
    V1_printf("hf_grid4 %s" EOL, hf_grid4);
    V1_printf("hf_power %s" EOL, hf_power);
    V1_print(F(EOL));
    V1_flush();
    // init to all zeroes just so we know what the encode is doing, when
    // if we get bad symbols when we send the symbols
    syncAndSendWspr(hf_freq, 0, hf_tx_buffer, hf_callsign, hf_grid4, hf_power, false);
    tx_cnt_0 += 1;
    // we have 10 secs or so at the end of WSPR to get this off?
    if (VERBY[1]) {
        StampPrintf("WSPR callsign Tx sent. now: minute: %d second: %d" EOL, minute(), second());
        DoLogPrint();
    }

    // we don't loop around again caring about gps fix, because we've saved
    // telemetry (and sensor data in there) right before the callsign tx
    setStatusLEDBlinkCount(LED_STATUS_TX_TELEMETRY);

    // output: modifies globals: hf_callsign, hf_grid4, hf_power
    txNum = 1;
    V1_printf("WSPR txNum %d Preparing with u4b_encode_std().." EOL, txNum);
    V1_flush();

    u4b_encode_std(hf_callsign, hf_grid4, hf_power,
        t_grid6, t_altitude, t_temp, t_voltage, t_speed, _id13);
    V1_print(F(EOL));
    V1_printf("WSPR txNum %d Prepared.." EOL, txNum);
    V1_printf("hf_callsign %-6s" EOL, hf_callsign);
    V1_printf("hf_grid4 %s" EOL, hf_grid4);
    V1_printf("hf_power %s" EOL, hf_power);
    V1_print(F(EOL));
    V1_flush();

    syncAndSendWspr(hf_freq, 1, hf_tx_buffer, hf_callsign, hf_grid4, hf_power, false);
    tx_cnt_1 += 1;
    // we have 10 secs or so at the end of WSPR to get this off?
    if (VERBY[1]) {
        StampPrintf("WSPR telemetry Tx sent. minute: %d second: %d" EOL, minute(), second());
        DoLogPrint();
    }

    // have to send this if telen1 or telen2 is enabled
    if ( (_TELEN_config[0] != '-' || _TELEN_config[1] != '-') ||
         (_TELEN_config[2] != '-' || _TELEN_config[3] != '-') ) {
        setStatusLEDBlinkCount(LED_STATUS_TX_TELEN1);

        txNum = 2;
        V1_printf("WSPR txNum %d Preparing with u4b_encode_telen().." EOL, txNum);
        V1_flush();

        u4b_encode_telen(hf_callsign, hf_grid4, hf_power, TELEN1_val1, TELEN1_val2, true, _id13);

        V1_print(F(EOL));
        V1_printf("WSPR txNum %d Prepared.." EOL, txNum);
        V1_printf("hf_callsign %-6s" EOL, hf_callsign);
        V1_printf("hf_grid4 %s" EOL, hf_grid4);
        V1_printf("hf_power %s" EOL, hf_power);
        V1_print(F(EOL));
        V1_flush();
        syncAndSendWspr(hf_freq, txNum, hf_tx_buffer, hf_callsign, hf_grid4, hf_power, false);
        tx_cnt_2 += 1;
        if (VERBY[1]) {
            StampPrintf("WSPR telen1 Tx sent. minute: %d second: %d" EOL, minute(), second());
            DoLogPrint();
        }
    }
    // have to send this if telen2 is enabled
    if ( (_TELEN_config[2] != '-' || _TELEN_config[3] != '-') ) {
        setStatusLEDBlinkCount(LED_STATUS_TX_TELEN2);
        // output: modifies globals: hf_callsign, hf_grid4, hf_power
        // input: TELEN2_val1/2 are ints?
        txNum = 3;
        V1_printf("WSPR txNum %d Preparing with u4b_encode_telen().." EOL, txNum);
        V1_flush();
        u4b_encode_telen(hf_callsign, hf_grid4, hf_power, TELEN1_val1, TELEN1_val2, false, _id13);
        V1_print(F(EOL));
        V1_printf("WSPR txNum %d Prepared.." EOL, txNum);
        V1_printf("hf_callsign %-6s" EOL, hf_callsign);
        V1_printf("hf_grid4 %s" EOL, hf_grid4);
        V1_printf("hf_power %s" EOL, hf_power);
        V1_print(F(EOL));
        V1_flush();
        syncAndSendWspr(hf_freq, txNum, hf_tx_buffer, hf_callsign, hf_grid4, hf_power, true);
        tx_cnt_3 += 1;
        // we have 10 secs or so at the end of WSPR to get this off?
        if (VERBY[1]) {
            StampPrintf("WSPR telen2 Tx sent. minute: %d second: %d" EOL, minute(), second());
            DoLogPrint();
        }
    }

    if (_morse_also[0] == '1') {
        setStatusLEDBlinkCount(LED_STATUS_TX_CW);
        txNum = 4;
        V1_printf("CW txNum %d using cw_send_message().." EOL, txNum);
        V1_flush();
        // picks a good HF freq for the config'ed _Band.
        // uses t_callsign a and t_grid6 in the message
        // NOTE: turns GPS back on at the end..so it's assuming it's last after wspr
        cw_send_message();
        // restore the wspr XMIT_FREQUENCY since cw changed it
        // sets minute/lane/id from chan number.
        // FIX! is it redundant at this point?..remove?
        XMIT_FREQUENCY = init_rf_freq(_Band, _lane);
        tx_cnt_4 += 1;
        if (VERBY[1]) {
            StampPrintf("CW Tx sent. minute: %d second: %d" EOL, minute(), second());
            DoLogPrint();
        }
    }

    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    // Now: don't turn GPS back on until beginning of loop
    // then we reset the fix time variables also (on the off -> on transition))
    // GpsON(false);  // no gps cold reset

    V1_println(F("alignAndDoAllSequentialTX END"));
    return 0;  // success
}

//***********************************************************
void sleepSeconds(int secs) {
    // this doesn't have an early out (although the updateGpsDataAndTime() can
    // so the delay will be >= secs
    V1_println(F("sleepSeconds START"));
    V1_flush();
    uint64_t start_millis = millis();
    uint64_t current_millis = start_millis;
    uint64_t duration_millis = (uint64_t) secs * 1000;
    float solarVoltage;
    do {
        Watchdog.reset();
        // can power off gps depending on voltage
        // normally keep gps on, tracking after first fix. we are moving!
        // uint32_t usec = time_us_32();
        GpsON(false);

        solarVoltage = readVoltage();
        if (solarVoltage < BattMin || solarVoltage < GpsMinVolt) {
            V1_printf("sleepSeconds() bad solarVoltage %.f ..(1) turn gps off" EOL,
                solarVoltage);
            GpsOFF(false);  // don't keep TinyGPS state
        }
        // FIX! should we unload/use GPS data during this?
        // gps could be on or off, so no?
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
        if (GpsIsOn()) {
            // does this have a updateStatusLED() ??
            // long enough to be sure to catch all NMEA during the broadcast interval of 1 sec
            // 1050: was this causing rx buffer overrun (21 to 32)
            // 1500 was good for 9600 and up
            // not long enough for 4800?
            // arg is milliseconds
            if ((!USE_SIM65M) && ATGM336H_BAUD_RATE == 4800) updateGpsDataAndTime(2000);
            else updateGpsDataAndTime(1500);
        } else {
            sleep_ms(1500);
        }
        current_millis = millis();
    } while ((current_millis - start_millis) < duration_millis);

    Watchdog.reset();


    // hmm. maybe we just toggle the current state
    // if it's off, we turn it on. If it's on, we turn it off
    // that should be good for power
    if (solarVoltage < BattMin || solarVoltage < GpsMinVolt) {
        V1_printf("sleepSeconds() bad solarVoltage %.f ..(2) turn gps off" EOL,
            solarVoltage);
        GpsOFF(false);  // don't keep TinyGPS state
    } else {
        // I suppose we really want to know how long it's been off
        // shouldn't keep it off if we don't have a valid fix
        // and shouldn't keep it off for more than 1 minute.
        if (!GpsIsOn()) GpsON(false);  // don't keep TinyGps state

        // all the data should be valid to consider it a good fix.
        // this doesn't need qualification on whether we got a good date/time
        // since we check that first, before we do any looking for a 3d fix

        // Should we keep keep on if we don't have a solid fix
        // otherwise save power for a cycle?
        // FIX! not worth it..power spike when turned back on?
        // better to just stay on except duing RF?

        // should hot fix within secs after we turn it back on?
    }

    // Gps gets left off it the voltage was low at any point
    V1_println(F("sleepSeconds END"));
}

//***********************************************************
// -1 is returned if anything illegal
// FIX! should check what caller does if -1
bool alignMinute(int offset) {
    bool aligned = false;
    // FIX! need good time. Does returning false help (i.e. not do any TX)
    if (timeStatus() != timeSet) return true;

    // offset can be -1, 0, 1, 2, 3, 4, 5, 6 (3 messages)
    // if not one of those, set the start minute to -1?
    // caller should detect that and not wait?
    if (offset < -1 || offset > 6) {
        offset = 0;
    }

    // this should have been only set to be char strs 0, 2, 4, 6, or 8
    // this it the channel config minute to align to, plus an offset
    // we only use offsets -1, 0, 2, 4, 6
    // the telemetry and other prep is al done during -1.
    // the u4b channel minutes should all be
    int align_minute = atoi(_start_minute);
    switch (align_minute) {
        case 0: {;}
        case 2: {;}
        case 4: {;}
        case 6: {;}
        case 8: {
            // add 10 to cover the wrap of 0 to -1 with offset -1 (goes to 9)
            align_minute = (10 + align_minute + offset) % 10;
            break;
        }
        default:
            V1_printf("ERROR: Illegal align_minute %d for u4b channel" EOL, align_minute);
            align_minute = 0;
    }

    // WARN: make sure _go_when_rdy is cleared before real balloon flight!
    if (_go_when_rdy[0] == '1') {
        // add 2 to cover the wrap of 0 to -1 with offset -1 (goes to 1)
        align_minute = (2 + align_minute + offset) % 2;
        aligned = (minute() % 2) == align_minute;
    } else {
        aligned = (minute() % 10) == align_minute;
    }
    return aligned;
}

//********************************************
// expected this is called at least 10 secs before starting minute
// if not in the minute before starting minute,
// it will wait until the right starting minute (depends on txNum)
// txNum can be 0, 1, 2, 3, or 4 for cw

void sendWspr(uint32_t hf_freq, int txNum, uint8_t *hf_tx_buffer, bool vfoOffWhenDone) {
    // currently don't do this, but good strategy?
    // Instead of delaying in for 1 sec: we can wait for PROCEED here.
    // If we wait for 2 PROCEEDs, it's okay if the first is short
    // because of unknown PWM counter initial state?

    // Two PROCEED delays kind of get us to the 1-1.5secs in
    // target..so okay?
    // Then we're going more closely interrupt to interrupt right from symbol[0]
    // each of these should just be symbol time delay
    // there will be two extra interrupts in our interrupt cnt in the handler?

    // 0.68266666... per symbol
    // so 2x -> 1.3653333..maybe a little shorter due to code delays
    // should we do one PROCEED plus a fixed delay?
    Watchdog.reset();

    //*******************************
    // Note we print this after the extra PROCEED delay(s). (or any additional fixed delay)
    V1_printf(EOL "sendWspr START now: minute: %d second: %d" EOL, minute(), second());
    if (VERBY[2]) {
        // do a StampPrintf, so we can measure usec duration from here to the first symbol
        // remember, it's usec running time, it's not aligned to the gps time.
        StampPrintf("sendWspr START now: minute: %d second: %d" EOL, minute(), second());
    }
    vfo_turn_on_clk_out(WSPR_TX_CLK_0_NUM, false);  // no print. clk0/1 both affected
    //*******************************
    // earliest time to start is some 'small' time after the 2 minute 0 sec real gps time.
    // i.e. code delays inherent in 'aligned to time' PWM interrupts and my resulting WSPR tx.

    // Assuming both gps/tracker and pc are time-synchronized
    // With no extra tx delay: wsjt-x/sdr says DT = -0.3 ..
    // I guess telling  me I start that much before 1 sec in?

    // If I wait additional "symbol time" delays before tx
    // (most precise running timer I have, interrupt driven)
    // Each additional "symbol time" delay is 0.682666... seconds
    // (symbol time: is duration for 1 wspr symbol tx 8192/12000 or equivalently 256/375)

    //******************
    // static int EXTRA_DELAY_AFTER_PROCEED = 300; // milliseconds
    // hmm..no benefit?
    static int EXTRA_DELAY_AFTER_PROCEED = 0;
    if (EXTRA_DELAY_AFTER_PROCEED<0 || EXTRA_DELAY_AFTER_PROCEED > 1000) {
        V1_printf("ERROR: bad EXTRA_DELAY_AFTER_PROCEED %d.. setting to 0" EOL,
            EXTRA_DELAY_AFTER_PROCEED);
        EXTRA_DELAY_AFTER_PROCEED = 0;
    }

    uint8_t PROCEEDS_TO_SYNC = 0;
    for (int i = 0; i < PROCEEDS_TO_SYNC; i++) {
        // we can sleep a little less than the symbol time,
        // after a PROCEED false -> true transition
        // intersymbol sleep: go as big as we can go safely and not be too big. save power!
        // we need to wake up at intervals though to do the led
        // otherwise we get wrong timing and 3-4 short look like 3-4 long!
        // the 3-4 long are for config/error cases!
        // 660 ms had worked here. but seems like 645 is better
        // to match the real symbol PROCEED 'coarse, then very-fine' alignment and rationale
        // function wakes to update led appropriately
        // resets watchdog too?
        wsprSleepForMillis(645);
        while (!PROCEED) tight_loop_contents();
        PROCEED = false;  // ? to 1 symbol time
    }
    // hmm. not updating led during this
    // this should be adjusted to give us DT=0 with PROCEEDS_TO_SYNC=0
    delay(EXTRA_DELAY_AFTER_PROCEED);

    //******************
    Watchdog.reset();
    uint8_t symbol_count = WSPR_SYMBOL_COUNT;
    uint8_t i;
    absolute_time_t wsprStartTime = get_absolute_time();  // usecs
    for (i = 0; i < symbol_count; i++) {
        uint8_t symbol = hf_tx_buffer[i];
        switch (symbol) {
            case 0: {;}
            case 1: {;}
            case 2: {;}
            case 3: break;
            default:
                V1_printf("ERROR: bad symbol i %u 0x%02x" EOL, i, symbol);
                symbol = 0;
        }

        // FIX! Does this affect drift at sdr?
        // We can get more timing info?
        // i.e. usec accuracy, but not aligned to gps-correct rtc)
        // kazu has his special library to get usec level detail..
        // with 75usec alignment to rtc?
        // these are amount the few VERBY[2] controlled prints
        if (VERBY[2])
            if ((i % 10 == 0) || i == 161) StampPrintf("b" EOL);

        //****************************************************
        startSymbolFreq(hf_freq, symbol, false);  // symbol 0 to 3, just change pll_num

        //****************************************************
        // Don't make StampPrintf log buffer bigger to try to save more
        // deferred printing during a whole wspr message, to avoid the slowdown
        // effects of printing here.
        // Can't make it bigger than 4096 because of ram problems!
        // With jtencode symbols going bad if so.

        // Checking the symbols (when we use them) for valid 0-3 detects the fail case..
        // they go bad if ram space issues!
        // jtencode github readme warns about this issue and corruption.

        // With this reduced rate printing, everything is good enough for the tx.
        // only log every 10 symbols and the last three (160 161)

        // FIX! Does this affect drift at sdr?
        // these are amount the few VERBY[2] controlled prints
        if (VERBY[2])
            if ((i % 10 == 0) || i == 161) StampPrintf("sym: %d" EOL, i);

        PROCEED = false;
        // Ideally we sleep during the symbol time. (a little less than symbol time)
        // hardwired 645 based on tweaking and seeing what SDR DT is reported by WSJT-X
        // we only use 10ms granularity in this routine, so need that slop subtracted
        // here..plus a little more?
        // basically this is a 'coarse', then 'very fine' alignment strategy.
        // resets watchdog too?
        wsprSleepForMillis(645);

        // Will watchdog reset if we don't get PROCEED, which would mean
        // something went wrong with the interrupt handler

        // FIX! we could sleep for a little less than the expected symbol time here
        // that would work..then check PROCEED on wakeup (spin loop here)
        // i.e. sleep a little less than this.
        // The symbol rate is approximately 1.4648 baud (4FSK symbols per second),
        // or exactly 12,000 Hz / 8192
        // 8192 / 12000 = 0.6826666.. secs. So could sleep for 660 millisecs ?

        // on sleeping: https://ghubcoder.github.io/posts/awaking-the-pico/
        // low cost sleep: waking and re-checking time on every processor event (WFE)
        // These functions should not be called from an IRQ handler.

        // static __always_inline void tight_loop_contents ( void )
        // No-op function intended to be called by any tight hardware polling loop.
        // Using this ubiquitously makes it much easier to find tight loops,
        // #ifdef-ed support for lockup debugging might be added

        // Whenever we have spin loops we need to updateStatusLED()
        // Leave it out for now, just in case it affects symbols
        // we know we're done coarse/very-fine split here. so maybe just 15-20 ms spin
        // leds won't blink right during tx?
        // updateStatusLED();
        Watchdog.reset();
        while (!PROCEED) tight_loop_contents();

        // hmm. not updating led during this
        // this should be adjusted to give us DT=0 with PROCEEDS_TO_SYNC=0
        // note we have this same delay before the first symbol, above before the loop!
        delay(EXTRA_DELAY_AFTER_PROCEED);

        // another VERBY[3] (or above) print (only VERBY[3] so far)
        if ((i % 10) == 1) V3_print(".");  // one per symbol
        Watchdog.reset();
    }
    // use deltatime.sh to check the symbol timing distribution after running with V3
    // EOL the dots
    V2_print(F(EOL));

    absolute_time_t wsprEndTime = get_absolute_time();  // usecs
    int64_t wsprDuration = absolute_time_diff_us(wsprStartTime, wsprEndTime);
    // WSPR transmission consists of 162 symbols, each has a duration of 256/375 seconds.
    // 0.68266666666
    // 162 * 256/375 = 110.592 secs total. Compare our duration to that
    float wsprDurationSecs = (float) wsprDuration / 1000000UL;

    if (wsprDurationSecs < 110.0 || wsprDurationSecs > 111.0) {
        V1_print(F(EOL "ERROR: 100.592 secs goal:"));
        V1_printf(EOL "wsprDurationSecs %.5f seems too big or small" EOL, wsprDurationSecs);
        V1_printf("ERROR: wsprDuration %" PRIu64 " usecs" EOL EOL, wsprDuration);
    }

    // if we gathered stuff to log above at VERBY[2] level
    if (VERBY[2]) DoLogPrint();
    disablePwmInterrupts();

    // FIX! leave on if we're going to do more telemetry?
    // or always turn off?
    if (vfoOffWhenDone) {
        vfo_turn_off();
    } else {
        vfo_turn_off_clk_out(WSPR_TX_CLK_0_NUM, true);  // print. clk0/1 both affected
    }

    Watchdog.reset();
    V1_println(F("sendWspr END"));
}

//**********************************
void syncAndSendWspr(uint32_t hf_freq, int txNum, uint8_t *hf_tx_buffer,
    char *hf_callsign, char *hf_grid4, char *hf_power, bool vfoOffWhenDone) {
    V1_printf("syncAndSendWSPR START now: minute: %d second: %d" EOL, minute(), second());
    if (txNum < 0 || txNum > 3) {
        V1_printf("syncAndSendWSPR() bad txNum %d, using 0" EOL, txNum);
        txNum = 0;
    }

    // actual freq for symbol 0 in the log buffer, eventually it will get printed
    // when we're not sending wspr, by something above
    uint8_t symbol = 0;  // can only be 0, 1, 2 or 3
    // don't need the symbol_freq for anything..just want a print here
    // FIX! we could pre-calc the 4 symbol freqs during the first warmup symbol.
    // so if only_pll_num, use the calc'ed freqs.
    // make them globals for use by sendWspr()?

    // get the vfo going!
    startSymbolFreq(hf_freq, symbol, true);  // only_pll_num (expected)

    // encode into 162 symbols (4 value? 4-FSK) for hf_tx_buffer
    // https://stackoverflow.com/questions/27260304/equivalent-of-atoi-for-unsigned-integers
    set_hf_tx_buffer(hf_tx_buffer, hf_callsign, hf_grid4, (uint8_t)atoi(hf_power));

    // this should be fine even if we wait a long time
    int i = 2 * txNum;  // 0, 2, 4, 6
    // FIX! in debug, why aren't we aligning to any even minute?
    V1_printf("waiting for alignMinute(%d) && second()==0)" EOL, i);
    while ( !(alignMinute(i) && (second() == 0)) ) {
        Watchdog.reset();
        // FIX! delay 1 sec? change to pico busy_wait_us()?
        sleep_ms(20);
        // delay(1);
        // whenever we have spin loops we need to updateStatusLED()
        updateStatusLED();
    }
    Watchdog.reset();

    // PWM_WRAP_CNT is full period value.
    // -1 before it's set as the wrap top value.
    PROCEED = false;
    // we constantly reset this for every wspr message,
    // so we know the first interrupt is a little ways
    // out from where we are now, then?
    setPwmDivAndWrap(PWM_DIV, PWM_WRAP_CNT);

    // Now align to 1 seconds in
    // We could adjust this so the wspr starts EXACTLY at 1 sec in or 2 sec in
    // we know we should have still second()==0 at this point

    // can't align by looking for usec offset from realtime
    // the usecs (or millis() we can read is not aligned
    // to the realtime gps time. those are "since program started running"

    sendWspr(hf_freq, txNum, hf_tx_buffer, vfoOffWhenDone);
    V1_println(F("syncAndSendWSPR END"));
}

//**********************************
void set_hf_tx_buffer(uint8_t *hf_tx_buffer,
    char *hf_callsign, char *hf_grid4, uint8_t power) {

    V1_println(F("set_hf_tx_buffer START"));
    // Clear out the transmit buffer
    memset(hf_tx_buffer, 0, 162);  // same number of bytes as hf_tx_buffer is declared

    // wspr_encode(const char * call, const char * loc, const uint8_t dbm, uint8_t * symbols)
    // Takes a callsign, grid locator, and power level and returns a WSPR symbol
    // table for a Type 1, 2, or 3 message.
    // call - Callsign (12 characters maximum.. we guarantee 6 max).
    // loc - Maidenhead grid locator (6 characters maximum).
    // dbm - Output power in dBm.
    // symbols - Array of channel symbols to transmit returned by the method.
    // Ensure that you pass a uint8_t array of at least size WSPR_SYMBOL_COUNT to the method.

    //******************
    bool fatalErrorReboot = false;
    // hf_power needs to be passed as uint8_t
    // were any spaces on the left due to snprintf to hf_callsign? (starting at hf_callsign[0])
    int l;
    l = strlen(hf_callsign);
    V1_printf("length check: hf_callsign %s before jtencode was strlen %d" EOL,
        hf_callsign, l);

    if (l < 3 || l > 6) {
        V1_printf("ERROR: bad length: hf_callsign %s before jtencode was strlen %d" EOL,
            hf_callsign, l);
        fatalErrorReboot = true;
    }

    // shouldn't be any spaces in hf_callsign. possible due to wrong use of snprintf()
    // already checked for any other badness
    for (uint32_t i = 0; i <= sizeof(hf_callsign); i++) {
        if (hf_callsign[i] == ' ') {
            V1_printf("ERROR: hf_callsign '%s' has <space> at %lu" EOL, hf_callsign, i);
            fatalErrorReboot = true;
        }
    }

    //******************
    // no checks on power uint8_t
    //******************
    l = strlen(hf_grid4);
    V1_printf("length check: hf_grid4 %s before jtencode was strlen %d" EOL,
        hf_grid4, l);

    if (l != 4) {
        V1_printf("ERROR: bad length: hf_grid4 %s before jtencode was strlen %d" EOL,
            hf_grid4, l);
        fatalErrorReboot = true;
    }

    for (uint32_t i = 0; i <= sizeof(hf_grid4); i++) {
        if (hf_grid4[i] == ' ') {
            V1_printf("ERROR: hf_grid4 '%s' has <space> at %lu" EOL, hf_grid4, i);
            fatalErrorReboot = true;
        }
    }

    if (fatalErrorReboot) {
        V0_println(F("ERROR: set_hf_tx_buffer() rebooting because of prior fatal error" EOL));
        V0_flush();
        Watchdog.enable(5000);  // milliseconds
        while (true) tight_loop_contents();
    }

    //******************
    jtencode.wspr_encode(hf_callsign, hf_grid4, power, hf_tx_buffer);

    // maybe useful python for testing wspr encoding
    // https://github.com/robertostling/wspr-tools/blob/master/README.md
    V1_println(F("set_hf_tx_buffer END"));
}

//**********************************
int initPicoClock(uint32_t PLL_SYS_MHZ) {
    V1_println(F("initPicoClock START"));
    // frequencies like 205 mhz will PANIC,
    // System clock of 205000 kHz cannot be exactly achieved
    // should detect the failure and change the nvram, otherwise we're stuck even on reboot

    // we know the config read fixed the clock so it's legal
    // we double check here and recover here, but don't update flash if it's wrong
    // to avoid flash conflict resolution issues (multi-core)
    uint32_t clk_khz = PLL_SYS_MHZ * 1000UL;
    if (!set_sys_clock_khz(clk_khz, false)) {
        V1_printf("ERROR: setup1(): RP2040 can't change clock to %lu Mhz. Using %lu instead" EOL,
            PLL_SYS_MHZ, DEFAULT_PLL_SYS_MHZ);
        PLL_SYS_MHZ = DEFAULT_PLL_SYS_MHZ;
        snprintf(_clock_speed, sizeof(_clock_speed), "%lu", PLL_SYS_MHZ);
        // FIX! hmm. this guy could try to write flash, while the other guy is reading?
        // write_FLASH();
        // check the default?
        clk_khz = PLL_SYS_MHZ * 1000UL;
        if (!set_sys_clock_khz(clk_khz, false)) {
            V1_println("ERROR: setup1() The DEFAULT_SYS_MHZ is not legal either. will use 125");
            PLL_SYS_MHZ = 125;
            snprintf(_clock_speed, sizeof(_clock_speed), "%lu", PLL_SYS_MHZ);
            // write_FLASH();
        }
    }

    V1_printf("Attempt to set rp2040 clock to PLL_SYS_MHZ %lu (legal)" EOL, PLL_SYS_MHZ);
    // 2nd arg is "required"
    set_sys_clock_khz(clk_khz, true);

    V1_println(F("initPicoClock END"));
    return 0;
}

//**********************************
void freeMem() {
    V1_print(F("freeMem START" EOL));
    if (!VERBY[1]) return;
    // Nice to use F() for strings that are constant
    // compiled string stays in flash.
    // does not get copied to SRAM during the C++ initialization
    // string it has the PROGMEM property and runs from flash.
    V1_print(F("Free RAM: "));
    V1_print(freeMemory(), DEC);
    V1_println(F(" byte. Why is this < 0?. Is it's calc wrong?"));

    V1_print(F("Free Heap: "));
    V1_print(rp2040.getFreeHeap(), DEC);
    V1_println(F(" byte"));

    V1_print(F("Used Heap: "));
    V1_print(rp2040.getUsedHeap(), DEC);
    V1_println(F(" byte"));

    V1_print(F("Total Heap: "));
    V1_print(rp2040.getTotalHeap(), DEC);
    V1_println(F(" byte"));

    // https://forum.arduino.cc/t/trying-to-make-sense-of-ram-usage/622666
    // char __stack = 0;

    // V1_print(F("__brkval="));
    // V1_println((unsigned int)__brkval);
    // V1_print(F("__malloc_heap_start="));
    // V1_println((unsigned int)__malloc_heap_start);
    // V1_print("__flp=");
    // V1_println((unsigned int)__flp);
    // V1_print("__stack=");
    // V1_println((unsigned int)&__stack);
    // V1_print("stack size=");
    // V1_println(RAM_end - (unsigned int)&__stack);
    // V1_print("Heap size=");
    // V1_println((unsigned int)__brkval - RAM_start);

    V1_print(F("freeMem END" EOL));
}


//**********************
// will just use c style string. stored in array of characters terminated by null: '\0'
// char e[] = "geeks";
// char el[] = {'g', 'f', 'g', '10'}'
// char* c = "geeksforgeeks"

//**********************
// should I work on a local branch and commit to repo less often?
// https://stackoverflow.com/questions/13276909/how-to-do-a-local-only-commit-in-git

// interesting old benchmark testing wiggling gpio
// should run it on the pi pico and see
// https://github.com/hzeller/rpi-gpio-dma-demo#direct-output-loop-to-gpio

//*****************************************************************
// wsprry-pi still lives?
// https://wsprry-pi.readthedocs.io/en/latest/About_Wsprry_Pi/index.html
// wsprry pi starts at 2 secs in
// https://wsprry-pi.readthedocs.io/en/latest/About_WSPR/index.html
// https://www.qrp-labs.com/ultimate3/u3info/dt.html

//*****************************************
// FIX! should replace all atoi() because of no error handling defn. for atoi()
// atoi()
// strtol()
// strtoll()
// strtoul()
// https://pubs.opengroup.org/onlinepubs/9699919799/functions/strtoul.html
// strtoull()
// strtoimax()
// strtoumax()
// https://pubs.opengroup.org/onlinepubs/9699919799/functions/strtoumax.html

//*****************************************
// KC3LBR has a MCP1640T boost converter
// what about the kazu boost converter (at his *gen1 repo as .eprj.)
