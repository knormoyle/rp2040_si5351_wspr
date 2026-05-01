// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// Uses 4 space indent, OTBS (One True Brace) indentation style
// Linted with Google's cpplint. using Googl'es C++ Style guide
// cpplint --linelength=100
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

// ms5351m doesn't appear to honor clock disable?
// honors PDN bit, but need to pll reset if that's off

// used in cw_functions.cpp only?
bool USE_SI5351A_CLK_POWERDOWN_MODE = true;

// FIX! if we use this close to where we shift freqs, is the pll unstable?
bool USE_SI5351A_CLK_POWERDOWN_FOR_WSPR_MODE = false;
// need this true if prior is true
bool DO_CLK_OFF_FOR_WSPR_MODE = false;

// These are in arduino-pio core
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries
// #include <SPI.h>
// #include <Wire.h>

// The Raspberry Pi Pico SDK is included with the arduino-pico core,
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

// not used yet
// #include <morse.h>  // https://github.com/markfickett/arduinomorse

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
// time of last PPS 0->1 from irq, so we can calc offset duration when we setTIme
uint32_t PPS_rise_millis = 0;
uint32_t PPS_rise_micros = 0;
bool PPS_rise_valid = false;
uint32_t PPS_rise_cnt = 0;

// if this is non-zero we've synced time
// we can tell how long it's been since we've synced, also
uint32_t setTime_millis = 0;

// if we need to ignore TinyGps++ state for a while, because
// we turned off the Gps, and then TinyGps++ won't change
// state until we give it a full broadcast's worth of sentences (1 sec of broadcast)
uint32_t GpsInvalidAllCnt = 0;
bool GpsInvalidAll = false;

// when should do a gps cold reset? when not getting a new fix we can use
uint32_t GpsWatchdogCnt = 0;

// gps_functions.cpp refers to this
TinyGPSPlus gps;

// FIX! why can't we do this in tinygps_functions.cpp ?
// somehow this has to be a global, that TinyGPS and tinygps_functions.cpp can see
// correctly?
bool USE_SIM65M = false;

TinyGPSCustom gp_sats(gps, "GPGSV", 3);
// FIX! do we only get GNGSA ?? ?? don't need?
TinyGPSCustom gp_mode_op(gps, "GPGSA", 1);
TinyGPSCustom gp_mode_nav(gps, "GPGSA", 2);
// the CASIC spec says different fields?
// $GPGSA,A,3,01,03,04,16,,,,,,,,,4.7,2.7,3.9*3F
TinyGPSCustom gp_pdop(gps, "GPGSA", 15);
TinyGPSCustom gp_hdop(gps, "GPGSA", 16);
TinyGPSCustom gp_vdop(gps, "GPGSA", 17);

// $GAGSV,1,1,03,10,61,127,30,25,48,313,41,05,10,227,,7*4D
// $GBGSV,1,1,03,35,74,049,36,44,54,226,42,45,23,137,40,1*48

// SIM65M
TinyGPSCustom ga_sats(gps, "GAGSV", 3);  // Galileo

// do both, because config can change USE_SIM65M ??
// SIM65M
TinyGPSCustom ggb_sats(gps, "GBGSV", 3);  // BeiDou
// ATGM336H
TinyGPSCustom gbd_sats(gps, "BDGSV", 3);  // BeiDou

// FIX! do we only get GNGSA ?? ?? don't need?
// no..was getting. on ATGM336H?
// eventually replaced by $GNGSA, maybe?
// $BDGSA,A,3,21,22,44,,,,,,,,,,3.8,1.9,3.3*23
// $GPGSA,A,3,10,18,27,32,,,,,,,,,3.8,1.9,3.3*3D

// SIM65M and ATGM336H ?
TinyGPSCustom gb_mode_op(gps, "BDGSA", 1);
TinyGPSCustom gb_mode_nav(gps, "BDGSA", 2);
TinyGPSCustom gb_pdop(gps, "BDGSA", 15);
TinyGPSCustom gb_hdop(gps, "BDGSA", 16);
TinyGPSCustom gb_vdop(gps, "BDGSA", 17);

TinyGPSCustom gl_sats(gps, "GLGSV", 3);
// FIX! do we only get GNGSA ?? ?? don't need?
TinyGPSCustom gl_mode_op(gps, "GLGSA", 1);
TinyGPSCustom gl_mode_nav(gps, "GLGSA", 2);
TinyGPSCustom gl_pdop(gps, "GLGSA", 15);
TinyGPSCustom gl_hdop(gps, "GLGSA", 16);
TinyGPSCustom gl_vdop(gps, "GLGSA", 17);

#include "print_functions.h"
#include "debug_functions.h"
#include "config_functions.h"
#include "tele_functions.h"
#include "mh_functions.h"
#include "adc_functions.h"
#include "wspr_functions.h"
#include "u4b_functions.h"
#include "bmp_functions.h"
#include "led_functions.h"
#include "keyboard_functions.h"
#include "gps_functions.h"
#include "si5351_functions.h"
#include "time_functions.h"
#include "tinygps_functions.h"
#include "cw_functions.h"
#include "sweep_functions.h"
#include "doug_functions.h"
#include "global_structs.h"
#include "pps_functions.h"

//*********************************
JTEncode jtencode;

//*********************************
// all extern consts can be externed by a function
// the linker will will fail on these global constants unless labelled
// extern here also (like in the other files that reference this)
extern const int STATUS_LED_PIN = 25;

//*********************************
// some stuff on using namespace
// https://forum.arduino.cc/t/using-a-constant-defined-in-the-header-file/380178

// flash background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/

//*********************************
// extern consts can be externed for reference in another file (like gps_functions.cpp)
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

// 7/10/25 was:
// extern const int SIM65M_BAUD_RATE = 9600;
extern const int SIM65M_BAUD_RATE = 9600;
// extern const int SIM65M_BAUD_RATE = 115200;

// extern const int SIM65M_BAUD_RATE = 19200;
// extern const int SIM65M_BAUD_RATE = 38400;
// extern const int SIM65M_BAUD_RATE = 57600;
// default at power up
// extern const int SIM65M_BAUD_RATE = 115200;

// does this close putty if true?
bool ALLOW_USB_DISABLE_MODE = false;
bool ALLOW_KAZU_12MHZ_MODE = false;
bool ALLOW_TEMP_12MHZ_MODE = true;
// causing intermittent fails if true?
// maybe okay now with proper sequencing in slow_clock_functions.c
bool ALLOW_LOWER_CORE_VOLTAGE_MODE = true;

//*********************************************

// legal choices? 9600 (default ATGM336H) 19200 38400
// is in receive fifo 12-bit wide default 32 deep ??
// https://arduino-pico.readthedocs.io/en/latest/serial.html

// hmm sometimes have to restart Arduino IDE when it gets stuck in
// bad state after changing baud rates
// is my full cold reset not working to gps and vbat is remembering the baud?
// so I can't talk to him without a full power off (vcc and vbat?)
// AG6NS board can't turn off VBAT under progam control

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
// hmm something was not working with gps hot reset with 4800. back to 9600

// seem to leave the polling for NMEA data too soon at 4800..miss some chars
// extern const int ATGM336H_BAUD_RATE = 4800;

extern const int ATGM336H_BAUD_RATE = 9600;
// extern const int ATGM336H_BAUD_RATE = 19200;

// can't seem to recover to 9600 after trying 38400?
// full cold reset not working?
// need to unplug usb to get back to 9600

// okay
// extern const int ATGM336H_BAUD_RATE = 38400;

// this is too fast. I can't keep up with incoming data
// GPS burst: duration is 165 millisecs for 646 chars. 3954 baud effective
// extern const int ATGM336H_BAUD_RATE = 57600;

// don't use. rx buffer overruns
// extern const int ATGM336H_BAUD_RATE = 115200;

//*********************************
// double precision fp mantissa only has 52 bits of precision
// scaled integer arith has 64 bits of precision because we have
// 64-bit integers to use.
// So scaled integer math can have more precision if you can cover the
// dynamic range of values needed.
// But: the numbers eventually go into fp doubles? so can't exceed 52 bits?

// The biggest numbers are the PLL freq. 900 Mhz at most.

// And if you want the possibility of 1e-6 precision for the wspr shifts,
// that's 10e6 more range needed (which is 20 bits!!)
// How many bits needed altogether:
// log2(900e6 * 10e6) = 52.99 bits needed.
// So actually, it's pushing the limits of what you can do with
// double precision fp (which only  has 52 bits of precision)

// Sure the fact that the si5351a divisor chops some low order bits out,
// in the output freq you get, means you can reduce the precision needed
// by those lost bits.
// But with a small divisor like 19, that's just 4 or 5 bits less precision
// needed.

// The integer-scaled arith is optimally a left shift/right shift,
// because that's powers of two,
// so less likely to misunderstand where bits get lost

// If I shift everything left by 15 bits when I do the integer-scaled arith
// to figure out the real I need to get a fraction for.
// so 900e6 * 2**15 = 2.95e13 and that needs log2(2.95e13) = 44.74 bits.

// I should be able to shift by 20 bits to get the 1e-6 precision noted above?

// Once I get that (scaled) remainder, I cast it as a double fp, then divide
// by pow(2, 15)
// To get back to a real I want to feed the Farey algo
// (which is between 0 and 1) to get fraction (num/denom).

// That gives me perfect accuracy for getting a Farey result that has perfect
// absolute accuracy and perfect symbol shift accuracy.

// Once you get to the 0-1 real for Farey,
// then you you have plenty of bits of precision if you use double fp.
// Since you're no longer dealing with the full 900e6 of the pll.

// Hmm ran into problem where compile was messing up an 64-bit integer divide,
// probably because it was optimizing away shifts in the same equation
// and losing precision changed to separate assigns,
// and used volatile on he divide.

// uint64_t PLL_CALC_SHIFT = 16;
uint64_t PLL_CALC_SHIFT = 15;

// 1/11/25 NEW: set PLL_FREQ_TARGET by band down below
// overwrites this.
uint64_t PLL_FREQ_TARGET = 600000000;

// nice 10M result with this
// symbolAbsoluteError: 0.000003 Hz
// symbolShiftError: 0.000006 Hz
// channel 0 symbol 3 actual 28126024.394533 actual_pll_freq 618772536.679732
// pll_mult 23 pll_num 742154 pll_denom 928919 ms_div 22
// uint64_t PLL_FREQ_TARGET = 650000000;

// target PLL freq when making muliplier/divider initial calculations
// could change this per band?
// the implied mul/div for 5 bands is covered by denom choices
// from spreadsheet for 700000000

// 15 (min multiplier) * 26Mhz = 390 Mhz
// the other (not used PLL) will run at this freq in default config?
// so what about targetting that? (will it error on the multiplier?)
// check that code (set to min 15 if too small in si5351_functions.cpp
// so we should be able

//***************************
// 390Mhz pll test:
// here's results from the magic spreadsheet showing denom values
// for 10/12/15/17/20M
// with mult/divisor selected to get pll in the 390Mhz region.
// 10M doesn't have a good value (uses max value:
// means there was no perfect value)
//
// It shows that I can't get optimal denominator on 10M
// ..so the target pll freq is too low.
// But there were numerator-step-1 values that worked for 12/15/17/20M
// I don't show actual pll freq in spreadsheet,
// because I generate the numerator elsewhere (in tracker code)
//
// no green cells on 10M (and the 12M value is small)
// so I won't use 390Mhz target because
// I want the perfect symbol shift on 10M too

//***************************
extern const int VFO_VDD_ON_N_PIN = 4;
// are these really on Wire1
extern const int VFO_I2C_SDA_PIN = 12;
extern const int VFO_I2C_SCL_PIN = 13;

// extern const int VFO_I2C_SCL_HZ = (1000 * 1000);
// maybe go lower frequency?
extern const int VFO_I2C_SCL_HZ = (100 * 1000);
extern const int BMP_I2C_SCL_HZ = (100 * 1000);

extern const int BMP_I2C_SDA_PIN = 2;
extern const int BMP_I2C_SCL_PIN = 3;

bool BMP_FOUND = false;
// this default doesn't get right? SDA/SDK?
Adafruit_BMP280 bmp(&Wire1);

// FIX! used in i2c_functions for test of both i2c0 and i2c1
// pullup resistors are different on each i2c bus on the pcb
extern const int PICO_I2C_CLK_HZ = (100 * 1000);

// FIX! are the pcb pullups less aggressive on BMP i2c?
// maybe have to stay slower on speed?

// The I2C address for the MS5351M is the same as the Si5351A-B-GT/GTR
extern const int SI5351A_I2C_ADDR = 0x60;

// config and telemetry (snapped) structs
ConfigStruct cc = { 0 };
TeleStruct tt = { 0 };

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

// decode of cc._clock_speed
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
// now we're going to calc it and not use the hardwired values
// will just use this instead? Doesn't work for when we do band sweep
// (currently disabled test)
uint32_t PLL_DENOM_OPTIMIZE_calced = 1048575;

// can be 0 1 2 or 3
uint8_t SOLAR_SI5351_TX_POWER = 0;

// bool USE_FAREY_WITH_PLL_REMAINDER = true;
// bool TEST_FAREY_WITH_PLL_REMAINDER = true;
// bool USE_FAREY_CHOPPED_PRECISION = true;
// bool DISABLE_VFO_CALC_CACHE = false;

bool USE_FAREY_WITH_PLL_REMAINDER = false;
bool USE_FAREY_CHOPPED_PRECISION = false;
bool TEST_FAREY_WITH_PLL_REMAINDER = false;
bool DISABLE_VFO_CALC_CACHE = false;
bool USE_MFSK16_SHIFT = false;

//*****************************
bool BALLOON_MODE = true;
bool CORE1_PROCEED = false;
// decode of cc._testmode
bool TESTMODE = false;
// decode of cc._verbose 0-9
bool VERBY[10] = { false };

//*****************************
// tt.power is clamped to string versions of these. use 0 if illegal
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
    // -----------------------------------------------------------------------
    // Flash / SMP safety references (not yet enabled):
    //   flash_safe_execute_core_init() — lets the other core lock this one out
    //   during flash_safe_execute(). See dire warnings at:
    //   https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html
    //
    // Thread-safety notes for time primitives (from same source):
    //   sleep_us            — Depends on children; NOT fully resolved (sleep_until)
    //   absolute_time_diff_us, get_absolute_time — Looks SMP/thread safe
    //   to_ms_since_boot, to_us_since_boot       — Looks SMP/thread safe
    //   time_reached, busy_wait_until             — Looks SMP/thread safe
    //
    // USB/watchdog edge case:
    //   FIX! If USB is connected on boot, some printing occurs, then USB is
    //   unplugged — does the watchdog correctly detect the Serial hang and reboot?
    //   Likely reboots from power loss anyway. Thread safety of Watchdog TBD.
    //   Ref: https://forums.raspberrypi.com/viewtopic.php?t=370841
    // -----------------------------------------------------------------------

    BALLOON_MODE = false;
    decodeVERBY();

    // -----------------------------------------------------------------------
    // Early init: LED and watchdog
    // -----------------------------------------------------------------------
    Watchdog.enable(30000);
    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

    // -----------------------------------------------------------------------
    // Wait up to 15 seconds for a USB serial connection.
    // If none appears, fall through and decide on BALLOON_MODE below.
    // FIX! Do we detect Serial if power+data USB is connected but no terminal opened?
    // FIX! In case user is frantically reaching for the config menu, any byte
    //      received above should route directly there.
    // -----------------------------------------------------------------------
    while (!Serial) {
        Watchdog.reset();   // FIX! verify Watchdog is thread-safe here
        updateStatusLED();
        if (millis() > 15000) break;
        sleep_ms(1000);
    }

    uint32_t sieValue = get_sie_status();
    V1_printf("SETUP() after finding Serial.* sieValue %" PRIx32 EOL, sieValue);
    // Ref for printing hex from uint32_t:
    // https://stackoverflow.com/questions/43028865/how-to-print-hex-from-uint32-t

    Watchdog.reset();
    updateStatusLED();

    // -----------------------------------------------------------------------
    // Read config from flash (NVRAM).
    // VERBY and TESTMODE are decoded from this; they control all subsequent printing.
    // Retry once on failure — errors are reset to defaults on the retry.
    // -----------------------------------------------------------------------
    read_FLASH_result1 = read_FLASH();
    if (read_FLASH_result1 == -1) {
        V1_println(F("SETUP() WARN: first read_FLASH_result1 -1 ..retrying. errors fixed to default"));
        read_FLASH_result2 = read_FLASH();
    }

    Watchdog.reset();
    if (read_FLASH_result2 == -1) {
        V1_println(F("SETUP() ERROR: retry read_FLASH_result2 -1 , ignore"));
    }

    show_values();

    // -----------------------------------------------------------------------
    // Determine USB connection state.
    // SIE_STATUS 0x40050009 is what appears when Putty is open and active.
    // FIX! Is checking 'Serial' alone sufficient? It may be true before a
    //      terminal window is actually opened.
    // -----------------------------------------------------------------------
    bool usbConnected = Serial && get_sie_connected();
    V0_printf("SETUP() usbConnected %u" EOL, usbConnected);

    // -----------------------------------------------------------------------
    // BALLOON_MODE vs. USB-connected (interactive) mode
    //
    // FORCE_BALLOON_MODE is a debug flag that forces balloon mode even when
    // plugged into USB power, so normal field operation can be simulated.
    //
    // Note: once enabled, the RP2040 watchdog CANNOT be disabled.
    //   Watchdog.disable() is a no-op — do not rely on it.
    // -----------------------------------------------------------------------
    if (FORCE_BALLOON_MODE || !usbConnected) {
        V0_print(F(EOL "SETUP() set BALLOON_MODE true" EOL));
        BALLOON_MODE = true;
        decodeVERBY();  // BALLOON_MODE forces all VERBY false; reprinting is harmless

        // Extend watchdog timeout for balloon operation (cannot disable it)
        Watchdog.enable(60000);

        // core1 handles Watchdog.reset() and serial printing from here on;
        // no keyboard input is expected in balloon mode.
        CORE1_PROCEED = true;

    } else {
        BALLOON_MODE = false;
        decodeVERBY();
        V0_print(F(EOL "SETUP() ..Found usb serial. set BALLOON_MODE false" EOL));

        Watchdog.reset();

        // Drain serial briefly to catch an immediate <Enter> keypress.
        // IGNORE_KEYBOARD_CHARS should always be false at this point.
        // FIX! IGNORE_KEYBOARD_CHARS is not factored into this path.
        char incomingByte = drainSerialTo_CRorNL(10);  // 10 ms drain

        if (incomingByte == '\n' || incomingByte == '\r') {
            // Enter pressed immediately — go to config UI before any clock speed
            // changes take effect, in case the clock speed setting is bad.
            V0_print(F(EOL "LEAVING SETUP() TO GO TO user_interface() (1)" EOL EOL));
            user_interface();
            // user_interface() never returns — all exits reboot the device.
        } else {
            V0_print(F(EOL "Hit <enter> to go to config mode."));
            V0_print(F(" Either before gps cold reset, or wait until after" EOL));
            V0_print(F("otherwise it's running (1)" EOL EOL));

            // core1 takes over Watchdog.reset() from here; setup() hangs are
            // acceptable since we only lose keyboard input, not balloon operation.
            // FIX! avoid calling Watchdog.reset() from setup()/loop() once core1 is running.
            CORE1_PROCEED = true;
        }
    }

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
// Can only reboot which is the normal behavior after messing with config state.

// So core0 can stop core1 at any time. Do stuff with config, then reboot.
// That should be safe.

//**************************************
// Here is our very basic way to not worry about thread-safe
// for keyboard usb serial -> user configuration changes

// rp2040.idleOtherCore()
// Sends a message to stop the other core
// (i.e. when called from core 0 it pauses core 1, and vice versa).
// Waits for the other core to acknowledge before returning.

// The other core will have its interrupts disabled and be busy-waiting
// in an RAM-based routine, so flash and other peripherals can be accessed.

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
    // -----------------------------------------------------------------------
    // BALLOON_MODE: core0 has nothing to do — core1 runs the tracker.
    // Just keep core0 alive with periodic sleeps.
    // FIX! Do we need to service any interrupts here?
    // FIX! Can a long sleep here cause USB printing to stall?
    // -----------------------------------------------------------------------
    if (BALLOON_MODE) {
        sleep_ms(30000);
        return;
    }

    // -----------------------------------------------------------------------
    // USB serial keyboard monitor (ground/bench mode only)
    //
    // Watches for a CR/LF keypress and, when found, idles core1 and hands
    // control to the interactive config UI.
    //
    // Notes:
    //   - In flight this code is unreachable (BALLOON_MODE is true).
    //   - IGNORE_KEYBOARD_CHARS is set by core1 around GPS cold resets to
    //     suppress spurious serial input from USB power glitches.
    //   - Watchdog.reset() deliberately avoided here — not thread-safe with core1.
    //   - rp2040.idleOtherCore() is safe because only core0 modifies config state;
    //     core1 stops promptly when idled.
    //   - user_interface() never returns; all its exit paths reboot the device,
    //     so rp2040.resumeOtherCore() is never needed.
    //   FIX! Is checking Serial.available() sufficient, or do we need more?
    //   FIX! Odd chars can arrive on USB data+power lines with no terminal open —
    //        verify we don't accidentally enter config mode from those.
    // -----------------------------------------------------------------------

    // FIX! this wouldn't work if we had been in 12Mhz mode
    bool usbConnected = get_sie_connected();
    while (!BALLOON_MODE && usbConnected && !IGNORE_KEYBOARD_CHARS) {

        // On the falling edge of IGNORE_KEYBOARD_CHARS (core1 cleared it after
        // a GPS cold reset), drain any garbage chars that accumulated.
        bool ignoreJustCleared = IGNORE_KEYBOARD_CHARS_last && !IGNORE_KEYBOARD_CHARS;
        if (ignoreJustCleared) {
            while (Serial.available()) Serial.read();
        }
        IGNORE_KEYBOARD_CHARS_last = IGNORE_KEYBOARD_CHARS;

        // Pace the loop regardless of core1 state.
        // 1-second sleep keeps this core mostly idle while core1 does real work.
        if (core1_idled) {
            // core1 is stopped; avoid F() / NVRAM fetches while idled
            V1_print(EOL "loop() LOOPING WITH core1_idled()" EOL EOL);
        }
        sleep_ms(1000);

        // Re-check connection state after sleeping
        usbConnected = get_sie_connected();

        // Check for incoming keypress
        int charsAvailable = (int)Serial.available();
        if (!usbConnected || IGNORE_KEYBOARD_CHARS || !charsAvailable) continue;

        // Drain until CR or LF; ignore anything else
        char incomingByte = drainSerialTo_CRorNL(1000);
        bool enterPressed = (incomingByte == '\n' || incomingByte == '\r');
        if (!enterPressed) continue;

        // ---- CR/LF received: take over from core1 and enter config UI ----
        V0_print(F(EOL "CR or LF detected: Core 0 WILL TRY TO TAKE OVER" EOL EOL));

        Watchdog.enable(30000);
        rp2040.idleOtherCore();
        core1_idled = true;

        V0_print(F(EOL "Core 1 IDLED" EOL EOL));

        // Core0 now owns the LED and watchdog interfaces
        Watchdog.reset();

        // FIX! Just-in-case fix: if core1 was mid-GPS-cold-reset and had slowed
        // the clock to 18 MHz, restore the correct clock speed before UI work.
        // FIX! doesn't work if ALLOW_KAZU_12MHZ_MODE forces/keeps 12MHZ (gps_functions.cpp)
        // ah..we can do because we block with IGNORE_KEYBOARD_CHARS while in 12MHZ

        // FIX! need to restore the clocks assuming it could have been in 12MHZ!
        initPicoClock(PLL_SYS_MHZ);
        initStatusLED();
        setStatusLEDBlinkCount(LED_STATUS_USER_CONFIG);
        updateStatusLED();

        V0_print(F(EOL "Core 0 TOOK OVER AFTER SUCCESSFULLY IDLING Core 1" EOL EOL));
        V0_println(F("tracker.ino: Going to user_interface() from loop()"));

        user_interface();
        // Never reached — user_interface() reboots on all exit paths
    }
}
//***********************************************************
void setup1() {
    // -----------------------------------------------------------------------
    // Core1 startup: wait for core0 to signal readiness.
    //
    // CORE1_PROCEED starts false, so we spin here until setup() on core0
    // has finished flash/config reads and set it true.
    //
    // Flash / SMP safety (not yet enabled):
    //   flash_safe_execute_core_init() — lets core0 lock out core1 during
    //   flash_safe_execute(). See:
    //   https://k1.spdns.de/Develop/Projects/pico/pico-sdk/build/docs/doxygen/html/group__pico__flash.html
    //
    // No printing inside the spin loop — BALLOON_MODE and VERBY may not be
    // configured yet. LED is held on as a visual "waiting" indicator.
    // FIX! core0 setup() also drives the LED here; thread-safety unclear.
    // -----------------------------------------------------------------------
    sleep_ms(1000);

    while (!CORE1_PROCEED) {
        turnOnLED(true);
        sleep_ms(10);
    }

    // -----------------------------------------------------------------------
    // Take over watchdog and LED from core0
    // -----------------------------------------------------------------------
    Watchdog.enable(30000);
    Watchdog.reset();
    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

    V1_println(F("setup1() START"));

    V0_flush();
    blockingLongBlinkLED(8);  // visual "setup1 running" indicator

    // -----------------------------------------------------------------------
    // Hardware init
    //
    // VFO (si5351a) is NOT initialised here — no reason to power it up during
    // setup; it will be initialised on first use.
    // Wire / I2C pin notes:
    //   Wire  default: SDA=4,  SCL=5  (wrong for our ISC1)
    //   Wire1 default: SDA=26, SCL=27 (matches our ISC0)
    //   Pins would be set via Wire1.setSDA / Wire1.setSCL before Wire1.begin().
    //   Not currently used since BMP and GPS use their own init paths.
    // -----------------------------------------------------------------------
    adc_INIT();
    Watchdog.reset();

    bmp_init();
    Watchdog.reset();

    // GpsINIT() includes a full GPS cold reset and covers GpsON() internally.
    // Do NOT call GpsON() separately after this — cold reset already done.
    GpsINIT();
    tinyGpsCustomInit();
    gpsPPS_init();

    // Initialise extended-telemetry codec objects (time slot set later)
    define_codecGpsMsg();
    define_codecBmpMsg();

    // -----------------------------------------------------------------------
    // Serial check (non-balloon mode only)
    //
    // Balloon mode overflows the serial output buffer silently, which is fine.
    // In bench/ground mode, missing Serial is a fatal error — blink and reboot.
    // -----------------------------------------------------------------------
    if (!BALLOON_MODE && !Serial) {
        setStatusLEDBlinkCount(LED_STATUS_REBOOT_NO_SERIAL);
        Watchdog.enable(10000);  // reboot after 10 seconds
        while (true) blockingLongBlinkLED(5);  // 1 sec on / 1 sec off until reboot
    }

    // -----------------------------------------------------------------------
    // RF frequency and clock configuration
    // -----------------------------------------------------------------------
    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    Watchdog.reset();

    // Derive TX frequency, minute, lane, and ID from the configured channel.
    // FIX! May be redundant at this point — consider removing.
    init_rf_freq(&XMIT_FREQUENCY, cc._Band, cc._lane);

    // Set PLL target frequency for the configured band.
    init_PLL_freq_target(&PLL_FREQ_TARGET, cc._Band);
    Watchdog.reset();

    // Log flash read results from core0 (no re-read needed here;
    // keeping flash access on core1 avoids "safe execute" concerns).
    V0_printf("prior read_FLASH() results: read_FLASH_result1: %d read_FLASH_result2: %d" EOL,
        read_FLASH_result1, read_FLASH_result2);
    show_values();

    // Set system clock. May be temporarily overridden to 18 MHz (or 12Mhz?) by
    // GpsINIT() (Kazu slow-clock GPS cold reset); this restores it.
    // FIX! shouldn't be needed now? the gps code should do the right thing?
    initPicoClock(PLL_SYS_MHZ);

    // Apply TCXO frequency correction once at boot.
    // Config changes always reboot, so correction never needs re-propagation.
    SI5351_TCXO_FREQ = doCorrection(SI5351_TCXO_FREQ);

    // -----------------------------------------------------------------------
    // PWM symbol-timing configuration
    //
    // INTERRUPTS_PER_SYMBOL scales with clock speed to keep interrupt rate
    // manageable. Tested values: 18 MHz→1, 125/133 MHz→8.
    // FIX! Not all frequency/interrupt combinations have been fully tested.
    // -----------------------------------------------------------------------
    // FIX! could we have have 12Mhz here? not unless we change initPicoClock() above
    if      (PLL_SYS_MHZ <=  18) INTERRUPTS_PER_SYMBOL = 1;
    else if (PLL_SYS_MHZ <=  33) INTERRUPTS_PER_SYMBOL = 2;
    else if (PLL_SYS_MHZ <=  66) INTERRUPTS_PER_SYMBOL = 4;
    else if (PLL_SYS_MHZ <= 133) INTERRUPTS_PER_SYMBOL = 8;
    else                         INTERRUPTS_PER_SYMBOL = 16;

    calcPwmDivAndWrap(&PWM_DIV, &PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL, PLL_SYS_MHZ);
    V1_printf("calcPwmDivAndWrap(): PLL_SYS_MHZ %lu PWM_DIV %lu PWM_WRAP_CNT %lu"
              " INTERRUPTS_PER_SYMBOL %lu" EOL,
              PLL_SYS_MHZ, PWM_DIV, PWM_WRAP_CNT, INTERRUPTS_PER_SYMBOL);

    // -----------------------------------------------------------------------
    // PLL denominator and symbol frequency optimisation
    //
    // Disabled sweep/search blocks below are retained for occasional manual
    // testing. The active path restores known-good values then optimises the
    // 4 channel symbols.
    //
    // FIX! si5351a_calc_sweep / _band / denom_optimize_search: verify whether
    //      vfo_calc_cache_flush() is needed before and/or after each call.
    // FIX! Should we pre-calculate all 4 symbol frequencies here?
    // -----------------------------------------------------------------------

    // Disabled: manual sweep diagnostics (uncomment to run)
    if (false && VERBY[1]) {
        set_PLL_DENOM_OPTIMIZE(cc._Band);
        vfo_calc_cache_flush();
        si5351a_calc_sweep();
        vfo_calc_cache_flush();
    }
    if (false && VERBY[1]) {
        vfo_calc_cache_flush();
        si5351a_calc_sweep_band();
        vfo_calc_cache_flush();
    }
    if (false && VERBY[1]) {
        vfo_calc_cache_flush();
        si5351a_denom_optimize_search();
        vfo_calc_cache_flush();
    }

    // Restore known-good per-band PLL values, then optimise for the 4 TX symbols
    set_PLL_DENOM_OPTIMIZE(cc._Band);
    init_PLL_freq_target(&PLL_FREQ_TARGET, cc._Band);

    double   symbolShiftError;
    double   symbolAbsoluteError;
    uint32_t pll_num;
    uint32_t pll_denom;
    si5351a_calc_optimize(&symbolShiftError, &symbolAbsoluteError, &pll_num, &pll_denom, true);

    if (USE_FAREY_WITH_PLL_REMAINDER) {
        V1_print(F("Using Farey optimal calculated num and denom:"));
    } else {
        V1_print(F("Using Numerator-Shift with either hard-wired or calculated denom:"));
    }
    V1_printf(" pll_denom %lu pll_num %lu", pll_denom, pll_num);
    V1_printf(" symbolAbsoluteError %.8f symbolShiftError %.8f" EOL,
              symbolAbsoluteError, symbolShiftError);

    // Report symbol shift error precision (target: better than 1e-4 Hz)
    int sse_1e6 = roundf(1e6f * symbolShiftError);
    int sse_1e5 = roundf(1e5f * symbolShiftError);
    int sse_1e4 = roundf(1e4f * symbolShiftError);
    if (sse_1e6 == 0) {
        V1_print(F("GOOD: symbolShiftError is better than 1e-6 Hz precision." EOL));
    } else if (sse_1e5 == 0) {
        V1_print(F("GOOD: symbolShiftError is better than 1e-5 Hz precision." EOL));
    } else if (sse_1e4 == 0) {
        V1_print(F("GOOD: symbolShiftError is better than 1e-4 Hz precision." EOL));
    } else {
        V1_print(F("ERROR: symbolShiftError is worse than 1e-4 Hz precision."));
        V1_printf(" Fix by changing PLL_FREQ_TARGET? %" PRIu64 EOL, PLL_FREQ_TARGET);
    }

    // -----------------------------------------------------------------------
    // setup1() complete
    // -----------------------------------------------------------------------
    V1_println(F(EOL "setup1() END"));

    Watchdog.reset();
    blockingLongBlinkLED(3);  // 2 sec on / 2 sec off: visual "setup1 done" indicator
    initStatusLED();
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

    if (VERBY[1]) {
        V1_print(F("setup1() freeMem()" EOL));
        freeMem();
    }
}

//*************************************************************************
uint32_t GpsTimeToLastFix = 0;  // milliseconds
uint32_t GpsTimeToLastFixMin = 999999;  // milliseconds
uint32_t GpsTimeToLastFixMax = 0;  // milliseconds
uint32_t GpsTimeToLastFixAvg = 0;  // milliseconds
// Sum and Cnt are for computing the incremntal Avg
uint64_t GpsTimeToLastFixSum = 0;
uint64_t GpsTimeToLastFixCnt = 0;

// FIX! right now, where are they set?
int ExtTelemetry1_val1 = 0;
int ExtTelemetry1_val2 = 0;
int ExtTelemetry2_val1 = 0;
int ExtTelemetry2_val2 = 0;

int tx_cnt_0 = 0;
int tx_cnt_1 = 0;
int tx_cnt_2 = 0;
int tx_cnt_3 = 0;
int tx_cnt_4 = 0;

//*************************************************************************
// GPS NMEA bursts: The thinking behind how we deal with it
// LightAPRS only looked for GPS NMEA data when it needed a fix.
// Not all the time.

// TinyGPS++ is not running as a separate task.
// Only does work when we call it.
// we call it with every new char, and to get data it creates from
// a history of NMEA sentences in those chars.

// NMEA sentence come in bursts at 1Hz ..
// And the pi pico only has 32 byte receive buffer on the UART
// that talks to ATGM336H-51

// Maybe not aligned to a second, but the burst is less than
// one full second of data. And burst intervals are at 1 sec.
// Data is not spread out over the full second.

// Order of each NMEA sentence is not random either.
// Order stays the same for each burst.
// Just a interesting note

// we can handle each char at about 300 usec avg.
// Obviously a char processing time is more when TinyGPS++ sees a char
// is a "end of NMEA sentence"
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

// Don't have to locally buffer chars to allow for backpressure from TinyGPS++
// Could create a local fifo,
// to effectively absorb more than the 32 deep uart rx fifo)
// If we created a secondary RX buffer to hold 600 plus chars
// (the entire burst of multiple NMEA sentences, per second,
// for the default enabled sentences (US and Baidu satellites)
// Then if we could absorb/empty it at least every second,
// we'd never lose anything.

// Less processing/power if we only absorb GPS data when we need it.
// Allows us to sleep when  we don't need a fix update.

// Key that TinyGPS++ absorbs our char send at bounded delays
// (per NMEA sentence).
// end of NMEA sentence has longer delay.
// Apparently the CR LF is needed as a 'boundary' post checksum?
// FIX! do we need both CR and LF or is one enough? Shouldn't matter.

//*********************************************************************************
// Now about sleepSeconds() for next beacon (HF or VHF).
// https://arduiniana.org/libraries/tinygps/

// If the $GPRMC sentence reports a validity of “V” (void)
// instead of “A” (active),
// or if the $GPGGA sentence reports fix type “0” (no fix)
// then those sentences are discarded.

// TinyGPS::GPS_INVALID_AGE is the value when you never got a valid fix.

// seconds sleep if super capacitors/batteries are below BattMin
uint16_t  BATT_WAIT = 1;  // secs

// GPS_LOCATION_AGE_MAX should a bit greater than GPS_WAIT_FOR_NMEA_BURST_MAX
// We could live with data that is more 'stale'
// but theoretically no older than this?
// how stale can it get? we might be waiting for a new hot fix?
// and the old one is in there and usable

// maybe make an old fix good for up to 5 secs?
// if we did a quick gps power off/on we won't get new
// one for maybe 5 secs.
// but maybe that says we don't want anything older than 1-2 secs?
// but who knows how tinyGPS++ creates the .age ??
// maybe it needs to be hot fix max time? (allow 5 secs?)
// worse would be if it only updates
// age to 0 when the new fix changes any of location/altitude etc?

// at 180mph we could move 3 horizontal miles in 1 minute?
// hmm. what about a descending balloon?
// fixes could be 1 minute old? that's kind of like a cold fix time

// Definitely do this. having a 70 sec old fix allowed, is good for lower power
// but then we could take a long time to get a cold fix going? doing force cold reset
// hot fix should be fast in GpsON()

// FIX! if we have a good fix, and good age, should we turn gps off
// and only turn it on again when the age is bad?
// was getting fix_age violations around 299824 millisecs max with 700000 here
// 5 minutes? could be due to the go_when_rdy testing...
// back to back, no room for gps

// const uint32_t GPS_LOCATION_AGE_MAX = 70000;
// lets go 5 minutes! can travel 10 miles in that time though? 2 subsquares?
// we'll be getting long back to back when we do "Extended Telemetry".
// so maybe big max age is needed?
extern const uint32_t GPS_LOCATION_AGE_MAX = 30000;

// FIX! since we break out of the sleepSeconds when gps data starts
// with Serial2.available(), we could make this bigger?
// needs to be at least 1 sec (a little more) since it
// wants to grab a full burst, and we don't know where we are in the repeating
// burst behavior when we start (idle or in the middle of a burst?)
// with all the constellations on SIM65M, seems like we should wait longer to get
// full set of NMEA sentences for a burst (plus the burst is only every 5 secs no)

// this is adjusted in gps_functions.cpp, depending on broadcast interval
uint32_t GPS_WAIT_FOR_NMEA_BURST_MAX = 1200;

//*************************************************************************
void loop1() {
    // -----------------------------------------------------------------------
    // GPS invalidation countdown
    // After turning GPS off then on, ignore TinyGPS++ state for a few iterations
    // to allow it to transition cleanly on fresh NMEA sentences (within ~2 secs).
    // -----------------------------------------------------------------------
    if (GpsIsOn() && GpsInvalidAllCnt > 0) GpsInvalidAllCnt--;
    GpsInvalidAll = (GpsInvalidAllCnt > 0);

    loopCnt++;
    Watchdog.reset();
    V1_printf(EOL "loopCnt %" PRIu64 " loop1() START" EOL, loopCnt);

    if (loop_us_start == 0) loop_us_start = get_absolute_time();
    updateStatusLED();

    // Always turn off TX before GPS work (double-check against bugs).
    vfo_turn_off();
    GpsON(false);  // no full cold reset

    // -----------------------------------------------------------------------
    // Bail early if battery/solar voltage is too low
    // -----------------------------------------------------------------------
    float solarVoltage = readVoltage();
    if (solarVoltage <= BattMin || solarVoltage <= GpsMinVolt) {
        sleepSeconds(BATT_WAIT);
        return;
    }
    V1_printf("loop1() good solarVoltage %.f" EOL, solarVoltage);

    // -----------------------------------------------------------------------
    // Pull a burst of NMEA data and update GPS state.
    // FIX! unload for 2 secs to capture 1-sec broadcast intervals.
    // NMEA checksums handle buffer overrun corruption.
    // FIX! should change baud back to 9600 (lower power?)
    //   only getting ~1800 baud during 300-400ms window
    // -----------------------------------------------------------------------
    updateGpsDataAndTime(GPS_WAIT_FOR_NMEA_BURST_MAX);
    gpsDebug();
    tinyGpsCustom();

    // -----------------------------------------------------------------------
    // Snapshot GPS fix status
    // -----------------------------------------------------------------------
    uint32_t fix_age     = gps.location.age();
    uint32_t fix_sat_cnt = gps.satellites.value();
    int      fix_altitude = (int)gps.altitude.meters();  // double in TinyGPS++

    // fix_mode / fix_qual are single chars:
    //   fix_qual: Invalid DGPS PPS RTK FloatRTK Estimated Manual Simulated
    //   fix_mode: N A D E
    char fix_mode = gps.fix.FixMode();
    char fix_qual = gps.location.FixQuality();

    // isUpdated() = value refreshed since last query (not necessarily changed)
    bool fix_updated = gps.location.isUpdated();
    gps.location.updated = false;

    // fix_valid_2d: bare 2D location check (subset of fix_valid_all)
    bool fix_valid_2d = gps.location.isValid() && !GpsInvalidAll;

    // How long since we last synced system time?
    uint32_t elapsed_setTime_secs = (millis() - setTime_millis) / 1000;
    if (elapsed_setTime_secs > (10 * 60)) {
        V1_printf(
            "WARNING: Should set time every interval? elapsed_setTime_secs %lu > (10 * 60)" EOL,
            elapsed_setTime_secs);
    }

    // -----------------------------------------------------------------------
    // Full validity check for a 3D fix.
    // Notes:
    //   - fix_age is NOT included here; checked separately below.
    //   - GPS chips commonly emit bogus dates (2080-01-01, 2000-00-00) when
    //     not yet locked; year range 2025-2035 guards against that.
    //   - Requiring sat count >= 3 prevents false fixes with high HDOP/0 sats.
    //   - FIX! 3/22/26: consider using FixQuality/FixMode enums directly.
    // -----------------------------------------------------------------------
    bool date_year_sane = (gps.date.year() >= 2025 && gps.date.year() <= 2035);

    bool fix_valid_all =
        !GpsInvalidAll                                    &&
        (setTime_millis != 0)                             &&  // system time synced
        (elapsed_setTime_secs < (30 * 60))                &&  // system time fresh (<30 min)
        gps.date.isValid()                                &&
        gps.time.isValid()                                &&
        date_year_sane                                    &&
        gps.satellites.isValid() && (fix_sat_cnt >= 3)    &&
        gps.hdop.isValid()                                &&
        gps.altitude.isValid()                            &&
        gps.location.isValid()                            &&
        gps.speed.isValid()                               &&
        gps.course.isValid();

    // -----------------------------------------------------------------------
    // GPS watchdog: reboot GPS if we go too many iterations without a fix.
    // FIX! loop iterations vary in length (could be 5–20+ mins each),
    //      so GpsWatchdogCnt > 15 may represent a large real-time span.
    // -----------------------------------------------------------------------
    if (fix_valid_all) {
        GpsWatchdogCnt = 0;  // cleared only on confirmed 3D fix
        setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
    } else {
        GpsWatchdogCnt++;
        V1_printf("loop1() GpsWatchdogCnt++ %lu" EOL, GpsWatchdogCnt);

        // Show time-only status if date looks valid, else show no-GPS
        if (date_year_sane)
            setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
        else
            setStatusLEDBlinkCount(LED_STATUS_NO_GPS);

        if (GpsWatchdogCnt > 15) {
            // Seen in the wild: TinyGPS++ reports "valid" but altitude wrong,
            // HDOP huge, sat count 0. A cold reset clears stale ephemeris.
            // (VBAT is always present, so without a cold reset this would be
            //  a warm/hot reset only.)
            V1_println(F("ERROR: loop1() GpsWatchdogCnt > 15 ..gps full cold reset"));
            GpsOFF();
            Watchdog.reset();
            sleep_ms(1000);
            GpsON(true);   // cold reset
            GpsWatchdogCnt = 0;
            setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
            return;
        }
    }

    updateStatusLED();

    // -----------------------------------------------------------------------
    // Fix summary log
    // fix_age == 4294967295 when location is not valid.
    // See: https://sites.google.com/site/wayneholder/self-driving-rc-car/getting-the-most-from-gps
    // Grep "fix_used" to trace which fixes were actually consumed.
    // -----------------------------------------------------------------------
    bool fix_used = fix_valid_all && (fix_age <= GPS_LOCATION_AGE_MAX);

    V1_print(F(EOL));
    V1_printf("fix_used %u"            EOL, fix_used);
    V1_printf("fix_valid_all %u"       EOL, fix_valid_all);
    V1_printf("fix_valid_2d %u"        EOL, fix_valid_2d);
    V1_printf("fix_updated %u"         EOL, fix_updated);
    V1_printf("fix_age %lu millisecs"  EOL, fix_age);
    V1_printf("fix_sat_cnt %lu"        EOL, fix_sat_cnt);
    V1_printf("fix_altitude %d"        EOL, fix_altitude);
    V1_printf("fix_mode %c"            EOL, fix_mode);
    V1_printf("fix_qual %c"            EOL, fix_qual);
    V1_printf("elapsed_setTime_secs %lu" EOL, elapsed_setTime_secs);
    V1_print(F(EOL));

    Watchdog.reset();

    // -----------------------------------------------------------------------
    // Branch on fix quality and decide whether to transmit WSPR
    // -----------------------------------------------------------------------

    // Helper: sleep 20 seconds into the next minute (used in several cases)
    // FIX! SMART_WAIT is not so good for 10-minute TX intervals; fine for
    //      test mode (cc._go_when_ready) which disables the channel-minute check.
    #define SLEEP_INTO_NEXT_MINUTE(caseNum) \
        do { \
            int SMART_WAIT = (60 - second() + 20); \
            V1_printf("loopCnt %" PRIu64 " case %d: sleepSeconds() 20 secs into next minute" \
                      " SMART_WAIT %d" EOL, loopCnt, caseNum, SMART_WAIT); \
            sleepSeconds(SMART_WAIT); \
        } while (0)

    if (!fix_used) {
        // ---- Case 4 / 5: no usable fix ----
        if (fix_valid_all) {
            V1_printf("loopCnt %" PRIu64
                      " WARN: GPS issue: valid but fix_age %lu millisecs" EOL,
                      loopCnt, fix_age);
        } else {
            V1_printf("loopCnt %" PRIu64 " WARN: invalid GPS fix." EOL, loopCnt);
        }
        // Turn off RF (reduces noise for GPS), flush stale TinyGPS++ state, keep GPS on.
        vfo_turn_off();
        invalidateTinyGpsState();
        GpsON(false);
        SLEEP_INTO_NEXT_MINUTE(4);

    } else if (fix_sat_cnt <= 3) {
        // ---- Case 5: 2D fix only (too few satellites) ----
        V1_printf("loopCnt %" PRIu64
                  " WARN: GPS fix issue: only %lu sats ..2d fix only" EOL,
                  loopCnt, fix_sat_cnt);
        vfo_turn_off();
        invalidateTinyGpsState();
        GpsON(false);
        SLEEP_INTO_NEXT_MINUTE(5);

    } else {
        // ---- Good 3D fix: proceed toward WSPR TX ----
        V1_printf("loopCnt %" PRIu64 " Good recent 3d fix" EOL, loopCnt);

        // Record time-to-first-fix stats (set once per GPS on→off→on cycle)
        if (GpsTimeToLastFix == 0) {
            GpsTimeToLastFix =
                absolute_time_diff_us(GpsStartTime, get_absolute_time()) / 1000ULL;

            V1_printf("loopCnt %" PRIu64
                      " first Gps Fix after off->on! GpsTimeToLastFix %lu ms" EOL,
                      loopCnt, GpsTimeToLastFix);

            if (GpsTimeToLastFix < GpsTimeToLastFixMin) {
                GpsTimeToLastFixMin = GpsTimeToLastFix;
                V1_printf("loopCnt %" PRIu64 " New GpsTimeToLastFixMin %lu ms" EOL,
                          loopCnt, GpsTimeToLastFixMin);
            }
            if (GpsTimeToLastFix > GpsTimeToLastFixMax) {
                GpsTimeToLastFixMax = GpsTimeToLastFix;
                V1_printf("loopCnt %" PRIu64 " New GpsTimeToLastFixMax %lu ms" EOL,
                          loopCnt, GpsTimeToLastFixMax);
            }

            GpsTimeToLastFixSum += (uint64_t)GpsTimeToLastFix;
            GpsTimeToLastFixCnt += 1;
            GpsTimeToLastFixAvg = (uint32_t)(GpsTimeToLastFixSum / GpsTimeToLastFixCnt);
            V1_printf("loopCnt %" PRIu64 " New GpsTimeToLastFixAvg %lu ms" EOL,
                      loopCnt, GpsTimeToLastFixAvg);
        }

        // Snapshot telemetry (all tt.* strings) as close to the valid-fix checks as possible.
        // Voltage is captured at this point, before GPS is turned off.
        snapForTelemetry();

        V1_printf("loopCnt %" PRIu64
                  " After snapTelemetry() timeStatus(): %u minute: %u second: %u" EOL,
                  loopCnt, timeStatus(), minute(), second());

        Watchdog.reset();

        // ---- Voltage check before transmitting ----
        float voltageBeforeWSPR = readVoltage();
        if (voltageBeforeWSPR < WsprBattMin) {
            V1_printf(EOL "loopCnt %" PRIu64
                      " WARN: no send: voltageBeforeWSPR %.f WsprBattMin %.f" EOL,
                      loopCnt, voltageBeforeWSPR, WsprBattMin);
            SLEEP_INTO_NEXT_MINUTE(3);  // case 3: low battery wait

        } else {
            // ---- Minute alignment check ----
            // FIX! HF prep starts one minute before TX minute.
            // TX minute is determined by channel starting minute - 1 (mod 10).
            // e.g., at minutes 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53, 57.
            // alignMinute(-1) checks we are in the minute preceding TX.
            bool in_pre_tx_minute = alignMinute(-1);

            if (!in_pre_tx_minute) {
                V1_printf(EOL "loopCnt %" PRIu64
                          " OKAY: wspr no send."
                          " minute() %d second: %d alignMinute(-1) %u" EOL,
                          loopCnt, minute(), second(), in_pre_tx_minute);
                // Sleep instead of busy-waiting to avoid spinning on alignment.
                SLEEP_INTO_NEXT_MINUTE(0);

            } else if (second() > 40) {
                // Too late in the pre-TX minute; skip this cycle.
                V1_printf(EOL "loopCnt %" PRIu64
                          " WARN: wspr no send, past 40 secs in pre-minute:"
                          " minute: %d second: %d alignMinute(-1) %u" EOL,
                          loopCnt, minute(), second(), in_pre_tx_minute);
                SLEEP_INTO_NEXT_MINUTE(1);

            } else {
                // ---- Wait until 40 seconds into the pre-TX minute, then warm up VFO ----
                V1_printf("loopCnt %" PRIu64
                          " wspr: waiting until second 40; now: minute: %d second: %d" EOL,
                          loopCnt, minute(), second());

                while (second() < 40) {
                    Watchdog.reset();
                    delay(10);
                    updateStatusLED();
                }

                V1_printf("loopCnt %" PRIu64
                          " wspr: 20 secs until TX minute. Turning on VFO for warmup."
                          " minute: %d second: %d" EOL,
                          loopCnt, minute(), second());

                uint32_t hf_freq = XMIT_FREQUENCY;
                int txResult = alignAndDoAllSequentialTx(hf_freq);

                if (txResult == -1) {
                    // TX did not complete cleanly; sleep to next minute.
                    SLEEP_INTO_NEXT_MINUTE(2);
                }
                // txResult == 0: success, fall through to bottom of loop1()
            }
        }
    }

    #undef SLEEP_INTO_NEXT_MINUTE

    // -----------------------------------------------------------------------
    // VFO calibration cache sanity check
    // Expected entries after first TX:
    //   4  if WSPR only
    //   5  if WSPR + CW (cc._morse_also[0] == '1')
    // -----------------------------------------------------------------------
    uint8_t VCC_valid_cnt = vfo_calc_cache_print_and_check();
    if (tx_cnt_0 > 0) {
        bool morse_also = (cc._morse_also[0] == '1');
        uint8_t expected = morse_also ? 5 : 4;
        const char* severity = morse_also ? "WARN" : "ERROR";
        if (VCC_valid_cnt != expected) {
            V1_printf("%s: loop1() VCC_valid_cnt %u != %u" EOL,
                      severity, VCC_valid_cnt, expected);
        }
    }

    // -----------------------------------------------------------------------
    // Loop timing
    // -----------------------------------------------------------------------
    absolute_time_t loop_us_end   = get_absolute_time();
    int64_t loop_us_elapsed       = absolute_time_diff_us(loop_us_start, loop_us_end);
    int64_t loop_ms_elapsed       = loop_us_elapsed / 1000ULL;

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
        "t_fixMode: %s "
        "t_fixQual: %s "
        "GpsTimeToLastFix %lu "
        "GpsWatchdogCnt %lu" EOL,
        loopCnt,
        tt.tx_count_0, tt.callsign, tt.temp, tt.voltage,
        tt.altitude, tt.grid6, tt.power, tt.sat_count,
        tt.fixMode, tt.fixQual,
        GpsTimeToLastFix, GpsWatchdogCnt);

    V1_println(F(EOL));
    V1_printf(
        "loopCnt %" PRIu64 " "
        "t_tx_count_0: %s "
        "_Band %s "
        "loop_ms_elapsed: %" PRId64 " millisecs "
        "loop_us_start: %llu microsecs "
        "loop_us_end: %llu microsecs" EOL,
        loopCnt, tt.tx_count_0, cc._Band,
        loop_ms_elapsed, loop_us_start, loop_us_end);

    updateStatusLED();

    loop_us_start = loop_us_end;  // next loop's start = this loop's end

    if (VERBY[1]) {
        V1_print(F("loop1() freeMem()" EOL));
        freeMem();
    }

    V1_printf("loopCnt %" PRIu64 " loop1() END" EOL, loopCnt);
}
//*******************************************************

// =============================================================================
// alignAndDoAllSequentialTx
// -----------------------------------------------------------------------------
// Aligns to a WSPR transmit window and sends a sequence of transmissions:
//   txNum 0: WSPR callsign
//   txNum 1: WSPR telemetry (u4b standard)
//   txNum 2: WSPR telen1   (sent only if slot 3 or slot 4 is enabled)
//   txNum 3: WSPR telen2   (sent only if telen2 is enabled)
//   txNum 4: CW message    (sent only if cc._morse_also[0] == '1')
// =============================================================================

// -----------------------------------------------------------------------------
// Helpers (kept as static inlines so the main flow reads top-to-bottom)
// -----------------------------------------------------------------------------

// Wait (with watchdog kicks) until we're past second 40 of the current minute.
// We could end up waiting for up to ~40 secs.
static void waitUntilPastSecond40(void) {
    while (second() < 40) {
        Watchdog.reset();
        sleep_ms(50);
        updateStatusLED();
    }
}

// Turn off the WSPR clock output(s) before we actually need to drive RF.
// new 1/9/25: we used to have the clocks on here.
static void disableWsprClocksIfConfigured(void) {
    if (!DO_CLK_OFF_FOR_WSPR_MODE) return;

    if (USE_SI5351A_CLK_POWERDOWN_FOR_WSPR_MODE) {
        // FIX! does this work on ms5351m. do we care?
        // print. this does a pllb reset too
        si5351a_power_down_clk01(true);
    } else {
        // no print. clk0/1 both affected
        vfo_turn_off_clk_out(WSPR_TX_CLK_0_NUM, true);
    }
}

// Quickly cycle through all 4 WSPR symbols so the Farey-algo cache is primed.
// This avoids per-symbol latency the first time each symbol is used.
// End with symbol 0 so the VFO is left on the right symbol.
// Returns true on success, false if any startSymbolFreq() returned non-zero.
static bool primeSymbolFreqCache(uint32_t hf_freq) {
    uint8_t rc3 = startSymbolFreq(hf_freq, 3, false, true);
    uint8_t rc2 = startSymbolFreq(hf_freq, 2, false, true);
    uint8_t rc1 = startSymbolFreq(hf_freq, 1, false, true);
    uint8_t rc0 = startSymbolFreq(hf_freq, 0, false, false);

    if (rc3 == 0 && rc2 == 0 && rc1 == 0 && rc0 == 0) return true;

    V1_print(F(EOL "ERROR: fatal. bad retcode(s) from startSymbolFREQ" EOL));
    V1_printf("retcode 0: %u 1: %u 2: %u 3: %u" EOL, rc0, rc1, rc2, rc3);
    V1_printf("Need to adjust PLL_FREQ_TARGET" EOL);
    return false;
}

// Build the left-justified callsign, 4-char grid, and power strings used
// by JTEncode and friends.
//
// BUG: reported by Rob KC3LBR
// JTEncode walks thru chars 0-12, so we really need a 14 char array with
// null term for hf_callsign.
static void buildCallsignGridPower(char *hf_callsign, size_t cs_size,
                                   char *hf_grid4,
                                   char *hf_power, size_t pwr_size) {
    // left-justify (pad with spaces) for what JTEncode expects
    snprintf(hf_callsign, cs_size, "%-13s", tt.callsign);

    double lat_double = atof(tt.lat);
    double lon_double = atof(tt.lon);

    // get_mh_6 uses the first arg as pointer to a char array for the return data
    char hf_grid6[7] = { 0 };
    get_mh_6(hf_grid6, lat_double, lon_double);
    // just the first 4 chars
    for (int i = 0; i < 4; i++) hf_grid4[i] = hf_grid6[i];
    hf_grid4[4] = 0;

    snprintf(hf_power, pwr_size, "%s", tt.power);
}

// Print the "WSPR txNum N Prepared..." block we emit before each tx.
static void printPreparedBlock(int txNum,
                               const char *hf_callsign,
                               const char *hf_grid4,
                               const char *hf_power) {
    V1_print(F(EOL));
    V1_printf("WSPR txNum %d Prepared.." EOL, txNum);
    V1_printf("hf_callsign %-6s" EOL, hf_callsign);
    V1_printf("hf_grid4 %s" EOL, hf_grid4);
    V1_printf("hf_power %s" EOL, hf_power);
    V1_print(F(EOL));
    V1_flush();
}

// Encode a telen slot using either the GPS or BMP codec, based on the
// ExtTelemetry config char ('0' = GPS, '1' = BMP).
// `defaultIsBmp` selects the codec used when the char is unrecognized,
// matching the original switch defaults for slot 2 (GPS) vs slot 3 (BMP).
static void encodeTelenSlot(int txNum, uint8_t slot, char cfg, bool defaultIsBmp,
                            char *hf_callsign, char *hf_grid4, char *hf_power) {
    bool useBmp;
    switch (cfg) {
        case '0': useBmp = false; break;
        case '1': useBmp = true;  break;
        default:  useBmp = defaultIsBmp; break;
    }

    if (useBmp) {
        V1_printf("WSPR txNum %d Preparing with encode_codecBmpMsg() slot %u" EOL,
                  txNum, slot);
        encode_codecBmpMsg(hf_callsign, hf_grid4, hf_power, slot);
    } else {
        V1_printf("WSPR txNum %d Preparing with encode_codecGpsMsg() slot %u" EOL,
                  txNum, slot);
        encode_codecGpsMsg(hf_callsign, hf_grid4, hf_power, slot);
    }
}

// Log "<label> Tx sent. ..." plus the current minute/second, if VERBY[1].
static void logTxSent(const char *label) {
    if (!VERBY[1]) return;
    StampPrintf("%s Tx sent. minute: %d second: %d" EOL,
                label, minute(), second());
    DoLogPrint();
}


// -----------------------------------------------------------------------------
// Main entry point
// -----------------------------------------------------------------------------
int alignAndDoAllSequentialTx(uint32_t hf_freq) {
    V1_print(F(EOL "alignAndDoAllSequentialTX START"));
    V1_printf(" now: minute: %d second: %d" EOL, minute(), second());

    // If we were called too early, bail out so we don't sit here for ~10 minutes.
    // It should loop around in loop1() ..after how much of a wait? smartWait?
    // (calculated delay above?)
    // At most we wait up to a minute if called the minute before start time.
    if (!alignMinute(-1)) {
        V1_print(F("FAIL: alignAndDoAllSequentialTX END early out: alignment wrong!"));
        V1_printf(" now: minute: %d second: %d" EOL, minute(), second());
        return -1;
    }

    waitUntilPastSecond40();

    // Don't want GPS power and TX power active at the same time.
    GpsOFF();

    // Start the VFO ~20 seconds before it's needed.
    // If it was off beforehand it has no clocks running.
    // We could turn it off then on to guarantee always starting from reset state.
    vfo_turn_on();
    sleep_ms(1000);

    disableWsprClocksIfConfigured();

    // -------------------------------------------------------------------------
    // Prime the symbol frequency cache (timed for diagnostics).
    // -------------------------------------------------------------------------
    absolute_time_t start_usecs_1 = get_absolute_time();

    // Will print programming if false, since we have time.
    // How many entries are in the cache to start?
    // (was getting 14MDA printed if I cycled these with RF?
    //  these are just filling the cache.)
    uint8_t VCC_init_valid_cnt = vfo_calc_cache_print_and_check();

    primeSymbolFreqCache(hf_freq);  // logs its own error on failure

    absolute_time_t end_usecs_1 = get_absolute_time();
    int64_t elapsed_usecs_1 = absolute_time_diff_us(start_usecs_1, end_usecs_1);
    float elapsed_millisecs_1 = (float)elapsed_usecs_1 / 1000.0;
    V1_printf("VCC cache initially had %u valid entries\n" EOL, VCC_init_valid_cnt);
    V1_print(F("Time to calc/lookup, 4 symbol freq si5351 reg values:"));
    V1_printf(" %.4f millisecs" EOL, elapsed_millisecs_1);

    // -------------------------------------------------------------------------
    // Build callsign / grid / power strings used by all the WSPR txs below.
    // -------------------------------------------------------------------------
    setStatusLEDBlinkCount(LED_STATUS_TX_WSPR);

    char hf_callsign[14] = { 0 };
    char hf_grid4[5]    = "AA00";
    char hf_power[3]    = { 0 };
    buildCallsignGridPower(hf_callsign, sizeof(hf_callsign),
                           hf_grid4,
                           hf_power, sizeof(hf_power));
    Watchdog.reset();

    // -------------------------------------------------------------------------
    // txNum 0: callsign
    // syncAndSendWspr will sync up to the right minute and second == 0.
    // -------------------------------------------------------------------------
    int txNum = 0;
    printPreparedBlock(txNum, hf_callsign, hf_grid4, hf_power);

    // init to all zeroes just so we know what the encode is doing,
    // if we get bad symbols when we send the symbols
    bool vfoOffAtEnd = false;
    syncAndSendWspr(hf_freq, txNum, hf_tx_buffer,
                    hf_callsign, hf_grid4, hf_power, vfoOffAtEnd);
    tx_cnt_0 += 1;
    // we have 10 secs or so at the end of WSPR to get this off?
    logTxSent("WSPR callsign");

    // We don't loop around again caring about gps fix, because we've saved
    // telemetry (and sensor data in there) right before the callsign tx.

    // -------------------------------------------------------------------------
    // txNum 1: standard u4b telemetry
    // -------------------------------------------------------------------------
    setStatusLEDBlinkCount(LED_STATUS_TX_TELEMETRY);

    txNum = 1;
    V1_printf("WSPR txNum %d Preparing with u4b_encode_std().." EOL, txNum);
    V1_flush();

    // output: modifies globals: hf_callsign, hf_grid4, hf_power
    u4b_encode_std(hf_callsign, hf_grid4, hf_power,
                   tt.grid6, tt.altitude, tt.temp, tt.voltage, tt.speed, cc._id13);
    printPreparedBlock(txNum, hf_callsign, hf_grid4, hf_power);

    vfoOffAtEnd = (cc._ExtTelemetry[0] == '-' && cc._ExtTelemetry[1] == '-');
    syncAndSendWspr(hf_freq, txNum, hf_tx_buffer,
                    hf_callsign, hf_grid4, hf_power, vfoOffAtEnd);
    tx_cnt_1 += 1;
    // we have 10 secs or so at the end of WSPR to get this off?
    logTxSent("WSPR telemetry");

    // -------------------------------------------------------------------------
    // txNum 2: telen1 -- only if slot 3 or slot 4 is enabled
    // -------------------------------------------------------------------------
    if (cc._ExtTelemetry[0] != '-' || cc._ExtTelemetry[1] != '-') {
        setStatusLEDBlinkCount(LED_STATUS_TX_TELEMETRY);

        txNum = 2;
        uint8_t slot = 2;
        // slot 2 default in the original switch was GPS (defaultIsBmp = false)
        encodeTelenSlot(txNum, slot, cc._ExtTelemetry[0], /*defaultIsBmp=*/false,
                        hf_callsign, hf_grid4, hf_power);
        printPreparedBlock(txNum, hf_callsign, hf_grid4, hf_power);

        vfoOffAtEnd = (cc._ExtTelemetry[1] == '-');
        syncAndSendWspr(hf_freq, txNum, hf_tx_buffer,
                        hf_callsign, hf_grid4, hf_power, vfoOffAtEnd);
        tx_cnt_2 += 1;
        logTxSent("WSPR telen1");
    }

    // -------------------------------------------------------------------------
    // txNum 3: telen2 -- only if telen2 is enabled
    // -------------------------------------------------------------------------
    if (cc._ExtTelemetry[1] != '-') {
        setStatusLEDBlinkCount(LED_STATUS_TX_TELEMETRY);

        txNum = 3;
        V1_printf("WSPR txNum %d Preparing." EOL, txNum);
        V1_flush();

        // output: modifies globals: hf_callsign, hf_grid4, hf_power
        // input: ExtTelemetry2_val1/2 are ints?
        // (legacy alternative kept for reference)
        // u4b_encode_telen(hf_callsign, hf_grid4, hf_power,
        //     ExtTelemetry1_val1, ExtTelemetry1_val2, false, cc._id13);

        uint8_t slot = 3;
        // slot 3 default in the original switch was BMP (defaultIsBmp = true)
        encodeTelenSlot(txNum, slot, cc._ExtTelemetry[1], /*defaultIsBmp=*/true,
                        hf_callsign, hf_grid4, hf_power);
        printPreparedBlock(txNum, hf_callsign, hf_grid4, hf_power);

        vfoOffAtEnd = true;
        syncAndSendWspr(hf_freq, txNum, hf_tx_buffer,
                        hf_callsign, hf_grid4, hf_power, vfoOffAtEnd);
        tx_cnt_3 += 1;
        // we have 10 secs or so at the end of WSPR to get this off?
        logTxSent("WSPR telen2");
    }

    // -------------------------------------------------------------------------
    // txNum 4: CW -- only if morse_also is enabled
    // -------------------------------------------------------------------------
    if (cc._morse_also[0] == '1') {
        setStatusLEDBlinkCount(LED_STATUS_TX_CW);

        txNum = 4;
        V1_printf("CW txNum %d using cw_send_message().." EOL, txNum);
        V1_flush();

        // Picks a good HF freq for the configured cc._Band.
        // Uses tt.callsign and tt.grid6 in the message.
        // NOTE: turns GPS back on at the end, so it's assuming it's last after wspr.
        cw_send_message();

        // Restore the wspr XMIT_FREQUENCY since cw changed it.
        // Sets minute/lane/id from chan number.
        // FIX! is it redundant at this point?..remove?
        init_rf_freq(&XMIT_FREQUENCY, cc._Band, cc._lane);
        tx_cnt_4 += 1;
        logTxSent("CW");
    }

    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------
    setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
    // Now: don't turn GPS back on until beginning of loop.
    // Reset the fix time variables also (on the off -> on transition).
    GpsTimeToLastFix = 0;

    V1_print(F("alignAndDoAllSequentialTX END" EOL));
    return 0;  // success
}

//***********************************************************
void sleepSeconds(int secs) {
    // No early exit — sleep duration is guaranteed >= secs.
    // updateGpsDataAndTime() may extend individual iterations slightly.
    V1_println(F("sleepSeconds START"));
    V1_flush();

    uint32_t start_millis    = millis();
    uint32_t duration_millis = (uint32_t)secs * 1000;  // secs fits in uint32_t

    float    solarVoltage    = 0.0f;
    uint32_t gdt_millis      = 0;
    uint32_t increasingWait  = 0;  // back-off added when GPS yields nothing

    // -----------------------------------------------------------------------
    // Sleep loop: keep GPS alive and serviced until duration has elapsed.
    // We are moving, so we want continuous GPS tracking after first fix.
    // -----------------------------------------------------------------------
    do {
        Watchdog.reset();

        GpsON(false);  // keep GPS on; no cold reset
        solarVoltage = readVoltage();

        // Turn GPS off if voltage is too low to sustain it.
        bool voltageLow = (solarVoltage < BattMin || solarVoltage < GpsMinVolt);
        if (voltageLow) {
            V1_printf("sleepSeconds() bad solarVoltage %.f ..(1) turn gps off" EOL,
                solarVoltage);
            GpsOFF();
        }

        // Always service the status LED in spin loops.
        updateStatusLED();

        if (GpsIsOn()) {
            // Drain a full NMEA broadcast burst (~1 sec interval).
            // Tested thresholds:
            //   1050 ms: caused RX buffer overrun (21–32 chars lost)
            //   1500 ms: reliable at 9600 baud and above
            //   may not be enough at 4800 baud
            // FIX! should we parse/use GPS data here, or just drain?
            //   GPS may be on or off across iterations, so deferred for now.
            gdt_millis = updateGpsDataAndTime(GPS_WAIT_FOR_NMEA_BURST_MAX);

            // If we got very little data, the GPS may not have a lock yet.
            // Add a randomised increasing back-off to avoid hammering it.
            // Back-off resets to zero once it reaches GPS_WAIT_FOR_NMEA_BURST_MAX.
            bool gpsBurstEmpty = (gdt_millis < 100);
            if (gpsBurstEmpty) {
                uint32_t jitter     = random(20, 25);
                increasingWait     += jitter;
                if (increasingWait > (uint32_t)GPS_WAIT_FOR_NMEA_BURST_MAX)
                    increasingWait = 0;
                sleep_ms(increasingWait);
            }
        } else {
            // GPS is off; just wait out the equivalent burst window.
            sleep_ms(GPS_WAIT_FOR_NMEA_BURST_MAX);
        }

    } while ((millis() - start_millis) < duration_millis);

    Watchdog.reset();

    // -----------------------------------------------------------------------
    // Post-sleep GPS state
    // GPS is left OFF if voltage was low at any point during the loop.
    // -----------------------------------------------------------------------
    bool voltageLowAtEnd = (solarVoltage < BattMin || solarVoltage < GpsMinVolt);
    if (voltageLowAtEnd) {
        V1_printf("sleepSeconds() bad solarVoltage %.f ..(2) turn gps off" EOL,
            solarVoltage);
        GpsOFF();
    } else {
        // Turn GPS back on if it went off mid-loop; don't preserve stale TinyGPS++ state.
        // FIX! worth considering: keep GPS off between cycles to save power?
        //   Pro: saves idle draw.
        //   Con: power spike on restart; should hot-fix within seconds anyway.
        //   Current policy: stay on except during RF TX.
        if (!GpsIsOn()) GpsON(false);  // no cold reset; don't restore TinyGPS++ state
    }

    V1_println(F("sleepSeconds END"));
}
// =============================================================================
// alignMinute
// -1 is returned if anything illegal
// FIX! should check what caller does if -1
// -----------------------------------------------------------------------------
// Returns true if the current wall-clock minute matches the WSPR/u4b
// transmit-window minute (channel config start minute, shifted by `offset`).
//
// `offset` selects which slot in the sequence we're aligning to:
//   -1            : prep window (telemetry/sensor capture, etc.)
//    0, 2, 4, 6   : the four 2-minute WSPR transmit slots
//    1, 3, 5      : (accepted but unused in normal flow -- see comment below)
//
// If the system clock isn't set, this returns true so the caller proceeds
// (FIX! see note below -- returning false might be safer).
// =============================================================================

// Channel config _start_minute must be one of these (as a numeric string):
// "0", "2", "4", "6", or "8".
static bool isValidStartMinute(int m) {
    return (m == 0 || m == 2 || m == 4 || m == 6 || m == 8);
}

bool alignMinute(int offset) {
    // FIX! need good time. Does returning false help (i.e. not do any TX)
    if (timeStatus() != timeSet) return true;

    // offset can be -1, 0, 1, 2, 3, 4, 5, 6 (3 messages)
    // if not one of those, set the start minute to -1?
    // caller should detect that and not wait?
    if (offset < -1 || offset > 6) {
        offset = 0;
    }

    // The u4b channel config minute to align to, before the offset is applied.
    // This should only ever be the char string "0", "2", "4", "6", or "8".
    // We only use offsets -1, 0, 2, 4, 6 in practice.
    // The telemetry and other prep is all done during offset == -1.
    int align_minute = atoi(cc._start_minute);

    if (isValidStartMinute(align_minute)) {
        // Add 10 so the modulo handles the wrap from 0 to -1 when offset == -1
        // (which would otherwise go to 9).
        align_minute = (10 + align_minute + offset) % 10;
    } else {
        V1_printf("ERROR: Illegal align_minute %d for u4b channel, using 0" EOL, align_minute);
        align_minute = (10 + 0 + offset) % 10;
    }

    // WARN: make sure cc._go_when_rdy is cleared before real balloon flight!
    // In "go when ready" mode we align on a 2-minute cadence instead of 10.
    if (cc._go_when_rdy[0] == '1') {
        // Add 2 to cover the wrap of 0 to -1 with offset -1 (goes to 1).
        align_minute = (2 + offset) % 2;
        return (minute() % 2) == align_minute;
    }

    return (minute() % 10) == align_minute;
}

//********************************************
// expected this is called at least 10 secs before starting minute
// if not in the minute before starting minute,
// it will wait until the right starting minute (depends on txNum)
// txNum can be 0, 1, 2, 3, or 4 for cw
// Expected to be called at least 10 secs before the starting minute.
// If not already in the minute before the starting minute,
// it will wait until the right starting minute (depends on txNum).
// txNum can be 0, 1, 2, 3, or 4 for CW.
void sendWspr(uint32_t hf_freq, int txNum, uint8_t *hf_tx_buffer, bool vfoOffAtEnd) {
    V1_print(F(EOL "sendWspr START now: "));
    printSystemDateTime();
    V1_print(F(EOL));

    // StampPrintf gives usec-resolution timing from this point to first symbol.
    // Note: usec wall-clock, NOT aligned to GPS time.
    if (VERBY[2])
        StampPrintf("sendWspr START now: minute: %d second: %d" EOL, minute(), second());

    Watchdog.reset();

    // -----------------------------------------------------------------------
    // Transmit all 162 WSPR symbols
    //
    // Symbol rate: exactly 12000/8192 Hz = 1.4648... baud
    // Symbol duration: 8192/12000 = 0.68266... secs (256/375 secs)
    // Total transmission: 162 * 256/375 = 110.592 secs
    //
    // Timing strategy: coarse sleep (wsprSleepForMillis) then spin on PROCEED.
    //   wsprSleepForMillis(645) = coarse wait slightly under one symbol period.
    //   PROCEED (set by interrupt handler) signals exact symbol boundary.
    //   645 ms chosen empirically; WSJT-X SDR DT reports confirm alignment.
    //   We use 10 ms granularity here, so slop is subtracted accordingly.
    //   Could sleep up to ~660 ms (< 682 ms symbol) then spin — same effect.
    //   Ref: https://ghubcoder.github.io/posts/awaking-the-pico/
    //
    // Logging notes:
    //   - StampPrintf log buffer capped at 4096 bytes due to RAM constraints.
    //     Exceeding it corrupts jtencode symbols (see jtencode README warning).
    //   - Only log every 10th symbol + symbol 161 to limit TX-time slowdown.
    //   - V3 dots logged every symbol for post-run timing analysis (deltatime.sh).
    //   - updateStatusLED() intentionally omitted inside the loop — may affect
    //     symbol timing; LEDs won't blink correctly during TX. 15–20 ms spin only.
    //
    // FIX! Measure usec-level drift impact of StampPrintf on SDR DT.
    // FIX! Kazu's library achieves ~75 usec alignment to GPS-corrected RTC.
    // -----------------------------------------------------------------------

    const uint8_t symbol_count = WSPR_SYMBOL_COUNT;
    absolute_time_t wsprStartTime = get_absolute_time();

    for (uint8_t i = 0; i < symbol_count; i++) {
        uint8_t symbol = hf_tx_buffer[i];

        // Validate symbol is in range 0–3.
        // Symbols go corrupt if RAM is tight (jtencode issue); this catches it.
        if (symbol > 3) {
            V1_printf("ERROR: bad symbol i %u 0x%02x" EOL, i, symbol);
            symbol = 0;
        }

        // Log symbol index at start of boundary (VERBY[2] only, sparse)
        if (VERBY[2] && ((i % 10 == 0) || i == 161))
            StampPrintf("b" EOL);

        // Set the transmit frequency for this symbol (changes PLL numerator only)
        startSymbolFreq(hf_freq, symbol, false, false);

        // Log symbol index after freq set (VERBY[2] only, sparse)
        if (VERBY[2] && ((i % 10 == 0) || i == 161))
            StampPrintf("sym: %d" EOL, i);

        // Coarse sleep: slightly under one symbol period (645 ms empirical).
        // wsprSleepForMillis also resets the watchdog.
        PROCEED = false;
        wsprSleepForMillis(645);  // was 645 as of 1/9/25

        // Fine alignment: spin until interrupt handler signals symbol boundary.
        // Watchdog will fire if PROCEED never comes (interrupt handler failure).
        Watchdog.reset();
        while (!PROCEED) tight_loop_contents();

        // V3 dot per symbol — use deltatime.sh to check symbol timing distribution
        if ((i % 10) == 1) V3_print(".");

        Watchdog.reset();
    }

    V2_print(F(EOL));  // terminate the V3 dot line

    // -----------------------------------------------------------------------
    // Verify total transmission duration
    // Target: 162 * 256/375 = 110.592 secs
    // -----------------------------------------------------------------------
    absolute_time_t wsprEndTime   = get_absolute_time();
    int64_t wsprDuration_us       = absolute_time_diff_us(wsprStartTime, wsprEndTime);
    float   wsprDurationSecs      = (float)wsprDuration_us / 1000000.0f;

    if (wsprDurationSecs < 110.0f || wsprDurationSecs > 111.0f) {
        V1_print(F(EOL "ERROR: 110.592 secs goal:"));
        V1_printf(EOL "wsprDurationSecs %.5f seems too big or small" EOL, wsprDurationSecs);
        V1_printf("ERROR: wsprDuration %" PRId64 " usecs" EOL EOL, wsprDuration_us);
    }

    // Flush any deferred VERBY[2] log entries accumulated during TX
    if (VERBY[2]) DoLogPrint();

    disablePwmInterrupts();

    // -----------------------------------------------------------------------
    // Post-TX VFO / clock output state
    // -----------------------------------------------------------------------
    if (vfoOffAtEnd) {
        vfo_turn_off();
    } else if (DO_CLK_OFF_FOR_WSPR_MODE) {
        // FIX! Should we set frequency to symbol 0 before clock-off so every
        // subsequent transition into a new message looks identical?
        // Probably moot since we're powering the clock down now.
        if (USE_SI5351A_CLK_POWERDOWN_FOR_WSPR_MODE) {
            // FIX! Confirm this works on ms5351m (may not matter in practice)
            si5351a_power_down_clk01(true);  // also performs PLLB reset; prints
        } else {
            vfo_turn_off_clk_out(WSPR_TX_CLK_0_NUM, true);  // affects clk0 and clk1; prints
        }
    }

    Watchdog.reset();
    V1_println(F("sendWspr END"));
}
//**********************************
void syncAndSendWspr(uint32_t hf_freq, int txNum, uint8_t *hf_tx_buffer,
    char *hf_callsign, char *hf_grid4, char *hf_power, bool vfoOffAtEnd) {

    V1_printf("syncAndSendWSPR START now: minute: %d second: %d" EOL, minute(), second());

    // -----------------------------------------------------------------------
    // Validate txNum (0–3 for WSPR, 0–4 for CW)
    // -----------------------------------------------------------------------
    if (txNum < 0 || txNum > 3) {
        V1_printf("syncAndSendWSPR() bad txNum %d, using 0" EOL, txNum);
        txNum = 0;
    }

    // -----------------------------------------------------------------------
    // Start VFO at symbol 0 frequency for warmup.
    // The actual symbol frequency for each of the 4 FSK tones could be
    // pre-calculated here during warmup and stored as globals for sendWspr().
    // FIX! what is the initial power level?
    // FIX! pre-calc all 4 symbol freqs during first warmup symbol if only_pll_num.
    // -----------------------------------------------------------------------
    uint8_t warmupSymbol = 0;  // valid range: 0–3
    startSymbolFreq(hf_freq, warmupSymbol, true, false);  // only_pll_num (expected)

    // Encode callsign/grid/power into 162 4-FSK symbols in hf_tx_buffer.
    // atoi() used for power since hf_power arrives as a string.
    // Ref: https://stackoverflow.com/questions/27260304/equivalent-of-atoi-for-unsigned-integers
    set_hf_tx_buffer(hf_tx_buffer, hf_callsign, hf_grid4, (uint8_t)atoi(hf_power));

    // -----------------------------------------------------------------------
    // Minute alignment
    // Each txNum maps to an even-minute TX slot: txNum 0→min 0, 1→min 2, etc.
    // We align to (2*txNum), waiting in the minute before (2*txNum - 1).
    // FIX! in debug mode, why aren't we aligning to any even minute?
    // -----------------------------------------------------------------------
    int txMinute     = 2 * txNum;       // 0, 2, 4, or 6
    int preTxMinute  = txMinute - 1;    // minute before TX; alignMinute() interprets mod-10

    V1_printf("waiting for alignMinute(%d) && second()==0)" EOL, txMinute);
    V1_flush();  // clear TX buffer before the tight loops ahead

    // Phase 1: wait until we are in the minute before the TX minute.
    // Must execute at least once.
    while (!alignMinute(preTxMinute)) {
        Watchdog.reset();
        updateStatusLED();
        sleep_ms(20);
    }

    // -----------------------------------------------------------------------
    // Phase 2: turn on clk0/1 once, then spin until the TX minute at second 0.
    //
    // We should always arrive here with clocks off — whether this is the first
    // or a subsequent WSPR message (check how sendWspr() leaves the clocks).
    //
    // Clock turn-on notes:
    //   - PDN-bit disable requires a PLL reset to stay in phase.
    //   - Hans G0UPL noted ~2 ms of garbage RF at PLL reset.
    //   - We turn on ~1 sec before TX and log how long RF is hot before use.
    //   - ms5351m: si5351a CLK power-down may not work; clocks may always be on.
    // FIX! Measure/verify turn-on glitch at SDR.
    // FIX! Could check millis() > 900 to ensure we're ~100 ms before alignment.
    // -----------------------------------------------------------------------
    bool clk01_turned_on       = false;
    absolute_time_t clkOnTime  = 0;

    while (!clk01_turned_on || !(alignMinute(txMinute) && (second() == 0))) {
        Watchdog.reset();
        updateStatusLED();

        if (!clk01_turned_on) {
            if (DO_CLK_OFF_FOR_WSPR_MODE) {
                if (USE_SI5351A_CLK_POWERDOWN_FOR_WSPR_MODE) {
                    si5351a_power_up_clk01(true);   // prints; performs PLL reset
                } else {
                    vfo_turn_on_clk_out(WSPR_TX_CLK_0_NUM, true);  // silent; performs PLL reset
                }
            }
            clkOnTime       = get_absolute_time();
            clk01_turned_on = true;
        }

        busy_wait_ms(10);
    }

    // -----------------------------------------------------------------------
    // Sanity checks and RF-on duration log
    // -----------------------------------------------------------------------
    if (!clk01_turned_on) {
        V1_print(F("ERROR: syncAndSendWspr big problem, no clk01_turned_on"));
    } else if (DO_CLK_OFF_FOR_WSPR_MODE) {
        if (clkOnTime == 0) {
            V1_printf("ERROR: syncAndSendWspr why is clkOnTime == 0 here?");
        } else {
            float rfOnMs = (float)absolute_time_diff_us(clkOnTime, get_absolute_time()) / 1000.0f;
            V1_printf("syncAndSendWspr rf is on for %.3f millisecs before real wspr msg" EOL,
                rfOnMs);
        }
    }

    turnOnLED(false);   // ensure LED is not stuck on during WSPR TX
    Watchdog.reset();

    // -----------------------------------------------------------------------
    // Sub-second alignment before first symbol
    //
    // We are at second() == 0 of the TX minute. WSPR convention starts symbols
    // ~1 sec into the minute. Two strategies:
    //
    // A) ALIGN_TO_1SEC_IN_MODE (spin until second() transitions to 1):
    //    - Most accurate in theory.
    //    - Loop guard of 101 iterations (~1010 ms) prevents infinite spin.
    //    - Error if we don't enter at second() == 0.
    //
    // B) Fixed busy-wait (EXTRA_DELAY_AFTER_ZERO_SEC ms):
    //    - Empirically tuned against WSJT-X reported DT.
    //    - millis()/usec counters are NOT GPS-aligned (they run from boot),
    //      so we cannot use them for absolute time offset.
    //    - Tuning history (target DT = 0.0):
    //        SIM65M:  900 ms → DT 0.2  (2/16/25)
    //                 700 ms  (2/17/25)
    //                 650 ms  (2/18/25)  occasionally late
    //                 700 ms  current

    //        ATGM336: 800 ms → DT 0.2  (2/16/25)
    //                 750 ms → DT 0.1  (2/16/25)
    //                 700 ms  (2/17/25)  occasionally early
    //                 700 ms  current
    // -----------------------------------------------------------------------
    static int EXTRA_DELAY_AFTER_ZERO_SEC;
    EXTRA_DELAY_AFTER_ZERO_SEC = 700;  // ms; same value for both GPS modules currently

    if (EXTRA_DELAY_AFTER_ZERO_SEC < 0 || EXTRA_DELAY_AFTER_ZERO_SEC > 1000) {
        V1_printf("ERROR: bad EXTRA_DELAY_AFTER_ZERO_SEC %d.. setting to 0" EOL,
            EXTRA_DELAY_AFTER_ZERO_SEC);
        EXTRA_DELAY_AFTER_ZERO_SEC = 0;
    }

    bool ALIGN_TO_1SEC_IN_MODE = false;

    if (ALIGN_TO_1SEC_IN_MODE) {
        // Spin until second() rolls over from 0 to 1 (at most ~1 sec / 100 iters)
        int      alignSecond  = second();
        uint32_t alignLoopCnt = 0;

        if (alignSecond != 0) {
            V1_printf("ERROR: 1-sec-in alignment started with non-zero second() %d" EOL,
                alignSecond);
        }

        while (alignSecond == 0) {
            if (++alignLoopCnt > 101) {
                V1_print(F("ERROR: 1-sec-in alignment looped > 101 times" EOL));
                break;
            }
            busy_wait_ms(10);
            alignSecond = second();
        }
    } else {
        busy_wait_ms(EXTRA_DELAY_AFTER_ZERO_SEC);
    }

    // -----------------------------------------------------------------------
    // Arm PWM interrupt and transmit
    //
    // PWM_WRAP_CNT is the full period value (-1 before set as wrap top).
    // We reset PROCEED and reinitialise the PWM divider/wrap for every WSPR
    // message so the first interrupt is a known distance from our position now.
    // -----------------------------------------------------------------------
    PROCEED = false;
    setPwmDivAndWrap(PWM_DIV, PWM_WRAP_CNT);
    sendWspr(hf_freq, txNum, hf_tx_buffer, vfoOffAtEnd);

    V1_println(F("syncAndSendWSPR END"));
}
//**********************************
// wspr_encode() signature for reference:
//   wspr_encode(const char *call, const char *loc, const uint8_t dbm, uint8_t *symbols)
//   call    - Callsign (12 chars max; we guarantee 6 max)
//   loc     - Maidenhead grid locator (6 chars max; we pass 4)
//   dbm     - Output power in dBm, passed as uint8_t
//   symbols - Caller-allocated array of at least WSPR_SYMBOL_COUNT uint8_t
// Produces a Type 1, 2, or 3 WSPR symbol table.
// Ref: https://github.com/robertostling/wspr-tools  (useful Python cross-check)
void set_hf_tx_buffer(uint8_t *hf_tx_buffer,
    char *hf_callsign, char *hf_grid4, uint8_t hf_power) {

    V1_println(F("set_hf_tx_buffer START"));

    memset(hf_tx_buffer, 0, 162);  // 162 == WSPR_SYMBOL_COUNT

    bool fatalErrorReboot = false;

    // -----------------------------------------------------------------------
    // Validate hf_callsign
    //
    // No leading spaces expected (would indicate wrong snprintf() usage).
    // Trailing spaces are intentional — we right-pad to work around a JTEncode
    // bug — so spaces are only an error if a non-space character follows one.
    // Valid non-space length: 3–6 characters.
    // -----------------------------------------------------------------------
    bool    spaceFound  = false;
    uint8_t notspaceCnt = 0;

    for (uint8_t i = 0; i < sizeof(hf_callsign); i++) {
        if (hf_callsign[i] == ' ') {
            spaceFound = true;
        } else {
            if (spaceFound) {
                V1_printf("ERROR: hf_callsign '%s' has non-space after space at %u" EOL,
                    hf_callsign, i);
                fatalErrorReboot = true;
            }
            notspaceCnt++;
        }
    }

    V1_printf("length check: hf_callsign %s before jtencode had notspaceCnt %d" EOL,
        hf_callsign, notspaceCnt);

    if (notspaceCnt < 3 || notspaceCnt > 6) {
        V1_printf("ERROR: bad notspaceCnt: hf_callsign %s before jtencode had notspaceCnt %d" EOL,
            hf_callsign, notspaceCnt);
        fatalErrorReboot = true;
    }

    // -----------------------------------------------------------------------
    // Validate hf_grid4
    //
    // Must be exactly 4 characters with no spaces.
    // hf_power (uint8_t) needs no range check here.
    // -----------------------------------------------------------------------
    int gridLen = strlen(hf_grid4);

    V1_printf("length check: hf_grid4 %s before jtencode was strlen %d" EOL,
        hf_grid4, gridLen);

    if (gridLen != 4) {
        V1_printf("ERROR: bad length: hf_grid4 %s before jtencode was strlen %d" EOL,
            hf_grid4, gridLen);
        fatalErrorReboot = true;
    }

    for (uint32_t i = 0; i <= sizeof(hf_grid4); i++) {
        if (hf_grid4[i] == ' ') {
            V1_printf("ERROR: hf_grid4 '%s' has <space> at %lu" EOL, hf_grid4, i);
            fatalErrorReboot = true;
        }
    }

    // -----------------------------------------------------------------------
    // Halt and reboot on any validation failure.
    // Corrupt inputs produce bad jtencode symbols and a broken transmission.
    // Also: jtencode corrupts symbols silently if RAM is tight — catching bad
    // inputs here rules out one source of symbol errors before encoding.
    // -----------------------------------------------------------------------
    if (fatalErrorReboot) {
        V0_println(F("ERROR: set_hf_tx_buffer() fatal error, rebooting."));
        V0_flush();
        Watchdog.enable(5000);  // trigger reboot after 5 seconds
        while (true) tight_loop_contents();
    }

    // -----------------------------------------------------------------------
    // Encode into 162 4-FSK channel symbols
    // -----------------------------------------------------------------------
    jtencode.wspr_encode(hf_callsign, hf_grid4, hf_power, hf_tx_buffer);

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

    // UART baud rate: if you called Serial.begin() / Serial2.begin() before the clock change, 
    // the UART divider was computed for the old clk_peri. 
    // After set_sys_clock_khz, do Serial2.end(); Serial2.begin(baud); to recompute.

    if (!set_sys_clock_khz(clk_khz, false)) {
        V1_printf("ERROR: setup1(): can't change clock to %lu Mhz. Using %lu instead" EOL,
            PLL_SYS_MHZ, DEFAULT_PLL_SYS_MHZ);
        PLL_SYS_MHZ = DEFAULT_PLL_SYS_MHZ;
        snprintf(cc._clock_speed, sizeof(cc._clock_speed), "%lu", PLL_SYS_MHZ);
        // FIX! hmm. this guy could try to write flash, while the other guy is reading?
        // write_FLASH();
        // check the default?
        clk_khz = PLL_SYS_MHZ * 1000UL;
        if (!set_sys_clock_khz(clk_khz, false)) {
            V1_println("ERROR: setup1() DEFAULT_SYS_MHZ not legal either. Will use 125");
            PLL_SYS_MHZ = 125;
            snprintf(cc._clock_speed, sizeof(cc._clock_speed), "%lu", PLL_SYS_MHZ);
            // write_FLASH();
        }
    }

    V1_printf("Attempt to set rp2040 clock to PLL_SYS_MHZ %lu (legal)" EOL, PLL_SYS_MHZ);
    // 2nd arg is "required"
    set_sys_clock_khz(clk_khz, true);

    V1_println(F("initPicoClock END"));
    return 0;
}

//**********************
// will just use c style string.
// stored in array of characters terminated by null: '\0'
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
