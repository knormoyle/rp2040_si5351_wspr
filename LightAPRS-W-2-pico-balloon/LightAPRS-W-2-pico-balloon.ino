#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>

#include <TinyGPS++.h>                      //https://github.com/mikalhart/TinyGPSPlus
#include <GEOFENCE.h>                       // Modified version of https://github.com/TomasTT7/TT7F-Float-Tracker/blob/master/Software/ARM_GEOFENCE.c
#include <Adafruit_SleepyDog.h>             //https://github.com/adafruit/Adafruit_SleepyDog
#include <Adafruit_BMP085.h>                //https://github.com/adafruit/Adafruit-BMP085-Library

#include <JTEncode.h>                       //https://github.com/etherkit/JTEncode (JT65/JT9/JT4/FT8/WSPR/FSQ Encoder Library)
#include <TimeLib.h>                        //https://github.com/PaulStoffregen/Time

#include <MemoryFree.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define SerialUSB   Serial
#define Si5351Pwr     4
#define BattPin       A3
#define GpsPwr        16

//macros
// FIX! not used?
#define Si5351ON    

#define Si5351OFF   vfo_turn_off()

#define GpsON                   \
  do {                          \
    Serial2.begin(9600);        \
    digitalWrite(GpsPwr, LOW);  \
    /*printf("GpsON\n");*/      \
  } while (false)

#define GpsOFF                  \
  do {                          \
    digitalWrite(GpsPwr, HIGH); \
    Serial2.end();              \
    gps.date.clear();           \
    /*printf("GpsOFF\n");*/     \
  } while (false)

#define DEVMODE // Development mode. Uncomment to enable for debugging.
#define DEVMODE2// Development mode. Uncomment to enable for debugging.

//******************************  CONFIG **********************************
char    CallSign[7]="NOCALL";//DO NOT FORGET TO CHANGE YOUR CALLSIGN
int8_t  CallNumber=11; //11; //SSID http://www.aprs.org/aprs11/SSIDs.txt
char    comment[50] = "LightAPRS-W 2.0";// Max 50 char
char    StatusMessage[50] = "LightAPRS-W 2.0 by TA2NHP & TA2MUN";

//*****************************************************************************

uint16_t  BeaconWait=50;  //seconds sleep for next beacon (HF or VHF). This is an optimized value, do not change this if possible.

uint16_t  BattWait=1;    //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel) 
float     BattMin=0.0;    //min Volts to wake up.
float     GpsMinVolt=0.0; //min Volts for GPS to wake up. (important if power source is solar panel) 
float     WsprBattMin=0.0;//min Volts for HF (WSPR) radio module to transmit (TX) ~10 mW
float     HighVolt=9.9;   //GPS is always on if the voltage exceeds this value to protect solar caps from overcharge

//******************************  HF (WSPR) CONFIG *************************************

char hf_call[7] = "NOCALL";// DO NOT FORGET TO CHANGE YOUR CALLSIGN

//#define WSPR_DEFAULT_FREQ       10140200UL //30m band
#define WSPR_DEFAULT_FREQ       14097100UL //20m band
//#define WSPR_DEFAULT_FREQ       18106100UL //17M band
//#define WSPR_DEFAULT_FREQ       21096100UL //15m band
//#define WSPR_DEFAULT_FREQ       24926100UL //12M band
//#define WSPR_DEFAULT_FREQ       28126100UL //10m band
//for all bands -> http://wsprnet.org/drupal/node/7352


// Supported modes, default HF mode is WSPR
enum mode {MODE_WSPR};
enum mode cur_mode = MODE_WSPR; //default HF mode

//*******************************************************************************
//******************************  APRS SETTINGS *********************************

boolean  aliveStatus = true; //for tx status message on first wake-up just once.
static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1; //increase +1 after every APRS transmission

//*******************************************************************************
//******************************  HF SETTINGS   *********************************

#define WSPR_TONE_SPACING  146          // ~1.46 Hz
#define WSPR_DELAY         683          // Delay value for WSPR
#define HF_CORRECTION      -13000       // Change this for your ref osc
  
// Global variables
unsigned long hf_freq;
char hf_message[13] = "NOCALL AA00";//for WSPR, updated by hf_call and GPS location
char hf_loc[] = "AA00";             //for WSPR, updated by GPS location. You don't have to change this.
uint8_t dbm = 10;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
volatile bool proceed = false;

//*******************************************************************************
//******************************  GPS SETTINGS   *********************************
int16_t   GpsResetTime=1800; // timeout for reset if GPS is not fixed

// GEOFENCE 
boolean arissModEnabled = false; //do not change this, temp value. 

boolean GpsFirstFix=false; //do not change this
boolean ublox_high_alt_mode_enabled = false; //do not change this
int16_t GpsInvalidTime=0; //do not change this

//********************************************************************************

TinyGPSPlus gps;
Adafruit_BMP085 bmp;
JTEncode jtencode;

// when we set both?
#define WSPR_TX_CLK_NUM     0
#define WSPR_TX_CLK_1_NUM     1
// this is the other differential clock for wspr? (was aprs)
#define WSPR_TX_CLK_0_NUM     0

#define GPS_VCC_ON_N_PIN            16
#define GPS_NRESET_PIN              5
#define GPS_ON_PIN                  6
#define GPS_UART1_TX_PIN            8
#define GPS_UART1_RX_PIN            9
#define GPS_1PPS_PIN                17

#define VFO_VDD_ON_N_PIN            4
#define VFO_I2C0_SDA_PIN            12
#define VFO_I2C0_SCL_PIN            13

#define BMP280_I2C1_SDA_PIN         2
#define BMP280_I2C1_SCL_PIN         3

#define PLL_CALCULATION_PRECISION   4

#define SI5351A_CLK_IDRV_8MA        (3 << 0)
#define SI5351A_CLK_IDRV_6MA        (2 << 0)
#define SI5351A_CLK_IDRV_4MA        (1 << 0)
#define SI5351A_CLK_IDRV_2MA        (0 << 0)

#define VFO_I2C_INSTANCE            i2c0
#define VFO_I2C0_SCL_HZ             (1000 * 1000)

static void vfo_init(void)
{
  // turn ON VFO VDD
  gpio_init(VFO_VDD_ON_N_PIN);
  gpio_pull_up(VFO_VDD_ON_N_PIN);
  gpio_put(VFO_VDD_ON_N_PIN, 0);

  // init I2C0 for VFO
  i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);

  gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
  gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

  gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
}

static void vfo_set_power_on(bool turn_on)
{
  static bool s_is_on = false;
  if (turn_on == s_is_on) return;
  s_is_on = turn_on;
  gpio_set_dir(VFO_VDD_ON_N_PIN, (turn_on ? GPIO_OUT : GPIO_IN));
}

#define SI5351A_I2C_ADDR                0x60

static uint8_t s_i2c_buf[16];

int i2cWrite(uint8_t reg, uint8_t val){    // write reg via i2c
  s_i2c_buf[0] = reg;
  s_i2c_buf[1] = val;

  int res;
  res = i2c_write_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, s_i2c_buf, 2, false, 1000);

  if (res < PICO_ERROR_NONE) {
    #if defined(DEVMODE)
    printf("I2C error %d: reg:%02x val:%02x\n", res, reg, val);
    #endif
  }
  return res;
}

int i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt){   // write array
  s_i2c_buf[0] = reg;
  memcpy(&s_i2c_buf[1], vals, vcnt);

  int res;
  res = i2c_write_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, s_i2c_buf, (vcnt + 1), false, 10000);

  if (res < PICO_ERROR_NONE) {
    #if defined(DEVMODE)
    printf("I2C error %d: reg:%02x\n", res, reg);
    #endif
  }
  return res;
}

/*
    Si5351A related functions
    Developed by Kazuhisa "Kazu" Terasaki AG6NS
    https://github.com/kaduhi/AFSK_to_FSK_VFO
    This code was developed originally for QRPGuys AFP-FSK Digital Transceiver III kit
    https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii
    https://qrpguys.com/wp-content/uploads/2022/09/ft8_v1.4_092522-1.zip
 */

#define SI5351_TCXO_FREQ                26000000

#define SI5351A_OUTPUT_ENABLE_CONTROL   3
#define SI5351A_CLK0_CONTROL            16
#define SI5351A_CLK1_CONTROL            17
#define SI5351A_CLK7_CONTROL            23
#define SI5351A_PLLB_BASE               34
#define SI5351A_MULTISYNTH0_BASE        42
#define SI5351A_MULTISYNTH1_BASE        50
#define SI5351A_PLL_RESET               177

#define SI5351A_CLK0_MS0_INT            (1 << 6)
#define SI5351A_CLK0_MS0_SRC_PLLB       (1 << 5)
#define SI5351A_CLK1_MS1_INT            (1 << 6)
#define SI5351A_CLK1_MS1_SRC_PLLB       (1 << 5)

#define SI5351A_CLK0_SRC_MULTISYNTH_0   (3 << 2)
#define SI5351A_CLK1_SRC_MULTISYNTH_0   (2 << 2)

#define SI5351A_CLK1_CLK1_INV           (1 << 4)
#define SI5351A_CLK1_SRC_MULTISYNTH_1   (3 << 2)

#define SI5351A_CLK0_IDRV_8MA           (3 << 0)
#define SI5351A_CLK0_IDRV_6MA           (2 << 0)
#define SI5351A_CLK0_IDRV_4MA           (1 << 0)
#define SI5351A_CLK0_IDRV_2MA           (0 << 0)
#define SI5351A_CLK1_IDRV_8MA           (3 << 0)
#define SI5351A_CLK1_IDRV_6MA           (2 << 0)
#define SI5351A_CLK1_IDRV_4MA           (1 << 0)
#define SI5351A_CLK1_IDRV_2MA           (0 << 0)

#define SI5351A_PLL_RESET_PLLB_RST      (1 << 7)

static uint32_t prev_ms_div = 0;
static uint8_t s_regs[8];
static uint8_t s_vfo_drive_strength[3];  // 0:2mA, 1:4mA, 2:6mA, 3:8mA

void si5351a_setup_PLLB(uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t p1 = 128 * mult + ((128 * num) / denom) - 512;
  uint32_t p2 = 128 * num - denom * ((128 * num) / denom);
  uint32_t p3 = denom;

  s_regs[0] = (uint8_t)(p3 >> 8);
  s_regs[1] = (uint8_t)p3;
  s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
  s_regs[3] = (uint8_t)(p1 >> 8);
  s_regs[4] = (uint8_t)p1;
  s_regs[5] = ((uint8_t)(p3 >> 12) & 0xf0) | ((uint8_t)(p2 >> 16) & 0x0f);
  s_regs[6] = (uint8_t)(p2 >> 8);
  s_regs[7] = (uint8_t)p2;

  static uint8_t s_regs_prev[8];
  uint8_t start = 0;
  uint8_t end = 7;
  if (prev_ms_div != 0) {
    for (; start < 8; start++) {
      if (s_regs[start] != s_regs_prev[start]) break;
    }
    if (start == 8) return;
    for (; end > start; end--) {
      if (s_regs[end] != s_regs_prev[end]) break;
    }
  }
  uint8_t reg = SI5351A_PLLB_BASE + start;
  uint8_t len = end - start + 1;
  i2cWriten(reg, &s_regs[start], len);
  *((uint64_t *)s_regs_prev) = *((uint64_t *)s_regs);

}

// div must be even number
void si5351a_setup_multisynth0(uint32_t div)
{
  uint32_t p1 = 128 * div - 512;

  s_regs[0] = 0;
  s_regs[1] = 1;
  s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
  s_regs[3] = (uint8_t)(p1 >> 8);
  s_regs[4] = (uint8_t)p1;
  s_regs[5] = 0;
  s_regs[6] = 0;
  s_regs[7] = 0;
  i2cWriten(SI5351A_MULTISYNTH0_BASE, s_regs, 8);
  i2cWrite(SI5351A_CLK0_CONTROL, (SI5351A_CLK0_MS0_INT | 
                                  SI5351A_CLK0_MS0_SRC_PLLB | 
                                  SI5351A_CLK0_SRC_MULTISYNTH_0 | 
                                  s_vfo_drive_strength[0]));

#ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
  i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);
  i2cWrite(SI5351A_CLK1_CONTROL, (SI5351A_CLK1_MS1_INT | 
                                  SI5351A_CLK1_MS1_SRC_PLLB | 
                                  SI5351A_CLK1_CLK1_INV | 
                                  SI5351A_CLK1_SRC_MULTISYNTH_1 | 
                                  s_vfo_drive_strength[0]));
#endif

#ifdef TEST_ONLY
  printf("VFO_DRIVE_STRENGTH: %d\n", (int)s_vfo_drive_strength[0]);
#endif //TEST_ONLY

}

static void si5351a_setup_multisynth1(uint32_t div)
{
  uint32_t p1 = 128 * div - 512;

  s_regs[0] = 0;
  s_regs[1] = 1;
  s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
  s_regs[3] = (uint8_t)(p1 >> 8);
  s_regs[4] = (uint8_t)p1;
  s_regs[5] = 0;
  s_regs[6] = 0;
  s_regs[7] = 0;
  i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);

  i2cWrite(SI5351A_CLK1_CONTROL, (SI5351A_CLK1_MS1_INT | 
                                  SI5351A_CLK1_MS1_SRC_PLLB | 
                                  SI5351A_CLK1_SRC_MULTISYNTH_1 | 
                                  s_vfo_drive_strength[1]));
#ifdef TEST_ONLY
  printf("VFO_DRIVE_STRENGTH: %d\n", (int)s_vfo_drive_strength[1]);
#endif //TEST_ONLY
}

static void si5351a_reset_PLLB(void)
{
  i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
}

// freq is in 28.4 fixed point number, 0.0625Hz resolution
void vfo_set_freq_x16(uint8_t clk_number, uint32_t freq)
{
  #define PLL_MAX_FREQ        900000000
  #define PLL_MIN_FREQ        600000000
  #define PLL_MID_FREQ        ((PLL_MAX_FREQ + PLL_MIN_FREQ) / 2)
  #define PLL_DENOM_MAX       0x000fffff

  uint32_t ms_div = PLL_MID_FREQ / (freq >> PLL_CALCULATION_PRECISION) + 1;
  ms_div &= 0xfffffffe;   // make it even number

  uint32_t pll_freq = ((uint64_t)freq * ms_div) >> PLL_CALCULATION_PRECISION;

  uint32_t tcxo_freq = SI5351_TCXO_FREQ;
  uint32_t pll_mult   = pll_freq / tcxo_freq;
  uint32_t pll_remain = pll_freq - (pll_mult * tcxo_freq);
  uint32_t pll_num    = (uint64_t)pll_remain * PLL_DENOM_MAX / tcxo_freq;
  si5351a_setup_PLLB(pll_mult, pll_num, PLL_DENOM_MAX);

  if (ms_div != prev_ms_div) {
    prev_ms_div = ms_div;
    if (clk_number == 0) {
      si5351a_setup_multisynth0(ms_div);
    }
    else {
      // this was for setting up the aprs clock on clk_num == 1?
      si5351a_setup_multisynth1(ms_div);
    }
    si5351a_reset_PLLB();
  }
}

static uint8_t  si5351bx_clken = 0xff;

void vfo_turn_on_clk_out(uint8_t clk_number)
{
  uint8_t enable_bit = 1 << clk_number;

#ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
  if (clk_number == 0) {
    enable_bit |= 1 << 1;
  }
#endif
  si5351bx_clken &= ~enable_bit;
  i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
}

void vfo_turn_off_clk_out(uint8_t clk_number)
{
  uint8_t enable_bit = 1 << clk_number;
#ifdef ENABLE_DIFFERENTIAL_TX_OUTPUT
  if (clk_number == 0) {
    enable_bit |= 1 << 1;
  }
#endif
  si5351bx_clken |= enable_bit;
  i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
}

void vfo_set_drive_strength(uint8_t clk_number, uint8_t strength)
{
  s_vfo_drive_strength[clk_number] = strength;

  // reset the prev_ms_div to force vfo_set_freq_x16() to call si5351a_setup_multisynth1() next time
  prev_ms_div = 0;
}

bool vfo_is_on(void)
{
  return gpio_is_dir_out(VFO_VDD_ON_N_PIN);
}

// what is vfo_clk2 ? is that another PLL? is that used for calibration?
void vfo_turn_on(uint8_t clk_number)
{
  if (vfo_is_on()) return;    // already on

  gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
  vfo_set_power_on(true);
  // sleep_ms(100);
  busy_wait_us_32(100000);

  // output 7MHz on CLK0
  uint8_t reg;
  while (i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff) < PICO_ERROR_NONE) {   // Disable all CLK output drivers
    i2c_deinit(VFO_I2C_INSTANCE);
    // sleep_ms(10);
    busy_wait_us_32(10000);
    i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);

    gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
    gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);

    // sleep_ms(10);
    busy_wait_us_32(10000);
  }

  for (reg = SI5351A_CLK0_CONTROL; reg <= SI5351A_CLK7_CONTROL; reg++) i2cWrite(reg, 0xCC);    // Powerdown CLK's

  static const uint8_t s_ms_values[] = { 0, 1, 0x0C, 0, 0, 0, 0, 0 };
  i2cWriten(42, (uint8_t *)s_ms_values, 8);   // set MS0 for div_4 mode (minimum division)
  i2cWriten(50, (uint8_t *)s_ms_values, 8);   // set MS1 for div_4 mode (minimum division)
  i2cWriten(58, (uint8_t *)s_ms_values, 8);   // set MS2 for div_4 mode (minimum division)

  static const uint8_t s_pll_values[] = { 0, 0, 0, 0x05, 0x00, 0, 0, 0 };
  i2cWriten(26, (uint8_t *)s_pll_values, 8);  // set PLLA for div_16 mode (minimum even integer division)
  i2cWriten(34, (uint8_t *)s_pll_values, 8);  // set PLLB for div_16 mode (minimum even integer division)

  i2cWrite(149, 0x00);  // Disable Spread Spectrum
  i2cWrite(177, 0xA0);  // Reset PLLA and PLLB
  i2cWrite(187, 0x00);  // Disable all fanout

  prev_ms_div = 0;

  // uint32_t freq = 7040000UL << PLL_CALCULATION_PRECISION;

  // FIX! make this the base freq for the band?
  uint32_t freq = 14097000UL << PLL_CALCULATION_PRECISION;
  vfo_set_freq_x16(clk_number, freq);

  si5351bx_clken = 0xff;
  vfo_turn_on_clk_out(clk_number);
}

void vfo_turn_off(void)
{
  if (!vfo_is_on()) return;   // already off

  // disable all clk output
  si5351bx_clken = 0xff;
  i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, si5351bx_clken);
  // sleep_ms(10);
  busy_wait_us_32(10000);

  vfo_set_power_on(false);
  gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_NULL);
  gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_NULL);
}


/*
  status indicator LED
*/

#define STATUS_LED_PIN              25

#define turnOnLED(turn_on)          digitalWrite(STATUS_LED_PIN, (turn_on) ? HIGH : LOW)
#define isLEDOn()                   (digitalRead(STATUS_LED_PIN) ? true : false)
// background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
#define flipLED()                   turnLedOn(!isLedOn())

#define LED_BLINK_ON_PERIOD_USEC    50000
#define LED_BLINK_OFF_PERIOD_USEC   300000
#define LED_BLINK_PAUSE_PERIOD_USEC 1000000

#define LED_STATUS_NO_GPS           1
#define LED_STATUS_GPS_TIME         2
#define LED_STATUS_GPS_FIX          3
#define LED_STATUS_TX_APRS          4
#define LED_STATUS_TX_WSPR          5

int statusLEDBlinkCnt = 0;

void initStatusLED(void)
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  turnOnLED(true);
}

void setStatusLEDBlinkCount(int cnt)
{
  statusLEDBlinkCnt = cnt;
}

void updateStatusLED(void)
{
  static uint32_t nextFlipUsec = 0;
  static int targetBlinkCnt = 0;
  static int currBlinkCnt = 0;

  uint32_t usec = time_us_32();
  if ((int32_t)(nextFlipUsec - usec) <= 0) {
    if (isLEDOn() == false) {
      // OFF to ON
      if (targetBlinkCnt == 0) {
        targetBlinkCnt = statusLEDBlinkCnt;
        currBlinkCnt = 0;
      }
      if (++currBlinkCnt <= targetBlinkCnt) {
        turnOnLED(true);
      }
      nextFlipUsec = usec + LED_BLINK_ON_PERIOD_USEC;
    }
    else {
      // ON to OFF
      turnOnLED(false);
      if (currBlinkCnt >= targetBlinkCnt) {
        nextFlipUsec = usec + LED_BLINK_PAUSE_PERIOD_USEC;
        targetBlinkCnt = 0;
      }
      else {
        nextFlipUsec = usec + LED_BLINK_OFF_PERIOD_USEC;
      }
    }
  }
}



void setup() {
  Watchdog.enable(30000);
  Watchdog.reset();
  // While the energy rises slowly with the solar panel, 
  // using the analog reference low solves the analog measurement errors.

  initStatusLED();
  setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
  // FIX! why is this commented out?
  // pinMode(Si5351Pwr, OUTPUT);
  pinMode(GpsPwr, OUTPUT);
  // FIX! why is this commented out?
  // pinMode(BattPin, INPUT);
  analogReadResolution(12);
  
  GpsOFF;
  Si5351OFF;  

  Serial2.setRX(GPS_UART1_RX_PIN);
  Serial2.setTX(GPS_UART1_TX_PIN);
  Serial2.begin(9600);//GPS

  SerialUSB.begin(115200);
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  Watchdog.reset();  
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB){;}
  Watchdog.reset(); 

  SerialUSB.println(F("Starting"));

  Wire.begin(); // somehow this is necessary for Serial2 to work properly
  vfo_init();

  SerialUSB.print(F("WSPR (HF) CallSign: "));
  SerialUSB.println(hf_call);
  SerialUSB.println(F(""));
  
}

void loop() {
  Watchdog.reset();
  updateStatusLED();

  if (((readBatt() > BattMin) && GpsFirstFix) || ((readBatt() > GpsMinVolt) && !GpsFirstFix)) {

    if (aliveStatus) {

      sendStatus();
      aliveStatus = false;

      while (readBatt() < BattMin) {
        sleepSeconds(BattWait); 
      }   
    }
    
      updateGpsData(1000);
      gpsDebug();

      if(gps.location.isValid() && gps.location.age()<1000){
        GpsInvalidTime=0;
        setStatusLEDBlinkCount(LED_STATUS_GPS_FIX);
      }else{
        GpsInvalidTime++;
        if (gps.date.year() != 2000) setStatusLEDBlinkCount(LED_STATUS_GPS_TIME);
        else setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
        if(GpsInvalidTime > GpsResetTime){
          GpsOFF; 
          ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
          Watchdog.reset();
          delay(1000);
          GpsON;
          GpsInvalidTime=0;     
        }
      }

      if ((gps.location.age() < 1000 || gps.location.isUpdated()) && gps.location.isValid()) {
        if (gps.satellites.isValid() && gps.satellites.value() > 3) {
          GpsFirstFix = true;
          if(readBatt() < HighVolt){
             GpsOFF; 
             ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
          }      
          GpsInvalidTime=0;

          // Checks if there is an HF (WSPR) TX window is soon
          if (!((minute() % 10 == 3 || minute() % 10 == 7) &&  second()>50 && readBatt() > WsprBattMin && timeStatus() == timeSet)){
            updateTelemetry();
            freeMem();
            SerialUSB.flush();

          }   

          // FIX! it should depend on the channel starting minute - 1 (modulo 10)
          // preparations for HF starts one minute before TX time at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57. 
          printf("timeStatus():%u minute():%u\n", timeStatus(), minute());
          if (readBatt() > WsprBattMin && timeStatus() == timeSet && ((minute() % 10 == 3) || (minute() % 10 == 7)) ) { 
            printf("start WSPR\n");
            GridLocator(hf_loc, gps.location.lat(), gps.location.lng());
            sprintf(hf_message,"%s %s",hf_call,hf_loc);
            
            #if defined(DEVMODE)
            SerialUSB.println(F("Digital HF Mode Preparing"));
            SerialUSB.print(F("Grid Locator: "));
            SerialUSB.println(hf_loc);
            #endif
            
            //HF transmission starts at minute 4, 8, 14, 18, 24, 28, 34, 38, 44, 48, 54 or 58 
            // FIX! start on hte starting minute of the channel
            while (((minute() % 10 != 4) || (minute() % 10 != 8)) && second() != 0) {
              Watchdog.reset();
              delay(1);
              updateStatusLED();
            }
            #if defined(DEVMODE)
            SerialUSB.println(F("Digital HF Mode Sending..."));
            #endif          
            setStatusLEDBlinkCount(LED_STATUS_TX_WSPR);
            encode();
            //HFSent=true;

            #if defined(DEVMODE)
            SerialUSB.println(F("Digital HF Mode Sent"));
            #endif             
            setStatusLEDBlinkCount(LED_STATUS_NO_GPS);
  
          } else {
            sleepSeconds(BeaconWait);
          }   
        }else {
          #if defined(DEVMODE)
          SerialUSB.println(F("Not enough satelites"));
          #endif
        }
      }
    
  } else {
    sleepSeconds(BattWait);
  }
}

void sleepSeconds(int sec) {
  Si5351OFF;
  SerialUSB.flush();
  for (int i = 0; i < sec; i++) {
    if (GpsFirstFix){ //sleep gps after first fix
      if (readBatt() < HighVolt){
        GpsOFF;
        ublox_high_alt_mode_enabled = false;
      }
    }else{
      if (readBatt() < BattMin){
        GpsOFF;
        ublox_high_alt_mode_enabled = false;
      }
    } 
     
    Watchdog.reset();

    uint32_t usec = time_us_32();
    while ((time_us_32() - usec) < 1000000) {
      updateStatusLED();
    }
    
  }
  Watchdog.reset();
}


void updatePosition(int high_precision, char *dao) {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  RawDegrees rawDeg = gps.location.rawLat();
  uint32_t min_nnnnn;
  char lat_dao = 0;
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100)*5)) ) {
    // round up. Avoid overflow (59.999999 should never become 60.0 or more)
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100)*5;
  }
  sprintf(latStr, "%02u%02u.%02u%c", (unsigned int ) (rawDeg.deg % 100), (unsigned int ) ((min_nnnnn / 100000) % 100), (unsigned int ) ((min_nnnnn / 1000) % 100), rawDeg.negative ? 'S' : 'N');
  if (dao)
    dao[0] = (char) ((min_nnnnn % 1000) / 11) + 33;

  // FIX! what is this
  // APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  rawDeg = gps.location.rawLng();
  min_nnnnn = rawDeg.billionths * 0.006;
  if ( ((min_nnnnn / (high_precision ? 1 : 100)) % 10) >= 5 && min_nnnnn < (6000000 - ((high_precision ? 1 : 100)*5)) ) {
    min_nnnnn = min_nnnnn + (high_precision ? 1 : 100)*5;
  }
  sprintf(lonStr, "%03u%02u.%02u%c", (unsigned int ) (rawDeg.deg % 1000), (unsigned int ) ((min_nnnnn / 100000) % 100), (unsigned int ) ((min_nnnnn / 1000) % 100), rawDeg.negative ? 'W' : 'E');
  if (dao) {
    dao[1] = (char) ((min_nnnnn % 1000) / 11) + 33;
    dao[2] = 0;
  }

  // FIX! what is this?
  // APRS_setLon(lonStr);
  // APRS_setTimeStamp(gps.time.hour(), gps.time.minute(),gps.time.second());
}


void updateTelemetry() {
  sprintf(telemetry_buff, "%03d", gps.course.isValid() ? (int)gps.course.deg() : 0);
  telemetry_buff[3] = '/';

  sprintf(telemetry_buff + 4, "%03d", gps.speed.isValid() ? (int)gps.speed.knots() : 0);
  telemetry_buff[7] = '/';

  telemetry_buff[8] = 'A';
  telemetry_buff[9] = '=';
  //sprintf(telemetry_buff + 10, "%06lu", (long)gps.altitude.feet());

  //fixing negative altitude values causing display bug on aprs.fi
  float tempAltitude = gps.altitude.feet();

  if (tempAltitude>0){
    //for positive values
    sprintf(telemetry_buff + 10, "%06lu", (long)tempAltitude);
  } else{
    //for negative values
    sprintf(telemetry_buff + 10, "%06d", (long)tempAltitude);
    } 
  
  telemetry_buff[16] = ' ';
  sprintf(telemetry_buff + 17, "%03d", TxCount);
  telemetry_buff[20] = 'T';
  telemetry_buff[21] = 'x';
  telemetry_buff[22] = 'C';
  Si5351ON;//little hack to prevent a BMP180 related issue 
  delay(1);

  telemetry_buff[23] = ' '; float tempC = bmp.readTemperature();
  // telemetry_buff[23] = ' '; float tempC = 0.f;
  dtostrf(tempC, 6, 2, telemetry_buff + 24);

  telemetry_buff[30] = 'C';
  telemetry_buff[31] = ' '; float pressure = bmp.readPressure() / 100.0; //Pa to hPa
  // telemetry_buff[31] = ' '; float pressure = 0.f; //Pa to hPa
  dtostrf(pressure, 7, 2, telemetry_buff + 32);

  Si5351OFF; 

  telemetry_buff[39] = 'h';
  telemetry_buff[40] = 'P';
  telemetry_buff[41] = 'a';
  telemetry_buff[42] = ' ';

  dtostrf(readBatt(), 5, 2, telemetry_buff + 43);
  telemetry_buff[48] = 'V';
  telemetry_buff[49] = ' ';

  sprintf(telemetry_buff + 50, "%02d", gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  telemetry_buff[52] = 'S';
  telemetry_buff[53] = ' ';
  sprintf(telemetry_buff + 54, "%s", comment);   
  // remove temperature and pressure info
  // memmove(&telemetry_buff[24], &telemetry_buff[43], (sizeof(telemetry_buff) - 43));

#if defined(DEVMODE)
  SerialUSB.println(telemetry_buff);
#endif

}

void sendLocation() {

#if defined(DEVMODE)
  SerialUSB.println(F("Location sending with comment"));
#endif

  // FIX! how do we end the telemetry_buff
  int GEOFENCE_APRS_frequency=0;

  // vfo_set_drive_strength(APRS_TX_CLK_NUM, SI5351A_CLK_IDRV_8MA);
  // vfo_turn_on(APRS_TX_CLK_NUM);
  // vfo_set_freq_x16(APRS_TX_CLK_NUM, (GEOFENCE_APRS_frequency << PLL_CALCULATION_PRECISION));
  {
    delay(500);
    // FIX! how do we end the telemetry_buff
    // APRS_sendLoc(telemetry_buff);
    delay(10);
    vfo_turn_off();
    SerialUSB.print(F("APRS Location sent (Freq: "));
    SerialUSB.print(GEOFENCE_APRS_frequency);
    SerialUSB.print(F(") - "));    
    SerialUSB.println(TxCount);
    TxCount++;
  }

}

void sendStatus() {
  // FIX! how do we end the telemetry_buff
  int GEOFENCE_APRS_frequency=0;

  // vfo_set_drive_strength(APRS_TX_CLK_NUM, SI5351A_CLK_IDRV_8MA);
  // vfo_turn_on(APRS_TX_CLK_NUM);
  // vfo_set_freq_x16(APRS_TX_CLK_NUM, (GEOFENCE_APRS_frequency << PLL_CALCULATION_PRECISION));
  {
    delay(500);
    // APRS_sendStatus(StatusMessage);
    delay(10);
    vfo_turn_off();
    SerialUSB.print(F("Status sent (Freq: "));
    SerialUSB.print(GEOFENCE_APRS_frequency);
    SerialUSB.print(F(") - "));
    SerialUSB.println(TxCount);
    TxCount++;
  }

}

static void updateGpsData(int ms)
{
  Watchdog.reset();
  GpsON;
  while (!Serial) {delay(1);} // wait for serial port to connect.  

  // FIX! do we need any config of the ATGM336?
  if(!ublox_high_alt_mode_enabled){
    //enable ublox high altitude mode
    setGPS_DynamicModel6();
    #if defined(DEVMODE)
      SerialUSB.println(F("ublox DynamicModel6 enabled..."));
    #endif      
    ublox_high_alt_mode_enabled = true;      
  }
  
  unsigned long start = millis();
  unsigned long bekle=0;
  do
  {

    while (Serial2.available()>0) {
      char c;
      c=Serial2.read();
      gps.encode(c);
      bekle= millis();
    }
    
    if (bekle!=0 && bekle+10<millis())break;
    updateStatusLED();
  } while (millis() - start < ms);

  #if defined(DEVMODE2)
  printf("gps.time.isValid():%u\n", gps.time.isValid());
  #endif
  if (gps.time.isValid())
  {
    // kevin 11_6_24 0 instead of NULL (not pointer)
    // causes problems with the routines declares?
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), 0, 0, 0);     
    #if defined(DEVMODE2)
    printf("setTime(%02u:%02u:%02u)\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    #endif
  }
}

//following GPS code from : https://github.com/HABduino/HABduino/blob/master/Software/habduino_v4/habduino_v4.ino
void setGPS_DynamicModel6()
{  
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
  0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };

  while(!gps_set_sucess)
  {
    #if defined(DEVMODE)
      SerialUSB.println(F("ublox DynamicModel6 try..."));
    #endif 
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial2.write(0xFF);
  delay(500);
  for(int i=0; i<len; i++) {
    Serial2.write(MSG[i]);
  }
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  boolean status =false;

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      status= true;
      break;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      status= false;
      break;
    }

    // Make sure data is available to read
    if (Serial2.available()) {
      b = Serial2.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
  return status;
}

void gpsDebug() {
#if defined(DEVMODE)
  SerialUSB.println();
  SerialUSB.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars Sentences Checksum"));
  SerialUSB.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  RX    RX        Fail"));
  SerialUSB.println(F("-----------------------------------------------------------------------------------------------------------------"));

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  SerialUSB.println();

#endif
}

static void printFloat(float val, bool valid, int len, int prec)
{
#if defined(DEVMODE)
  if (!valid)
  {
    while (len-- > 1)
      SerialUSB.print('*');
    SerialUSB.print(' ');
  }
  else
  {
    SerialUSB.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      SerialUSB.print(' ');
  }
#endif
}

static void printInt(unsigned long val, bool valid, int len)
{
#if defined(DEVMODE)
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  SerialUSB.print(sz);
#endif
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
#if defined(DEVMODE)
  if (!d.isValid())
  {
    SerialUSB.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    SerialUSB.print(sz);
  }

  if (!t.isValid())
  {
    SerialUSB.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    SerialUSB.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
#endif
}

static void printStr(const char *str, int len)
{
#if defined(DEVMODE)
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    SerialUSB.print(i < slen ? str[i] : ' ');
#endif
}


float readBatt() {
  int adc_val = 0;
  adc_val = analogRead(BattPin);
  adc_val += analogRead(BattPin);
  adc_val += analogRead(BattPin);
  // The Raspberry Pi Pico's analog to digital converter (ADC) can measure voltages between 0 and 3.3 volts. 
  // The ADC uses a 3.3V reference voltage, 
  // and a read operation returns a number between 0 and 4095. 
  // The ADC's resolution is 3.3/4096, or roughly 0.8 millivolts. 
  // is the precision set to 4096? (12 not 16 bits resolution)
  // 4096/3.3 = 1241
  // 1241 / 3 = 413.66

  // FIX! this doesn't seem right. should I just multiply by the conversion factor
  // he's got some special 1/3 voltage divider for VBUS to BATT_V
  // you leave it open than the ADC converter voltage reference is the 3.3V .
  // In reality it is the voltage of the pin 3V3 - ( ~150uA * 200) which is roughly a 30mv drop. (0.8mv * 30 = 24 steps)

  // this must be a calibrated linear equation? only need to calibrate between 2.8v and 5v?
  float solar_voltage = ((float)adc_val / 3.0f - 27.0f) / 412.0f;
  // there is a 200 ohm resistor between 3V3 and ADC_AVDD
  // we did 3 reads above ..averaging? so don't need the 3x because of onboard voltage divider
  // pico-WSPRer does this (no use of ADC_AVDD) ?
  // const float conversionFactor = 3.3f / (1 << 12);  
  // float solar_voltage = 3 * (float)adc_read() * conversionFactor;   

  // if (solar_voltage < 0.0f) solar_voltage = 0.0f;
  // if (solar_voltage > 9.9f) solar_voltage = 9.9f;
  return solar_voltage;
}

#define WSPR_PWM_SLICE_NUM  4
static pwm_config wspr_pwm_config;
void PWM4_Handler(void) {
  pwm_clear_irq(WSPR_PWM_SLICE_NUM);
  static int cnt = 0;
  if (++cnt >= 500) {
    cnt = 0;
    proceed = true;
  }
}

void zeroTimerSetPeriodMs(float ms){
  wspr_pwm_config = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&wspr_pwm_config, 250); // 2uS
  pwm_config_set_wrap(&wspr_pwm_config, ((uint16_t)ms - 1));
  pwm_init(WSPR_PWM_SLICE_NUM, &wspr_pwm_config, false);

  irq_set_exclusive_handler(PWM_IRQ_WRAP, PWM4_Handler);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_clear_irq(WSPR_PWM_SLICE_NUM);
  pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, true);
  pwm_set_enabled(WSPR_PWM_SLICE_NUM, true);
}

void encode()
{
  Watchdog.reset();
  vfo_set_drive_strength(WSPR_TX_CLK_0_NUM, SI5351A_CLK_IDRV_8MA);
  vfo_set_drive_strength(WSPR_TX_CLK_1_NUM, SI5351A_CLK_IDRV_8MA);
  vfo_turn_on(WSPR_TX_CLK_NUM);
  // do we have to turn on the differential clock?
  // they share a pll ? so the differential enable ifdef should handle that?
  // vfo_turn_on(WSPR_TX_CLK_1_NUM);
  uint8_t i;

  #if defined(DEVMODE2)
  printf("cur_mode:%u\n", cur_mode);
  #endif
  switch(cur_mode)
  {
  case MODE_WSPR:
    hf_freq = WSPR_DEFAULT_FREQ - 100 + (rand() % 200);

    #if defined(DEVMODE2)
    printf("WSPR freq: %u\n", hf_freq);
    #endif

    symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;
    break;
  }
  set_tx_buffer();
  zeroTimerSetPeriodMs(tone_delay); 
  
  for(i = 0; i < symbol_count; i++)
  {
      uint32_t freq_x16 = (hf_freq << PLL_CALCULATION_PRECISION) + (tx_buffer[i] * (12000L << PLL_CALCULATION_PRECISION) + 4096) / 8192L;
      // printf("%s vfo_set_freq_x16(%u)\n", __func__, (freq_x16 >> PLL_CALCULATION_PRECISION));
      vfo_set_freq_x16(WSPR_TX_CLK_NUM, freq_x16);
      proceed = false;

      while (!proceed) {
        updateStatusLED();
      }
      Watchdog.reset();
  }
  pwm_set_enabled(WSPR_PWM_SLICE_NUM, false);
  pwm_set_irq_enabled(WSPR_PWM_SLICE_NUM, false);
  pwm_clear_irq(WSPR_PWM_SLICE_NUM);
  irq_set_enabled(PWM_IRQ_WRAP, false);
  irq_remove_handler(PWM_IRQ_WRAP, PWM4_Handler);
  vfo_turn_off();
  Watchdog.reset();
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  case MODE_WSPR:
    jtencode.wspr_encode(hf_call, hf_loc, dbm, tx_buffer);
    break;
  }
}

void GridLocator(char *dst, float latt, float lon) {
  int o1, o2;
  int a1, a2;
  float remainder;
  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (float)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  // latitude
  remainder = latt + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (float)a1 * 10.0;
  a2 = (int)(remainder);

  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)0;
}

void freeMem() {
#if defined(DEVMODE)
  SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
#endif

}
