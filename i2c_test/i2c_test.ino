//*******************************************
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdio.h"
#include "hardware/gpio.h"

#include <stdlib.h>
// #include <string.h>

#include <SPI.h>
#include <Wire.h>

#include <SPI.h>
#include <Wire.h>

#include "hardware/i2c.h"

// hmm. why is \n not sufficient?
#define EOL "\r\n"
// ascii 13
#define CR "\r"
// ascii 10
#define LF "\n"
// #define EOL "\n"

#define kHz 1000U

#define Si5351Pwr 4
#define VFO_I2C_INSTANCE i2c0
#define VFO_VDD_ON_N_PIN 4
#define VFO_I2C0_SDA_PIN 12
#define VFO_I2C0_SCL_PIN 13
#define SI5351A_I2C_ADDR 0x60
// #define VFO_I2C0_SCL_HZ 100000
#define VFO_I2C0_SCL_HZ 50000
#define SI5351A_OUTPUT_ENABLE_CONTROL 3


void setup() {
    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__gpio.html
    // https://arduino-pico.readthedocs.io/en/latest/wire.html
    // RP2040 has i2c0 and i2c1
    // have to use the pins labeled I2C0 for Wire
    // have to use the pins labeleds I2C1 for Wire1

    // https://docs.arduino.cc/language-reference/en/functions/communication/wire/
    // Wire.setSDA(VFO_I2C0_SDA_PIN);
    // Wire.setSCL(VFO_I2C0_SCL_PIN);
    // Wire.begin()

    gpio_init(VFO_I2C0_SCL_PIN);
    gpio_init(VFO_I2C0_SDA_PIN);

    gpio_init(Si5351Pwr);
    pinMode(Si5351Pwr, OUTPUT_8MA);
    digitalWrite(Si5351Pwr, LOW);
    while (!Serial) {;}
    // GPIO function selectors
    // Each GPIO can have one function selected at a time.
    // Likewise, each peripheral input (e.g. UART0 RX)
    // should only be selected on one GPIO at a time.
    // If the same peripheral input is connected to multiple GPIOs,
    // the peripheral sees the logical OR of these GPIO inputs.

    // Please refer to the datasheet for more information on GPIO function selection.

    // https://arduino-pico.readthedocs.io/en/latest/digital.html
    // default is 4mA
    // pinMode(VFO_I2C0_SCL_PIN, INPUT_PULLUP);
    // pinMode(VFO_I2C0_SDA_PIN, INPUT_PULLUP);

    // gpio_pull_up(Si5351Pwr);
    // gpio_put(Si5351Pwr, 0);
    // gpio_set_dir(Si5351Pwr, true); // true for out, false for in

    // void gpio_set_pulls(uint gpio, bool up, bool down )
    // Select up and down pulls on specific GPIO.
    // gpio GPIO number
    // up If true set a pull up on the GPIO
    // down If true set a pull down on the GPIO
    // On the RP2040, setting both pulls enables a "bus keep" function,
    // i.e. a weak pull to whatever is current high/low state of GPIO.
    // gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
    // gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

    // static void gpio_pull_up(uint gpio)
    // Set specified GPIO to be pulled up.
    // Parameters
    // gpio GPIO number


    // i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
    Serial.println("i2c_deinit() start");
    i2c_deinit(VFO_I2C_INSTANCE);
    Serial.println("i2c_deinit() complete");
    busy_wait_ms(1000);

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
    Serial.println("i2c_init() start");
    i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
    Serial.println("i2c_init() complete");
    busy_wait_ms(1000);

    gpio_pull_up(VFO_I2C0_SDA_PIN);
    gpio_pull_up(VFO_I2C0_SCL_PIN);
    gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
    i2c_set_slave_mode(VFO_I2C_INSTANCE, false, 0);
    busy_wait_ms(1000);

    // power the vfo off/on
    digitalWrite(Si5351Pwr, HIGH);
    busy_wait_ms(1000);
    digitalWrite(Si5351Pwr, LOW);
    busy_wait_ms(2000);
    // uint i2c_init ( i2c_inst_t *  i2c,
    // uint  baudrate
    // )
    // Initialise the I2C HW block. Put the I2C hardware into a known state, and enable it.
    // Must be called before other functions. By default, the I2C is configured to operate as a master.
    // The I2C bus frequency is set as close as possible to requested, and the actual rate set is returned

    // Parameters
    // i2c	Either i2c0 or i2c1
    // baudrate	Baudrate in Hz (e.g. 100kHz is 100000)

}

// https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_i2c

//****************************************************
int i2cWrite(uint8_t reg, uint8_t val) {  // write reg via i2c
    Serial.printf("i2cWrite START reg %02x val %02x" EOL, reg, val);
    // FIX! shouldn't this be local ? or does it setup data for i2cWriten
    // moved here to be local, and not static (shared) anymore
    // only need length 2!
    uint8_t i2c_buf[2];
    i2c_buf[0] = reg;
    i2c_buf[1] = val;

    // examples
    // https://github.com/raspberrypi/pico-examples/blob/master/i2c/mpu6050_i2c/mpu6050_i2c.c

    Serial.printf("i2cWrite doing i2c_write_blocking() reg %02x val %02x" EOL, reg, val);
    // Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged, no device present.

    int res = i2c_write_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false);

    Serial.print(EOL);
    if (res==PICO_ERROR_GENERIC)  Serial.printf("ERROR: i2cWrite() got bad res %d reg %02x val %02x" EOL, res, reg, val);
    else if (res==2) Serial.printf("GOOD: i2cWrite() got good res %d reg %02x val %02x" EOL, res, reg, val);
    else Serial.printf("UNEXPECTED: i2cWRite() got unexpected res %d reg %02x val %02x" EOL, res, reg, val);
    Serial.print(EOL);

    Serial.printf("i2cWrite END reg %02x val %02x" EOL, reg, val);
    return res;
}


// just reads two byte2
int i2cRead(uint8_t reg, uint8_t val) {  // read reg via i2c
    Serial.printf("i2cRead START reg %02x val %02x" EOL, reg, val);

    // FIX! shouldn't this be local ? or does it setup data for i2cWriten
    // moved here to be local, and not static (shared) anymore
    // only need length 2!
    uint8_t i2c_buf[2];
    i2c_buf[0] = reg;
    i2c_buf[1] = 254; // a fixed value that should be overwritten?

    // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
    // res = i2c_read_timeout_us(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false, 1000);
    Serial.print(F("Doing i2c_read_blocking()" EOL));

    // do you have to to write then read to get read data?
    
    // i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    // i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    // another example getting 6 bytes
    // val = 0x43;
    // i2c_write_blocking(i2c_default, addr, &val, 1, true);
    // i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus
    // attaching a bmp280 with i2c
    // https://github.com/raspberrypi/pico-examples/tree/master/i2c/bmp280_i2c

    // grab this code
    // https://github.com/raspberrypi/pico-examples/blob/master/i2c/bmp280_i2c/bmp280_i2c.c

    // raspberry pi bus scan
    // https://github.com/raspberrypi/pico-examples/blob/master/i2c/bus_scan/bus_scan.c

    // 3-axis digital accelerometer
    // https://github.com/raspberrypi/pico-examples/tree/master/i2c/mma8451_i2c



    int res = i2c_read_blocking(VFO_I2C_INSTANCE, SI5351A_I2C_ADDR, i2c_buf, 2, false);
    val = i2c_buf[1];
    Serial.print(EOL);
    if (res==PICO_ERROR_GENERIC)  
        Serial.printf("ERROR: i2cRead() got bad res %d reg %02x val %02x" EOL, res, reg, val);
    else if (res==2) 
        Serial.printf("GOOD: i2cRead() got good res %d reg %02x val %02x" EOL, res, reg, val);
    else 
        Serial.printf("UNEXPECTED: i2cRead() got unexpected res %d reg %02x val %02x" EOL, res, reg, val);
    Serial.print(EOL);


    Serial.printf("i2cRead END reg %02x val %02x" EOL, reg, val);
    return res;
}
//****************************************
void loop() {
    int i = 0;
    static uint8_t reg_addr = 15;
    while (true) {
        i++;
        Serial.printf("while loop %d start" EOL, i);
        if (i > 25) {
            Serial.print(EOL);
            Serial.print("start over at 0" EOL);
            Serial.print(EOL);
            reg_addr = 0;
            break;
        }
        int result;

        bool bad = false;
        Serial.print(EOL);
        Serial.print("i2cWrite() start" EOL);
        // result = i2cWrite(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff) ;
        result = i2cWrite(reg_addr, 0x55) ;
        Serial.print("i2cWrite() complete" EOL);
        if (result == 2) {
            Serial.printf("GOOD: result %d after i2cWrite" EOL, result);
            busy_wait_ms(3000);
        } else if (result == PICO_ERROR_GENERIC) {
            Serial.printf("ERROR: result %d after i2cWrite" EOL, result);
            bad = true;
        } else {
            Serial.printf("UNEXPECTED: result %d after i2cWrite" EOL, result);
            bad = true;
        }

        if (!bad) {
            Serial.print(EOL);
            Serial.print("i2cRead() start" EOL);
            // result = i2cRead(SI5351A_OUTPUT_ENABLE_CONTROL, 0xff) ;
            result = i2cRead(reg_addr, 0xaa) ;
            Serial.print("i2cRead() complete" EOL);

            if (result == 2) {
                Serial.printf("GOOD: result %d after i2cRead" EOL, result);
                busy_wait_ms(3000);
            } else if (result == PICO_ERROR_GENERIC) {
                Serial.printf("ERROR: result %d after i2cRead" EOL, result);
                bad = true;
            } else {
                Serial.printf("UNEXPECTED: result %d after i2cRead" EOL, result);
                bad = true;
            }
        }

        //******************************************
        if (!bad) {
            Serial.print(EOL);
            reg_addr = reg_addr + 1;
            Serial.printf("incremented reg_addr %u" EOL, reg_addr);
            Serial.print(EOL);
            break;
        }

        //******************************************
        Serial.print(EOL);
        Serial.println("i2c_deinit() start" EOL);
        i2c_deinit(VFO_I2C_INSTANCE);
        Serial.println("i2c_deinit() complete" EOL);
        busy_wait_ms(1000);

        // https://cec-code-lab.aps.edu/robotics/resources/pico-c-api/group__hardware__i2c.html
        Serial.println("i2c_init() start" EOL);
        i2c_init(VFO_I2C_INSTANCE, VFO_I2C0_SCL_HZ);
        Serial.println("i2c_init() complete" EOL);

        gpio_init(VFO_I2C0_SCL_PIN);
        gpio_init(VFO_I2C0_SDA_PIN);

        Serial.println("no pullup or pulldowns by RP2040 on both SDA and SCL.. external R on pcb")
        gpio_set_pulls(VFO_I2C0_SDA_PIN, false, false);
        gpio_set_pulls(VFO_I2C0_SCL_PIN, false, false);

        // gpio_pull_up(VFO_I2C0_SDA_PIN);
        // gpio_pull_up(VFO_I2C0_SCL_PIN);

        Serial.println("i2c_set_function() start" EOL);
        gpio_set_function(VFO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(VFO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
        i2c_set_slave_mode(VFO_I2C_INSTANCE, false, 0);
        Serial.println("i2c_set_function() complete" EOL);
        digitalWrite(Si5351Pwr, HIGH);

        // power the vfo off/on
        digitalWrite(Si5351Pwr, HIGH);
        busy_wait_ms(1000);
        digitalWrite(Si5351Pwr, LOW);
        busy_wait_ms(2000);

        Serial.print(EOL);
        Serial.printf("while loop end %d" EOL, i);

    }
}

