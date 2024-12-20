// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024
// See acknowledgements.txt for the lengthy list of contributions/dependencies.

// https://arduino-pico.readthedocs.io/en/latest/usb.html

//********************************************
// what do we need here
#include <stdint.h>

// stdlib https://www.tutorialspoint.com/c_standard_library/stdlib_h.htm
// #include <stdlib.h>
// #include <string.h>
// #include <cstdio.h>

#include "debug_functions.h"
#include "print_functions.h"
#include "led_functions.h"
#include "keyboard_functions.h"
#include <Adafruit_SleepyDog.h>  // https://github.com/adafruit/Adafruit_SleepyDog

// decode of _verbose 0-9
extern bool VERBY[10];
extern bool BALLOON_MODE;

//***********************************************************
char drainSerialTo_CRorNL(uint32_t millis_max) {
    // Support hitting <enter> frantically to get to config menu right away on boot
    // this approximates 100 millis per loop iteration
    int max_iter = millis_max / 100;
    char incomingByte = 0;
    Watchdog.reset();
    if (!BALLOON_MODE) {
        int i;
        for (i = 0; i <= max_iter ; i++) {
            if (!Serial.available()) {
                updateStatusLED();
            } else {
                incomingByte = Serial.read();
                V1_println(incomingByte);
                // FIX! 13 is ascii CR \r.
                // FIX! 10 is ascii LF \n.
                // we don't drain past CR/LF.
                // If you hit enter, stuff after that stays as input
                // readStringUntil() reads characters from the serial buffer into a String.
                // The function terminates if it times out (see setTimeout()).
                // Serial.setTimeout() sets the maximum milliseconds to wait for serial data.
                // defaults to 1000 milliseconds.

                if (incomingByte != 13) {
                    Serial.setTimeout(100);
                    Serial.readStringUntil(13);  // empty readbuffer if there's data
                }
                if (incomingByte == 13) {
                    V1_println(F("Found Serial incomingByte == 13 (CR)..will not drain the rest"));
                    // what happens if there is \r\n...
                    // I guess it will go to the setup menu with the \n
                    break;
                }
                if (incomingByte == 10) {
                    V1_println(F("Found Serial incomingByte == 10 (LF)..will not drain the rest"));
                    break;
                }
            }
            if (incomingByte == 10 || incomingByte == 13) break;
            Watchdog.reset();
            sleep_ms(100);
        }
        if (i > max_iter) {
            // V1_println("Must have timed out looking for input char(s) on Serial");
        }
    }

    // will always return single byte '0' if none found
    // otherwise return single byte char
    return incomingByte;
}

//***********************************************************
char getOneChar(uint32_t millis_max) {
    // this approximates 1000 millis per loop iteration
    int max_iter = millis_max / 100;
    char incomingByte = 0;
    Watchdog.reset();
    if (!BALLOON_MODE) {
        int i;
        for (i = 0; i <= max_iter ; i++) {
            if (!Serial.available()) {
                updateStatusLED();
            } else {
                incomingByte = Serial.read();
                // keep waiting if CR or LF
                if (incomingByte != 13 && incomingByte != 10) {
                    V1_println(incomingByte);
                    break;
                }
            }
            Watchdog.reset();
            sleep_ms(100);
        }

        if (i > max_iter) {
            V1_println("Must have timed out looking for input char on Serial");
        }
    }
    // will always return single byte '0' if none found
    // otherwise return single byte char
    return incomingByte;
}


//***********************************************************
uint32_t get_sie_status(void) {
    // Get the SIE_STATUS to see if we're connected or what?
    // why am I not getting bit 16 when connected?

    // see bottom of tracker.ino for details about memory mapped usb SIE_STATUS register
    // 0x00010000 [16]    CONNECTED    (0) Device: connected
    // 0x00000010 [4]     SUSPENDED    (0) Bus in suspended state
    // 0x0000000c [3:2]   LINE_STATE   (0x0) USB bus line state
    // 0x00000001 [0]     VBUS_DETECTED (0) Device: VBUS Detected
    #define sieStatusPtr ((uint32_t*)0x50110050)
    uint32_t sieValue = *sieStatusPtr;

    // FIX! why is it 0? we seem to have the right addr?
    // bool usbConnected = Serial && ((sieValue && 0x0001000) != 0) ;
    // https://forum.arduino.cc/t/solved-serialusb-checking-if-connection-is-still-present/582448/3

    return sieValue;
}

//***********************************************************
bool get_sie_connected(void) {
    uint32_t sieValue = get_sie_status();
    bool usbConnected = Serial && ((sieValue && 0x0000001) == 0x1);
    return usbConnected;
}

//***********************************************************
//
// https://stackoverflow.com/questions/74323515/how-can-a-usb-device-tell-if-it-is-connected-to-a-port
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2040/hardware_structs/include/hardware/structs/usb.h


// In the RP2040 datasheet,
// the USB device connectivity status is bit 16 of the SIE_STATUS register.
// Serial Interface engine
// From section 4.1.4:
// The USB registers start at a base address of 0x50110000
// (defined as USBCTRL_REGS_BASE in SDK).
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2040/hardware_regs/include/hardware/regs/addressmap.h

// In the following table, we find that the SIE status register is at offset 0x50.
// So to find out if the device is connected,
// read the 32-bit register with (micropython machine.mem32),
// mask out bit 16 and cast the result to a boolean.
// https://sourcevu.sysprogs.com/rp2040/picosdk/symbols/usb_hw_t
// https://lorenz-ruprecht.at/docu/pico-sdk/1.4.0/html/group__hardware__base.html
// memory mapped hardware register?
// https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_base/include/hardware/address_mapped.h

// #define sieStatus ((uint32_t*)0x50110050)
// uint32_t value = *sieStatus


// include "hardware/usb.h"
//  io_rw_32 sie_status;
//    _REG_(USB_SIE_STATUS_OFFSET) // USB_SIE_STATUS
//     // SIE status register
//     // 0x80000000 [31]    DATA_SEQ_ERROR (0) Data Sequence Error
//     // 0x40000000 [30]    ACK_REC      (0) ACK received
//     // 0x20000000 [29]    STALL_REC    (0) Host: STALL received
//     // 0x10000000 [28]    NAK_REC      (0) Host: NAK received
//     // 0x08000000 [27]    RX_TIMEOUT   (0) RX timeout
//     // 0x04000000 [26]    RX_OVERFLOW  (0) RX overflow
//     // 0x02000000 [25]    BIT_STUFF_ERROR (0) Bit Stuff Error
//     // 0x01000000 [24]    CRC_ERROR    (0) CRC Error
//     // 0x00080000 [19]    BUS_RESET    (0) Device: bus reset received
//     // 0x00040000 [18]    TRANS_COMPLETE (0) Transaction complete
//     // 0x00020000 [17]    SETUP_REC    (0) Device: Setup packet received
//     // 0x00010000 [16]    CONNECTED    (0) Device: connected
//     // 0x00000800 [11]    RESUME       (0) Host: Device initiated a remote resume
//     // 0x00000400 [10]    VBUS_OVER_CURR (0) VBUS over current detected
//     // 0x00000300 [9:8]   SPEED        (0x0) Host: device speed
//     // 0x00000010 [4]     SUSPENDED    (0) Bus in suspended state
//     // 0x0000000c [3:2]   LINE_STATE   (0x0) USB bus line state
//     // 0x00000001 [0]     VBUS_DETECTED (0) Device: VBUS Detected

// io_rw_32 sie_status;
