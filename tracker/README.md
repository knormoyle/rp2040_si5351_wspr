// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// tracker firmware:
// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

// Open source c/c++/arduino ide with arduino-pico core allows customization if you have software skills

// No auto-calibration of Si5351 Tx frequency (yet?). 
// Manual config for passing 'correction" (parts per billion) to Si5351, in case Tx Frequency is outside of U4B channel bin. 
// Traquito website can fingerprint callsign/telemetry frequency even if out of bin. 
// LU7AA website not so well (although 'wide' helps). 
// SDR can be used to report actual TX freq and correction applied to adjust.

// Any correction should be fixed: the same for all bands for a particular tracker.

// No auto-calibration of RP2040 clock frequency
// Manual config for setting rp2040 clock frequency, 115 to 250 Mhz. 115Mhz default
// (could calibrate/adjust with GPS PPS and then calibrate the Si5351 Tx frequency (signals go to RP2040 input)

//*******************************************
// Arduino IDE created by many
// Adafruit libraries created by many
// arduino-pico core https://github.com/earlephilhower/arduino-pico
// JTEncode library: https://github.com/etherkit/JTEncode
// https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/blob/main/pcb/tracker/v0.4_kbn/corrected_placement_jlcpcb.png// TinyGPSPlus library: //https://github.com/mikalhart/TinyGPSPlus 
// Time library: //https://github.com/PaulStoffregen/Time
// Si5351 programming: based on work by: Kazuhisa “Kazu” Terasaki AG6NS
// U4B telemetry protocol defined by Hans Summers G0UPL
// WSPR protocol defined by Joe Taylor K1JT

// Thanks to all authors and contributors
// Thanks to the entire WSPR RX ecosystem of spotters and database maintainers

// Thanks to the tracker websites and authors/maintainers
// http://lu7aa.org/wsprx.asp Pedro Converso LU7AA
// https://traquito.github.io/channelmap/ Doug Malnati KD2KDD
// https://amateur.sondehub.org

// Thanks for knowledge/support from everyone at https://groups.io/g/picoballoonhttps://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/blob/main/pcb/tracker/v0.4_kbn/corrected_placement_jlcpcb.png

//*******************************************
// tracker pcb: (fork from AG6NS rp2040_picoballoon_tracker_pcb_gen1j)
// https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/tree/main/pcb/tracker/v0.4
// schematic: https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/blob/main/pcb/tracker/v0.4_kbn/SCH_Schematic1_2023-09-11.pdf
// jlcpcb parts placement with '_kbn' bom/cpl https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/blob/main/pcb/tracker/v0.4_kbn/corrected_placement_jlcpcb.png

// Uses unmodified AG6NS 0.4 pcb with alternate assembly bom/cpl (Manfuctured by https://jlcpcb.com)
// that:
// 1) removes low pass Tx filters, 
// 2) depopulates all APRS support on AG6NS 0.4 pcb (or firmware)
// 3) enables differential Tx drive for higher Tx power
// 4) capacitive coupling on Tx to both HF antenna legs, and lighter dc load across legs (larger R)
// 3) optionally populates the temp/pressure sensor BMP280
// 4) uses the default LDO between solar power and 3V3. AG6NS board uses voltage monitor (resets)
// 5) User config menu over usb serial to config 
// 6) Configurable channel config, and bands 20M/17M/15M/12M/10M supported
// 7) Configurable U4B TELEN support for extra telemetry transmissions (temp/pressure from sensor, or sat counts or ...)
// 8) GPS is powered off during Si5351 Tx (external mosfet)
// 9) Si5351 is powered off during GPS acquistion/tracking (external mosfet)https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/blob/main/pcb/tracker/v0.4_kbn/corrected_placement_jlcpcb.png
// 10) clamping of speed/temp/voltage (3-4.95v) telemetry per traquito u4b-like definition.
// 11) compared to traquito: no buck/boost, just LDO. pressure/external temp sensor. Configurable extra telemetry. 
// 12) USB serial for config input and output. No need to modify software for callsign/band etc
// 13) Uses MS5351M instead of Si5351a.  Hans Summers G0UPL did a review here: https://qrp-labs.com/synth/ms5351m.html 

/* key parts withi jlpcb #
C90770   ATGM336H-5N31
C2040    RP2040
C5546    3.3V LDO
C9965    MAX809STRG
C132296  26Mhz
C83291   BMP280
C2843335 W25Q16JVUXIQ
C1509083 MS5351M
*/
