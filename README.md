# LightAPRS-W-2.0 ported to sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1

[This branch](https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker) is a ported version of [LightAPRS-W-2.0](https://github.com/lightaprs/LightAPRS-W-2.0) for [sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1).

[sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1) is an open-hardware project intended for STEM (Science, Technology, Engineering and Mathematics) educational purposes.

## Differences

There are several differences between **LightAPRS-W 2.0 Tracker** and **sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1**:

|   |LightAPRS-W 2.0 Tracker|sf-hab.org RP2040 based Tracker gen1|
|---|---|---|
|**Weight**|4.61 g|3.94 g (3.57 g after cut-out USB connector portion, with GPS antenna wires)|
|**Size**|32 mm x 55 mm|29 mm x 55 mm (48 mm after cut-out USB connector portion)|
|**MCU**|Microchip ATSAMD21G18 (ARM Cortex-M0)|Raspberry Pi RP2040 (ARM Cortex-M0+ dual core)|
|**Flash**|256 KB (internal)|2 MB (external)|
|**RAM**|32 KB|264 KB|
|**MCU Clock Freq.**|48 MHz|125 MHz|
|**VHF Radio Module**|Si4463 (Max 100 mWatt)|Si5351A-B-GT / MS5351M (Max 10 mWatt)|
|**VHF LPF**|Available (7 elements)|5 elements (separate from HF LPF)|
|**HF Radio Module**|Si5351A-B-GT (Max 10mWatt)|Si5351A-B-GT / MS5351 (Max 10 mWatt)|
|**HF LPF**|No|5 elements (separate from VHF LPF)|
|**Sensor**|BMP180 (pressure and temperature)|BMP280 (pressure and temperature)|

The original LightAPRS-W 2.0 Tracker uses two different chips (Si5351A and Si4463) for supporting both HF and VHF bands, but my ported version only uses the Si5351A-B-GT/MS5351M to generate both HF (WSPR in 20m band) and VHF (APRS in 2m band).

The source code for controlling the Si5351A/MS5351M chip is ported from my [AFSK_to_FSK_VFO](https://github.com/kaduhi/AFSK_to_FSK_VFO) repository, it was developed originally for [**QRPGuys AFP-FSK Digital Transceiver III kit**](https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii) ( [source code](https://qrpguys.com/wp-content/uploads/2022/09/ft8_v1.4_092522-1.zip) ).

## How to compile & build the source code

Same as the LightAPRS-W-2.0, you need to use [Arduino IDE](https://www.arduino.cc/en/Main/Software) to compile & build this project.

### 1. Install Arduino IDE

Download and install [Arduino IDE](https://www.arduino.cc/en/Main/Software). If you have already installed Arduino, please check for updates. Its version should be at least v2.3.1 or newer.

### 2. Configure Board

- Open the **Tools > Board > Boards Manager...** menu item.
- Type "raspberry pi pico" in the search bar until you see the **Raspberry Pi Pico - Arduino Mbed OS RP2040 Boards** entry and click on it.
  - For more details, please refer to [Arduino-Pico documentation](https://arduino-pico.readthedocs.io/en/latest/index.html).
  - If you use Arduino IDE v1.*, please refer to [here](https://arduino-pico.readthedocs.io/en/latest/install.html#installing-via-arduino-boards-manager).
- Click **Install** .
- After installation is complete, close the **Boards Manager** window.
- Open the **Tools > Board** menu item and select **Raspberry Pi Pico - Arduino Mbed OS RP2040 Boards** from the the list.
- Open the **Tools** menu again to select values below:
  - Debug Level: "None"
  - Debug Port: "Serial"
  - C++ Exceptions: "Disabled"
  - Flash Size: "2MB (no FS)"
  - CPU Speed: "125MHz"
  - IP/Bluetooth Stack: "IPv4 Only"
  - Optimize: "Small (-Os) (standard)"
  - RTTI: "Disabled"
  - Stack Protection: "Disabled"
  - Upload Method: "Default (UF2)"
  - USB Stack: "Pico SDK"

### 3. Copy Libraries & Compile Source Code 

- First download the repository to your computer using the green "[clone or download](https://github.com/kaduhi/LightAPRS-W-2.0/archive/refs/heads/port_to_ag6ns_rp2040_picoballoon_tracker.zip)" button.
- There are only one Arduino projects "[LightAPRS-W-2-pico-balloon](LightAPRS-W-2-pico-balloon)" folder.
- You will notice some folders in the "libraries" folder. You have to copy these folders (libraries) into your Arduino libraries folder on your computer. Path to your Arduino libraries:
- **Windows** : This PC\Documents\Arduino\libraries\
- **Mac** : /Users/\<username\>/Documents/Arduino/libraries/
- **Ubuntu** : /Users/\<username\>/Arduino/libraries/

**IMPORTANT :** If you already have folders that have same name, you still need to overwrite them. Otherwise you get a compile error.

- Then open the LightAPRS-W-2-pico-balloon.ino file with Arduino IDE and change your settings (Callsign, SSID, comment, etc.)
- Click **Verify** (If you get compile errors, check the steps above)

### 4. Upload

- First attach an VHF antenna (at least 50cm monopole wire) to your tracker. Radio module may be damaged when not attaching an antenna, since power has nowhere to go. 
- Connect sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1 board to your computer with a micro USB cable, then you should see a COM port under **Tools->Port** menu item. Select that port. (e.g. "/dev/cu.usbmodem141101")
- Click **Upload**
- If you see an error, you may need to put the tracker board into "Bootloader" mode before uploading:
  - Disconnect the USB cable
  - While shorting the H5 (two thru-hole next to the USB connector, maybe with pins or tweezers), connect the USB cable
  - After your PC recognized the board, release the shorting

## Questions?

I will only answer questions from people who are a part of a STEM education program (student, teacher, mentor, advisor, ...), please send them via email (*my call sign* @ arrl.net).
The answers to these questions will also be added to the [Wiki](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/wiki) for the benefit of all other STEM education groups.

**Note:** ***I will most likely ignore all emails from people who are not part of a STEM education program. If you are not a part of a STEM education program, please do not waste your time sending any questions.***

## Project Background and History
**Oct 2021** - attend to [SF-HAB (San Francisco Bay Area High Altitude Balloon) group](https://sf-hab.org/)'s Amateur Radio Pico Balloon presentation at Pacificon 2021, then joined the group

**Dec 2021** - start writing firmware for existing W6MRR V6.6 Pico Balloon Tracker boards

**Oct 2022** - at Pacificon 2022, meet a group of people from San Diego doing Pico Balloon / Ocean Buoy STEM educational programs for local high school students. They are looking for a new Tracker board that is specialized for their STEM education programs. They mentioned about the idea of using RP2040 as a controller chip, run tracker software in 1st CPU Core and MicroPython in 2nd Core for students to customize / extend the tracker functionalities. The tracker should be open source and open hardware, and not expensive

**Jan 2023** - during a SF-HAB online meeting, I was assigned to design a RP2040 based next generation Pico Balloon Tracker board

**Jan 2023** - start designing a tracker board

**Feb 2023** - order and receive the v0.1 prototype boards

**Mar 2023** - update design, order and receive the v0.2 prototype boards

**Apr 2023** - launch the v0.2 tracker [AG6NS-11](https://amateur.sondehub.org/#!mt=Mapnik&mz=8&qm=366d&f=AG6NS-11&q=AG6NS-11) from Hayward California, flown for 12 days then stop working above Iran

**May 2023** - launch another v0.2 tracker [K6EAU-11](https://amateur.sondehub.org/#!mt=Mapnik&mz=8&qm=366d&f=K6EAU-11&q=K6EAU-11) from Milpitas California, flown for 77.9 days (2.7 circumnavigations)

**May 2023** - update design, order and receive the v0.3 prototype boards

**Jun 2023** - launch the v0.3 tracker [W6MRR-27](https://amateur.sondehub.org/#!mt=Mapnik&mz=8&qm=366d&f=W6MRR-27&q=W6MRR-27) from Milpitas California, flown for 1.5 days (accumulated ice destroyed the balloon? altitude dropped from 13km to 3km during night, then slowly back to 13km)

**Jun 2023** - launch another v0.3 tracker [AG6NS-12](https://amateur.sondehub.org/#!mt=Mapnik&mz=11&qm=366d&f=AG6NS-12&q=AG6NS-12) from Milpitas California, flown for 1 day

**Jul 2023** - launch another v0.3 tracker [AG6NS-13](https://amateur.sondehub.org/#!mt=Mapnik&mz=8&qm=366d&f=AG6NS-13&q=AG6NS-13) from Milpitas California, flown for 1 day (balloon failure, landed in Mexico then keep transmitting signal for 3 days)

**Jul 2023** - update design, order and receive the v0.3.1 prototype boards

**Aug 2023** - launch another v0.3 tracker [WB6TOU-14](http://lu7aa.org/wsprx.asp?banda=20m&other=wb6tou&balloonid=q8&timeslot=8&repito=on&wide=&detail=&SSID=14&launch=20230808170001&tracker=wb8elk) from Lodi California, as of Mar 30 2024 **still flying for 234 days.** It requires high sun angle due to the tiny single solar cell therefore reports are limited during winter.

**Sep 2023** - launch the v0.3.1 tracker [AG6NS-14](https://amateur.sondehub.org/#!mt=Mapnik&mz=8&qm=366d&f=AG6NS-14&q=AG6NS-14) from Milpitas, California, flown for 1 day (balloon failure, landed near Mono Lake then keep transmitting signal for 53 days)

**Oct 2023** - launch another v0.3.1 tracker [AG6NS-15](https://amateur.sondehub.org/#!mt=Mapnik&mz=8&qm=366d&f=AG6NS-15&q=AG6NS-15) from Milpitas California, as of Mar 30 2024 **still flying for 174 days and completed 16.8 circumnavigations**

**Nov 2023** - release an **Ocean Buoy** with the v0.3 tracker [KQ6RS-12](https://aprs.fi/#!call=a%2FKQ6RS-12&timerange=604800&tail=604800) at 420 miles WSW of San Diego California (from a boat), as of Mar 30 2024 **still floating on Pacific Ocean for 144 days**

#

Kazuhisa "Kazu." Terasaki, AG6NS

if you are insterested in, [here](https://www.instagram.com/kazuterasaki/) is my latest updates
