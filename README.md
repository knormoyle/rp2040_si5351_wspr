# Origins
This is a fork by AD6Z of the Kazu AG6NS fork of LightAPRS-2.0 for Kazu's [sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1).
The shorthand name for this pcb is AG6NS 0.4 pcb. The bom and cpl files (BOM* and PickAndPlace* xlsx) were modified to remove the low pass filters and vhf output, and have full differential tx for the WSPR RF. There are multiple choices due to experiments. Contact me to clarify which to use.

[sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1) is an open-hardware project.

The pcb used for this firmware is currently at [AD6Z tracker](https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/tree/main/pcb/tracker/v0.4_kbn)
(v0.4_kbn dir). There are multiple BOM and CPL files for jlcpcb.com there. Contact me by email to find out which you should use if interested in building some at jlcpcb.com.
Schematic and board png from jlcpcb are there also. The schematic has a buck/boost converter that's not used, some supercaps that are not used, an the LPFs on the si5351a clk0/clk1 have been changed in the bom/cpl files. But the schematic is pretty usable for understanding what's going on.. There are some datasheets for parts there also. Alternatives and other things being investigated.

## PCB highlights

**sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1**:

<img src="https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/blob/main/pcb/tracker/v0.4_kbn/corrected_placement_jlcpcb.png" height="500"/>


|   |sf-hab.org RP2040 based Tracker gen1|
|---|---|
|**Weight**|3.94 g (3.57 g after cut-out USB connector portion, with GPS antenna wires)|
|**Size**|29 mm x 55 mm (48 mm after cut-out USB connector portion)|
|**MCU**|Raspberry Pi RP2040 (ARM Cortex-M0+ dual core)|
|**Flash**|2 MB (external)|
|**RAM**|264 KB|
|**MCU Clock Freq.**|18-250 MHz|
|**HF Radio Module**|Si5351A-B-GT or MS5351 (Max 10 mWatt)|
|**Sensor**|optional BMP280 (pressure and temperature)|

<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/config_screenshot.png"/>

The source code started with AG6NS port of the LightAPRS firmware to his HF + APRS SF-HAB pcb v0.4

U4B extensions were added, multiband configuration etc, etc. by AD6Z in 11/2024

## How to compile & build the source code
You need to use [Arduino IDE](https://www.arduino.cc/en/Main/Software) to compile & build this project.

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

- First download the repository to your computer using the green "[clone or download]" button.
- There are only one Arduino projects rp2040_si5351_wspr/tracker folder.
- You will notice some folders in the "libraries" folder. You have to copy these folders (libraries) into your Arduino libraries folder on your computer. Path to your Arduino libraries:
- **Windows** : This PC\Documents\Arduino\libraries\
- **Mac** : /Users/\<username\>/Documents/Arduino/libraries/
- **Ubuntu** : /Users/\<username\>/Arduino/libraries/

**IMPORTANT :** If you already have folders that have same name, you still need to overwrite them. Otherwise you get a compile error.

- Then open the tracker.ino file with Arduino IDE
- Click **Verify** (If you get compile errors, check the steps above)

### 4. Upload

- First attach a GPS antenna and a short HF antenna (6" dipole wires?) to the AG6NS 0.4 pcb with new kbn BOM/CPL files, assembled by jlcpcb.com
- Connec board to your computer with a micro USB cable, then you should see a COM port under **Tools->Port** menu item. Select that port. (e.g. "/dev/cu.usbmodem141101")
- Click **Upload**
- 
- The firmware turns USB pll off at times. If you can't upload, unplug the usb (better yet get a usb cable with switch to avoid plug/unplug cycles). Then plug it back in. The green led should be on steady. immediately do the compile upload. After that you have 15 secs to open a putty window (setup a fast way to start putty with a session defined. save output (logging) to a putty.log for later analysis/debug. You will interact with the putty window for keyboard entry (if necessary). Hit enter to interrupt to get to config screen and keyboard entry. X to leave config mode and reboot. You will have to repeat opening putty at this point because of the reboot. If there is no usb, but just power, the tracker will enter balloon mode. Also, all usb input/output will be disabled if you don't open the putty session within 15 secs of boot. There is much debug output. it can be disabled with the V command in the config (VERBY). 0 is least output. 1 is more. 2 is more than that. 3 is most so far. The printing does not interfere with wspr operations. You can set K (clock) to 18 Mhz. There is a factory reset in the config selection. It will default any bad/illegal entries in the config.
- 
- If you see an error, you may need to put the tracker board into "Bootloader" mode before uploading:
  - Disconnect the USB cable
  - While shorting the H5 (two thru-hole next to the USB connector, maybe with pins or tweezers), connect the USB cable
  - After your PC recognized the board, release the shorting

## Questions?
I (Kevin Normoyle) will try to answer all questions or email.
 

## Project Background and History (Kazu AG6NS prehistory to this fork)
**Nov 2024** - This fork initiated by Kevin Normoyle AD6Z

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
