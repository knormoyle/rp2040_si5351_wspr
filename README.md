# Origins
This is a fork by AD6Z of the Kazu AG6NS fork of LightAPRS-2.0 for Kazu's [sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1).
The shorthand name for this pcb is AG6NS 0.4 pcb. The bom and cpl files (BOM* and PickAndPlace* xlsx) were modified to remove the low pass filters and vhf output, and have full differential tx for the WSPR RF. There are multiple choices due to experiments. Contact me to clarify which to use.

[sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1) is an open-hardware project. Kazu AG6NS released the pcb schematics/gerber etc and his port of LightAPRS firmware as open source. Thank you Kazu. Note there were 4 revisions of his work. I forked 0.4 of his pcb work.

The pcb used for this firmware is currently at [AD6Z tracker](https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/tree/main/pcb/tracker/v0.4_kbn)
(v0.4_kbn dir). There are multiple BOM and CPL files for jlcpcb.com there. Contact me by email to find out which you should use if interested in building some at jlcpcb.com.
Schematic and board png from jlcpcb are there also. The schematic has a buck/boost converter that's not used, some supercaps that are not used, an the LPFs on the si5351a clk0/clk1 have been changed in the bom/cpl files. But the schematic is pretty usable for understanding what's going on.. There are some datasheets for parts there also. Alternatives and other things being investigated.

## PCB highlights

**sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1 with mods**:
Note LPF has been removed. Capacitive coupling for full differential (antiphase) dipole HF antenna (for max power out of Si5351a-compatible ms5351m).
Hans G0UPL did a review of m5351m in 2021. Other reviews exist (Google). I mention that only because that part is in the current BOM, but a si5351a could just be swapped in, if desired. The part seems good, potentially less phase noise?
[ms5351m review](https://qrp-labs.com/synth/ms5351m.html)

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

### 2. The lay of the land, Arduino style
You want to be using the arduino-pico core (that's a layer on top of the raspberry pi sdk).

the base layer Raspberry Pi Pico SDK (PICO-SDK) docs are
[Pi Pico SDK](https://arduino-pico.readthedocs.io/en/latest/sdk.html)

Earle Philhower did the arduino-pico layer on top of that
[arduino-pico](https://github.com/earlephilhower/arduino-pico)

this is the arduino-pico core docs for the install
[arduino-pico install](https://arduino-pico.readthedocs.io/en/latest/install.html)

top level is here. you can peruse that to see ALL the stuff this layer gives you
[arduino-pico](https://arduino-pico.readthedocs.io/en/latest/)

### 2. Configure Board



Earle says above at the arduino-pico github how to get the arduino-pico stuff in the Arduino IDE. Open up the Arduino IDE and go to File->Preferences. In the dialog that pops up, enter the following URL in the "Additional Boards Manager URLs" field:

<img src="https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json"/>

Hit OK to close the dialog.
Go to Tools->Boards->Board Manager in the IDE
Type "pico" in the search box and select "Add":
<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/boardmanager.png"/>

I muddled thru the instructions on installing the arduino-pico core as above.....(look at the readme on his github)
<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/board_screenshot1.png"/>

You'll get to where under Tools it will look like this and you'll select the board (I've only done the setup once, so my memory is a little foggy...once you get it right it's sticky so your good). It will do some fetching and compiling if I remember right at some point

<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/board_screenshot2.png"/>

that's the board you want

here's another step-by-step

- Open the **Tools > Board > Boards Manager...** menu item.
- Type "pico" in the search bar until you see the **Raspberry Pi Pico/RP2040** entry and click on it.
  - For more details, please refer to [Arduino-Pico documentation](https://arduino-pico.readthedocs.io/en/latest/index.html).
  - Make sure you're using the latest Arduino IDE version that's available for download. Old versions had a different board manager.
    
- Click **Install** .
- After installation is complete, close the **Boards Manager** window.
- Open the **Tools > Board** menu item and select **Raspberry Pi Pico/RP2040** from the the list.
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


To give an example of where the files should be and the libraries issue:


You have to have a Arduino dir for libraries. I copied Kazu notes on where the Arduino dir is on different systems in my readme

I just created links (ln -s) from the Arduino libraries dir to the files in my repo libraries. You can just cp them though

```
kevin@pc8c:~$ cd Arduino
kevin@pc8c:~/Arduino$ cd libraries
kevin@pc8c:~/Arduino/libraries$ ls -ltr
total 4
lrwxrwxrwx 1 kevin kevin 64 Dec 12 01:08 Adafruit_BMP280_Library -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_BMP280_Library
lrwxrwxrwx 1 kevin kevin 55 Dec 12 01:08 Adafruit_BusIO -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_BusIO
lrwxrwxrwx 1 kevin kevin 56 Dec 12 01:08 Adafruit_Sensor -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_Sensor
lrwxrwxrwx 1 kevin kevin 59 Dec 12 01:08 Adafruit_SleepyDog -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_SleepyDog
lrwxrwxrwx 1 kevin kevin 59 Dec 12 01:08 Arduino-MemoryFree -> /home/kevin/rp2040_si5351_wspr/libraries/Arduino-MemoryFree
lrwxrwxrwx 1 kevin kevin 49 Dec 12 01:08 JTEncode -> /home/kevin/rp2040_si5351_wspr/libraries/JTEncode
lrwxrwxrwx 1 kevin kevin 49 Dec 12 01:08 not_used -> /home/kevin/rp2040_si5351_wspr/libraries/not_used
lrwxrwxrwx 1 kevin kevin 45 Dec 12 01:08 Time -> /home/kevin/rp2040_si5351_wspr/libraries/Time
lrwxrwxrwx 1 kevin kevin 52 Dec 12 01:08 TinyGPSPlus -> /home/kevin/rp2040_si5351_wspr/libraries/TinyGPSPlus
kevin@pc8c:~/Arduino/libraries$

```
Then you get to where you use File to open the tracker dir of my repo, and open the tracker.ino
this is where my copy of the repo is, and the files
one tracker.ino
all the *.cpp
all the *.h

```
kevin@pc8c:~/rp2040_si5351_wspr/tracker$ 

-rw-rw-r-- 1 kevin kevin  1702 Dec  4 11:38 mh_functions.cpp
-rw-rw-r-- 1 kevin kevin   429 Dec  4 11:38 adc_functions.h
-rw-rw-r-- 1 kevin kevin  1010 Dec  4 11:38 config_functions.h
-rw-rw-r-- 1 kevin kevin   663 Dec  4 11:38 bmp_functions.h
-rw-rw-r-- 1 kevin kevin   431 Dec  4 11:38 mh_functions.h
-rw-rw-r-- 1 kevin kevin   911 Dec  4 11:38 led_functions.h
-rw-rw-r-- 1 kevin kevin   782 Dec  4 11:38 i2c_functions.h
-rw-rw-r-- 1 kevin kevin   445 Dec  4 11:38 tele_functions.h
-rw-rw-r-- 1 kevin kevin   793 Dec  4 11:38 u4b_functions.h
-rw-rw-r-- 1 kevin kevin 15801 Dec  5 12:45 u4b_functions.cpp
-rw-rw-r-- 1 kevin kevin  9731 Dec  5 18:34 bmp_functions.cpp
-rw-rw-r-- 1 kevin kevin 21517 Dec  5 23:17 i2c_functions.cpp
-rw-rw-r-- 1 kevin kevin   775 Dec  7 12:41 debug_functions.h
-rw-rw-r-- 1 kevin kevin 10447 Dec  7 12:41 debug_functions.cpp
-rw-rw-r-- 1 kevin kevin 14046 Dec  8 17:41 tele_functions.cpp
-rw-rw-r-- 1 kevin kevin  3114 Dec  9 01:29 print_functions.cpp
-rw-rw-r-- 1 kevin kevin  3821 Dec  9 01:32 adc_functions.cpp
-rw-rw-r-- 1 kevin kevin  3162 Dec  9 02:08 led_functions.cpp
-rw-rw-r-- 1 kevin kevin   521 Dec  9 11:22 keyboard_functions.h
-rw-rw-r-- 1 kevin kevin   631 Dec  9 13:19 wspr_functions.h
-rw-rw-r-- 1 kevin kevin  2543 Dec  9 17:58 print_functions.h
-rw-rw-r-- 1 kevin kevin  1351 Dec  9 21:17 gps_functions.h
-rw-rw-r-- 1 kevin kevin 18871 Dec 10 16:05 wspr_functions.cpp
-rw-rw-r-- 1 kevin kevin  3248 Dec 10 19:13 si5351_functions.h
-rw-rw-r-- 1 kevin kevin  7897 Dec 11 11:40 keyboard_functions.cpp
-rw-rw-r-- 1 kevin kevin 43608 Dec 11 14:27 config_functions.cpp
-rw-rw-r-- 1 kevin kevin 97164 Dec 11 14:29 tracker.ino
-rw-rw-r-- 1 kevin kevin 68514 Dec 11 23:59 gps_functions.cpp
-rw-rw-r-- 1 kevin kevin 47261 Dec 12 00:39 si5351_functions.cpp
```


a WATCH OUT FOR: whenever the IDE is downloading something like the arduino-pico json, if you have the serial monitor open, the download can hang. so if it doesn't download, make sure you don't have serial montitor open.

if I cd ../libraries from there, I get:
```
kevin@pc8c:~/rp2040_si5351_wspr/tracker$ cd ../libraries
 
kevin@pc8c:~/rp2040_si5351_wspr/libraries$ ls -ltr
total 36
drwxrwxr-x 4 kevin kevin 4096 Nov  7 21:30 TinyGPSPlus
drwxrwxr-x 4 kevin kevin 4096 Nov  8 00:01 Adafruit_SleepyDog
drwxrwxr-x 3 kevin kevin 4096 Nov  8 05:05 Adafruit_BusIO
drwxrwxr-x 4 kevin kevin 4096 Nov  8 05:53 Adafruit_BMP280_Library
drwxrwxr-x 3 kevin kevin 4096 Nov  8 06:16 Adafruit_Sensor
drwxrwxr-x 4 kevin kevin 4096 Nov  8 13:50 JTEncode
drwxrwxr-x 4 kevin kevin 4096 Nov 20 14:28 Arduino-MemoryFree
drwxrwxr-x 4 kevin kevin 4096 Nov 22 18:41 Time
```



you should be able to see "Raspberry Pi Pico" in the top left white box when you have a connected board

### 4. Upload

- First attach a GPS antenna and a short HF antenna (6" dipole wires?) to the AG6NS 0.4 pcb with new kbn BOM/CPL files, assembled by jlcpcb.com
- Connect board to your computer with a micro USB cable, then you should see a COM port under **Tools->Port** menu item. Select that port. (e.g. "/dev/cu.usbmodem141101")
- Click **Upload**
- 
- The firmware turns USB pll off at times. If you can't upload, unplug the usb (better yet get a usb cable with switch to avoid plug/unplug cycles). Then plug it back in. The green led should be on steady. immediately do the compile upload. After that you have 15 secs to open a putty window (setup a fast way to start putty with a session defined. save output (logging) to a putty.log for later analysis/debug. You will interact with the putty window for keyboard entry (if necessary). Hit enter to interrupt to get to config screen and keyboard entry. X to leave config mode and reboot. You will have to repeat opening putty at this point because of the reboot. If there is no usb, but just power, the tracker will enter balloon mode. Also, all usb input/output will be disabled if you don't open the putty session within 15 secs of boot. There is much debug output. it can be disabled with the V command in the config (VERBY). 0 is least output. 1 is more. 2 is more than that. 3 is most so far. The printing does not interfere with wspr operations. You can set K (clock) to 18 Mhz. There is a factory reset in the config selection. It will default any bad/illegal entries in the config.
- 
- If you see an error, you may need to put the tracker board into "Bootloader" mode before uploading:
  - Disconnect the USB cable
  - While shorting the H5 (two thru-hole next to the USB connector, maybe with pins or tweezers), connect the USB cable
  - After your PC recognized the board, release the shorting

### 5. More detail on compile/upload and getting to a serial window for keyboard input and print output
Forgive the wild descriptions here, will improve this shortly. Excerpts from late night email with a collaborator here:

I went hog wild playing with clock speeds and turning usb pll off and sys pll off.
I settled on running at 18 mhz (you can change the freq in the config up to 250mhz..default is 18mhz now)

I did have it running at 12mhz on the crystall oscillator like kazu suggested but couldn't get Serial2 working to the gps with that. so I just went to the pll being on for sys and 18 mhz (the lowest legal

I do some funky stuff on the first gps cold reset to try to get min power.

since it messes with usb pll, there's some timing to get firmware compiled and uploaded
I found once I started with the code messing with usb, i had to unplug/replug in the usb to get the sequence right
I ordered a usb plug that has a on/off switch that I'm going to use so I don't wear out my usb on my pc


in any case...this is the way to do it
do the compile-only. you will compile a lot the first time.
then unplug your usb to the rp2040 and plug it back in. This resets things if you were running the code before

now hit the compile/upload icon
if you had my pcb, this would be while a green led was lit

you should see it upload
have a script ready to open a putty session and start it. You have 15 secs to open the putty session otherwise it decides it's in balloon mode and you won't get any serial

I could change this to 30 secs until you get the hang of it, if you're continually getting a blank screen when you open the putty window after upload

I couldn't think of a good way for deciding when to ignore the usb serial fully in balloon mode..this timeout method is what I came up 


## Interesting current info
I think I'm seeing 45ma on usb power when sending RF now. GPS cold reset current still is the peak current though, I think.

The 45ma, I think that's 8ma si5351a output drive. Been testing on all bands: 20/17/15/12/10m.

that's at 5v but I think it's the same at 3.6v

the thing seems to run at 3.3v, there's a voltage reset monitor that keeps things off if the voltage is too low.

but I think the target with the ldo is around 3.6v voltage for normal balloon operation.

I went hog wild playing with clock speeds and turning usb pll off and sys pll off.
I settled on running at 18 mhz (you can change the freq in the config up to 250mhz..default is 18mhz now)

I did have it running at 12mhz on the crystal oscillator like kazu suggested but couldn't get Serial2 working to the gps with that. so I just went to the pll being on for sys and 18 mhz (the lowest legal

I do some funky stuff on the first gps cold reset to try to get min power.

since it messes with usb pll, there's some timing to get firmware compiled and uploaded
I found once I started with the code messing with usb, i had to unplug/replug in the usb to get the sequence right
I ordered a usb plug that has a on/off switch that I'm going to use so I don't wear out my usb on my pc

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

prehistory by Kazuhisa "Kazu." Terasaki, AG6NS
