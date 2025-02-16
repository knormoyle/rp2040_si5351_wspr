# Origins
This is a fork by AD6Z of the Kazu AG6NS fork of LightAPRS-2.0 for Kazu's [sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1). Every line has been touched, and lots of changes to firmware architecture, strategy, file organization, etc.

The shorthand name for this pcb is AG6NS 0.4 pcb. The bom and cpl files (BOM* and PickAndPlace* xlsx) were modified to remove the low pass filters and vhf output, and have full differential tx for the WSPR RF. There are multiple choices due to experiments. Contact me to clarify which to use.

[sf-hab.org RP2040 based PicoBalloon Tracker PCB generation 1](https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1) is an open-hardware project. Kazu AG6NS released the pcb schematics/gerber etc and his port of LightAPRS firmware as open source. Thank you Kazu. Note there were 4 revisions of his work. I forked 0.4 of his pcb work.

The PCB gerber/boms==/placement files  for this firmware is currently at [AD6Z tracker](https://github.com/knormoyle/sf-hab_rp2040_picoballoon_tracker_pcb_gen1/tree/main/pcb/tracker/v0.4_kbn)
(v0.4_kbn dir). 
There are multiple BOM and CPL files that work for jlcpcb.com there. Nothing special for using and ordering from jlcpcb.com. Pretty much just like ordering traquito pcbs, if you're familiar with that. Economic assembly works, so stays cheap.

Contact me by email to find out which you should use if interested in building some at jlcpcb.com. I left the original AG6NS bom in there for comparison, but don't use that one!

Schematic and board png from jlcpcb are there also. The schematic has a buck/boost converter that's not used, some supercaps that are not used, an the LPFs on the si5351a clk0/clk1 have been changed in the bom/cpl files. But the schematic is pretty usable for understanding what's going on.. There are some datasheets for parts there also. Alternatives and other things being investigated.

I think the power consumption, at 3.6V, doesn't exceed 40mA during gps or rf or idle times. Gps is left on except when rf'ing, so theoretically a battery powered tracker could be more agressive at saving energy by using gps less. I try to slow turn-on stuff to minimize the effect of gps chip surge currents during cold gps reset (start of day). (not an issue I think with gps warm reset, the normal gps off->on transition with VBAT power)

## Reading/editting the code with an editor
I dislike reading code with tab chars or inconsistent style. Here's the target:

Uses 4 space indent, OTBS (One True Brace) indentation style.

Single statement blocks can exclude braces.
See style forms at https://en.wikipedia.org/wiki/Indentation_style

Linted with Google's cpplint. using Google's C++ Style guide, allowing 100 char lines
````
   cpplint --linelength=100
````

vim users can use this in .vimrc to good effect:
````
   set tabstop=4 softtabstop=0 expandtab shiftwidth=4 smarttab
````

## Costs
I could post invoices from some recent orders from jlcpcb.com. Cheaper with jlcpcb.com discounts and larger quantities. But even quantity 5 (the minimum) is cheap. Shouldn't be more than $10-$12 a board. (no bmp280, atgm336n gps). Shipping costs are $20 if you want fast shipping, but only $1.52 !! if you are willing to wait a bit for Global Standard Direct Line shipping.
No extra pico board needed. RP2040 is integrated. You only have solder gps antenna and hf antenna and power connections. Debug done with usb cable for power/data, and gps antenna and short HF antenna stubs.

## no-code USB serial configuration
<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/config_screenshot.png"/>
<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/config_screenshot2.png"/>

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

### 3. Configure Board

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

That's the board you want

Here's another step-by-step

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


you should be able to see "Raspberry Pi Pico" in the top left white box when you have a connected board

### 4. Copy Libraries 

- First download the repository to your computer using the green "[clone or download]" button.
- There are only one Arduino projects rp2040_si5351_wspr/tracker folder.
- You will notice some folders in the "libraries" folder. You have to copy these folders (libraries) into your Arduino libraries folder on your computer. Path to your Arduino libraries:
- **Windows** : This PC\Documents\Arduino\libraries\
- **Mac** : /Users/\<username\>/Documents/Arduino/libraries/
- **Ubuntu** : /Users/\<username\>/Arduino/libraries/


**IMPORTANT :** If you already have folders that have same name, you still need to overwrite them. Otherwise you get a compile error.

### 5. Use Library Manager to get Doug Malnati's WsprEncode library
Instead of copying this library, I'm trying to use with the IDE's Library Manager. Go to Tools, Manage Libraries at the top of the IDE. Type WsprEncoded into the search box under "LIBRARY MANAGER". You'll get Doug's library as a search result. Should be version 4.2.3. Hit Install.
<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/library_manager.png"/>

The black screen should say:

```
Downloading WsprEncoded@4.2.3
WsprEncoded@4.2.3
Installing WsprEncoded@4.2.3
Installed WsprEncoded@4.2.3
```
I believe the IDE will notify you when a new version of the library is available and should be installed.

### 6. Check the library files are where you expect
To give an example of where the files should be and the libraries issue:

You have to have a Arduino dir for libraries. I copied Kazu notes on where the Arduino dir is on different systems in my readme

I just created links (ln -s) from the Arduino libraries dir to the files in my repo libraries. You can just cp them though.
I put 'cp_libraries.sh' in the repo's tracker dir. I use 'cp_libraries.sh to create links in my Arduino directory to the libraries. This makes it easy for me to be stupid if I add libraries. You can modify it to work for whatever your local config is. You should only have to redo the Arduino libraries links or copies, if I had any new library usage (i just 

Added a library for blinking led with morse code, which I may use for detailed error messaging in the future. (that library seems cool. can be non-blocking!)

```
kevin@pc8c:~$ cd Arduino
kevin@pc8c:~/Arduino$ cd libraries
kevin@pc8c:~/Arduino/libraries$ ls -ltr
lrwxrwxrwx 1 kevin kevin   64 Feb 13 22:29 Adafruit_BMP280_Library -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_BMP280_Library
lrwxrwxrwx 1 kevin kevin   55 Feb 13 22:29 Adafruit_BusIO -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_BusIO
lrwxrwxrwx 1 kevin kevin   56 Feb 13 22:29 Adafruit_Sensor -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_Sensor
lrwxrwxrwx 1 kevin kevin   59 Feb 13 22:29 Adafruit_SleepyDog -> /home/kevin/rp2040_si5351_wspr/libraries/Adafruit_SleepyDog
lrwxrwxrwx 1 kevin kevin   59 Feb 13 22:29 Arduino-MemoryFree -> /home/kevin/rp2040_si5351_wspr/libraries/Arduino-MemoryFree
lrwxrwxrwx 1 kevin kevin   53 Feb 13 22:29 arduinomorse -> /home/kevin/rp2040_si5351_wspr/libraries/arduinomorse
lrwxrwxrwx 1 kevin kevin   53 Feb 13 22:29 JTEncode_mod -> /home/kevin/rp2040_si5351_wspr/libraries/JTEncode_mod
lrwxrwxrwx 1 kevin kevin   49 Feb 13 22:29 Time_mod -> /home/kevin/rp2040_si5351_wspr/libraries/Time_mod
lrwxrwxrwx 1 kevin kevin   56 Feb 13 22:29 TinyGPSPlus_mod -> /home/kevin/rp2040_si5351_wspr/libraries/TinyGPSPlus_mod
drwxr-xr-x 5 kevin kevin 4096 Feb 13 22:45 WsprEncoded


```
Then you get to where you use File to open the tracker dir of my repo, and open the tracker.ino
this is where my copy of the repo is, and the files
one tracker.ino
all the *.cpp
all the *.h

```
kevin@pc8c:~/rp2040_si5351_wspr/tracker$ 
-rw-rw-r-- 1 kevin kevin   1627 Nov 16 04:43 freq_count_functions.cpp
-rw-rw-r-- 1 kevin kevin    429 Dec  4 11:38 adc_functions.h
-rw-rw-r-- 1 kevin kevin    663 Dec  4 11:38 bmp_functions.h
-rw-rw-r-- 1 kevin kevin    431 Dec  4 11:38 mh_functions.h
-rw-rw-r-- 1 kevin kevin    782 Dec  4 11:38 i2c_functions.h
-rw-rw-r-- 1 kevin kevin   3114 Dec  9 01:29 print_functions.cpp
-rw-rw-r-- 1 kevin kevin   9710 Dec 12 17:47 bmp_functions.cpp
-rw-rw-r-- 1 kevin kevin    631 Dec 19 08:51 wspr_functions.h
-rw-rw-r-- 1 kevin kevin   7836 Dec 20 11:30 keyboard_functions.cpp
-rw-rw-r-- 1 kevin kevin    517 Dec 20 11:31 keyboard_functions.h
-rw-rw-r-- 1 kevin kevin   1701 Dec 20 11:47 mh_functions.cpp
-rw-rw-r-- 1 kevin kevin   1314 Jan  1 12:20 solar2_functions.h
-rw-rw-r-- 1 kevin kevin   6322 Jan  2 13:11 solar2_functions.cpp
-rw-rw-r-- 1 kevin kevin    577 Jan  2 13:16 solar5_functions.h
-rw-rw-r-- 1 kevin kevin   2119 Jan  2 19:16 solar4_functions.h
-rw-rw-r-- 1 kevin kevin    780 Jan  3 13:12 cw_functions.h
-rw-rw-r-- 1 kevin kevin   2271 Jan  4 16:44 solar_functions.h
-rw-rw-r-- 1 kevin kevin  14105 Jan  4 16:44 solar_functions.cpp
-rw-rw-r-- 1 kevin kevin    532 Jan  5 01:56 farey_functions.h
-rw-rw-r-- 1 kevin kevin   3859 Jan  6 17:12 adc_functions.cpp
-rw-rw-r-- 1 kevin kevin    826 Jan  7 21:06 u4b_functions.h
-rw-rw-r-- 1 kevin kevin   8104 Jan 10 12:03 farey_functions.cpp
-rw-rw-r-- 1 kevin kevin   3418 Jan 10 23:02 led_functions.cpp
-rw-rw-r-- 1 kevin kevin    618 Jan 17 23:09 sweep_functions.h
-rw-rw-r-- 1 kevin kevin   5253 Jan 17 23:09 si5351_functions.h
-rw-rw-r-- 1 kevin kevin   9667 Jan 18 22:37 solar5_functions.cpp
-rw-rw-r-- 1 kevin kevin    879 Jan 20 23:07 debug_functions.h
-rw-rw-r-- 1 kevin kevin   6555 Jan 21 22:57 solar4_functions.cpp
-rw-rw-r-- 1 kevin kevin  21490 Jan 22 14:42 i2c_functions.cpp
-rw-rw-r-- 1 kevin kevin    991 Jan 22 15:57 config_functions.h
-rw-rw-r-- 1 kevin kevin    730 Jan 22 15:58 tele_functions.h
-rw-rw-r-- 1 kevin kevin  22608 Jan 22 16:20 sweep_functions.cpp
-rw-rw-r-- 1 kevin kevin   2609 Jan 22 16:23 print_functions.h
-rw-rw-r-- 1 kevin kevin    705 Jan 24 23:29 doug_functions.h
-rw-rw-r-- 1 kevin kevin 112511 Feb  5 22:14 si5351_functions.cpp
-rw-rw-r-- 1 kevin kevin   1207 Feb  7 12:04 led_functions.h
-rw-rw-r-- 1 kevin kevin   7532 Feb  9 12:48 doug_functions.cpp
-rw-rw-r-- 1 kevin kevin  16261 Feb  9 14:31 u4b_functions.cpp
-rw-rw-r-- 1 kevin kevin   2561 Feb 13 00:01 time_functions.cpp
-rw-rw-r-- 1 kevin kevin    568 Feb 13 00:05 time_functions.h
-rw-rw-r-- 1 kevin kevin    450 Feb 13 01:48 tinygps_functions.h
-rw-rw-r-- 1 kevin kevin  23390 Feb 13 02:56 cw_functions.cpp
-rw-rw-r-- 1 kevin kevin   7004 Feb 13 09:39 tinygps_functions.cpp
-rw-rw-r-- 1 kevin kevin   4954 Feb 13 10:09 global_structs.h
-rw-rw-r-- 1 kevin kevin  55520 Feb 13 14:39 config_functions.cpp
-rw-rw-r-- 1 kevin kevin    475 Feb 13 16:52 pps_functions.h
-rw-rw-r-- 1 kevin kevin  18368 Feb 13 17:08 wspr_functions.cpp
-rw-rw-r-- 1 kevin kevin  31877 Feb 14 16:01 tele_functions.cpp
-rw-rw-r-- 1 kevin kevin  17577 Feb 14 16:22 debug_functions.cpp
-rw-rw-r-- 1 kevin kevin   1332 Feb 16 11:34 gps_functions.h
-rw-rw-r-- 1 kevin kevin 110702 Feb 16 13:53 tracker.ino
-rw-rw-r-- 1 kevin kevin   7449 Feb 16 14:54 pps_functions.cpp
-rw-rw-r-- 1 kevin kevin 119776 Feb 16 15:18 gps_functions.cpp
```


WATCH OUT FOR: whenever the IDE is downloading something like the arduino-pico json, if you have the serial monitor open, the download can hang. so if it doesn't download, make sure you don't have serial montitor open.

If I cd ../libraries from there, I get:
```
kevin@pc8c:~/rp2040_si5351_wspr/tracker$ cd ../libraries
 
kevin@pc8c:~/rp2040_si5351_wspr/libraries$ ls -ltr
drwxrwxr-x 4 kevin kevin 4096 Nov  7 21:30 TinyGPSPlus_mod
drwxrwxr-x 4 kevin kevin 4096 Nov  8 00:01 Adafruit_SleepyDog
drwxrwxr-x 4 kevin kevin 4096 Nov  8 05:53 Adafruit_BMP280_Library
drwxrwxr-x 6 kevin kevin 4096 Dec  5 11:31 not_used
drwxrwxr-x 4 kevin kevin 4096 Jan  1 12:05 Arduino-MemoryFree
drwxrwxr-x 3 kevin kevin 4096 Jan  6 13:10 arduinomorse
drwxrwxr-x 3 kevin kevin 4096 Jan  8 01:47 Adafruit_Sensor
drwxrwxr-x 3 kevin kevin 4096 Jan 19 18:35 Adafruit_BusIO
drwxrwxr-x 4 kevin kevin 4096 Feb  9 13:23 JTEncode_mod
drwxrwxr-x 4 kevin kevin 4096 Feb 16 11:45 Time_mod
```


### 7. Now you're ready to compile tracker.ino

- Then open the tracker.ino file with Arduino IDE
- Click **Verify** (If you get compile errors, check the steps above)


### 8. Compile and Upload

- First attach a GPS antenna and a short HF antenna (6" dipole wires?) to the AG6NS 0.4 pcb with new kbn BOM/CPL files, assembled by jlcpcb.com
- Connect board to your computer with a micro USB cable, then you should see a COM port under **Tools->Port** menu item. Select that port. (e.g. "/dev/cu.usbmodem141101")
- Click **Upload**
- 
- The firmware turns USB pll off at times. If you can't upload, unplug the usb (better yet get a usb cable with switch to avoid plug/unplug cycles). Then plug it back in. The green led should be on steady. immediately do the compile upload. After that you have 15 secs to open a putty window (setup a fast way to start putty with a session defined. save output (logging) to a putty.log for later analysis/debug. You will interact with the putty window for keyboard entry (if necessary). Hit enter to interrupt to get to config screen and keyboard entry. X to leave config mode and reboot. You will have to repeat opening putty at this point because of the reboot. If there is no usb, but just power, the tracker will enter balloon mode. Also, all usb input/output will be disabled if you don't open the putty session within 15 secs of boot. There is much debug output. it can be disabled with the V command in the config (VERBY). 0 is least output. 1 is more. 2 is more than that. 3 is most so far. The printing does not interfere with wspr operations. You can set K (clock) to 18 Mhz. There is a factory reset in the config selection. It will default any bad/illegal entries in the config.
- 
Here's the usb cable with on/off switch I ordered. I think it will be very useful!
[USB cable with on/off switch](https://www.amazon.com/dp/B0CVZQ66C6) Note your existing USB/A micro-usb cable will plug into the female jack. Pick an appopriate length for comfort!

- 
- If you see an error, you may need to put the tracker board into "Bootloader" mode before uploading:
  - Disconnect the USB cable
  - While shorting the H5 (two thru-hole next to the USB connector, maybe with pins or tweezers), connect the USB cable
  - After your PC recognized the board, release the shorting
  - 
### 9. Setup putty to have a session that is config'ed to log to putty.log
I set up a session with these configurations and save to a session named putty_config_for_ad6z_tracker. Remember to enter the name of the config you want to save: 'putty_config_for_ad6z_tracker' and click save to save it.  On Ubuntu (linux) the usb serial port should always be created as /dev/ttyACM0. If you plug in multiple usb serial devices, or unplug and replug too quickly, it might use /dev/ttyACM1, etc. 

If using Windows, or maybe Mac, you probably have different usb serial ports. I expect you've dealt with this before, so do the right thing!
If there's a problem and maybe the port isn't being created, check like this at the command line
```
$ ls /dev/ttyA*
/dev/ttyACM0
```

the putty config setup I use..everything else is same as default.

<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/putty1.png"/>
<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/putty2.png"/>
<img src="https://github.com/knormoyle/rp2040_si5351_wspr/blob/main/tracker/putty3.png"/>

Then I create a myputty.sh in my home directory.

Then I create an entry in my .bashrc so that I can type 'mp" at the command line, and it goes to my home dir and executes my putty.sh

That way, putty.log will always overwrite any putty.log in my home dir, and I know where to find it. So you don't have to remember to log, you just always do it this way, and you always have a log if there was some issue with the session. Notably if the tracker reboots unexpectedly.

When you 'X' to force reboot after config, or the config times out, it will reboot and close the putty session. Just redo 'mp' or whatever, and open a new session within 10 secs.
Windows should have similar thing you can do to have a script to execute putty.
myputty.sh looks like this:

```
#!/usr/bin/env bash

# SESSION="/home/kevin/.putty/sessions/Default%20Settings"
# SESSION="/home/kevin/.putty/sessions/putty_config_kc3lbr.10_20_24"
SESSION="/home/kevin/.putty/sessions/putty_config_for_ad6z_tracker"

# don't include the path above
# this autostarts
SESSION="putty_config_kc3lbr.10_20_24"

# in case there was an existing putty session writing to putty.log
pkill putty.log
echo "putty -load $SESSION" 
putty -load "$SESSION"
```

So the combination of the on/off switch on the usb cable, and my fast 'mp' to start a putty session, means I can reboot (power on/off) the tracker quickly when necessary, like right before a compile upload, or if I missed the putty window open timing ,and have to reboot the tracker to get it out of BALLOON_MODE.

This is why I just have short 10-15 sec timeouts on things, to enter BALLOON_MODE etc.

It might be maddening at first to see how the putty window closes on reboot, but that's safest for propagating configuration changes. They are just written to flash, and then the tracker reboots (after timeout or when you give teh command 'X'.. Then the new config is used on reboot, just like normal!


### 10. More detail on compile/upload and getting to a serial window for keyboard input and print output
Forgive the wild descriptions here, will improve this shortly. Excerpts from late night email with a collaborator here:

I went hog wild playing with clock speeds and turning usb pll off and sys pll off.
I settled on running at 18 mhz (you can change the freq in the config up to 250mhz..default is 18mhz now)

I did have it running at 12mhz on the crystall oscillator like kazu suggested but couldn't get Serial2 working to the gps with that. so I just went to the pll being on for sys and 18 mhz (the lowest legal

I do some funky stuff on the first gps cold reset to try to get min power.

Since it messes with usb pll, there's some timing to get firmware compiled and uploaded
I found once I started with the code messing with usb, i had to unplug/replug in the usb to get the sequence right
I ordered a usb cable from amazon that has a on/off switch that I'm going to use so I don't wear out my usb on my pc

In any case...this is the way to do it
do the compile-only. you will compile a lot the first time.
then unplug your usb to the rp2040 and plug it back in. This resets things if you were running the code before

Now hit the compile/upload icon
if you had my pcb, this would be while a green led was lit

You should see it upload.
Have a script ready to open a putty session and start it. You have 15 secs to open the putty session otherwise it decides it's in balloon mode and you won't get any serial

I could change this to 30 secs until you get the hang of it, if you're continually getting a blank screen when you open the putty window after upload

I couldn't think of a good way for deciding when to ignore the usb serial fully in balloon mode..this timeout method is what I came up 

## Interesting current info
I think I'm seeing 40mA a on usb power when sending RF now. GPS cold reset current still is the peak current though, I think? (have to get better power measuring device).

The 4mA output drive on si5351 seems like it's only 3db down from the configurable higher power 8mA choice.  So the default is low power drive which is 4mA output drive.
For reference, on estimated power. This is singled ended drive. 

The si5351a(ms5351m) is doing double-ended drive: clk0/clk1 with clk1 180 degrees out of phase.  So the power should be twice these numbers, ideally?

````
https://rfzero.net/documentation/rf/
The table below shows the typical output power vs. current in the output stages
running in push-pull with a T1 transformer
Current [mA] 
    137 kHz  1 MHz     10 MHz    30 MHz    50 MHz    200 MHz
8   9,5 dBm  14,2 dBm  14,5 dBm  15,0 dBm  14,5 dBm  13,3 dBm
6   9,2 dBm  12,8 dBm  13,3 dBm  13,7 dBm  13,0 dBm  11,8 dBm
4   8,3 dBm  10,3 dBm  10,7 dBm  11,0 dBm  10,5 dBm   9,7 dBm
2   5,2 dBm   4,7 dBm   5,0 dBm   5,5 dBm   5,0 dBm   4,5 dBm
````

The 40ma, I think that's 4ma si5351a output drive. Been testing on all bands: 20/17/15/12/10m.
That's at 5v but I think it's the same at 3.6v

The thing seems to run at 3.3v, there's a voltage reset monitor that keeps things off if the voltage is too low.

But I think the target with the ldo is around 3.6v voltage for normal balloon operation.

I went hog wild playing with clock speeds and turning usb pll off and sys pll off.
I settled on running at 18 mhz (you can change the freq in the config up to 250mhz..default is 18mhz now)

I did have it running at 12mhz on the crystal oscillator like kazu suggested but couldn't get Serial2 working to the gps with that. so I just went to the pll being on for sys and 18 mhz (the lowest legal

I do some funky stuff on the first gps cold reset to try to get min power.

Since it messes with usb pll, there's some timing to get firmware compiled and uploaded
I found once I started with the code messing with usb, i had to unplug/replug in the usb to get the sequence right
I ordered a usb plug that has a on/off switch that I'm going to use so I don't wear out my usb on my pc

Exciting update: I've updated constants and the symbol freq algo, to use optimal denominators for the symbol freq generation, so that the symbol shifts are exactly = the desired wspr shifts of 12000/8196 = ~1.4648 Hz,
Basically get microHertz level accuracy on the symbol shifts, and absolute errors of < 1Hz.
Different denominators targeted for different bands, using the spreadsheet data for deciding optimal denominator (per band). Same denominator used for all 4 u4b frequency bins on a band. By getting exact symbol shifts, I think the DT and SNR and frequency report from my WSJT-X sdr testing, has improved (more energy into the ffts being done by WSJT-X)

## Questions?
I (Kevin Normoyle) will try to answer all questions or email. Feel free to use Issues or Discussions here to everyone can benefit!
[Issues](https://github.com/knormoyle/rp2040_si5351_wspr/discussions)
[Discussions](https://github.com/knormoyle/rp2040_si5351_wspr/issues)
 

## Project Background and History (Kazu AG6NS prehistory to this fork)
**Jan 2024** - Seems to have all the features I wanted. Can do CW also, and uses Farey algo for si5351a config. Changes both numerator/denominator on symbol shift frequencies. So 400 Mhz Si5351a PLL target frequency is used with pretty accurate WSPR symbol shifts. Can target higher PLL frequencies if wanted, (600-900 is si5351a/ms5351m spec). Interestingly I couldn't key the CW on the ms5351m with the clk output enable reg. Used the clk PDN (power down) instead. requires PLL reset to keep clk0/clk1 phase relationship for the dipole antiphase drive, but that's okay. CW timing is perfect at 12 wpm, but can be changed.

Alternate symbol shift strategy of numerator-only shift is still in the code (mode that can be changed with recompile). I have fixed denominator per band constants for that mode, that allow PERFECT wspr symbol shifts of 1.4641...Hz for a couple of different target PLL frequencies (also choosable with constants, with recompile). But that shouldn't be necessary (unless we run into a lock problem with the si5351a/ms5351m at 400Mhz, or increased phase noise or ??. Right now, the 393Mhz pll frequency seems to save a couple mA of power, so good!

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

Prehistory by Kazuhisa "Kazu." Terasaki, AG6NS
