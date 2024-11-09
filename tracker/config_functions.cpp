// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php
// Author: Kevin Normoyle AD6Z initial 11/2024

// Arduino IDE main: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/tracker
// Arduino IDE libraries: https://github.com/knormoyle/rp2040_si5351_wspr/tree/main/libraries

// Incorporates work by: Kazuhisa “Kazu” Terasaki AG6NS. Thank you.
// https://github.com/kaduhi/sf-hab_rp2040_picoballoon_tracker_pcb_gen1
// https://github.com/kaduhi/LightAPRS-W-2.0/tree/port_to_ag6ns_rp2040_picoballoon_tracker

// Incorporates work by: Rob Votin KC3LBR. Thank you.
// https://github.com/EngineerGuy314/pico-WSPRer

// Incorporates work by: Roman Piksaykin R2BDY. Thank you.
// https://github.com/RPiks/pico-WSPR-tx

#include <stdint.h>

// any of this needed
// #include <stdio.h>
// #include <string.h>
// #include <ctype.h>
// #include <defines.h>
// #include "pico/stdlib.h"


 // User Input. echoes input to stdout and sets input_variable
 // prompt: Prompt to display to user <input>
 // input_variable: Variable to which we want to read input <output>
 // max_length: Maximum length of input string <input>
void get_user_input(const char *prompt, char *input_variable, int max_length) 
{
    int index = 0;
    int ch;
    
    printf("%s", prompt);  // Display the prompt to the user
    fflush(stdout);

    while (1) {
        ch = getchar();
        if (ch == '\n' || ch == '\r') {  // Enter key pressed
            break;
        } else if (ch == 127 || ch == 8) {  // Backspace key pressed (127 for most Unix, 8 for Windows)
            if (index > 0) {
                index--;
                printf("\b \b");  // Move back, print space, move back again
            }
        } else if (isprint(ch)) {
            if (index < max_length - 1) {  // Ensure room for null terminator
                input_variable[index++] = ch;
                printf("%c", ch);  // Echo character
            }
        }
        fflush(stdout);
    }

    input_variable[index] = '\0';  // Null-terminate the string
    printf("\n");
}

// hex listing of the settings NVRAM to stdio
// buf: Address of NVRAM to list <input>
// len: Length of storage to list <input>
void print_buf(const uint8_t *buf, size_t len) 
{
    printf(CLEAR_SCREEN);printf(BRIGHT);
    printf(BOLD_ON);printf(UNDERLINE_ON);
    printf("\nNVRAM dump: \n");printf(BOLD_OFF); printf(UNDERLINE_OFF);
    for (size_t i = 0; i < len; ++i) {
        printf("%02x", buf[i]);
        if (i % 16 == 15) printf("\n");
        else printf(" ");
    }
    printf(NORMAL);
}

void display_intro(void)
{
    printf(CLEAR_SCREEN);
    printf(CURSOR_HOME);
    printf(BRIGHT);
    printf("\n\n\n\n\n\n\n\n\n\n\n\n");
    printf("================================================================================\n\n");printf(UNDERLINE_ON);
    printf("AD6Z tracker for AG6NS 0.04 pcb,  version: %s %s\n\n",__DATE__ ,__TIME__);printf(UNDERLINE_OFF);
    printf("\n================================================================================\n");
    printf(RED);printf("press anykey to continue");printf(NORMAL); 
    char c=getchar_timeout_us(60000000);	//wait 
    printf(CLEAR_SCREEN);
}

void show_TELEN_msg()
{
    printf(BRIGHT);
    printf("\n\n\n\n");printf(UNDERLINE_ON);
    printf("TELEN CONFIG INSTRUCTIONS:\n\n");printf(UNDERLINE_OFF);
    printf(NORMAL); 
    printf("* There are 4 possible TELEN values, corresponding to TELEN 1 value 1,\n");
    printf("  TELEN 1 value 2, TELEN 2 value 1 and TELEN 2 value 2.\n");
    printf("* Enter 4 characters (legal 0-9 or -) in TELEN_config. use a '-' (minus) to disable one \n");
    printf("  or more values.\n* example: '----' disables all telen \n");
    printf("* example: '01--' sets Telen 1 value 1 to type 0, \n  Telen 1 val 2 to type 1,  disables all of TELEN 2 \n"); 
    printf(BRIGHT);printf(UNDERLINE_ON);
    printf("\nTelen Types:\n\n");printf(UNDERLINE_OFF);printf(NORMAL); 
    printf("-: disabled, 0: ADC0, 1: ADC1, 2: ADC2, 3: ADC3,\n");
    printf("4: minutes since boot, 5: minutes since GPS fix aquired \n");
    printf("6-9: OneWire temperature sensors 1 though 4 \n");
    printf("A: custom: OneWire temperature sensor 1 hourly low/high \n");
    printf("B-Z: reserved for Future: I2C devices, other modes etc \n");
    printf("\n(ADC values come through in units of mV)\n");
    printf("See the Wiki for more info.\n\n");
}

// Function that implements simple user interface via UART
/* For every new config variable to be added to the interface:
	1: create a global character array at top of main.c 
	2: add entry in read_NVRAM()
	3: add entry in write_NVRAM()
	4: add limit checking in check_data_validity()
	5: add limit checking in check_data_validity_and_set_defaults()
	6: add TWO entries in show_values() (to display name and value, and also to display which key is used to change it)
	7: add CASE statement entry in user_interface()
	8: Either do something with the variable locally in Main.c, or if needed elsewhere:
		-- add a member to the GPStimeContext or WSPRbeaconContext structure
		-- add code in main.c to move the data from the local _tag to the context structure
		-- do something with the data elsewhere in the program
 */

//called if keystroke from terminal on USB detected during operation.
void user_interface(void) 
{
    int c;
    char str[10];

    // FIX! call right thing with gpsPwr
    gpio_put(GPS_ENABLE_PIN, 0); //shutoff gps to prevent serial input  (probably not needed anymore)
    sleep_ms(100);
    // FIX! right thing for LED on
    gpio_put(LED_PIN, 1); //LED on.	
    display_intro();
    show_values(); /* shows current VALUES  AND list of Valid Commands */

    // background
    // https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
    for(;;)
	{	
        printf(UNDERLINE_ON);printf(BRIGHT);
        // kevin 10_30_24
		printf("\nEnter the command (X,C,S,U,[I,M,L],V,P,T,B,D,K,F,A):");
        printf(UNDERLINE_OFF);
        printf(NORMAL);	
		c=getchar_timeout_us(60000000);	//just in case user setup menu was enterred during flight, this will reboot after 60 secs
		printf("%c\n", c);
		if (c==PICO_ERROR_TIMEOUT) {
            printf(CLEAR_SCREEN);
            printf("\n\n TIMEOUT WAITING FOR INPUT, REBOOTING FOR YOUR OWN GOOD!\n");
            sleep_ms(100);
            watchdog_enable(100, 1);
            for(;;)	{}}

		if (c>90) c-=32; //make it capital either way
		switch(c)
		{
			case 'X':
                printf(CLEAR_SCREEN);
                printf("\n\nGOODBYE");
                watchdog_enable(100, 1);
                for(;;)	{}
			case 'C':
                get_user_input("Enter callsign: ", _callsign, sizeof(_callsign)); 
                convertToUpperCase(_callsign); 
                write_NVRAM(); 
                break;
			case 'U':
                get_user_input("Enter U4B channel: ", _U4B_chan, sizeof(_U4B_chan)); 
                process_chan_num(); 
                write_NVRAM(); 
                break;
			case 'V':
                get_user_input("Verbosity level (0-9): ", _verbosity, sizeof(_verbosity)); 
                write_NVRAM(); 
                break;
			case 'T':
                show_TELEN_msg();
                get_user_input("TELEN config: ", _TELEN_config, sizeof(_TELEN_config)); 
                convertToUpperCase(_TELEN_config); 
                write_NVRAM(); 
                break;
			case 'K':
                get_user_input("Klock speed (default 115): ", _Klock_speed, sizeof(_Klock_speed)); 
                write_NVRAM(); 
                // frequencies like 205 mhz will PANIC, System clock of 205000 kHz cannot be exactly achieved
                // should detect the failure and change the nvram, otherwise we're stuck even on reboot
                const uint32_t clkhz =  atoi(_Klock_speed) * 1000000L;
                // this doesn't change the pll, just checks
                if (!set_sys_clock_khz(clkhz / kHz, false))
                {
                    printf("\n NOT LEGAL TO SET SYSTEM KLOCK TO %dMhz. Cannot be achieved. Using 115\n", PLL_SYS_MHZ);
                    strcpy(_Klock_speed,"115");
                    write_NVRAM();
                }
                break;
            //********************
            // kevin 10_30_24
			case 'A':
                get_user_input("Enter Band (10,12,15,17,20): ", _Band, sizeof(_Band)); 
                // redo the channel selection if we change bands, since U4B definition changes per band 
                process_chan_num(); 
                write_NVRAM(); 
                init_rf_freq();
                _u32_dialfreqhz = XMIT_FREQUENCY = init_rf_freq();
                break;

            //********************
			case 13:  break;
			case 10:  break;
			default: 
                printf(CLEAR_SCREEN); 
                printf("\nYou pressed: %c - (0x%02x), INVALID choice!! ",c,c);
                sleep_ms(1000);
                break;		
		}
		int result = check_data_validity_and_set_defaults();
		show_values();
	}
}

// Reads flash where the user settings are saved
// prints hexa listing of data 
// calls function which check data validity
// background
// https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
void read_NVRAM(void)
{
    // pointer to a safe place after the program memory
    const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET); 
    print_buf(flash_target_contents, FLASH_PAGE_SIZE); //256

    // kevin 10_31_24 null terminate in case it's printf'ed with %s
    // FIX! shouldn't all of these (all chars) have null terminate?
    // FIX! left gaps (unused)
    // multichar that have room have a null in the flash?
    strncpy(_callsign,    flash_target_contents,6);     _callsign[6]=0;
    strncpy(_verbosity,   flash_target_contents+11, 1); _verbosity[1]=0;
    strncpy(_TELEN_config,flash_target_contents+14, 4); _TELEN_config[4]=0;
    strncpy(_Klock_speed, flash_target_contents+19, 3); _Klock_speed[3]=0;
    PLL_SYS_MHZ =atoi(_Klock_speed); 
    strncpy(_U4B_chan,    flash_target_contents+23, 3); _U4B_chan[3]=0;
    strncpy(_Band,        flash_target_contents+26, 2); _Band[2]=0;
 
}

// Write the user entered data into NVRAM
void write_NVRAM(void)
{
    uint8_t data_chunk[FLASH_PAGE_SIZE];  //256 bytes

	strncpy(data_chunk,_callsign,        6);
	strncpy(data_chunk+11,_verbosity,    1);
	strncpy(data_chunk+14,_TELEN_config, 4);
	strncpy(data_chunk+19,_Klock_speed,  3);
	strncpy(data_chunk+22,_Datalog_mode, 1);
	strncpy(data_chunk+26,_Band, 2);
	

    //you could theoretically write 16 pages at once (a whole sector). don't interrupt
	uint32_t ints = save_and_disable_interrupts();
    //a "Sector" is 4096 bytes FLASH_TARGET_OFFSET,FLASH_SECTOR_SIZE,FLASH_PAGE_SIZE = 040000x, 4096, 256
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);  
    //writes 256 bytes (one "page") (16 pages per sector)
	flash_range_program(FLASH_TARGET_OFFSET, data_chunk, FLASH_PAGE_SIZE);  
	restore_interrupts (ints);

}
// Checks validity of user settings and if something is wrong, 
// sets "factory defaults" and writes it back to NVRAM
// create result to return
int check_data_validity_and_set_defaults(void)
{
    int result=1;	
    //*************************
    // kevin 10_31_24 
    // do some basic plausibility checking on data, set reasonable defaults if memory was uninitialized							
    // or has bad values for some reason
    // create result to return like check_data_validity does
    // FIX! should do full legal callsign check? (including spaces at end)
    // be sure to null terminate so we can print the callsign
	if ( ((_callsign[0]<'A') || (_callsign[0]>'Z')) && ((_callsign[0]<'0') || (_callsign[0]>'9')) ) {
        strcpy(_callsign,"AB1CDE"); 
        write_NVRAM(); 
        result=-1;
    } 
    // change to strcpy for null terminate
	if ( (_verbosity[0]<'0') || (_verbosity[0]>'9')) {
        //set default verbosity to 1
        strcpy(_verbosity,"1"); 
        write_NVRAM(); 
        result=-1;
    } 

    // 0-9 and - are legal. _
    // make sure to null terminate
    int i
    for(i = 1; i <= 3; ++i)
	if ( (_TELEN_config[i]<'0' || _TELEN_config[i]>'9') && _TELEN_config[i]!='-') {
        strcpy(_TELEN_config,"----"); 
        write_NVRAM(); 
        result=-1;
    }

    //*********
    // kevin 10_31_24 . keep the upper limit at 250 to avoid nvram getting
    // a freq that won't work. will have to load flash nuke uf2 to clear nram
    // if that happens, so that default Klock will return?
    // if so: Download the [UF2 file]
    // https://datasheets.raspberrypi.com/soft/flash_nuke.uf2
    // code is
    // https://github.com/raspberrypi/pico-examples/blob/master/flash/nuke/nuke.c
    // may require some iterations of manually setting all the configs by hand 
    // after getting the nuke uf2 (it autoruns) and then reloading pico-WSPRer.uf2
    // hmm. I suppose we could call this routine to fix nvram at the beginning, so if the 
    // clock gets fixed, then the defaults will get fixed (where errors exist)
    // be sure to null terminate
	if ( (atoi(_Klock_speed)<100) || (atoi(_Klock_speed)>250)) {strcpy(_Klock_speed,"115"); write_NVRAM(); result=-1;} 
    //*********
    // be sure to null terminate
	if ( (atoi(_U4B_chan)<0) || (atoi(_U4B_chan)>599)) {strcpy(_U4B_chan,"599"); write_NVRAM(); result=-1;} 
    //****************
    // kevin 10_30_24
    switch(atoi(_Band))
    {
        case 10: break;
        case 12: break;
        case 15: break;
        case 17: break;
        case 20: break;
        default: 
            strcpy(_Band,"20"); 
            write_NVRAM(); 
            // figure out the XMIT_FREQUENCY for new band, and set _32_dialfreqhz
            // have to do this whenever we change bands
            XMIT_FREQUENCY = init_rf_freq();
		    pWSPR->_pTX->_u32_dialfreqhz = XMIT_FREQUENCY;
            result=-1;
            break;
    }
    return result;
    //****************
}

 // Function that writes out the current set values of parameters
void show_values(void) /* shows current VALUES  AND list of Valid Commands */
{
								printf(CLEAR_SCREEN);printf(UNDERLINE_ON);printf(BRIGHT);
printf("\n\nCurrent values:\n");printf(UNDERLINE_OFF);printf(NORMAL);
printf("\n\tCallsign:%s\n\t",_callsign);
printf("Suffix:%s\n\t",_suffix);
printf("U4b channel:%s",_U4B_chan);
printf(" (Id13:%s",_id13);
printf(" Start Minute:%s",_start_minute);
printf(" Lane:%s)\n\t",_lane);
printf("Verbosity:%s\n\t",_verbosity);
// this isn't modifiable by user but still checked for correct default value
/*printf("Oscillator Off:%s\n\t",_oscillator);*/
printf("custom Pcb IO mappings:%s\n\t",_custom_PCB);
printf("TELEN config:%s\n\t",_TELEN_config);
printf("Klock speed:%sMhz  (default: 115)\n\t",_Klock_speed);
printf("Datalog mode:%s\n\t",_Datalog_mode);
//*************
// kevin 10_30_24
printf("Band:%s\n\t",_Band);
printf("XMIT_FREQUENCY:%d\n\t",XMIT_FREQUENCY);
//*************
printf("Battery (low power) mode:%s\n\n",_battery_mode);
printf(UNDERLINE_ON);printf(BRIGHT);
printf("VALID commands: ");printf(UNDERLINE_OFF);printf(NORMAL);

printf("\n\n\tX: eXit configuraiton and reboot\n\tC: change Callsign (6 char max)\n\t");
printf("S: change Suffix ( for WSPR3/Zachtek) use '-' to disable WSPR3\n\t");
printf("U: change U4b channel # (0-599)\n\t");
printf("A: change bAnd (10,12,15,17,20 default 20)\n\t");
/*printf("I: change Id13 (two alpha numeric chars, ie Q8) use '--' to disable U4B\n\t");
printf("M: change starting Minute (0,2,4,6,8)\n\tL: Lane (1,2,3,4) corresponding to 4 frequencies in 20M band\n\t");*/ //it is still possible to directly change these, but its not shown
printf("V: Verbosity level (0 for no messages, 9 for too many) \n\t");
/*printf("O: Oscillator off after transmission (default: 1) \n\t");*/
printf("P: custom Pcb mode IO mappings (0,1)\n\t");
printf("T: TELEN config\n\t");
printf("K: Klock speed  (default: 115)\n\t");
printf("D: Datalog mode (0,1,(W)ipe memory, (D)ump memory) see wiki\n\t");
printf("B: Battery (low power) mode \n\t");
printf("F: Frequency output (antenna tuning mode)\n\n");


}
/**
 * @brief Converts string to upper case
 * 
 * @param str string to convert
 * @return No return value, string is converted directly in the parameter *str  
 */
void convertToUpperCase(char *str) {
    while (*str) {
        *str = toupper((unsigned char)*str);
        str++;
    }
}
