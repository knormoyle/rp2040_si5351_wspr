// Project: https://github.com/knormoyle/rp2040_si5351_wspr
// Distributed with MIT License: http://www.opensource.org/licenses/mit-license.php 
// Author/Gather: Kevin Normoyle AD6Z initially 11/2024

// See acknowledgements.txt for the lengthy list of contributions/dependencies.

#ifndef CW_FUNCTIONS_H
#define CW_FUNCTIONS_H
#include <stdint.h>

// state of the tx (si5351a)
enum tx_state_e {
    E_STATE_RX,
    E_STATE_TX
};      
        
// state of the key line
enum key_state_e {
    E_KEY_UP,
    E_KEY_DOWN
};

void cw_keyer_speed(uint8_t wpm);
void cw_key_state(key_state_e k);
void cw_tx_state(tx_state_e s);
void cw_init();
void cw_high_drive_strength();
void cw_restore_drive_strength();

void cw_send(char *m);
void cw_send_message();


#endif  // CW_FUNCTIONS_H
