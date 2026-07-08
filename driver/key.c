#include "key.h"
extern int status;

uint8_t get_key_state(uint32_t key) {
    uint32_t high_bits = DL_GPIO_readPins(KEY_PORT, key); //0x00000040 0b01000000 PB6 0~31
    if((high_bits & key) != 0) return 1;
    else return 0;
}





