#include "common.h"
#include <stdint.h>

void memcpy(char* dest, char* src, uint32_t size){
    uint32_t i;
    for(i = 0; i < size; i++)
        dest[i] = src[i];
}

void memset(char* dest, char val, uint32_t size){
    uint32_t i;
    for(i = 0; i < size; i++)
        dest[i] = val;
}
