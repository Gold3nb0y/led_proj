#ifndef _COMMON_H
#define _COMMON_H

#define STM32F4

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>

void memcpy(char* dest, char* src, uint32_t size);

void memset(char* dest, char val, uint32_t size);

#endif 
