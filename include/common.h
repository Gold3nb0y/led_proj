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
#include <stdarg.h>

//defines
#define NUM_LEDS 3
#define WRITE_DELAY 105
#define CLK_MHZ 16
#define CLK_HZ 84000000
#define RR_HZ 30
#define UPDATE_INTERVAL 2800000
#define SYSCFG 0x40013C00
#define SYSCFG_EXTCR3 0x10
#define LED_WAIT (UPDATE_INTERVAL - (NUM_LEDS * WRITE_DELAY))
//use a 32bit timer
#define LED_TIM TIM5

#define GYRO_ADDR 0x68
#define ACC_X 0x3B
#define ACC_XH 0x3B
#define ACC_XL 0x3C

#define ACC_Y 0x3D
#define ACC_YH 0x3D
#define ACC_YL 0x3E

#define ACC_Z 0x3F
#define ACC_ZH 0x3F
#define ACC_ZL 0x40

#define GYRO_X 0x43
#define GYRO_XH 0x43
#define GYRO_XL 0x44

#define GYRO_Y 0x45
#define GYRO_YH 0x45
#define GYRO_YL 0x46

#define GYRO_Z 0x47
#define GYRO_ZH 0x47
#define GYRO_ZL 0x48

#define LCD_GPIO GPIOA
#define LCD_CMD_GPIO GPIOB
#define DATA0 GPIO0
#define DATA1 GPIO1
#define DATA2 GPIO2
#define DATA3 GPIO3
#define DATA4 GPIO4
#define DATA5 GPIO5
#define DATA6 GPIO6
#define DATA7 GPIO7
#define LCD_ENABLE GPIO12
#define LCD_RW GPIO13
#define LCD_RS GPIO14

void msleep(uint32_t delay);
uint32_t strlen(char* str);
void memcpy(void* dest, void* src, uint32_t size);
void memset(void* dest, char val, uint32_t size);
int sprintf(char *buf, const char *fmt, ...);


#endif 












