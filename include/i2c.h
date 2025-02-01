#ifndef _I2C_H
#define _I2C_H
#include <stdint.h>

#define MY_I2C1_BASE 0x40005400
#define MY_I2C2_BASE 0x40005800
#define MY_I2C3_BASE 0x40005C00

#define I2C_CR1 0x0
#define I2C_CR2 0x4
#define I2C_OAR1 0x8
#define I2C_OAR2 0xc
#define I2C_DR 0x10
#define I2C_SR1 0x14
#define I2C_SR2 0x18
#define I2C_CCR 0x1c
#define I2C_TRISE 0x20
#define I2C_FLTR 0x24

#define MY_GPIOB_BASE 0x40020400
#define MY_GPIO_MODER 0x0
#define MY_GPIO_AFLR 0x20
#define I2C1_GPIO MY_GPIOB_BASE
#define I2C_GPIO(gpio, offset) (*(uint32_t *)(gpio + offset))

#define I2C(i2c, offset) (*(uint32_t *)(i2c + offset))
#define I2C1(offset) (*(uint32_t *)(MY_I2C1_BASE + offset))
#define I2C2(offset) (*(uint32_t *)(MY_I2C2_BASE + offset))
#define I2C3(offset) (*(uint32_t *)(MY_I2C3_BASE + offset))

void i2c_enable(uint32_t i2c);
void i2c_disable(uint32_t i2c);
void i2c1_clock_setup(void);
void i2c1_set_alternate_functions(void);
void i2c1_set_filter(void);

#endif
