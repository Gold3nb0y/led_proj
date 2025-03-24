#include <stdint.h>
#include "i2c.h"

void i2c_enable(uint32_t i2c){
    I2C(i2c, I2C_CR1) |= 1;
}

void i2c_disable(uint32_t i2c){
    I2C(i2c, I2C_CR1) &= ~1;
}

void i2c1_clock_setup(void){
    uint16_t tmp;
    //setup the peripheral clock to 10mhz
    tmp = I2C1(I2C_CR2) & ~(0x3F);
    I2C1(I2C_CR2) = tmp | 0xa;
    tmp = 3 << 14; //set fast mode and duty = 1
    tmp |= 25; //400khz * 25 = 10mhz
    I2C1(I2C_CCR) = tmp;
}

void i2c1_set_alternate_functions(void){
    uint32_t tmp;
    tmp = I2C_GPIO(I2C1_GPIO, MY_GPIO_MODER) & ~(0xF << 12);    
    tmp |= (0xa << 12);
    I2C_GPIO(I2C1_GPIO, MY_GPIO_MODER) = tmp;
    tmp = I2C_GPIO(I2C1_GPIO, MY_GPIO_AFLR) & 0xFFFFFF;
    tmp |= 0x44 << 24;
    I2C_GPIO(I2C1_GPIO, MY_GPIO_AFLR) = tmp;
}

void i2c1_set_filter(void){
    I2C1(I2C_FLTR) = 0;
}
