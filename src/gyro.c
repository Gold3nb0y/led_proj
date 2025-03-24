#include "gyro.h"
#include "lcd.h"
#include "led.h"
#include "libopencm3/stm32/i2c.h"
#include <stdint.h>
#include <math.h>

gyro_t gyro;

static float calc_pitch(float ax, float ay, float az) {
    return atan2f(-ax, az) * 57.4f;
}

static float calc_roll(float ax, float ay, float az) {
    return atan2f(-ay, az) * 57.4f;
}

void gyro_print_data(void){
    char buf[0x20];
    lcd_clear();
    memset(buf, 0, 0x20);
    sprintf(buf, "P:%d", gyro.pitch);
    lcd_write_line(buf, 0);
    memset(buf, 0, 0x20);
    sprintf(buf, "R:%d", gyro.roll);
    lcd_write_line(buf, 1);
    lcd_update_display();
}

uint8_t gyro_read_u8(uint8_t reg){
    uint8_t ret;
    i2c_transfer7(I2C1, GYRO_ADDR, &reg, 1, &ret, 1);
    return ret;
}

uint16_t gyro_read_u16(uint8_t reg){
    uint16_t ret = 0;
    ret = gyro_read_u8(reg) << 8; 
    ret |= gyro_read_u8(reg + 1); 
    return ret;
}

void gyro_init(void){
    uint8_t data;
    uint8_t pwr_mgmt[] = {0x6b, 0x00};

    //whoami
    data = 0x75;
    i2c_transfer7(I2C1, GYRO_ADDR, &data, 1, NULL, 0);
    i2c_transfer7(I2C1, GYRO_ADDR, NULL, 0, &data, 1);

    i2c_transfer7(I2C1, GYRO_ADDR, pwr_mgmt, 2, NULL, 0);
    msleep(100);

    memset(&gyro, 0, sizeof(gyro));
}

void gyro_update(void){

    gyro.acc_x = gyro_read_u16(ACC_X);
    gyro.acc_y = gyro_read_u16(ACC_Y);
    gyro.acc_z = gyro_read_u16(ACC_Z);

    gyro.pitch_f = calc_pitch((float)gyro.acc_x, (float)gyro.acc_y, (float)gyro.acc_z);
    gyro.pitch = (int32_t)gyro.pitch_f;

    gyro.roll_f = calc_roll((float)gyro.acc_x, (float)gyro.acc_y, (float)gyro.acc_z);
    gyro.roll = (int32_t)gyro.roll_f;

    //gyro.gyro_x = gyro_read_u16(GYRO_X);
    //gyro.gyro_y = gyro_read_u16(GYRO_Y);
    //gyro.gyro_z = gyro_read_u16(GYRO_Z);
    //if(back_x != gyro.gyro_x || back_y != gyro.gyro_y || back_z != gyro.gyro_z){
    //    gyro.update_display = true;
    //    update = true;
    //}
}

static int32_t sandwich(int32_t min, int32_t max, int32_t value){
    int32_t ret = value;
    if(value < min) {
        ret = min;
    } else if(value > max) {
        ret = max;
    }
    return ret;
}

void gyro_calc_led_color(led_t *led){
    led->r = sandwich(0, 0xff, BASE_RED + gyro.pitch + gyro.roll);
    led->g = 0x0; //sandwich(0, 0xff, BASE_GREEN + ((gyro.pitch + gyro.roll)));
    led->b = sandwich(0, 0xff, BASE_BLUE + gyro.pitch - gyro.roll);
}
