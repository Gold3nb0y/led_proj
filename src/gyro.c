#include "gyro.h"
#include "lcd.h"
#include "led.h"
#include "common.h"
#include "libopencm3/stm32/i2c.h"
#include <stdint.h>
#include <math.h>

gyro_t gyro;

//int err = 0;
//
//int* __errno(void){
//    return &err;
//}

static float calc_pitch(int16_t ax, int16_t ay, int16_t az) {
    float x, y, z;
    x = (float)ax / ACC_SENSITIVITY;
    y = (float)ay / ACC_SENSITIVITY;
    z = (float)az / ACC_SENSITIVITY;
    if (!isfinite(-x) || !isfinite(y) || !isfinite(z)) return 0.0f;
    return atan2f(-x, sqrtf(y*y + z*z)) * 57.4f;
}

static float calc_roll(int16_t ax, int16_t ay, int16_t az) {
    float x, y, z;
    x = (float)ax / ACC_SENSITIVITY;
    y = (float)ay / ACC_SENSITIVITY;
    z = (float)az / ACC_SENSITIVITY;
    return atan2f(-y, z) * 57.4f;
}

bool __inline gyro_lock(void){
    return lock(&gyro.gyro_lock);
}

void __inline gyro_unlock(void){
    unlock(&gyro.gyro_lock);
}

void gyro_print_data(void){
    char buf[0x10];
    lcd_clear();
    memset(buf, 0, 0x10);
    snprintf(buf, 0xf, "P:%d", gyro.pitch);
    lcd_write_line(buf, 0);
    memset(buf, 0, 0x10);
    snprintf(buf, 0xf, "R:%d", gyro.roll);
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

static void gyro_write_reg(uint8_t *cmd, uint8_t len){
    i2c_transfer7(I2C1, GYRO_ADDR, cmd, len, NULL, 0);
    msleep(100); //give some time to command to register
}

void gyro_init(void){
    uint8_t data;
    uint8_t pwr_mgmt[] = {0x6b, 0x00};
    uint8_t accel_cfg[] = {0x1c, 0xe0}; //set all input with 

    //whoami
    data = 0x75;
    i2c_transfer7(I2C1, GYRO_ADDR, &data, 1, NULL, 0);
    i2c_transfer7(I2C1, GYRO_ADDR, NULL, 0, &data, 1);

    gyro_write_reg(pwr_mgmt, 2);
    gyro_write_reg(accel_cfg, 2);
    data = gyro_read_u8(0x1c);
    if(data != 0xe0)
        for(;;);

    memset(&gyro, 0, sizeof(gyro));
}

void gyro_update(void){
    gyro.acc_x = gyro_read_u16(ACC_X);
    gyro.acc_y = gyro_read_u16(ACC_Y);
    gyro.acc_z = gyro_read_u16(ACC_Z);

    //calc_pitch (-340, -616, 19188);
    gyro.pitch_f = calc_pitch(gyro.acc_x, gyro.acc_y, gyro.acc_z);
    gyro.pitch = (int32_t)gyro.pitch_f;

    gyro.roll_f = calc_roll(gyro.acc_x, gyro.acc_y, gyro.acc_z);
    gyro.roll = (int32_t)gyro.roll_f;
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
