#ifndef _GYRO_H
#define _GYRO_H

#include "common.h"
#include <stdbool.h>
#include "led.h"
#include <stdint.h>

typedef struct gyro_struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float pitch_f;
    int32_t pitch;
    float roll_f;
    int32_t roll;
    bool update_display;
} gyro_t;

void gyro_print_data(void);
void gyro_init(void);
uint8_t gyro_read_u8(uint8_t reg);
uint16_t gyro_read_u16(uint8_t reg);
bool gyro_update(void);
void gyro_calc_led_color(led_t *led);

#endif
