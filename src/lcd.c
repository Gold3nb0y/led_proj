#include "lcd.h"
#include "common.h"
#include <stdint.h>
#include "libopencm3/stm32/f4/gpio.h"

uint8_t lcd_display[LCD_COLS][LCD_ROWS];
bool lcd_dirty;

void _write_nibble(uint8_t nibble){
    gpio_clear(LCD_GPIO, DATA0 | DATA1 | DATA2 | DATA3); //send the data accross
    if(nibble & 1) gpio_set(LCD_GPIO, DATA0);
    if(nibble & 2) gpio_set(LCD_GPIO, DATA1);
    if(nibble & 4) gpio_set(LCD_GPIO, DATA2);
    if(nibble & 8) gpio_set(LCD_GPIO, DATA3);
    gpio_set(LCD_CMD_GPIO, LCD_ENABLE);
    msleep(5);

    gpio_clear(LCD_CMD_GPIO, LCD_ENABLE); 
}

void _write_lcd(bool rs, bool rw, uint8_t data){
    uint8_t low;
    uint8_t high;
    low = data & 0xf;
    high = (data & 0xf0) >> 4;

    GPIO_ODR(LCD_CMD_GPIO) |= ((rw ? LCD_RW : 0) | (rs ? LCD_RS : 0)); //clear read and write

    _write_nibble(high);
    _write_nibble(low);

    GPIO_ODR(LCD_CMD_GPIO) &= ~(LCD_RW | LCD_RS); //clear read and write
}

//init in 4 bit mode
void lcd_init(void){
    //init gpio
    gpio_mode_setup(LCD_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DATA0 | DATA1 | DATA2 | DATA3); //setup all values
    gpio_set_output_options(LCD_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, DATA0 | DATA1 | DATA2 | DATA3);
    gpio_mode_setup(LCD_CMD_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LCD_ENABLE | LCD_RS | LCD_RW);
    gpio_set_output_options(LCD_CMD_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, LCD_ENABLE | LCD_RS | LCD_RW);

    _write_nibble(0x3);
    msleep(5);

    _write_nibble(0x3);
    msleep(5);

    _write_nibble(0x3);
    msleep(5);

    _write_nibble(0x2);
    msleep(5);

    lcd_write_command(LCD_FUNC_SET | 0x8);
    lcd_write_command(LCD_DISPLAY_CTRL | 0x4);
    lcd_write_command(LCD_ENTRY_MODE_SET | 0x2);
    lcd_write_command(LCD_CLEAR);
}

void lcd_write_command(LCD_cmd cmd){
    _write_lcd(0, 0, cmd);
}

void set_ddram_addr(uint8_t address){
    uint8_t addr = address & 0x7f;
    lcd_write_command(LCD_DDRAM_SET | addr);
}

void lcd_write_data(uint8_t data){
    _write_lcd(1, 0, data);
}

void lcd_update_display(void){
    uint8_t j = 0;

    if(!lcd_dirty)
        return;
    
    lcd_write_command(LCD_CLEAR);
    lcd_write_command(LCD_HOME);

    for(; j < LCD_COLS; j++){
        lcd_write_data(lcd_display[j][0]);
    }
    set_ddram_addr(0x40); 
    msleep(5);
    for(j = 0; j < LCD_COLS; j++){
        lcd_write_data(lcd_display[j][1]);
    }

    lcd_dirty = false;
}

void lcd_clear(void){
    memset(lcd_display, ' ', sizeof(lcd_display));
    lcd_write_command(LCD_CLEAR);
}

uint8_t lcd_write_display(uint8_t col, uint8_t row, uint8_t data){
    if(col >= LCD_COLS || row >= LCD_ROWS)
        return -1;

    lcd_display[col][row] = data;
    lcd_dirty = true;
    return 0;
}

bool inline lcd_check_dirty(void){
    return lcd_dirty;
}

uint8_t lcd_write_string(uint8_t start, char* data){
    uint8_t len;
    uint8_t end;

    len = strlen(data);
    end = len + start;
    if(end >= 0x20){
        end = 0x1f;
    }

    for(uint8_t i = start; i < end; i++){
        if(i < 0x10) {
            lcd_write_display(i%LCD_COLS, 0, data[i-start]);
        } else {
            lcd_write_display(i%LCD_COLS, 1, data[i-start]);
        }
    }
    return len;
}

uint8_t lcd_write_line(char* data, uint8_t line){
    if(line >= LCD_ROWS)
        return -1;

    lcd_write_string((line ? 0x10 : 0), data);

    return 0;
}
