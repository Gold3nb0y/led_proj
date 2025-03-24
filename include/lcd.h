#ifndef _LCD_H
#define _LCD_H
#include "stdbool.h"
#include "stdint.h"

/*
 * This file offers up the basic functionality for implementing a TC1602A LCD screen for the STM32
 * Other configuration is necessary for this device to function
 */

#define LCD_ROWS 2
#define LCD_COLS 16
typedef enum{
    LCD_CLEAR           = 1<<0,
    LCD_HOME            = 1<<1,
    LCD_ENTRY_MODE_SET  = 1<<2,
    LCD_DISPLAY_CTRL    = 1<<3,
    LCD_SHIFT           = 1<<4,
    LCD_FUNC_SET        = 1<<5,
    LCD_CGRAM_SET       = 1<<6,
    LCD_DDRAM_SET       = 1<<7,
}LCD_cmd;

void lcd_write_command(LCD_cmd cmd);
void lcd_write_data(uint8_t data);
void lcd_init(void);
bool lcd_check_dirty(void);
void lcd_update_display(void);
void lcd_clear(void);
uint8_t lcd_write_display(uint8_t col, uint8_t row, uint8_t data);
uint8_t lcd_write_string(uint8_t start, char* data);
uint8_t lcd_write_line(char* data, uint8_t line);

#endif
