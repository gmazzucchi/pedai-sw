#ifndef LCD_1602A_H
#define LCD_1602A_H

#include "ped_config.h"

void lcd_1602a_init(void);
void lcd_1602a_write_text(const char *str);
void lcd_1602a_clear_screen(void);

#endif  // LCD_1602A_H
