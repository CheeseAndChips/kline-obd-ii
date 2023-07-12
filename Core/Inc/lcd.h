#ifndef INC_LCD_H_
#define INC_LCD_H_

void lcd_init(void);
void lcd_text_init(void);
// void lcd_text_newline(void);
// void lcd_text_backspace(void);
void lcd_text_write_symbol(char c);
void lcd_text_write_string(const char *str);

#endif /* INC_LCD_H_ */
