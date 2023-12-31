#ifndef INC_LCD_H_
#define INC_LCD_H_
#include <stdint.h>

typedef struct {
	uint8_t row, col;
} cursor_pos_t;

void lcd_init(void);
void lcd_text_init(void);
// void lcd_text_newline(void);
// void lcd_text_backspace(void);
void lcd_text_update_cursor(void);
void lcd_clear(void);
void lcd_text_putc(char c);
void lcd_text_puts(const char *str);
void lcd_text_printf(const char *format, ...);
uint8_t lcd_text_lines_left(void);

#endif /* INC_LCD_H_ */
