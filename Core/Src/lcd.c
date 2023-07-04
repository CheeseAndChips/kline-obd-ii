#include "lcd.h"
#include "main.h"

/* FONT FUNCTIONS BEGIN */

#define FONT_H 8
#define FONT_W 5

// size of a single character in bytes
static const int CHAR_SIZE = (FONT_H * FONT_W) / 8 + (((FONT_H * FONT_W) % 8) != 0);

extern char _binary_font_hex_start;
extern char _binary_font_hex_end;
extern char _binary_font_hex_size;

size_t get_font_size() {
	return (size_t)(&_binary_font_hex_size);
}

const char *get_char_data(uint8_t c) {
	const char *start = &_binary_font_hex_start;
	const char *ret = start + c * CHAR_SIZE;
	assert(ret <= &_binary_font_hex_end);
	return ret;
}

/* FONT FUNCTIONS END */

#define SET_PIN(pin, state) HAL_GPIO_WritePin(pin ## _GPIO_Port, pin ## _Pin, state)

void lcd_write(uint8_t data) {
	SET_PIN(LCD_WR, 0);

	SET_PIN(LCD_D0, (data >> 0) & 1);
	SET_PIN(LCD_D1, (data >> 1) & 1);
	SET_PIN(LCD_D2, (data >> 2) & 1);
	SET_PIN(LCD_D3, (data >> 3) & 1);
	SET_PIN(LCD_D4, (data >> 4) & 1);
	SET_PIN(LCD_D5, (data >> 5) & 1);
	SET_PIN(LCD_D6, (data >> 6) & 1);
	SET_PIN(LCD_D7, (data >> 7) & 1);

	SET_PIN(LCD_WR, 1);
}

static inline void lcd_command_write(uint8_t command) {
	SET_PIN(LCD_RS, 0);
	lcd_write(command);
}

static inline void lcd_data_write(uint8_t data) {
	SET_PIN(LCD_RS, 1);
	lcd_write(data);
}

void lcd_set_address(int16_t y1, int16_t y2, int16_t x1, int16_t x2) {
	 lcd_command_write(0x2a);
	 lcd_data_write(y1 >> 8);
	 lcd_data_write(y1);
	 lcd_data_write(y2 >> 8);
	 lcd_data_write(y2);

	 lcd_command_write(0x2b);
	 lcd_data_write(x1 >> 8);
	 lcd_data_write(x1);
	 lcd_data_write(x2 >> 8);
	 lcd_data_write(x2);
}

void lcd_set_pixel(int16_t x, int16_t y, int16_t color) {
	SET_PIN(LCD_CS, 0);
	lcd_set_address(y, y+1, x, x+1);
	lcd_command_write(0x2c);
	lcd_data_write(color >> 8);
	lcd_data_write(color);
	SET_PIN(LCD_CS, 1);
}

void lcd_init(void) {
	SET_PIN(LCD_RST, 1);
	HAL_Delay(20);
	SET_PIN(LCD_RST, 0);
	HAL_Delay(20);
	SET_PIN(LCD_RST, 1);
	HAL_Delay(20);

	SET_PIN(LCD_CS, 1);
	SET_PIN(LCD_WR, 1);
	SET_PIN(LCD_RD, 1);
	SET_PIN(LCD_CS, 0);

	// VCOM1
	lcd_command_write(0xc5);
	lcd_data_write(0x3f);
	lcd_data_write(0x3c);

	// Memory access control
	lcd_command_write(0x36);
	lcd_data_write(0x00);

	// pixel format
	lcd_command_write(0x3a);
	lcd_data_write(0x55); // 16 bit

	lcd_command_write(0x11); // disable sleep
	HAL_Delay(10);
	lcd_command_write(0x29); // display on

	lcd_command_write(0x2c); // write memory
	for(int i = 0; i < 2*240*320; i++) {
		lcd_data_write(0x0);
	}

	lcd_set_address(10, 9 + FONT_H, 0, 320);
	lcd_command_write(0x2c); // write memory
	for(int curr_ch = '0'; curr_ch < 'z'; curr_ch++){
		if(curr_ch == '0' + 40) {
			lcd_set_address(20, 19 + FONT_H, 0, 320);
			lcd_command_write(0x2c);
		}

		const char *ch = get_char_data(curr_ch - ' ');
		for(int i = 0; i < CHAR_SIZE; i++) {
			printf("%x ", ch[i]);
		}
		putc('\n', stdout);
		int8_t shift = 7;
		for(int i = 0; i < FONT_W * FONT_H; i++) {
			if(*ch & (1 << shift)) {
				lcd_data_write(0xff);
				lcd_data_write(0xff);
			} else {
				lcd_data_write(0x00);
				lcd_data_write(0x00);
			}
			if(--shift < 0) {
				shift = 7;
				ch++;
			}
		}
	}
	SET_PIN(LCD_CS, 1);
}
