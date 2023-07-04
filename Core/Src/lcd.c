#include "lcd.h"
#include "main.h"

/* FONT BEGIN */

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

/* FONT END */

#define GET_GPIO_PORT(pin) (pin ## _GPIO_Port)
#define GET_GPIO_PIN_NUM(pin) (pin ## _Pin)

#define GET_GPIO_MASK(pin) GET_GPIO_PIN_NUM(pin)
#define GET_GPIO_MASK_INV(pin) (0xffff ^ GET_GPIO_MASK(pin))

#define SET_H(pin) GET_GPIO_PORT(pin)->ODR |= GET_GPIO_MASK(pin)
#define SET_L(pin) GET_GPIO_PORT(pin)->ODR &= GET_GPIO_MASK_INV(pin)
#define SET_PIN(pin, state) HAL_GPIO_WritePin(pin ## _GPIO_Port, pin ## _Pin, state)
#define GET_BSRR_MASK(pin, data) (GET_GPIO_MASK(LCD_D ## pin) << (16 * !(((data) >> pin) & 1)))

void lcd_write(uint8_t data) {
	SET_L(LCD_WR);

	GPIOF->BSRR |= \
		GET_BSRR_MASK(0, data) | \
		GET_BSRR_MASK(2, data) | \
		GET_BSRR_MASK(4, data) | \
		GET_BSRR_MASK(7, data);

	GPIOE->BSRR |= \
		GET_BSRR_MASK(3, data) | \
		GET_BSRR_MASK(5, data) | \
		GET_BSRR_MASK(6, data);

	GPIOD->BSRR |= \
		GET_BSRR_MASK(1, data);

//	SET_PIN(LCD_D0, (data >> 0) & 1);
//	SET_PIN(LCD_D1, (data >> 1) & 1);
//	SET_PIN(LCD_D2, (data >> 2) & 1);
//	SET_PIN(LCD_D3, (data >> 3) & 1);
//	SET_PIN(LCD_D4, (data >> 4) & 1);
//	SET_PIN(LCD_D5, (data >> 5) & 1);
//	SET_PIN(LCD_D6, (data >> 6) & 1);
//	SET_PIN(LCD_D7, (data >> 7) & 1);

	SET_H(LCD_WR);
}

static inline void lcd_command_write(uint8_t command) {
	SET_L(LCD_RS);
	lcd_write(command);
}

static inline void lcd_data_write(uint8_t data) {
	SET_H(LCD_RS);
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
	SET_L(LCD_CS);
	lcd_set_address(y, y+1, x, x+1);
	lcd_command_write(0x2c);
	lcd_data_write(color >> 8);
	lcd_data_write(color);
	SET_H(LCD_CS);
}

void lcd_init(void) {
	SET_H(LCD_RST);
	HAL_Delay(20);
	SET_L(LCD_RST);
	HAL_Delay(20);
	SET_H(LCD_RST);
	HAL_Delay(20);

	SET_H(LCD_CS);
	SET_H(LCD_WR);
	SET_H(LCD_RD);
	SET_L(LCD_CS);

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
	SET_H(LCD_CS);
}
