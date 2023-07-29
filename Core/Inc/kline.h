#ifndef KLINE_H
#define KLINE_H
#include "main.h"

enum kline_command_status {
	COMMAND_WAITING,
	COMMAND_TIMEOUT,
	COMMAND_SUCCESS,
};

void kline_setup(TIM_HandleTypeDef *_htim, UART_HandleTypeDef *_huart);
void kline_tim_callback(void);
void kline_run_init();
void kline_5baud_gpio_init(void);
void kline_5baud_gpio_deinit(void);
enum kline_command_status kline_get_command_status();
uint8_t kline_sync_received(void);
uint8_t kline_get_kw2(void);
uint8_t kline_kw2_ready(void);
uint8_t kline_not_addr_ready(void);
void kline_send_command(uint8_t service, uint8_t pid, uint8_t expected_bytes);
uint8_t kline_get_bytes_received(void);
uint8_t kline_get_byte(uint8_t i);

#endif
