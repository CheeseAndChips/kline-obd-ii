#include "kline.h"
// #include "lcd.h"

#define COMMAND_BYTES_CNT 6
#define RX_BUFFER_LEN 64
#define KLINE_TIMEOUT 200

enum kline_uart_read_state {
	WAITING_FOR_SYNC,
	WAITING_FOR_KW1,
	WAITING_FOR_KW2,
	WAITING_FOR_NOT_ADDR,	// inverse of KLINE_INIT_ADDRESS
	READ_BYTES,
	IGNORE_BYTES,
};

static TIM_HandleTypeDef *htim = NULL;
static UART_HandleTypeDef *huart = NULL;
volatile uint8_t kline_init_data;
volatile uint8_t kline_init_bits_left = 0;
volatile static enum kline_uart_read_state uart_state;
volatile static uint8_t kw2;	// inverse of the KeyWord2, supplied by the car during initialization

volatile uint32_t last_byte_timestamp;
volatile uint8_t bytes_received[RX_BUFFER_LEN];
volatile uint8_t cnt_received;
volatile enum kline_command_status command_status;
volatile uint8_t expected_bytes;

void kline_setup(TIM_HandleTypeDef *_htim, UART_HandleTypeDef *_huart) {
	htim = _htim;
	huart = _huart;
	uart_state = WAITING_FOR_SYNC;
}

void kline_tim_callback(void) {
	// aaargh
	if(kline_init_bits_left == 9) {
		kline_init_bits_left--;
		return;
	}

	if(!kline_init_bits_left) {
    	HAL_GPIO_WritePin(KLINE_OUT_GPIO_Port, KLINE_OUT_Pin, GPIO_PIN_SET);
		kline_5baud_gpio_deinit();
		HAL_ASSERT(HAL_TIM_Base_Stop_IT(htim));
		//lcd_text_puts("\nDone sending init\n");

		// start listening
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
	} else {
		GPIO_PinState new_state = (kline_init_data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET;
		kline_init_data <<= 1;
		HAL_GPIO_WritePin(KLINE_OUT_GPIO_Port, KLINE_OUT_Pin, new_state);
		//lcd_text_putc(new_state ? '1' : '0');
		kline_init_bits_left--;
	}
}

void kline_run_init() {
	uint8_t address = KLINE_INIT_ADDRESS;
	kline_init_data = ~address;
	kline_init_bits_left = 9;
	HAL_ASSERT(HAL_TIM_Base_Start_IT(htim));
	htim->Instance->CNT = 0;
	HAL_GPIO_WritePin(KLINE_OUT_GPIO_Port, KLINE_OUT_Pin, GPIO_PIN_RESET);
}

void kline_5baud_gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = KLINE_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(KLINE_OUT_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(KLINE_OUT_GPIO_Port, KLINE_OUT_Pin, GPIO_PIN_SET);
}

void kline_5baud_gpio_deinit(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = KLINE_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(KLINE_OUT_GPIO_Port, &GPIO_InitStruct);
}

void handle_new_byte(uint8_t byte) {
	const uint8_t KLINE_INIT_ADDRESS_INVERTED = (~(KLINE_INIT_ADDRESS)) & 0xff;
	switch(uart_state) {
	case WAITING_FOR_SYNC:
		if(byte == KLINE_INIT_SYNC_BYTE) {
			uart_state = WAITING_FOR_KW1;
		} else {
			LOG("WARN: Expecting %x, found %x\n", KLINE_INIT_SYNC_BYTE, byte);
		}
		break;
	case WAITING_FOR_KW1:
		uart_state = WAITING_FOR_KW2;
		break;
	case WAITING_FOR_KW2:
		uart_state = WAITING_FOR_NOT_ADDR;
		kw2 = byte;
		break;
	case WAITING_FOR_NOT_ADDR:
		if(byte != KLINE_INIT_ADDRESS_INVERTED) {
			LOG("WARN: Expecting %x, found %x\n", KLINE_INIT_ADDRESS_INVERTED, byte);
		} else {
			uart_state = IGNORE_BYTES;
		}
		break;
	case READ_BYTES:
		assert(cnt_received < RX_BUFFER_LEN);
		bytes_received[cnt_received++] = byte;
		LOG("%d: %x\n", cnt_received, byte);
		if(cnt_received >= expected_bytes) {
			uart_state = IGNORE_BYTES;
			LOG("SUCCESS %u\n", command_status);
			command_status = COMMAND_SUCCESS;
			LOG("SUCCESS %u\n", command_status);
		}
		last_byte_timestamp = HAL_GetTick();
		break;
	case IGNORE_BYTES:
		break;
	}
}

uint8_t kline_sync_received(void) {
	switch(uart_state) {
	case WAITING_FOR_SYNC:
		return 0;
	default:
		return 1;
	}
}

uint8_t kline_kw2_ready(void) {
	switch(uart_state) {
	case WAITING_FOR_SYNC:
	case WAITING_FOR_KW1:
	case WAITING_FOR_KW2:
		return 0;
	default:
		return 1;
	}
}

uint8_t kline_not_addr_ready(void) {
	switch(uart_state) {
	case WAITING_FOR_SYNC:
	case WAITING_FOR_KW1:
	case WAITING_FOR_KW2:
	case WAITING_FOR_NOT_ADDR:
		return 0;
	default:
		return 1;
	}
}

uint8_t kline_get_kw2(void) {
	assert(kline_kw2_ready());
	return kw2;
}

void kline_write_command(uint8_t *out, uint8_t service, uint8_t pid) {
	out[0] = 0x68;
	out[1] = 0x6a;
	out[2] = 0xf1;
	out[3] = service;
	out[4] = pid;

	uint8_t checksum = 0;
	for(uint8_t i = 0; i < 5; i++) {
		checksum += out[i];
	}

	out[5] = checksum;
}

void kline_send_command(uint8_t service, uint8_t pid, uint8_t _expected_bytes) {
	uint8_t command[COMMAND_BYTES_CNT];
	kline_write_command(command, service, pid);
	command_status = COMMAND_WAITING;
	expected_bytes = _expected_bytes;

	uart_state = IGNORE_BYTES;
	cnt_received = 0;
	printf("Doing send: ");
	for(uint8_t i = 0; i < COMMAND_BYTES_CNT; i++) {
		HAL_UART_Transmit(huart, &command[i], 1, HAL_MAX_DELAY);
		printf("0x%02x ", command[i]);
		HAL_Delay(6);
	}
	uart_state = READ_BYTES;
	printf("\n");
}

enum kline_command_status kline_get_command_status() {
	if(command_status != COMMAND_WAITING) LOG("Status: %d\n", command_status);
	if(command_status == COMMAND_WAITING && HAL_GetTick() - last_byte_timestamp > KLINE_TIMEOUT) {
		LOG("E: Timeout with %d bytes received\n", cnt_received);
		command_status = COMMAND_TIMEOUT;
	}
	if(command_status != COMMAND_WAITING) LOG("Status: %d\n", command_status);

	return command_status;
}

uint8_t kline_get_bytes_received(void) {
	return cnt_received;
}

uint8_t kline_get_byte(uint8_t i) {
	return bytes_received[i];
}
