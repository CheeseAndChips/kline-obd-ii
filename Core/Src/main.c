/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "kline.h"
#include "usbh_hid.h"
#include "usbh_hid_keybd.h"
#include "hid_keys.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART4_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim != &htim7)
		return;

	kline_tim_callback();
}

char stdinbuffer[256];
extern ApplicationTypeDef Appli_state;
extern cursor_pos_t cursor_pos;
void update_lcd_command(uint8_t);
//extern USBH_HandleTypeDef hUsbHostFS;

uint8_t buttons_held[256 / 8];

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost) {
	HID_KEYBD_Info_TypeDef *k_pinfo = USBH_HID_GetKeybdInfo(phost);
	if(k_pinfo != NULL) {
		uint8_t new_button = 0;
		for(uint8_t i = 0; i < sizeof(k_pinfo->keys); i++) {
			char c = k_pinfo->keys[i];
			if(c) {
				uint8_t ind = c / 8;
				uint8_t shift = c % 8;
				if((buttons_held[ind] >> shift) == 0) {
					new_button = c;
				}
			}
		}

		memset(buttons_held, 0, sizeof(buttons_held));

		for(uint8_t i = 0; i < sizeof(k_pinfo->keys); i++) {
			uint8_t c = k_pinfo->keys[i];
			if(c) {
				uint8_t ind = c / 8;
				uint8_t shift = c % 8;
				buttons_held[ind] |= (1 << shift);
			}
		}

		if(new_button) {
			/*if(new_button == KEY_ENTER && main_status == MAIN_LOOP_WAITING_FOR_ENTER) {
				main_status = MAIN_LOOP_RUNNING_INIT;
			} else if(new_button == KEY_SPACEBAR && expecting_chars == 0) {
				update_single_char('U', 0);
				expecting_chars = 2;
			} else if(expecting_chars) {
				char ascii_pressed = HID_KEYBRD_Key[HID_KEYBRD_Codes[new_button]];
				if(ascii_pressed >= '0' && ascii_pressed <= '9') {
					expecting_chars--;
					new_command = (new_command << 4) | (ascii_pressed - '0');
					update_single_char(ascii_pressed, 1);
				} else if (ascii_pressed >= 'a' && ascii_pressed <= 'f') {
					expecting_chars--;
					new_command = (new_command << 4) | (ascii_pressed - 'a' + 10);
					update_single_char(ascii_pressed, 1);
				} else if (ascii_pressed >= 'A' && ascii_pressed <= 'F') {
					expecting_chars--;
					new_command = (new_command << 4) | (ascii_pressed - 'A' + 10);
					update_single_char(ascii_pressed, 1);
				}

				if(!expecting_chars) {
					main_status = MAIN_LOOP_COMMANDS;
					current_command = new_command;
					update_lcd_command(new_command);
				}
			}*/

			/*
			uint8_t result;
			if(k_pinfo->lshift || k_pinfo->rshift) {
				result = HID_KEYBRD_ShiftKey[HID_KEYBRD_Codes[new_button]];
			} else {
				result = HID_KEYBRD_Key[HID_KEYBRD_Codes[new_button]];
			}
			if(result)
				lcd_text_putc(result);
			*/
		}
	}
}

/*void update_lcd_command(uint8_t pid) {
	lcd_clear();
	cursor_pos.row = 0;
	cursor_pos.col = 0;
	lcd_text_update_cursor();

	kline_update_command(pid);

	lcd_text_puts("Kline command: ");
	printf("Kline command: ");
	for(int i = 0; i < kline_command_len; i++) {
		lcd_text_printf("0x%02x ", kline_command[i]);
		printf("0x%02x ", kline_command[i]);
	}
	lcd_text_putc('\n');
	printf("\n");
}*/

enum main_loop_state {
	MAIN_LOOP_DELAY,
	MAIN_LOOP_GPIO_INIT,
	MAIN_LOOP_RUN_INIT,
	MAIN_LOOP_WAIT_FOR_RESPONSE,
	MAIN_LOOP_WAIT_UNTIL_INIT_DONE,
};

enum main_loop_state main_status;
struct {
	uint32_t switch_timestamp;
	enum main_loop_state new_state;
} delay_info;

void start_delay(uint32_t time, enum main_loop_state new_state) {
	delay_info.switch_timestamp = HAL_GetTick() + time;
	delay_info.new_state = new_state;
	main_status = MAIN_LOOP_DELAY;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  setvbuf(stdin, stdinbuffer, _IOLBF, sizeof(stdinbuffer));

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  printf("\n----- PROGRAM BEGIN -----\n");
  fflush(stdout);

  lcd_init();
  kline_setup(&htim7, &huart4);
  kline_5baud_gpio_init();
  start_delay(6000, MAIN_LOOP_GPIO_INIT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  const uint8_t STARTING_ADDRESS = 0x30;
  uint8_t write_bytes = 0;
	uint32_t timestamp = 0;
	uint8_t address = STARTING_ADDRESS;

	kline_run_init(0xf7);
	while(!kline_init_done())
		;
	while(1) { }

  while (1)
  {
	// ...
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
	/*if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET) {
		if(main_status == MAIN_LOOP_DELAY) {
			main_status = MAIN_LOOP_SEND_REGULAR_COMMAND;
		}
	}*/

	switch(main_status) {
	case MAIN_LOOP_DELAY:
		if(HAL_GetTick() > delay_info.switch_timestamp) {
			main_status = delay_info.new_state;
		}
		break;
	case MAIN_LOOP_GPIO_INIT:
		start_delay(50, MAIN_LOOP_RUN_INIT);
		break;
	case MAIN_LOOP_RUN_INIT:
		if(timestamp && address == STARTING_ADDRESS){
			printf("Done. %u %u\n", timestamp, address);
			Error_Handler();
		}
		printf("Address 0x%02x ", address);
		fflush(stdout);
		kline_run_init(address);
		address++;
		main_status = MAIN_LOOP_WAIT_UNTIL_INIT_DONE;
		break;
	case MAIN_LOOP_WAIT_UNTIL_INIT_DONE:
		if(kline_init_done()) {
			main_status = MAIN_LOOP_WAIT_FOR_RESPONSE;
			timestamp = HAL_GetTick();
		}
		break;
	case MAIN_LOOP_WAIT_FOR_RESPONSE:
		if(HAL_GPIO_ReadPin(KLINE_IN_GPIO_Port, KLINE_IN_Pin) == GPIO_PIN_RESET) {
			printf(" OK!\n");
			start_delay(10000, MAIN_LOOP_GPIO_INIT);
		} else if(HAL_GetTick() > timestamp + 1100) {
			putchar('\n');
			main_status = MAIN_LOOP_GPIO_INIT;
		}
		fflush(stdout);

		break;
	}

	/*
	printf("Sending data\n");
	kw2_state = WAITING_FOR_SYNC;
	printf("Running init with address " STR(KLINE_INIT_ADDRESS) "\n");
	kline_run_init(KLINE_INIT_ADDRESS);

	while(kw2_state != WAITING_FOR_NOT_ADDR)
	  ;

	HAL_Delay(30);

	uint8_t data = not_kw2;
	HAL_ASSERT(HAL_UART_Transmit(&huart4, &data, 1, 1000));
	lcd_text_printf("Sent 0x%x\n", data);

	while(kw2_state != INIT_DONE)
	  ;


	HAL_Delay(2000);
	lcd_text_puts("Init successful\n");
	HAL_Delay(2000);
	update_lcd_command(current_command);
	lcd_clear();
	main_status = MAIN_LOOP_SEND_SPEED;
	*/
  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 479;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 34999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 4800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_CS_Pin|LCD_RST_Pin|LCD_D0_Pin|LCD_D7_Pin
                          |LCD_D4_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_WR_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_D6_Pin|LCD_D5_Pin|LCD_D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_D1_GPIO_Port, LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RST_Pin LCD_D0_Pin LCD_D7_Pin
                           LCD_D4_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RST_Pin|LCD_D0_Pin|LCD_D7_Pin
                          |LCD_D4_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_WR_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LCD_WR_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RD_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_RD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D6_Pin LCD_D5_Pin LCD_D3_Pin */
  GPIO_InitStruct.Pin = LCD_D6_Pin|LCD_D5_Pin|LCD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_D1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
	uint8_t c = ch;
	HAL_UART_Transmit(&huart3, &c, 1, 0xffff);
	return 1;
}

int __io_getchar(void) {
	uint8_t data;
	HAL_UART_Receive(&huart3, &data, 1, HAL_MAX_DELAY);
	return data;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  fflush(stdout);
  __disable_irq();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  while (1)
  {
	  for(uint32_t i = 0; i < 1e6; i++)
		  ;
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
