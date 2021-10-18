/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "btn_config.h"
#include "encoder_config.h"
#include "lcd_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB   2
#define TASK  7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if TASK == 4

char msg_str[32];
int msg_len = 1;

#endif

#if TASK == 5

char msg_str[32];
int msg_len = 5;

#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#if TASK == 5

/**
  * @brief  This function parse LED control message
  * @note   Control message is 5 characters long with first two characters being "LD",
  *         third defines LED number ("1", "2", "3"), fourth is "_" separator and
  *         fifth defines LED state ("0" or "1")
  * @param[in] msg_str     : control message C-string
  * @param[out] led_number : number of LED (1 : Green, 2 : Blue, 3 : Red)
  * @param[out] led_state  : state of LED (0 : Off, 1 : On)
  * @retval None
  */
void parse_control_message(char* msg_str, int* led_number, _Bool* led_state);

/**
  * @brief  This function control LEDs
  * @param[in] led_number : number of LED (1 : Green, 2 : Blue, 3 : Red)
  * @param[in] led_state  : state of LED (0 : Off, 1 : On)
  * @retval None
  */
void control_leds(int led_number, _Bool led_state);

#endif



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if TASK == 4

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		HAL_UART_Transmit_IT(&huart3, (uint8_t*)msg_str, msg_len);
		//HAL_UART_Transmit(&huart3, (uint8_t*)msg_str, msg_len, 100 /* ms */);
		//HAL_UART_Receive_IT(&huart3, (uint8_t*)msg_str, msg_len);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, (uint8_t*)msg_str, msg_len);
	}
}

#endif

#if TASK == 5

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		int led_number = 0;   /* 0 : None / 1 : Green / 2 : Blue / 3 : Red */
		_Bool led_state = 0;  /* 0 : Off  / 1 : On  */

		parse_control_message(msg_str, &led_number, &led_state);
		control_leds(led_number, led_state);

		HAL_UART_Receive_IT(&huart3, (uint8_t*)msg_str, msg_len);
	}
}

#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */

  // Initialize LCD1
  LCD_Init(&hlcd1);
  // Print laboratory task info on LCD1
  LCD_printf(&hlcd1, "L%02d: TASK %d", LAB, TASK);

#if TASK == 1

  char msg_str[] = "Hello from STM32!\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_str, sizeof(msg_str), 100 /* ms */);

#endif

#if TASK == 2

  int n = 0;

#endif

#if TASK == 4 || TASK == 5

	HAL_UART_Receive_IT(&huart3, (uint8_t*)msg_str, msg_len);

#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

#if TASK == 2

  	if(BTN_EdgeDetected(&hbtn1))
  	{
  		n++;
  		char msg_str[32];
  		int msg_len = sprintf(msg_str, "Hello #%d\n", n);
  		HAL_UART_Transmit(&huart3, (uint8_t*)msg_str, msg_len, 100 /* ms */);
  	}
  	HAL_Delay(100);

#endif

#if TASK == 3

  	char msg_char;
  	if(HAL_UART_Receive(&huart3, &msg_char, sizeof(msg_char), 100 /* ms */) == HAL_OK)
  		HAL_UART_Transmit(&huart3, &msg_char, sizeof(msg_char), 100 /* ms */);

#endif

#if TASK == 7

  	char msg_char[32];
  	scanf("%s", msg_char);
  	printf("%s", msg_char);

#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if TASK == 5

/**
  * @brief  This function parse LED control message
  * @note   Control message is 5 characters long with first two characters being "LD",
  *         third defines LED number ("1", "2", "3"), fourth is "_" separator and
  *         fifth defines LED state ("0" or "1")
  * @param[in] msg_str     : control message C-string
  * @param[out] led_number : number of LED (1 : Green, 2 : Blue, 3 : Red)
  * @param[out] led_state  : state of LED (0 : Off, 1 : On)
  * @retval None
  */
void parse_control_message(char* msg_str, int* led_number, _Bool* led_state)
{
	int ld_n = 0, ld_s = 0;
	if(msg_str[0] == 'L' && msg_str[1] == 'D')
		sscanf(&msg_str[2], "%d_%d", &ld_n, &ld_s);

	if(ld_n >= 1 && ld_n <= 3)
		*led_number = ld_n;

	if(ld_s == 1 || ld_s == 0)
		*led_state = (_Bool)ld_s;
}

/**
  * @brief  This function control LEDs
  * @param[in] led_number : number of LED (1 : Green, 2 : Blue, 3 : Red)
  * @param[in] led_state  : state of LED (0 : Off, 1 : On)
  * @retval None
  */
void control_leds(int led_number, _Bool led_state)
{
	switch(led_number)
	{
		case 1:
			HAL_GPIO_WritePin(LD1EX_GPIO_Port, LD1EX_Pin, led_state);
			break;
		case 2:
			HAL_GPIO_WritePin(LD2EX_GPIO_Port, LD2EX_Pin, led_state);
			break;
		case 3:
			HAL_GPIO_WritePin(LD3EX_GPIO_Port, LD3EX_Pin, led_state);
			break;
		default:
			break;
	}
}

#endif

#if TASK == 7

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, 0xFFFF);
  return len;
}

int _read(int file, char *ptr, int len)
{
	int msg_len = 0;
	while(msg_len <= len)
	{
		if(HAL_UART_Receive(&huart3, (uint8_t*)ptr, 1, 0xFFFF) == HAL_OK)
		{
			msg_len++;
			if(*ptr == '\r')
				break;
			ptr++;
		}
	}
  return msg_len;
}

#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
