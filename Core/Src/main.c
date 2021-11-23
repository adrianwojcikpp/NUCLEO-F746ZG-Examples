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
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "common.h"
#include "led_config.h"
#include "btn_config.h"
#include "encoder_config.h"
#include "lcd_config.h"
#include "lamp_config.h"
#include "led_rgb_config.h"
#include "bh1750_config.h"

#define BMP2_VER_2021

#ifdef BMP2_VER_2021
#include "bmp2_config.h"    // Active library
#else
#include "bmp280_config.h"  // Archive library
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { ENCODER, BH1750, BMP280_TEMP, BMP280_PRESS  } Input_TypeDef;
typedef PWM_OUTPUT_HandleTypeDef HEATER_HandleTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB   6
#define TASK  4

#define HEATER_Init     PWM_OUTPUT_Init
#define HEATER_SetPower PWM_OUTPUT_SetDuty
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
HEATER_HandleTypeDef hheater1 = {
	.Timer = &htim4, .Channel = TIM_CHANNEL_4, .Duty = 0
};
Input_TypeDef input = ENCODER;
char cmd_msg[] = "000\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
#if TASK >= 5
  	// Convert C-string to float
  	control = atof(cmd_msg);
  	// Heater power
  	HEATER_SetPower(&hheater1, control);
  	// Rise flag
  	rx_flag = 1;
#endif
  }
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* User interface: low priority */
  if(htim->Instance == TIM10)
  {
  	LCD_SetCursor(&hlcd1, 1, 0);
  	char str_buffer[32];
  	int n;

  	/* Encoder button */
  	if(BTN_EdgeDetected(&hbtn3))
  	  input = (input < BMP280_PRESS) ? (input + 1) : (ENCODER);

  	/* Selected measurement in JSON format */
  	switch(input)
  	{
			case ENCODER:
			{
				uint32_t enc = ENC_GetCounter(&henc1);
				n = sprintf(str_buffer, "{\"Encoder\":%3lu} ", enc);
				break;
			}
			case BH1750: /* Light sensor */
			{
				float light = BH1750_ReadIlluminance_lux(&hbh1750_1);
				n = sprintf(str_buffer, "{\"Light\":%6d}", (int)light);
				break;
			}
			case BMP280_TEMP: /* Temperature sensor */
			{
				#ifdef BMP2_VER_2021
				double temp = BMP2_ReadTemperature_degC(&hbmp2_1);
				n = sprintf(str_buffer, "{\"Temp\":%2.02f,\"Duty\":%3d}", temp, (int)ENC_GetCounter(&henc1));
				break;
				#else
				float temp = BMP280_ReadTemperature_degC(&hbmp280_1);
				n = sprintf(str_buffer, "{\"Temp\":%2.02f,\"Duty\":%3d}", temp, (int)ENC_GetCounter(&henc1));
				break;
				#endif
  		}
			case BMP280_PRESS: /* Pressure sensor */
			{
				#ifdef BMP2_VER_2021
				double press = BMP2_ReadPressure_hPa(&hbmp2_1);
				n = sprintf(str_buffer, "{\"Press\":%4.02f}", press);
				break;
				#else
				float press = BMP280_ReadPressure_hPa(&hbmp280_1);
				n = sprintf(str_buffer, "{\"Press\":%4.02f}", press);
				break;
				#endif
  		}
  	default: break;
  	}

  	/* Embedded display */
  	LCD_SetCursor(&hlcd1, 1, 0);
  	LCD_printStr(&hlcd1, str_buffer);

#if TASK < 5
  	// Heater power
  	HEATER_SetPower(&hheater1, (float)ENC_GetCounter(&henc1));
#endif

  	/* Serial port streaming */
  	str_buffer[n] = '\n'; // add new line
  	HAL_UART_Transmit(&huart3, (uint8_t*)str_buffer, n+1, 1000);
  }
}

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
  MX_TIM2_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ETH_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */

  // Initialize PWM-controlled heater (power resistor)
  HEATER_Init(&hheater1);
  // Initialize light sensor
  BH1750_Init(&hbh1750_1);

#ifdef BMP2_VER_2021
  // Initialize pressure and temperature sensor
  BMP2_Init(&hbmp2_1);
#else
	// Initialize pressure and temperature sensor
  BMP280_Init(&hbmp280_1);
#endif

  // Initialize LCD1
  LCD_Init(&hlcd1);
  // Print laboratory task info on LCD1
  LCD_printf(&hlcd1, "L%02d: TASK %d", LAB, TASK);
  // Initialize RGB LED
  LED_RGB_Init(&hledrgb1);
  // Set RGB LED color: disable all channels
  LED_RGB_SetColor(&hledrgb1, 0x000000);
  // Initialize rotary encoder
  ENC_Init(&henc1);
  // Start UI timer
  HAL_TIM_Base_Start_IT(&htim10);
  // Start UI serial port
  HAL_UART_Receive_DMA(&huart3, (uint8_t*)cmd_msg, strlen(cmd_msg));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
