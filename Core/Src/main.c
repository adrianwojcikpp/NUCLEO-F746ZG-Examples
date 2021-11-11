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

#include "led_config.h"
#include "btn_config.h"
#include "encoder_config.h"
#include "lcd_config.h"
#include "lamp_config.h"
#include "led_rgb_config.h"
#include "bh1750_config.h"
#include "bmp280_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { ENCODER, BH1750, BMP280 } Input_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB   5
#define TASK  4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @brief Linear transformation of 'x' from <amin, amax> to <bmin, bmax>.
 * @param[in] x Input variable
 * @param[in] amin Minimum of input range
 * @param[in] amax Maximum of input range
 * @param[in] bmin Minimum of output range
 * @param[in] bmax Maximum of output range
 * @return Scaled output variable in <bmin, bmax> range
 */
#define LINEAR_TRANSFORM(x,amin,amax,bmin,bmax) (((x-amin)/(amax-amin))*(bmax-bmin)+bmin)

#define ENC_TO_TRIAC_ANGLE(__ENC_HANDLE__, __LAMP_HANDLE__) \
	LINEAR_TRANSFORM((float)ENC_GetCounter(__ENC_HANDLE__), \
	(float)(__ENC_HANDLE__)->CounterMin, \
	(float)(__ENC_HANDLE__)->CounterMax, \
	(float)(__LAMP_HANDLE__)->TriacFiringAngleMin, \
	(float)(__LAMP_HANDLE__)->TriacFiringAngleMax)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Input_TypeDef input = ENCODER;

#if TASK < 6

char cmd_msg[] = "000000";

#elif TASK >= 6

char cmd_msg[] = "000\n";
_Bool rx_flag = 0;
float control;

#endif

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
#if TASK < 6

  	uint32_t color;

  	// Convert C-string to integer
  	sscanf(cmd_msg, "%lx", &color);
  	// Set color
  	LED_RGB_SetColor(&hledrgb1, color);
    // Receive next command
  	HAL_UART_Receive_DMA(&huart3, (uint8_t*)cmd_msg, strlen(cmd_msg));

#else

  	// Convert C-string to float
  	control = atof(cmd_msg);
#if TASK == 6
  	// Set light intensity
  	LED_PWM_SetDuty(&hledw1, control);
#elif TASK == 7
  	// Set TRIAC angle
    hlamp1.TriacFiringAngle = control;
#endif
  	// Rise flag
  	rx_flag = 1;

#endif

  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Dimmer (LAMP) handling */
  if(GPIO_Pin == hlamp1.SYNC_Pin)
  {
#if TASK < 7
  	hlamp1.TriacFiringAngle = ENC_TO_TRIAC_ANGLE(&henc1, &hlamp1);
#endif
  	LAMP_StartTimer(&hlamp1);
  }
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Dimmer (LAMP) handling */
  if(htim->Instance == hlamp1.Timer->Instance)
  {
  	LAMP_StopTimer(&hlamp1);
    LAMP_TriacFiring(&hlamp1);
  }

  /* User interface: low priority */
  if(htim->Instance == TIM10)
  {
  	LCD_SetCursor(&hlcd1, 1, 0);
  	char str_buffer[32];
  	int n;

  	struct bmp280_uncomp_data bmp280_data;
  	int32_t temp;

  	/* Encoder button */
  	if(BTN_EdgeDetected(&hbtn3))
  	  input = (input < BMP280) ? (input + 1) : (ENCODER);

  	/* Selected measurement in JSON format */
  	switch(input)
  	{
  	case ENCODER:
  		n = sprintf(str_buffer, "{\"Encoder\":%3lu} ", ENC_GetCounter(&henc1));
  		break;
  	case BH1750: /* Light sensor */
  		n = sprintf(str_buffer, "{\"Light\":%6d}", (int)BH1750_ReadLux(&hbh1750_1));
  		break;
  	case BMP280: /* Temperature sensor */
  		bmp280_get_uncomp_data(&bmp280_data, &hbmp280_1);
  		bmp280_get_comp_temp_32bit(&temp, bmp280_data.uncomp_temp, &hbmp280_1);
  		n = sprintf(str_buffer, "{\"Temp\":%2d.%02d}  ", (int)(temp/100), (int)(temp%100));
  		break;
  	default: break;
  	}

  	/* Embedded display */
  	LCD_SetCursor(&hlcd1, 1, 0);
  	LCD_printStr(&hlcd1, str_buffer);

#if TASK < 6

  	// Set light intensity
  	LED_PWM_SetDuty(&hledw1, (float)ENC_GetCounter(&henc1));

  	/* Serial port streaming */
  	str_buffer[n] = '\n'; // add new line
  	HAL_UART_Transmit(&huart3, (uint8_t*)str_buffer, n+1, 1000);

#endif

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

  // Initialize PWM-controlled LED
  LED_PWM_Init(&hledw1);
  // Initialize light sensor
  BH1750_Init(&hbh1750_1);
  // Initialize pressure and temperature sensor
  BMP280_Init(&hbmp280_1);
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

#if TASK >= 6

  	if(rx_flag) // Read flag
  	{
  		// Clear flag
  		rx_flag = 0;
  		// Wait 1 second
  		HAL_Delay(1000);
  		// Read light measurement
  		float light = BH1750_ReadLux(&hbh1750_1);
  		// Send response
  		char data_msg[32];
  		int n = sprintf(data_msg, "%3d, %6d\n", (int)control, (int)light);
  		HAL_UART_Transmit(&huart3, (uint8_t*)data_msg, n, 0xffff);
  		// Receive next command
  		HAL_UART_Receive_DMA(&huart3, (uint8_t*)cmd_msg, strlen(cmd_msg));
  	}

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
