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
#include "adc.h"
#include "dac.h"
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
#define _USE_MATH_DEFINES
#include <math.h>

#include "common.h"
#include "led_config.h"
#include "btn_config.h"
#include "encoder_config.h"
#include "lcd_config.h"
#include "lamp_config.h"
#include "led_rgb_config.h"
#include "bh1750_config.h"
#include "bmp2_config.h"
#include "analog_input.h"
#include "analog_output.h"
#include "sine_wave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ENCODER, BH1750, BMP280_TEMP, BMP280_PRESS, ANALOG_INPUT1, ANALOG_INPUT2
} Input_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB   8
#define TASK  3

#define ADC1_TIMEOUT        100 // [ms]
#define ADC1_NUMBER_OF_CONV   2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc1_conv_rslt[ADC1_NUMBER_OF_CONV];

// Harmonic signal parameters
const float ts = 0.001f; // [s]
float A  = 1.0f;   // [V]
const float A0 = 1.0f;   // [V]
const float f  = 10.0f;  // [Hz]
const float T  = 1/f-ts; // [s]

// Harmonic signal
float y = 0.0f; // [V]

// Time variable
float t = 0.0;  // [s]

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Low-level user interface routine.
  *        Embedded display and serial port handling
  */
void ui_routine(void)
{
  static Input_TypeDef input = ANALOG_INPUT1;

  /* Encoder button */
  if(BTN_EdgeDetected(&hbtn3))
    input = (input < ANALOG_INPUT2) ? (input + 1) : (ENCODER);

  /* Selected measurement in JSON format */
  char str_buffer[32];
  int n;

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
      double temp = BMP2_ReadTemperature_degC(&hbmp2_1);
      n = sprintf(str_buffer, "{\"Temp\":%2.02f}  ", temp);
      break;
    }
    case BMP280_PRESS: /* Pressure sensor */
    {
      double press = BMP2_ReadPressure_hPa(&hbmp2_1);
      n = sprintf(str_buffer, "{\"Press\":%4.02f}", press);
      break;
    }
    case ANALOG_INPUT1: /* Analog input #1: potentiometer #1 */
    {
      n = sprintf(str_buffer, "{\"POT1\":%4d mV}", (int)ADC_REG2VOLTAGE(adc1_conv_rslt[0]));
      break;
    }
    case ANALOG_INPUT2: /* Analog input #2: potentiometer #2 */
    {
      n = sprintf(str_buffer, "{\"POT2\":%4d mV}", (int)ADC_REG2VOLTAGE(adc1_conv_rslt[1]));
      break;
    }
  default: break;
  }

  /* Embedded display */
  LCD_SetCursor(&hlcd1, 1, 0);
  LCD_printStr(&hlcd1, str_buffer);

  /* Serial port streaming */
  str_buffer[n] = '\n'; // add new line
  HAL_UART_Transmit(&huart3, (uint8_t*)str_buffer, n+1, 1000);
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
  	ui_routine();
  }
}


/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#if TASK == 3
  /* Analog output: high priority */
  if(htim->Instance == TIM6)
  {
  	if(t == 0)
  		A = ADC_REG2VOLTAGE(adc1_conv_rslt[0]) / 2200.0f;

    y = A*sinf(2*M_PI*f*t) + A0;
    t = (t < T) ? (t+ts) : (0);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_VOLTAGE2REG(y));
  }
#endif
  /* User interface: low priority */
  if(htim->Instance == TIM10)
  {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_conv_rslt, ADC1_NUMBER_OF_CONV);
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Initialize light sensor
  BH1750_Init(&hbh1750_1);
  // Initialize pressure and temperature sensor
  BMP2_Init(&hbmp2_1);
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

#if TASK != 5
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
#endif

#if TASK == 1
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_VOLTAGE2REG(1.0f));
#elif TASK == 3
  HAL_TIM_Base_Start_IT(&htim6);
#elif TASK == 5
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)SINE_WAVE, 100, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim6);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if TASK == 0
    y = A*sinf(2*M_PI*f*t) + A0;
    t = (t < T) ? (t+ts) : (0);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_VOLTAGE2REG(y));
    HAL_Delay(1000*ts - 1);
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
