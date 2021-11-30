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
  * TODO : analog_input compontnet
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
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
#include "bmp2_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ENCODER, BH1750, BMP280_TEMP, BMP280_PRESS, ANALOG_INPUT1, ANALOG_INPUT2
} Input_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB   7
#define TASK  5

#define ADC_BIT_RES      12      // [bits]
#define ADC_REG_MAX      (float)((1ul << ADC_BIT_RES) - 1)
#define ADC_VOLTAGE_MAX  3.3f    // [V]

#define ADC1_NUMBER_OF_CONV  2
#define ADC1_TIMEOUT         100 // [ms]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//! ADC ----------------------------------------------------------------------

/**
 * @brief ADC data register to voltage in millivolts.
 * @param[in] reg  Data register
 * @return Input voltage in millivolts
 */
#define ADC_REG2VOLTAGE(reg) (uint32_t)(1000*LINEAR_TRANSFORM((float)reg,  \
                                                     0.0f, ADC_REG_MAX,    \
                                                     0.0f, ADC_VOLTAGE_MAX))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc1_conv_rslt[ADC1_NUMBER_OF_CONV];
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

#if TASK >= 5
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
#if TASK == 5
    // Reading 'adc1_conv_rank' conversion result from Data Register
    adc1_conv_rslt[hadc->NbrOfCurrentConversionRank] = HAL_ADC_GetValue(&hadc1);

    // Increment rank
    hadc->NbrOfCurrentConversionRank++;
    if( hadc->NbrOfCurrentConversionRank == hadc->Init.NbrOfConversion )
    	hadc->NbrOfCurrentConversionRank = 0;

    // Update UI after last conversion
    if(hadc->NbrOfCurrentConversionRank == 0) /* Refresh rate divided by ADC1_NUMBER_OF_CONV ! */
#endif
      ui_routine();
  }
}
#endif

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
    /* Starting analog inputs conversions  */
#if TASK == 4

    // Blocking mode
    HAL_ADC_Start(&hadc1);

    // Iterate over all conversions
    for(hadc1.NbrOfCurrentConversionRank = 0;
    		hadc1.NbrOfCurrentConversionRank < hadc1.Init.NbrOfConversion;
    		hadc1.NbrOfCurrentConversionRank++)
    {
      // Poll for i-ranked conversion
      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
      {
        // Reading i-ranked conversion result from Data Register
        adc1_conv_rslt[hadc1.NbrOfCurrentConversionRank] = HAL_ADC_GetValue(&hadc1);
      }
    }

    ui_routine();

#elif TASK == 5

    // Non-blocking mode #1: interrupt
    HAL_ADC_Start_IT(&hadc1);

#elif TASK == 6

    // Non-blocking mode #2: direct memory access
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_conv_rslt, ADC1_NUMBER_OF_CONV);

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
  MX_ADC1_Init();
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
