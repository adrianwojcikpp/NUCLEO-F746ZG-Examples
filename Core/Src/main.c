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

#include "arm_math.h" // CMSIS DSP

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
//#include "sine_wave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ENCODER, BH1750, BMP280_TEMP, BMP280_PRESS, ANALOG_INPUT1, ANALOG_INPUT2
} Input_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB   10
#define TASK  2

#define ADC1_NUMBER_OF_CONV   2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENC2DAC(enc) LINEAR_TRANSFORM(enc, (float)henc1.CounterMin, \
		                                       (float)henc1.CounterMax, \
		                                       0.0f, DAC_REG_MAX)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc1_conv_rslt[ADC1_NUMBER_OF_CONV];

float adc1_fir;

// FIR filter
#define FIR_NUM_TAPS    58

arm_fir_instance_f32 fir;

// Filter coefficients
const float32_t b[FIR_NUM_TAPS] = {
	#include "../../MATLAB/fir_b.csv"
};

// Filter state
float32_t fir_state[FIR_NUM_TAPS] = {
  #include "../../MATLAB/fir_state_init.csv"
};

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
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ENC2DAC(enc));
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
    	static float32_t ain1_x;
    	static float32_t ain1_y;
#if TASK == 2
    	ain1_x = ADC_REG2VOLTAGE(adc1_conv_rslt[0]);
    	arm_fir_f32(&fir, &ain1_x, &ain1_y, 1);
#endif
      n = sprintf(str_buffer, "{\"POT1\":%4d mV}", (int)ain1_y);
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
  /* User interface: low priority */
  if(htim->Instance == TIM10)
  {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_conv_rslt, ADC1_NUMBER_OF_CONV);
  }
}

/**
 * @brief Root mean square error of vector 'y' with vector 'yref' as reference
 * @param[in] y    : Input vector
 * @param[in] yref : Reference vector
 * @param[in] len  : Vectors length
 * @return Root mean square error: sqrt(sum( (yref - y)^2 ))
 */
float32_t RMSE(float32_t* y, float32_t* yref, uint32_t len)
{
	float32_t sum_sq_error = 0;
	for(uint32_t i = 0; i < len; i++)
		sum_sq_error += (yref[i]-y[i])*(yref[i]-y[i]);
	return sqrtf(sum_sq_error / len);
}

int8_t UNIT_TEST_FIR()
{
	/* LOCAL VARIABLES */
	arm_status status = ARM_MATH_TEST_FAILURE;
	float32_t rmse = 1.0;
	const float32_t rmse_max = 1e-6; //1e-10

#define FIR_NUM_SAMPLES 1000

	/* FIR INIT */
	arm_fir_init_f32(&fir, FIR_NUM_TAPS, b, fir_state, 1);

	float32_t x[FIR_NUM_SAMPLES] = {
		#include "../../MATLAB/fir_x.csv"
	};

	// Output
	float32_t y[FIR_NUM_SAMPLES];

	// Reference output
	float32_t yref[FIR_NUM_SAMPLES] = {
		#include "../../MATLAB/fir_yref.csv"
	};

	/* DIGITAL SIGNAL FILTRATION */
	for(uint32_t i = 0; i < FIR_NUM_SAMPLES; i++)
	{
	  arm_fir_f32(&fir, &x[i], &y[i], 1);
	  //SWV_VAR = y[i]; HAL_Delay(0); // for SWV
	}

	/* ROOT MEAN SQUARE ERROR */
	rmse = RMSE(y, yref, FIR_NUM_SAMPLES);

	/* RMSE TRESHOLD */
	if(rmse < rmse_max) status = ARM_MATH_SUCCESS;

	return status;
}

void CMSIS_UnitTests(void)
{
  arm_status TEST_RESULT = ARM_MATH_TEST_FAILURE;
  LCD_SetCursor(&hlcd1, 1, 0);

#if TASK == 0

  float32_t cmplx_var[2] = {1.0f, 1.0f};
  float32_t cmplx_var_mag = 0.0f;

  arm_cmplx_mag_f32(cmplx_var, &cmplx_var_mag, 1);
  float32_t cmplx_var_mag_ref = sqrtf(2.0f);

  if(fabs(cmplx_var_mag - cmplx_var_mag_ref) < 1e-6)
  	TEST_RESULT = ARM_MATH_SUCCESS;

  LCD_printStr(&hlcd1, "TEST #0: ");

#endif

#if TASK == 2
  TEST_RESULT = UNIT_TEST_FIR();
  LCD_printStr(&hlcd1, "TEST #2: ");
#endif

  if(TEST_RESULT == ARM_MATH_SUCCESS)
  {
  	LCD_printStr(&hlcd1, "SUCESS");
  	LED_On(&hledg2);
  }
  else
  {
  	LCD_printStr(&hlcd1, "FAIL");
  	LED_On(&hledr2);
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

  // CMSIS unit testing
  CMSIS_UnitTests();
  // Wait for button
  while(!BTN_EdgeDetected(&hbtn1))
  	HAL_Delay(0);

  // Start UI timer
  HAL_TIM_Base_Start_IT(&htim10);
  // Start analog output
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  //HAL_TIM_Base_Start(&htim6);
  //HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, SINE_WAVE, 100, DAC_ALIGN_12B_R);

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
