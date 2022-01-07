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
#include "arm_math.h" // CMSIS DSP

#include "common.h"
#include "led_config.h"
#include "led_rgb_config.h"
#include "lamp_config.h"
#include "btn_config.h"
#include "encoder_config.h"
#include "disp_config.h"
#include "lcd_config.h"
#include "bh1750_config.h"
#include "bmp2_config.h"
#include "analog_input.h"
#include "analog_output.h"
//#include "sine_wave.h"
#include "menu_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAB   12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc1_conv_rslt[ADC1_NUMBER_OF_CONV];
float adc1_voltages[ADC1_NUMBER_OF_CONV];

extern arm_fir_instance_f32 fir;
extern arm_biquad_casd_df1_inst_f32 iir;
extern arm_pid_instance_f32 pid;

//uint8_t END_OF_MSG = '\r';
uint8_t END_OF_MSG = '\n';
//uint8_t END_OF_MSG[] = {'\r','\n'};

uint8_t RX_DATA[SERIAL_PORT_MSG_BUF_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart3)
  {
    unsigned int dac_reg;
    int rx_n = sscanf((char*)RX_DATA, "%3x", &dac_reg);

    if(rx_n == 1)
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t)dac_reg);

    HAL_UART_Receive_DMA(huart, RX_DATA, SERIAL_PORT_DAC_MSG_SIZE);
  }
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc == &hadc1)
  {
  	float ain1_tmp = ADC_REG2VOLTAGE(adc1_conv_rslt[0]);
  	arm_fir_f32(&fir, &ain1_tmp, &adc1_voltages[0], 1);                // FIR filter

  	ain1_tmp = ADC_REG2VOLTAGE(adc1_conv_rslt[1]);
  	arm_biquad_cascade_df1_f32(&iir, &ain1_tmp, &adc1_voltages[1], 1); // IIR filter
  }
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* User menu: low priority */
  if(htim == hmenu.Timer)
  {
	/* UI timer period: display refresh rate [x100us] */
	__HAL_TIM_SET_AUTORELOAD(htim, MAX(hmenu.Item->RefreshRate, hmenu.Item->Next->RefreshRate));

	/* Menu items routines: main application tasks */
	MENU_ROUTINE(&hmenu);

	/* ADC1 conversion result on 7-segment display */
	DISP_printDecUInt(&hdisp1, adc1_voltages[0]);
  }
  /* User inputs & low-level display multiplexing: high priority timer */
  if(htim == hdisp1.Timer)
  {
	/* ADC1 DMA software triggering */
	ADC1_READ(adc1_conv_rslt);

    /* Menu buttons */
    if(BTN_EdgeDetected(&hbtn1) && hmenu.Item->Next != NULL)
	  hmenu.Item = hmenu.Item->Next;
    if(BTN_EdgeDetected(&hbtn2) && hmenu.Item->Prev != NULL)
      hmenu.Item = hmenu.Item->Prev;

    /* 7-segment display routine */
	DISP_ROUTINE(&hdisp1);
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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  // Initialize light sensor
  BH1750_Init(&hbh1750_1);
  // Initialize pressure and temperature sensor
  BMP2_Init(&hbmp2_1);
  // Initialize LCD1
  LCD_Init(&hlcd1);
  // Print laboratory task info on LCD1
  LCD_printf(&hlcd1, "L%02d: CMSIS DSP", LAB);
  // Initialize RGB LED
  LED_RGB_Init(&hledrgb1);
  // Set RGB LED color: disable all channels
  LED_RGB_SetColor(&hledrgb1, 0x000000);
  // Initialize rotary encoder
  ENC_Init(&henc1);

  // Start analog output
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_VOLTAGE2REG(0.0f) /* V */);
  //HAL_TIM_Base_Start(&htim6);
  //HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, SINE_WAVE, 100, DAC_ALIGN_12B_R);

  // CMSIS unit testing
  CMSIS_UnitTests();

  // Wait for button
  while(!BTN_EdgeDetected(&hbtn1))
  	HAL_Delay(0);

  // Turn LEDs off after testing
  LED_OFF_ALL();
  // Initialize 7-segment display
  DISP_Init(&hdisp1);
  // User menu initialization
  MENU_Init(&hmenu);

  HAL_UART_Receive_DMA(&huart3, RX_DATA, SERIAL_PORT_DAC_MSG_SIZE);

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
