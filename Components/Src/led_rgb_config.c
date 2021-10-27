/**
  ******************************************************************************
  * @file    led_rgb_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    02-Nov-2020
  * @brief   Simple tricolor (RGB) LED driver library configuration file.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "led_rgb.h"
#include "led_rgb_config.h"
#include "main.h" 
#include "tim.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
LED_RGB_HandleTypeDef hledrgb1 = {
  .Timer = &htim4,
  .ChannelR = TIM_CHANNEL_1,
  .ChannelG = TIM_CHANNEL_2,
  .ChannelB = TIM_CHANNEL_3,
  .DutyR = 0, .DutyG = 0, .DutyB = 0
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
