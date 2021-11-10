/**
  ******************************************************************************
  * @file    led_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    3-Oct-2020
  * @brief   Simple LED driver library configuration file.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "led_config.h"
#include "main.h"
#include "tim.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/*                           GPIO PORT    |   PIN      |  LOGIC */
LED_HandleTypeDef hledg1 = { LD1_GPIO_Port,   LD1_Pin,    1 }; //! LD1: Green on-board LED
LED_HandleTypeDef hledb1 = { LD2_GPIO_Port,   LD2_Pin,    1 }; //! LD2: Blue on-board LED
LED_HandleTypeDef hledr1 = { LD3_GPIO_Port,   LD3_Pin,    1 }; //! LD3: Red on-board LED
LED_HandleTypeDef hledg2 = { LD1EX_GPIO_Port, LD1EX_Pin,  1 }; //! LD1EX: Green breadboard LED
LED_HandleTypeDef hledb2 = { LD2EX_GPIO_Port, LD2EX_Pin,  1 }; //! LD2EX: Blue breadboard LED
LED_HandleTypeDef hledr2 = { LD3EX_GPIO_Port, LD3EX_Pin,  1 }; //! LD3EX: Red breadboard LED

LED_PWM_HandleTypeDef hledw1 = {
  .Timer = &htim4, .Channel = TIM_CHANNEL_4, .Duty = 0
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
