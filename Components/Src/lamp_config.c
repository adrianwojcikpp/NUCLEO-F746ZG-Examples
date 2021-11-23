/**
  ******************************************************************************
  * @file    lamp_config.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    30-Oct-2020
  * @brief   Simple dimmer (incandescent light bulb controller board) driver
  *          library configuration file.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "lamp.h"
#include "main.h"
#include "tim.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
LAMP_HandleTypeDef hlamp1 = {
  .Timer = &htim2,
  .SYNC_Port  = LAMP_SYNC_GPIO_Port,  .SYNC_Pin  = LAMP_SYNC_Pin,
  .TRIAC_Port = LAMP_TRIAC_GPIO_Port, .TRIAC_Pin = LAMP_TRIAC_Pin, 
  .EXTI_IRQn = EXTI9_5_IRQn,
  .TriacFiringAngle = 160.0,
  .TriacFiringAngleMin = 10.0, .TriacFiringAngleMax = 160.0
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
