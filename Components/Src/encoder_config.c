/**
  ******************************************************************************
  * @file    encoder_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    30-Oct-2020
  * @brief   Simple rotary encoder driver library configuration file. 
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "encoder.h"
#include "encoder_config.h"
#include "main.h"
#ifdef ENC_HARDWARE_COUNTER
#include "tim.h"
#endif

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
#ifdef ENC_HARDWARE_COUNTER

ENC_HandleTypeDef henc1 = {
  .Timer = &htim4,
  .Counter  = 0,
  .CounterMax = 100, .CounterMin = 0,
  .CounterInc = 0, .CounterDec = 0
};

#else

ENC_HandleTypeDef henc1 = {
  .CLK_Port = ENC_CLK_GPIO_Port, .CLK_Pin = ENC_CLK_Pin,
  .DT_Port  = ENC_DT_GPIO_Port,  .DT_Pin  = ENC_DT_Pin,
  .Counter  = 100,
  .CounterMax = 1000, .CounterMin = 0, .CounterStep = 10,
  .CounterInc = 0, .CounterDec = 0
};

#endif

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
