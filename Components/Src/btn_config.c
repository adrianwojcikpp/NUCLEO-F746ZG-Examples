/**
  ******************************************************************************
  * @file    btn_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    30-Oct-2020
  * @brief   Simple push button driver library configuration file.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "btn.h"
#include "btn_config.h"
#include "main.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
BTN_HandleTypeDef hbtn1 = {
  .Port = BTN1_GPIO_Port, .Pin = BTN1_Pin,
  .Edge = FALLING_EDGE, .State = GPIO_PIN_SET
};

BTN_HandleTypeDef hbtn2 = {
  .Port = BTN2_GPIO_Port, .Pin = BTN2_Pin,
  .Edge = RISING_EDGE, .State = GPIO_PIN_RESET
};

BTN_HandleTypeDef hbtn3 = {
  .Port = BTN3_GPIO_Port, .Pin = BTN3_Pin,
  .Edge = FALLING_EDGE, .State = GPIO_PIN_SET
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
