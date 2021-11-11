/**
  ******************************************************************************
  * @file    btn_config.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    30-Oct-2020
  * @brief   Simple push button driver library configuration file.
  *
  ******************************************************************************
  */
#ifndef INC_BTN_CONFIG_H_
#define INC_BTN_CONFIG_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "btn.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern BTN_HandleTypeDef hbtn1; // Red
extern BTN_HandleTypeDef hbtn2; // Black
extern BTN_HandleTypeDef hbtn3; // Encoder button

/* Public function prototypes ------------------------------------------------*/

#endif /* INC_BTN_CONFIG_H_ */
