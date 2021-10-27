/**
  ******************************************************************************
  * @file    lcd_config.c
  * @author  AW
  * @version V2.0
  * @date    30-Oct-2020
  * @brief   Simple HD44780 driver library for STM32F7 configuration file.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "lcd.h"
#include "lcd_config.h"
#include "main.h"

#ifdef LCD_USE_TIMER
#include "tim.h"
#endif

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
LCD_PortType LCD_DATA_Ports[] = {
  LCD_D4_GPIO_Port, LCD_D5_GPIO_Port, LCD_D6_GPIO_Port, LCD_D7_GPIO_Port
};

LCD_PinType  LCD_DATA_Pins[]  = {
  LCD_D4_Pin,       LCD_D5_Pin,       LCD_D6_Pin,       LCD_D7_Pin
};

/* Public variables ----------------------------------------------------------*/
LCD_HandleTypeDef hlcd1 = {
  .DATA_Ports = LCD_DATA_Ports, 
  .DATA_Pins = LCD_DATA_Pins,
  .RS_Port = LCD_RS_GPIO_Port, 
  .RS_Pin = LCD_RS_Pin,
  .E_Port = LCD_E_GPIO_Port,   
  .E_Pin = LCD_E_Pin,
  .Mode = LCD_4_BIT_MODE,
  .Timer = &htim5
};

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
