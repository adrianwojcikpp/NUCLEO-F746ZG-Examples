/**
  ******************************************************************************
  * @file    led_config.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    3-Oct-2020
  * @brief   Simple LED driver library configuration file.
  *
  ******************************************************************************
  */
#ifndef INC_LED_CONFIG_H_
#define INC_LED_CONFIG_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "led.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/
#define LED_ON_ALL()  LED_On(&hledg1); \
                      LED_On(&hledb1); \
                      LED_On(&hledr1); \
                      LED_On(&hledg2); \
                      LED_On(&hledb2); \
                      LED_On(&hledr2); \
                      LED_SetBrightness(&hledw1, 100.0f)

#define LED_OFF_ALL() LED_Off(&hledg1); \
                      LED_Off(&hledb1); \
                      LED_Off(&hledr1); \
                      LED_Off(&hledg2); \
                      LED_Off(&hledb2); \
                      LED_Off(&hledr2); \
                      LED_SetBrightness(&hledw1, 0.0f)

/* Public variables ----------------------------------------------------------*/
extern LED_HandleTypeDef hledg1; //! LD1: Green on-board LED
extern LED_HandleTypeDef hledb1; //! LD2: Blue on-board LED
extern LED_HandleTypeDef hledr1; //! LD3: Red on-board LED
extern LED_HandleTypeDef hledg2; //! LD1EX: Green breadboard LED
extern LED_HandleTypeDef hledb2; //! LD2EX: Blue breadboard LED
extern LED_HandleTypeDef hledr2; //! LD3EX: Red breadboard LED
extern LED_PWM_HandleTypeDef hledw1;  //! LD4EX: White breadboard LED (PWM)

/* Public function prototypes ------------------------------------------------*/

#endif /* INC_LED_CONFIG_H_ */
