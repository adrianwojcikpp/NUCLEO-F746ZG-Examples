/**
  ******************************************************************************
  * @file    led.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    3-Oct-2020
  * @brief   Simple LED driver library.
  *
  ******************************************************************************
  */
#ifndef INC_LED_H_
#define INC_LED_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "digital_output.h"
#include "pwm_output.h"

/* Typedef -------------------------------------------------------------------*/
typedef DIGITAL_OUTPUT_HandleTypeDef LED_HandleTypeDef;
typedef PWM_OUTPUT_HandleTypeDef LED_PWM_HandleTypeDef;

/* Define --------------------------------------------------------------------*/

/**
 * @brief Turns LED on. Alias of 'DIGITAL_OUTPUT_SetLogicHigh' function.
 * @param[in] __HANDLE__ : Digital output handle
 * @return None
 */
#define LED_On(__HANDLE__) DIGITAL_OUTPUT_SetLogicHigh(__HANDLE__)

/**
 * @brief Turns LED off. Alias of 'DIGITAL_OUTPUT_SetLogicLow' function.
 * @param[in] __HANDLE__ : Digital output handle
 * @return None
 */
#define LED_Off(__HANDLE__) DIGITAL_OUTPUT_SetLogicLow(__HANDLE__)

/**
 * @brief Toggles LED state. Alias of 'DIGITAL_OUTPUT_Toggle' function.
 *        NOTE:! LED logic state; not GPIO state!
 * @param[in] __HANDLE__ : Digital output handle
 * @return None
 */
#define LED_Toggle(__HANDLE__) DIGITAL_OUTPUT_Toggle(__HANDLE__)

/**
 * @brief Sets LED to selected state (0/1). Alias of 'DIGITAL_OUTPUT_SetTo' function.
 *        NOTE:! LED logic state; not GPIO state!
 * @param[in] __HANDLE__ : Digital output handle
 * @param[in] __State__  : LED logic state (0/1).
 * @return None
 */
#define LED_SetTo(__HANDLE__, __STATE__) DIGITAL_OUTPUT_SetTo(__HANDLE__, __STATE__)

/**
 * @brief Read LED state (0/1).
 *        NOTE:! LED logic state; not GPIO state!
 * @param[in] __HANDLE__ : Digital output handle
 * @return LED state
 * @retval 0 -> LED is off
 * @retval 1 -> LED is on
 */
#define LED_Read(__HANDLE__) DIGITAL_OUTPUT_Read(__HANDLE__)

/**
 * @brief Initialize PWM channel for LED control. Alias of 'PWM_OUTPUT_Init' function.
 * @param[in] __HANDLE__ : PWM output handle
 */
#define LED_PWM_Init(__HANDLE__) PWM_OUTPUT_Init(__HANDLE__)

/**
 * @brief Control of LED power with PWM channel. Alias of 'PWM_OUTPUT_SetDuty' function.
 * @param[in] __HANDLE__ : PWM output handle
 * @param[in] __BTIGHTNESS__ : LED brightness expresses in percents <0-100>
 */
#define LED_SetBrightness(__HANDLE__, __BTIGHTNESS__) PWM_OUTPUT_SetDuty(__HANDLE__, __BTIGHTNESS__)

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

#endif /* INC_LED_H_ */
