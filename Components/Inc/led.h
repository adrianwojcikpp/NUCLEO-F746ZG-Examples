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

/* Typedef -------------------------------------------------------------------*/
#define LED_PortType GPIO_TypeDef*
#define LED_PinType uint16_t

typedef struct {
	LED_PortType Port;
	LED_PinType Pin;
	uint8_t  Logic;  //! LED logic: 1 if LED is ON when pin is high, 0 otherwise
} LED_HandleTypeDef;

#define LED_PWM_TimerType   TIM_HandleTypeDef*
#define LED_PWM_ChannelType uint16_t

typedef struct {
  LED_PWM_TimerType Timer;
  LED_PWM_ChannelType Channel;
  float Duty;
} LED_PWM_HandleTypeDef;

/* Define --------------------------------------------------------------------*/

/**
 * @brief Control of LED power with PWM channel. Alias of 'LED_PWM_SetDuty' function.
 * @param[in] __HANDLE__ : Input LED handle
 * @param[in] __BTIGHTNESS__ : LED brightness expresses in percents <0-100>
 */
#define LED_SetBrightness(__HANDLE__, __BTIGHTNESS__) LED_PWM_SetDuty(__HANDLE__, __BTIGHTNESS__)

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Turns LED on.
 * @param[in] hled LED handle
 * @return None
 */
void LED_On(LED_HandleTypeDef* hled);

/**
 * @brief Turns LED off
 * @param[in] hled LED handle
 * @return None
 */
void LED_Off(LED_HandleTypeDef* hled);

/**
 * @brief Toggles LED state.
 * @param[in] hled LED handle
 * @return None
 */
void LED_Toggle(LED_HandleTypeDef* hled);

/**
 * @brief Sets LED to selected state (0/1)
 * @param[in] hled LED handle
 * @return None
 */
void LED_SetTo(LED_HandleTypeDef* hled, unsigned char state);

/**
 * @brief Read LED state (0/1) (NOTE:! not GPIO state!)
 * @param[in] hled LED handle
 * @return LED state
 * @retval 0 -> LED is off
 * @retval 1 -> LED is on
 */
uint8_t LED_Read(LED_HandleTypeDef* hled);

/**
 * @brief PWM-controlled LED initialization.
 * @param[in] hledpwm LED PWM handler
 * @return None
 */
void LED_PWM_Init(LED_PWM_HandleTypeDef* hledpwm);

/**
 * @brief Sets duty of PWM-controlled LED.
 * @param[in] hledpwm LED PWM handler
 * @param[in] duty    PWM duty in percents
 * @return None
 */
void LED_PWM_SetDuty(LED_PWM_HandleTypeDef* hledpwm, float duty);

#endif /* INC_LED_H_ */
