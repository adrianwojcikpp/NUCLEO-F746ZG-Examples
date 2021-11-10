/**
  ******************************************************************************
  * @file    led.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    3-Oct-2020
  * @brief   Simple LED driver library.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "led.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/
//! Set GPIO Pin to high
#define __led_seth(hled)       (hled)->Port->BSRR |= ((hled)->Pin)

//! Set GPIO Pin to low
#define __led_setl(hled)       (hled)->Port->BSRR |= ((hled)->Pin << 0x10)

//! Turn LED on
#define __led_on(hled)         ((hled)->Logic ? (__led_seth(hled)) : (__led_setl(hled)))

//! Turn LED off
#define __led_off(hled)        ((hled)->Logic ? (__led_setl(hled)) : (__led_seth(hled)))

//! Read LED state
#define __led_read(hled)       ((hled)->Port->ODR & (hled)->Pin)

//! Set LED to selected state (on/off)
#define __led_setto(hled, st)  (st ? __led_on(hled) : __led_off(hled))

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/

/**
 * @brief Turns LED on.
 * @param[in] hled LED handle
 * @return None
 */
void LED_On(LED_HandleTypeDef* hled)
{
	__led_on(hled);
}

/**
 * @brief Turns LED off
 * @param[in] hled LED handle
 * @return None
 */
void LED_Off(LED_HandleTypeDef* hled)
{
	__led_off(hled);
}

/**
 * @brief Toggles LED state.
 * @param[in] hled LED handle
 * @return None
 */
void LED_Toggle(LED_HandleTypeDef* hled)
{
	__led_setto(hled, !__led_read(hled));
}

/**
 * @brief Sets LED to selected state (0/1)
 * @param[in] hled LED handle
 * @return None
 */
void LED_SetTo(LED_HandleTypeDef* hled, unsigned char state)
{
	__led_setto(hled, state);
}

/**
 * @brief Read LED state (0/1) (NOTE:! not GPIO state!)
 * @param[in] hled LED handle
 * @return LED state
 * @retval 0 -> LED is off
 * @retval 1 -> LED is on
 */
uint8_t LED_Read(LED_HandleTypeDef* hled)
{
  if((hled)->Logic)
    return (__led_read(hled)) ? (1) : (0);
  else
    return (__led_read(hled)) ? (0) : (1);
}

/**
 * @brief PWM-controlled LED initialization.
 * @param[in] hledpwm LED PWM handler
 * @return None
 */
void LED_PWM_Init(LED_PWM_HandleTypeDef* hledpwm)
{
  HAL_TIM_PWM_Start(hledpwm->Timer, hledpwm->Channel);
}

/**
 * @brief Sets duty of PWM-controlled LED.
 * @param[in] hledpwm LED PWM handler
 * @param[in] duty    PWM duty in percents
 * @return None
 */
void LED_PWM_SetDuty(LED_PWM_HandleTypeDef* hledpwm, float duty)
{
  hledpwm->Duty = duty;
  int COMPARE = (duty * (__HAL_TIM_GET_AUTORELOAD(hledpwm->Timer)+1)) / 100;
  __HAL_TIM_SET_COMPARE(hledpwm->Timer, hledpwm->Channel, COMPARE);
}
