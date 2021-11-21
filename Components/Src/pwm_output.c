/**
  ******************************************************************************
  * @file    pwm_output.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    21-Nov-2021
  * @brief   Simple PWM output driver library.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "pwm_output.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/

/**
 * @brief PWM output initialization.
 * @param[in] hpwmout : PWM output handler
 * @return None
 */
void PWM_OUTPUT_Init(PWM_OUTPUT_HandleTypeDef* hpwmout)
{
  HAL_TIM_PWM_Start(hpwmout->Timer, hpwmout->Channel);
}

/**
 * @brief Sets duty of PWM output.
 * @param[in] hpwmout : PWM output handler
 * @param[in] duty    : PWM duty in percents
 * @return None
 */
void PWM_OUTPUT_SetDuty(PWM_OUTPUT_HandleTypeDef* hpwmout, float duty)
{
  hpwmout->Duty = duty;
  int COMPARE = (duty * (__HAL_TIM_GET_AUTORELOAD(hpwmout->Timer)+1)) / 100;
  __HAL_TIM_SET_COMPARE(hpwmout->Timer, hpwmout->Channel, COMPARE);
}