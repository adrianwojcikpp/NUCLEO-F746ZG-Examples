/**
  ******************************************************************************
  * @file    pwm_output.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    21-Nov-2021
  * @brief   Simple PWM output driver library.
  *
  ******************************************************************************
  */
#ifndef INC_PWM_OUTPUT_H_
#define INC_PWM_OUTPUT_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Typedef -------------------------------------------------------------------*/
#define PWM_OUTPUT_TimerType   TIM_HandleTypeDef*
#define PWM_OUTPUT_ChannelType uint16_t

typedef struct {
  PWM_OUTPUT_TimerType Timer;
  PWM_OUTPUT_ChannelType Channel;
  float Duty;
} PWM_OUTPUT_HandleTypeDef;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief PWM output initialization.
 * @param[in] hpwmout : PWM output handler
 * @return None
 */
void PWM_OUTPUT_Init(PWM_OUTPUT_HandleTypeDef* hpwmout);

/**
 * @brief Sets duty of PWM output.
 * @param[in] hpwmout : PWM output handler
 * @param[in] duty    : PWM duty in percents
 * @return None
 */
void PWM_OUTPUT_SetDuty(PWM_OUTPUT_HandleTypeDef* hpwmout, float duty);

#endif /* INC_PWM_OUTPUT_H_ */
