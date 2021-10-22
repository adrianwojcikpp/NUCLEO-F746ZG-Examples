/**
  ******************************************************************************
  * @file    lamp.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    30-Oct-2020
  * @brief   Simple dimmer (lamp controller board) driver library.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "lamp.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

//! For timer with base frequency (ARR = 0) of 1 MHz (period of 1 us)
#define __LAMP_DEG_TO_MICROSECONDS(A) ((1000000ul*A)/(2ul*LAMP_LINE_FREQ*180ul))

//! Simple software delay
#define __LAMP_SOFT_DELAY(N) for(uint32_t i = 0; i < N; i++){ asm("nop"); }

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
/**
 * @brief Start lamp controller timer.
 * @param[in] ang   Triac firing angle [degrees]
 * @return None
 */
void LAMP_StartTimer(LAMP_HandleTypeDef* hlamp)
{
	// Disable EXTI on lamps SYNC line
	//EXTI->IMR &= ~(hlamp->EXTI_LINE);
	NVIC_DisableIRQ(EXTI9_5_IRQn);

	// Saturate firing angle
  if(hlamp->TriacFiringAngle > hlamp->TriacFiringAngleMax)
  	hlamp->TriacFiringAngle = hlamp->TriacFiringAngleMax;
  else if(hlamp->TriacFiringAngle < hlamp->TriacFiringAngleMin)
  	hlamp->TriacFiringAngle = hlamp->TriacFiringAngleMin;
  
  // Compute and set timer ARR value
  uint32_t CounterPeriod = __LAMP_DEG_TO_MICROSECONDS(hlamp->TriacFiringAngle);
  __HAL_TIM_SetAutoreload(hlamp->Timer, CounterPeriod);
  
  // Start timer in non-blocking mode
  HAL_TIM_Base_Start_IT(hlamp->Timer);
}

/**
 * @brief Stop lamp controller timer.
 * @param[in] hlamp Lamp handler
 * @return None
 */
void LAMP_StopTimer(LAMP_HandleTypeDef* hlamp)
{
	// Enable EXTI on lamps SYNC line
	//EXTI->IMR |= (hlamp->EXTI_LINE);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	// Stop timer in non-blocking mode
  HAL_TIM_Base_Stop_IT(hlamp->Timer);
}

/**
 * @brief Triac firing procedure: sets TRIAC output on high for short period (<100us).
 * @param[in] hlamp Lamp handler
 * @return None
 */
void LAMP_TriacFiring(LAMP_HandleTypeDef* hlamp)
{
  HAL_GPIO_WritePin(hlamp->TRIAC_Port, hlamp->TRIAC_Pin, GPIO_PIN_SET);
  __LAMP_SOFT_DELAY(LAMP_TRIAC_DELAY);
  HAL_GPIO_WritePin(hlamp->TRIAC_Port, hlamp->TRIAC_Pin, GPIO_PIN_RESET);
}
