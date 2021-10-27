/**
  ******************************************************************************
  * @file    lamp.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    30-Oct-2020
  * @brief   Simple dimmer (lamp controller board) driver library.
  *
  ******************************************************************************
  */
#ifndef INC_LAMP_H_
#define INC_LAMP_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Typedef -------------------------------------------------------------------*/
#define LAMP_TimerType TIM_HandleTypeDef*
#define LAMP_PortType GPIO_TypeDef*
#define LAMP_PinType uint16_t

typedef struct {
  LAMP_TimerType Timer;
  LAMP_PortType SYNC_Port;
  LAMP_PinType SYNC_Pin;
  LAMP_PortType TRIAC_Port;
  LAMP_PinType TRIAC_Pin;
  uint32_t EXTI_LINE;
  float TriacFiringAngle;
  float TriacFiringAngleMin;
  float TriacFiringAngleMax;
} LAMP_HandleTypeDef; 

/* Define --------------------------------------------------------------------*/
#define LAMP_LINE_FREQ   50ul  // [Hz]
#define LAMP_TRIAC_DELAY 1000  // [ticks]

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief Start lamp controller timer.
 * @param[in] hlamp Lamp handler
 * @return None
 */
void LAMP_StartTimer(LAMP_HandleTypeDef* hlamp);

/**
 * @brief Stop lamp controller timer.
 * @param[in] hlamp Lamp handler
 * @return None
 */
void LAMP_StopTimer(LAMP_HandleTypeDef* hlamp);

/**
 * @brief Triac firing procedure: sets TRIAC output on high for short period (<100us).
 * @param[in] hlamp Lamp handler
 * @return None
 */
void LAMP_TriacFiring(LAMP_HandleTypeDef* hlamp);

#endif /* INC_LAMP_H_ */
