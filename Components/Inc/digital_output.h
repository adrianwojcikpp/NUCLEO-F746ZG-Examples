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
#ifndef INC_DIGITAL_OUTPUT_H_
#define INC_DIGITAL_OUTPUT_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Typedef -------------------------------------------------------------------*/
#define DIGITAL_OUTPUT_PortType GPIO_TypeDef*
#define DIGITAL_OUTPUT_PinType  uint16_t

typedef struct {
	DIGITAL_OUTPUT_PortType Port;
	DIGITAL_OUTPUT_PinType Pin;
	uint8_t  Logic; 
} DIGITAL_OUTPUT_HandleTypeDef;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Set digital output to logic hight state.
 *        NOTE:! not GPIO state!
 * @param[in] hout : Digital output handle
 * @return None
 */
void DIGITAL_OUTPUT_SetLogicHigh(DIGITAL_OUTPUT_HandleTypeDef* hout);

/**
 * @brief Set digital output to logic low state.
 *        NOTE:! not GPIO state!
 * @param[in] hout : Digital output handle
 * @return None
 */
void DIGITAL_OUTPUT_SetLogicLow(DIGITAL_OUTPUT_HandleTypeDef* hout);

/**
 * @brief Toggle digital output logic state.
 *        NOTE:! not GPIO state!
 * @param[in] hout : Digital output handle
 * @return None
 */
void DIGITAL_OUTPUT_Toggle(DIGITAL_OUTPUT_HandleTypeDef* hout);

/**
 * @brief Set digital output logic state.
 *        NOTE:! not GPIO state!
 * @param[in] hout  : Digital output handle
 * @param[in] state : Selected state (0/1) 
 * @return None
 */
void DIGITAL_OUTPUT_SetTo(DIGITAL_OUTPUT_HandleTypeDef* hout, unsigned char state);

/**
 * @brief Read digital output logic state (0/1). 
 *        NOTE:! not GPIO state!
 * @param[in] hout LED handle
 * @return Digital output state
 * @retval 0 (logic low)
 * @retval 1 (logic high)
 */
uint8_t DIGITAL_OUTPUT_Read(DIGITAL_OUTPUT_HandleTypeDef* hout);

#endif /* INC_DIGITAL_OUTPUT_H_ */
