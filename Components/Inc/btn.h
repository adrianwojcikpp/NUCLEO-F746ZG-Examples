/**
  ******************************************************************************
  * @file    btn.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    30-Oct-2020
  * @brief   Simple push button driver library.
  *
  ******************************************************************************
  */
#ifndef INC_BTN_H_
#define INC_BTN_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Typedef -------------------------------------------------------------------*/
#define BTN_PortType GPIO_TypeDef*
#define BTN_PinType uint16_t
#define BTN_PinStateType GPIO_PinState

typedef enum {
	RISING_EDGE,
	FALLING_EDGE
} BTN_EdgeTypeDef;

typedef struct {
  BTN_PortType Port;
  BTN_PinType  Pin;
  BTN_EdgeTypeDef Edge;
  BTN_PinStateType State;
} BTN_HandleTypeDef;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief Push button edge detection.
 * @param[in] hbtn Push button handler
 * @return Edge detection flag
 * @retval 1 -> Sensitive edge detected
 * @retval 0 -> Sensitive edge not detected
 */
uint8_t BTN_EdgeDetected(BTN_HandleTypeDef* hbtn);

#endif /* INC_BTN_H_ */
