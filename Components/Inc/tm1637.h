/**
 ******************************************************************************
 * @file    tm1637.h
 * @author  AW		Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    08 Sep 2022
 * @brief   TM1637 driver: 4-digit 7-segment display module
 ******************************************************************************
 */

#ifndef __TM1637_H__
#define __TM1637_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Typedef -------------------------------------------------------------------*/
#define TM1637_PortType GPIO_TypeDef*
#define TM1637_PinType uint16_t

typedef enum {
  TM1637_SEP_NONE = 0, TM1637_SEP_COLON
} TM1637_Separator_TypeDef;

typedef struct {
	TM1637_PortType CLK_Port;
	TM1637_PinType CLK_Pin;
	TM1637_PortType DIO_Port;
	TM1637_PinType DIO_Pin;
	TM1637_Separator_TypeDef Separator;
} TM1637_HandleTypeDef;

/* Public function prototypes ------------------------------------------------*/
/**
 * @brief TODO
 * @param[in] hdisp : TODO
 * @param[in] dec   : TODO
 * @return None
 */
void TM1637_printDecUInt(TM1637_HandleTypeDef* hdisp, uint16_t dec);

/**
 * @brief TODO
 * @param[in] hdisp      : TODO
 * @param[in] brightness : TODO
 * @return None
 */
void TM1637_SetBrightness(TM1637_HandleTypeDef* hdisp, uint8_t brightness);

#endif /* __TM1637_H__ */
