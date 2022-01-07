/**
  ******************************************************************************
  * @file    analog_input.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    01-Dec-2021
  * @brief   Analog input definitions and macroinstructions.
  *
  ******************************************************************************
  */

#ifndef INC_ANALOG_INPUT_H_
#define INC_ANALOG_INPUT_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
#define ADC_BIT_RES      12      // [bits]
#define ADC_REG_MAX      (float)((1ul << ADC_BIT_RES) - 1)
#define ADC_VOLTAGE_MAX  3.3f    // [V]

/* Macro ---------------------------------------------------------------------*/
/**
 * @brief ADC data register to voltage in millivolts.
 * @param[in] reg : Data register
 * @return Input voltage in millivolts
 */
#define ADC_REG2VOLTAGE(reg) (1000.0f*LINEAR_TRANSFORM((float)reg,  \
                                                       0.0f, ADC_REG_MAX,    \
                                                       0.0f, ADC_VOLTAGE_MAX))

/**
 * @brief ADC voltage in volts to data register.
 * @param[in] vol : Output voltage in millivolts
 * @return Data register
 */
#define ADC_VOLTAGE2REG(vol) (uint16_t)(LINEAR_TRANSFORM(vol/1000.0f, \
                                               0.0, ADC_VOLTAGE_MAX, \
                                               0.0, ADC_REG_MAX      ))

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/


#endif /* INC_ANALOG_INPUT_H_ */
