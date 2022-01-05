/**
  ******************************************************************************
  * @file    analog_output.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    01-Dec-2021
  * @brief   Analog output definitions and macroinstructions.
  *
  ******************************************************************************
  */

#ifndef INC_ANALOG_OUTPUT_H_
#define INC_ANALOG_OUTPUT_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
#define DAC_BIT_RES      12    // [bits]
#define DAC_REG_MAX      (float)((1ul << DAC_BIT_RES) - 1)
#define DAC_VOLTAGE_MAX  3.3f  // [V]

/* Macro ---------------------------------------------------------------------*/
/**
 * @brief DAC voltage in volts to data register.
 * @param[in] vol : Output voltage in volts
 * @return Data register
 */
#define DAC_VOLTAGE2REG(vol) (uint16_t)(LINEAR_TRANSFORM((float)vol, \
                                               0.0, DAC_VOLTAGE_MAX, \
                                               0.0, DAC_REG_MAX      ))

/**
 * @brief DAC data register to voltage in volts.
 * @param[in] reg : Data register
 * @return Input voltage in volts
 */
#define DAC_REG2VOLTAGE(reg) (LINEAR_TRANSFORM((float)reg,  \
                                                0.0f, DAC_REG_MAX,    \
                                                0.0f, DAC_VOLTAGE_MAX))

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

#endif /* INC_ANALOG_OUTPUT_H_ */
