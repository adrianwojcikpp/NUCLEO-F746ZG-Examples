/**
  ******************************************************************************
  * @file    digital_output.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    21-Nov-2021
  * @brief   Simple digital output driver library.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "digital_output.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
//! Set GPIO Pin to high state
#define __out_set_high(hout)        (hout)->Port->BSRR |= ((hout)->Pin)

//! Set GPIO Pin to low state
#define __out_set_low(hout)         (hout)->Port->BSRR |= ((hout)->Pin << 0x10)

//! Set digital output to logic hight state
#define __out_set_logic_high(hout)  ((hout)->Logic ? (__out_set_high(hout)) : (__out_set_low(hout)))

//! Set digital output to logic low state
#define __out_set_logic_low(hout)   ((hout)->Logic ? (__out_set_low(hout)) : (__out_set_high(hout)))

//! Read GPIO Pin state 
#define __out_read(hout)            ((hout)->Port->ODR & (hout)->Pin)

//! Set digital output to selected logic state
#define __out_set_to(hout, st)       (st ? __out_set_logic_high(hout) : __out_set_logic_low(hout))

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/

/**
 * @brief Set digital output to logic hight state.
 *        NOTE:! not GPIO state!
 * @param[in] hout : Digital output handle
 * @return None
 */
void DIGITAL_OUTPUT_SetLogicHigh(DIGITAL_OUTPUT_HandleTypeDef* hout)
{
  __out_set_logic_high(hout);
}

/**
 * @brief Set digital output to logic low state.
 *        NOTE:! not GPIO state!
 * @param[in] hout : Digital output handle
 * @return None
 */
void DIGITAL_OUTPUT_SetLogicLow(DIGITAL_OUTPUT_HandleTypeDef* hout)
{
  __out_set_logic_low(hout);
}

/**
 * @brief Toggle digital output logic state.
 *        NOTE:! not GPIO state!
 * @param[in] hout : Digital output handle
 * @return None
 */
void DIGITAL_OUTPUT_Toggle(DIGITAL_OUTPUT_HandleTypeDef* hout)
{
  __out_set_to(hout, !DIGITAL_OUTPUT_Read(hout));
}


/**
 * @brief Set digital output logic state.
 *        NOTE:! not GPIO state!
 * @param[in] hout  : Digital output handle
 * @param[in] state : Selected state (0/1) 
 * @return None
 */
void DIGITAL_OUTPUT_SetTo(DIGITAL_OUTPUT_HandleTypeDef* hout, unsigned char state)
{
  __out_set_to(hout, state);
}

/**
 * @brief Read digital output logic state (0/1). 
 *        NOTE:! not GPIO state!
 * @param[in] hout LED handle
 * @return Digital output state
 * @retval 0 (logic low)
 * @retval 1 (logic high)
 */
uint8_t DIGITAL_OUTPUT_Read(DIGITAL_OUTPUT_HandleTypeDef* hout)
{
  if((hout)->Logic)
    return (__out_read(hout)) ? (1) : (0);
  else
    return (__out_read(hout)) ? (0) : (1);
}