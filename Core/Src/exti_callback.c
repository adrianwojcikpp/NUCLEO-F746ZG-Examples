/*
 * exti_callback.c
 *
 *  Created on: 15 paź 2021
 *      Author: AW
 */

#include "main.h"
#include "btn_config.h"

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == hbtn1.Pin)
  	HAL_GPIO_TogglePin(LD1EX_GPIO_Port, LD1EX_Pin);
}
