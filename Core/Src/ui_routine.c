/*
 * ui_routine.c
 *
 *  Created on: 3 sty 2022
 *      Author: AW
 */

#include "main.h"
#include "dac.h"
#include "usart.h"

extern arm_fir_instance_f32 fir;
extern arm_biquad_casd_df1_inst_f32 iir;
extern arm_pid_instance_f32 pid;
extern uint16_t adc1_conv_rslt[ADC1_NUMBER_OF_CONV];

/**
  * @brief Low-level user interface routine.
  *        Embedded display and serial port handling
  */
void ui_routine(void)
{
  /* Selected measurement in JSON format */
  char str_buffer[32];
  int n;

    	static float32_t ain1_x;
    	static float32_t ain1_y;

    	ain1_x = ADC_REG2VOLTAGE(adc1_conv_rslt[0]);

    	//arm_fir_f32(&fir, &ain1_x, &ain1_y, 1);                // FIR filter
    	arm_biquad_cascade_df1_f32(&iir, &ain1_x, &ain1_y, 1); // IIR filter
    	//ain1_y = ain1_x;                                       // no filter

      n = sprintf(str_buffer, "{\"POT1\":%4d mV}", (int)ain1_y);


  /* Embedded display */
  LCD_SetCursor(&hlcd1, 1, 0);
  LCD_printStr(&hlcd1, str_buffer);

  /* Serial port streaming */
  str_buffer[n] = '\n'; // add new line
  HAL_UART_Transmit(&huart3, (uint8_t*)str_buffer, n+1, 1000);
}
