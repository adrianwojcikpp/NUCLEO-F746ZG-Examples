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
  static Input_TypeDef input = ANALOG_INPUT1;

  /* Encoder button */
  if(BTN_EdgeDetected(&hbtn3))
    input = (input < ANALOG_INPUT2) ? (input + 1) : (ENCODER);

  /* Selected measurement in JSON format */
  char str_buffer[32];
  int n;

  switch(input)
  {
    case ENCODER:
    {
      uint32_t enc = ENC_GetCounter(&henc1);
      n = sprintf(str_buffer, "{\"Encoder\":%3lu} ", enc);
      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ENC2DAC(enc));
      break;
    }
    case BH1750: /* Light sensor */
    {
      float light = BH1750_ReadIlluminance_lux(&hbh1750_1);
      n = sprintf(str_buffer, "{\"Light\":%6d}", (int)light);
      break;
    }
    case BMP280_TEMP: /* Temperature sensor */
    {
      double temp = BMP2_ReadTemperature_degC(&hbmp2_1);
      n = sprintf(str_buffer, "{\"Temp\":%2.02f}  ", temp);
      break;
    }
    case BMP280_PRESS: /* Pressure sensor */
    {
      double press = BMP2_ReadPressure_hPa(&hbmp2_1);
      n = sprintf(str_buffer, "{\"Press\":%4.02f}", press);
      break;
    }
    case ANALOG_INPUT1: /* Analog input #1: potentiometer #1 */
    {
    	static float32_t ain1_x;
    	static float32_t ain1_y;

    	ain1_x = ADC_REG2VOLTAGE(adc1_conv_rslt[0]);

    	//arm_fir_f32(&fir, &ain1_x, &ain1_y, 1);                // FIR filter
    	arm_biquad_cascade_df1_f32(&iir, &ain1_x, &ain1_y, 1); // IIR filter
    	//ain1_y = ain1_x;                                       // no filter

      n = sprintf(str_buffer, "{\"POT1\":%4d mV}", (int)ain1_y);
      break;
    }
    case ANALOG_INPUT2: /* Analog input #2: potentiometer #2 */
    {
      n = sprintf(str_buffer, "{\"POT2\":%4d mV}", (int)ADC_REG2VOLTAGE(adc1_conv_rslt[1]));
      break;
    }
  default: break;
  }

  /* Embedded display */
  LCD_SetCursor(&hlcd1, 1, 0);
  LCD_printStr(&hlcd1, str_buffer);

  /* Serial port streaming */
  str_buffer[n] = '\n'; // add new line
  HAL_UART_Transmit(&huart3, (uint8_t*)str_buffer, n+1, 1000);
}
