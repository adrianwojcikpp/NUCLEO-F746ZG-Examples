/**
  ******************************************************************************
  * @file    menu_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version V2.0
  * @date    12-Jan-2021
  * @brief   Simple LCD menu example configuration file: menu structure,
  *          content and behavior
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "adc.h"
#include "dac.h"
#include "usart.h"

#include "common.h"
#include "led_config.h"
#include "led_rgb_config.h"
//#include "lamp_config.h"
#include "btn_config.h"
#include "encoder_config.h"
//#include "disp_config.h"
#include "lcd_config.h"
#include "bh1750_config.h"
#include "bmp2_config.h"
#include "analog_input.h"
#include "analog_output.h"
//#include "sine_wave.h"
#include "menu_config.h"

#include <stdio.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
#define ENC2DUTY(__HENC__) LINEAR_TRANSFORM((float)(__HENC__)->Counter, \
		                                    (float)(__HENC__)->CounterMin, \
		                                    (float)(__HENC__)->CounterMax, \
		                                    0.0f, 100.0f)

#define ENC2VOLTAGE(__HENC__) LINEAR_TRANSFORM((float)(__HENC__)->Counter, \
		                                    (float)(__HENC__)->CounterMin, \
		                                    (float)(__HENC__)->CounterMax, \
		                                    0.0f, 3.3f)
/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void menu_led_routine(MenuItem_TypeDef* hmenuitem, LED_HandleTypeDef* hled, const char* led_name);
void menu_ledrgb_routine(MenuItem_TypeDef* hmenuitem, LED_RGB_HandleTypeDef* hledrgb, LED_RGB_ChannelType channel);
void menu_ain_routine(MenuItem_TypeDef* hmenuitem, float value, int ain_n);
void menu_aout_routine(MenuItem_TypeDef* hmenuitem, DAC_HandleTypeDef *hdac, uint32_t Channel);
void menu_enc_routine(MenuItem_TypeDef* hmenuitem, ENC_HandleTypeDef* henc, int enc_n);
void menu_digital_sensor_routine(MenuItem_TypeDef* hmenuitem, float value, const char* format);

/* Public variables ----------------------------------------------------------*/
Menu_TypeDef hmenu = { .Item = &menu_ledr1, .Display = &hlcd1, .Timer = &htim10, .SerialPort = &huart3 };

extern float adc1_voltages[ADC1_NUMBER_OF_CONV];

/** MENU LED CODE BEGIN *************************************
 * ****************************************************************/

void __menu_ledr1_routine(MenuItem_TypeDef* hmenuitem){ menu_led_routine(hmenuitem, &hledr1, "LDR1"); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledr1, &menu_ledg1, &menu_aout2, 100 /* ms */);

void __menu_ledg1_routine(MenuItem_TypeDef* hmenuitem){ menu_led_routine(hmenuitem, &hledg1, "LDG1"); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledg1, &menu_ledb1, &menu_ledr1, 100 /* ms */);

void __menu_ledb1_routine(MenuItem_TypeDef* hmenuitem){ menu_led_routine(hmenuitem, &hledb1, "LDB1"); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledb1, &menu_ledr2, &menu_ledg1, 100 /* ms */);

void __menu_ledr2_routine(MenuItem_TypeDef* hmenuitem){ menu_led_routine(hmenuitem, &hledr2, "LDR2"); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledr2, &menu_ledg2, &menu_ledb1, 100 /* ms */);

void __menu_ledg2_routine(MenuItem_TypeDef* hmenuitem){ menu_led_routine(hmenuitem, &hledg2, "LDG2"); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledg2, &menu_ledb2, &menu_ledr2, 100 /* ms */);

void __menu_ledb2_routine(MenuItem_TypeDef* hmenuitem){ menu_led_routine(hmenuitem, &hledb2, "LDB2"); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledb2, &menu_ledrgb_r, &menu_ledg2, 100 /* ms */);

/** MENU LED CODE END *******************************************************************************************************/

/** MENU LED RGB CODE BEGIN *************************************************************************************************/

void __menu_ledrgb_r_routine(MenuItem_TypeDef* hmenuitem){ menu_ledrgb_routine(hmenuitem, &hledrgb1, LED_CHANNEL_R); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledrgb_r, &menu_ledrgb_g, &menu_ledb2, 100 /* ms */);

void __menu_ledrgb_g_routine(MenuItem_TypeDef* hmenuitem){ menu_ledrgb_routine(hmenuitem, &hledrgb1, LED_CHANNEL_G); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledrgb_g, &menu_ledrgb_b, &menu_ledrgb_r, 100 /* ms */);

void __menu_ledrgb_b_routine(MenuItem_TypeDef* hmenuitem){ menu_ledrgb_routine(hmenuitem, &hledrgb1, LED_CHANNEL_B); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ledrgb_b, &menu_enc1, &menu_ledrgb_g, 100 /* ms */);

/** MENU LED RGB CODE END ***************************************************************************************************/

/** MENU ENCODER CODE BEGIN *************************************************************************************************/

void __menu_enc1_routine(MenuItem_TypeDef* hmenuitem){ menu_enc_routine(hmenuitem, &henc1, 1); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_enc1, &menu_ain1, &menu_ledrgb_b, 100 /* ms */);

/** MENU ENCODER CODE END ***************************************************************************************************/

/** MENU ANALOG INPUT CODE BEGIN ********************************************************************************************/

void __menu_ain1_routine(MenuItem_TypeDef* hmenuitem){ menu_ain_routine(hmenuitem, adc1_voltages[0], 1); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ain1, &menu_ain2, &menu_enc1, 100 /* ms */);

void __menu_ain2_routine(MenuItem_TypeDef* hmenuitem){ menu_ain_routine(hmenuitem, adc1_voltages[1], 2); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_ain2, &menu_bmp2, &menu_ain1, 100 /* ms */);

/** MENU ANALOG INPUT CODE END **********************************************************************************************/

/** MENU DIGITAL SENSORS CODE BEGIN *****************************************************************************************/

void __menu_bmp2_routine(MenuItem_TypeDef* hmenuitem){
	menu_digital_sensor_routine(hmenuitem, (float)BMP2_ReadTemperature_degC(&hbmp2_1), "TEMP:  %2.02f \xdf\x43");
}
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_bmp2, &menu_bh1750, &menu_ain2, 250 /* ms */);

void __menu_bh1750_routine(MenuItem_TypeDef* hmenuitem){
	menu_digital_sensor_routine(hmenuitem, BH1750_ReadIlluminance_lux(&hbh1750_1), "LIGTH: %5.0f lx");
}
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_bh1750, &menu_aout2, &menu_bmp2, 200 /* ms */);

/** MENU DIGITAL SENSORS CODE END *******************************************************************************************/

/** MENU ANALOG OUTPUT CODE BEGIN *******************************************************************************************/

void __menu_aout2_routine(MenuItem_TypeDef* hmenuitem){ menu_aout_routine(hmenuitem, &hdac, DAC_CHANNEL_2); }
MenuItem_TypeDef MENU_ITEM_CONTRUCTOR(menu_aout2, &menu_ledr1, &menu_bh1750, 100 /* ms */);


/** MENU ANALOG OUTPUT CODE END *********************************************************************************************/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/
void menu_led_routine(MenuItem_TypeDef* hmenuitem, LED_HandleTypeDef* hled, const char led_name[3])
{
  if(hmenu.Item == hmenuitem) // If active component
  {
	  ENC_GetCounter(&henc1);
	  if(henc1.CounterInc)
		  LED_On(hled);   // If counter has increased - turn LED on
	  else if(henc1.CounterDec)
		  LED_Off(hled);  // If counter has decreased - turn LED off
  }

  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, "%s: %s", led_name, LED_Read(hled) ? "ON" : "OFF");
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = 0;
}

void menu_ledrgb_routine(MenuItem_TypeDef* hmenuitem, LED_RGB_HandleTypeDef* hledrgb, LED_RGB_ChannelType channel)
{
  if(hmenu.Item == hmenuitem) // If active component
  {
	ENC_GetCounter(&henc1);
	if(henc1.CounterInc || henc1.CounterDec) // If counter changed - update duty
		LED_RGB_SetDuty(hledrgb, channel, ENC2DUTY(&henc1));
  }

  const char label[][6] = { "[R]GB", "R[G]B", "RG[B]" };

  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, "LD %s: %d", label[channel], (int)LED_RGB_GetDuty(hledrgb, channel));
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = 0;
}

void menu_enc_routine(MenuItem_TypeDef* hmenuitem, ENC_HandleTypeDef* henc, int enc_n)
{
  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, "ENC%d: %3d", enc_n, (int)ENC_GetCounter(henc));
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = 0;
}

void menu_ain_routine(MenuItem_TypeDef* hmenuitem, float value, int ain_n)
{
  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, "AIN%d: %4d mV", ain_n, (int)value);
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = snprintf(hmenuitem->SerialPortStr, LCD_LINE_LEN, "%03x", ADC_VOLTAGE2REG(value));
}

void menu_digital_sensor_routine(MenuItem_TypeDef* hmenuitem, float value, const char* format)
{
  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, format, value);
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = 0;
}

void menu_aout_routine(MenuItem_TypeDef* hmenuitem, DAC_HandleTypeDef *hdac, uint32_t Channel)
{
  if(hmenu.Item == hmenuitem) // If active component
  {
    ENC_GetCounter(&henc1);
    if(henc1.CounterInc || henc1.CounterDec) // If counter changed - update duty
      HAL_DAC_SetValue(hdac, Channel, DAC_ALIGN_12B_R, DAC_VOLTAGE2REG(ENC2VOLTAGE(&henc1) /* V */));
  }

  float value = 1000.0f*DAC_REG2VOLTAGE(HAL_DAC_GetValue(hdac, Channel)); /* [mV] */
  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, "AOUT%d: %4d mV", (Channel == DAC_CHANNEL_1) ? 1 : 2, (int)value);
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  //hmenuitem->SerialPortStrLen = 0;
  hmenuitem->SerialPortStrLen = snprintf(hmenuitem->SerialPortStr, LCD_LINE_LEN, "%03x", ADC_VOLTAGE2REG(adc1_voltages[0]));
}

/* Public function -----------------------------------------------------------*/
