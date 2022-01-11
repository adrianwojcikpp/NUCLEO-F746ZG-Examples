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

#define MOD(a,b) (a>0) ? (a%b) : (a%b)+b

/* Macro ---------------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void menu_led_routine(MenuItem_TypeDef* hmenuitem, LED_HandleTypeDef* hled, const char* led_name);
void menu_ledrgb_routine(MenuItem_TypeDef* hmenuitem, LED_RGB_HandleTypeDef* hledrgb, LED_RGB_ChannelType channel);
void menu_ain_routine(MenuItem_TypeDef* hmenuitem, float value, int ain_n);
void menu_aout_routine(MenuItem_TypeDef* hmenuitem, DAC_HandleTypeDef *hdac, uint32_t Channel);
void menu_enc_routine(MenuItem_TypeDef* hmenuitem, ENC_HandleTypeDef* henc, int enc_n);
void menu_digital_sensor_routine(MenuItem_TypeDef* hmenuitem, float value, const char* format);

/* Private variables ---------------------------------------------------------*/
extern float adc1_voltages[ADC1_NUMBER_OF_CONV];

/** MENU LED CODE BEGIN ******************************************************************************************************/

MENU_ITEM_CONTRUCTOR(menu_ledr1, 100 /* ms */, { menu_led_routine(hmenuitem, &hledr1, "LDR1"); } );
MENU_ITEM_CONTRUCTOR(menu_ledg1, 100 /* ms */, { menu_led_routine(hmenuitem, &hledg1, "LDG1"); } );
MENU_ITEM_CONTRUCTOR(menu_ledb1, 100 /* ms */, { menu_led_routine(hmenuitem, &hledb1, "LDB1"); } );
MENU_ITEM_CONTRUCTOR(menu_ledr2, 100 /* ms */, { menu_led_routine(hmenuitem, &hledr2, "LDR2"); } );
MENU_ITEM_CONTRUCTOR(menu_ledg2, 100 /* ms */, { menu_led_routine(hmenuitem, &hledg2, "LDG2"); } );
MENU_ITEM_CONTRUCTOR(menu_ledb2, 100 /* ms */, { menu_led_routine(hmenuitem, &hledb2, "LDB2"); } );

/** MENU LED CODE END *******************************************************************************************************/

/** MENU LED RGB CODE BEGIN *************************************************************************************************/

MENU_ITEM_CONTRUCTOR(menu_ledrgb_r, 100 /* ms */, { menu_ledrgb_routine(hmenuitem, &hledrgb1, LED_CHANNEL_R); } );
MENU_ITEM_CONTRUCTOR(menu_ledrgb_g, 100 /* ms */, { menu_ledrgb_routine(hmenuitem, &hledrgb1, LED_CHANNEL_G); } );
MENU_ITEM_CONTRUCTOR(menu_ledrgb_b, 100 /* ms */, { menu_ledrgb_routine(hmenuitem, &hledrgb1, LED_CHANNEL_B); } );

/** MENU LED RGB CODE END ***************************************************************************************************/

/** MENU ENCODER CODE BEGIN *************************************************************************************************/

MENU_ITEM_CONTRUCTOR(menu_enc1, 100 /* ms */, { menu_enc_routine(hmenuitem, &henc1, 1); } );

/** MENU ENCODER CODE END ***************************************************************************************************/

/** MENU ANALOG INPUT CODE BEGIN ********************************************************************************************/

MENU_ITEM_CONTRUCTOR(menu_ain1, 100 /* ms */, { menu_ain_routine(hmenuitem, adc1_voltages[0], 1); } );
MENU_ITEM_CONTRUCTOR(menu_ain2, 100 /* ms */, { menu_ain_routine(hmenuitem, adc1_voltages[1], 2); } );

/** MENU ANALOG INPUT CODE END **********************************************************************************************/

/** MENU DIGITAL SENSORS CODE BEGIN *****************************************************************************************/

MENU_ITEM_CONTRUCTOR(menu_bmp2, 250 /* ms */, {
  menu_digital_sensor_routine(hmenuitem, (float)BMP2_ReadTemperature_degC(&hbmp2_1), "TEMP:  %2.02f \xdf\x43");
});

MENU_ITEM_CONTRUCTOR(menu_bh1750, 200 /* ms */, {
  menu_digital_sensor_routine(hmenuitem, BH1750_ReadIlluminance_lux(&hbh1750_1), "LIGTH: %5.0f lx");
});

/** MENU DIGITAL SENSORS CODE END *******************************************************************************************/

/** MENU ANALOG OUTPUT CODE BEGIN *******************************************************************************************/

MENU_ITEM_CONTRUCTOR(menu_aout2, 100 /* ms */, { menu_aout_routine(hmenuitem, &hdac, DAC_CHANNEL_2); } );

/** MENU ANALOG OUTPUT CODE END *********************************************************************************************/

#define MENU_MAIN_LEN (sizeof(MENU_MAIN_ARRAY)/sizeof(MENU_MAIN_ARRAY[0]))
MenuItem_TypeDef* MENU_MAIN_ARRAY[] = { /* Main menu list */
	&menu_ledr1,    /* LED Red #1: on-board red LED (LD3) */
	&menu_ledr2,    /* LED Red #2: breadboard red LED (LD3EX) */
	&menu_ledg1,    /* LED Green #1: on-board green LED (LD1) */
	&menu_ledg2,    /* LED Green #2: breadboard green LED (LD1EX) */
	&menu_ledb1,    /* LED Blue #1: on-board blue LED (LD2) */
	&menu_ledb2,    /* LED Blue #2: breadboard blue LED (LD2EX) */
	&menu_ledrgb_r, /* LED RGB: channel R */
	&menu_ledrgb_g, /* LED RGB: channel G */
	&menu_ledrgb_b, /* LED RGB: channel B */
	&menu_enc1,     /* Rotary encoder #1 */
	&menu_ain1,     /* Analog input #1 */
	&menu_ain2,     /* Analog input #2 */
	&menu_bmp2,     /* Temperature & pressure sensor: BMP280 */
	&menu_bh1750,   /* Light sensor: BH1750 */
	&menu_aout2     /* Analog output #2 */
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/**
 * @brief Common LED RGB menu routine.
 * @param[in/out] hmenuitem : menu item handler
 * @param[in]     hled      : LED handler
 * @param[in]     led_name  : LED display name
 */
void menu_led_routine(MenuItem_TypeDef* hmenuitem, LED_HandleTypeDef* hled, const char* led_name)
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

/**
 * @brief Common LED RGB menu routine.
 * @param[in/out] hmenuitem : menu item handler
 * @param[in]     hledrgb   : LED RGB handler
 * @param[in]     channel   : encoder number
 *   This parameter can be one of the following values:
 *     @arg LED_CHANNEL_R: Red channel selected
 *     @arg LED_CHANNEL_G: Green channel selected
 *     @arg LED_CHANNEL_B: Blue channel selected
 */
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

/**
 * @brief Common encoder menu routine.
 * @param[in/out] hmenuitem : menu item handler
 * @param[in]     henc      : encoder handler
 * @param[in]     enc_n     : encoder number
 */
void menu_enc_routine(MenuItem_TypeDef* hmenuitem, ENC_HandleTypeDef* henc, int enc_n)
{
  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, "ENC%d: %3d", enc_n, (int)ENC_GetCounter(henc));
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = 0;
}

/**
 * @brief Common digital sensor menu routine.
 * @param[in/out] hmenuitem : menu item handler
 * @param[in]     value     : analog input reading [mV]
 * @param[in]     ain_n     : input number
 */
void menu_ain_routine(MenuItem_TypeDef* hmenuitem, float value, int ain_n)
{
  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, "AIN%d: %4d mV", ain_n, (int)value);
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = snprintf(hmenuitem->SerialPortStr, LCD_LINE_LEN, "%03x", ADC_VOLTAGE2REG(value));
}

/**
 * @brief Common digital sensor menu routine.
 * @param[in/out] hmenuitem : menu item handler
 * @param[in]     value     : sensor reading
 * @param[in]     format    : printing format
 */
void menu_digital_sensor_routine(MenuItem_TypeDef* hmenuitem, float value, const char* format)
{
  char temp_str[LCD_LINE_BUF_LEN];
  hmenuitem->DisplayStrLen = snprintf(temp_str, LCD_LINE_LEN, format, value);
  MENU_ITEM_SetDisplayBuffer(hmenuitem, temp_str); // Set display buffer
  hmenuitem->SerialPortStrLen = 0;
}

/**
 * @brief Common analog output menu routine.
 * @param[in/out] hmenuitem : menu item handler
 * @param[in]     hdac      : DAC handler
 * @param[in]     Channel   : DAC Channel
 *   This parameter can be one of the following values:
 *     @arg DAC_CHANNEL_1: DAC Channel1 selected
 *     @arg DAC_CHANNEL_2: DAC Channel2 selected
 */
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

/* Public variables ----------------------------------------------------------*/
Menu_TypeDef hmenu = {
  .Item = &menu_ledr1, .Display = &hlcd1, .Timer = &htim10, .SerialPort = &huart3,
};

/* Public function -----------------------------------------------------------*/
/**
 * @brief TODO
 * @param[in/out] hmenuitem :
 */
void MENU_Init(Menu_TypeDef* hmenu)
{
  /* Active element initialization */
  hmenu->Item = MENU_MAIN_ARRAY[0];

  /* Main menu initialization */
  for(int i = 0; i < (int)MENU_MAIN_LEN; i++)
  {
	MENU_MAIN_ARRAY[i]->Next = MENU_MAIN_ARRAY[MOD((i+1), MENU_MAIN_LEN)];
	MENU_MAIN_ARRAY[i]->Prev = MENU_MAIN_ARRAY[MOD((i-1), MENU_MAIN_LEN)];
  }

  /* LCD set-up */
  LCD_SetCursor(hmenu->Display, 0, 0);
  LCD_printf(hmenu->Display, "%c", LCD_MENU_CURSOR_CHAR);
  LCD_SetCursor(hmenu->Display, 1, 0);
  LCD_printStr(hmenu->Display, " ");

  HAL_TIM_Base_Start_IT(hmenu->Timer);
}

/**
 * @brief TODO
 * @param[in/out] hmenuitem :
 */
void MENU_ROUTINE(Menu_TypeDef* hmenu)
{
  hmenu->Item->Routine(hmenu->Item);
  hmenu->Item->Next->Routine(hmenu->Item->Next);

  // #1 line - active item
  LCD_SetCursor(hmenu->Display, 0, 1);
  LCD_printStr(hmenu->Display, hmenu->Item->DisplayStr);

  // #1 line - next item
  LCD_SetCursor(hmenu->Display, 1, 1);
  LCD_printStr(hmenu->Display, hmenu->Item->Next->DisplayStr);

  // Serial port streaming
  HAL_UART_Transmit(hmenu->SerialPort, (uint8_t*)hmenu->Item->SerialPortStr, hmenu->Item->SerialPortStrLen, 10);
}

