/**
  ******************************************************************************
  * @file    led_rgb.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version V1.0
  * @date    02-Nov-2020
  * @brief   Simple tricolor (RGB) LED driver library.
  *
  ******************************************************************************
  */
#ifndef INC_LED_RGB_H_
#define INC_LED_RGB_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Typedef -------------------------------------------------------------------*/
#define LED_RGB_TimerType   TIM_HandleTypeDef*
#define LED_RGB_ChannelType uint16_t

typedef struct {
  LED_RGB_TimerType Timer;
  LED_RGB_ChannelType ChannelR;
  float DutyR;
  LED_RGB_ChannelType ChannelG;
  float DutyG;
  LED_RGB_ChannelType ChannelB;
  float DutyB;
} LED_RGB_HandleTypeDef;

typedef enum {
	LED_CHANNEL_R,
	LED_CHANNEL_G,
	LED_CHANNEL_B
} LED_Channel;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief LED RGB initialization.
 * @param[in] hledrgb LED RGB handler
 * @return None
 */
void LED_RGB_Init(LED_RGB_HandleTypeDef* hledrgb);

/**
 * @brief Sets duty of selected channel of LED RGB.
 * @param[in] hledrgb LED RGB handler
 * @param[in] ch      LED RGB channel (color)
 *     @arg LED_CHANNEL_R: Red channel selected
 *     @arg LED_CHANNEL_G: Green channel selected
 *     @arg LED_CHANNEL_B: Blue channel selected
 * @param[in] duty    PWM duty in percents
 * @return None
 */
void LED_RGB_SetDuty(LED_RGB_HandleTypeDef* hledrgb, LED_Channel ch, float duty);

/**
 * @brief Gets duty of selected channel of LED RGB.
 * @param[in] hledrgb LED RGB handler
 * @param[in] ch      LED RGB channel (color)
 *     @arg LED_CHANNEL_R: Red channel selected
 *     @arg LED_CHANNEL_G: Green channel selected
 *     @arg LED_CHANNEL_B: Blue channel selected
 * @return PWM duty in percents. Negative number if error.
 */
float LED_RGB_GetDuty(LED_RGB_HandleTypeDef* hledrgb, LED_Channel ch);

/**
 * @brief Sets color LED RGB.
 * @param[in] hledrgb LED RGB handler
 * @param[in] color   RGB color as word (00RRGGBB)
 * @return None
 */
void LED_RGB_SetColor(LED_RGB_HandleTypeDef* hledrgb, uint32_t color);

#endif /* INC_LED_RGB_H_ */
