/**
 ******************************************************************************
 * @file    tm1637.c
 * @author  AW		Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    08 Sep 2022
 * @brief   TM1637 driver: 4-digit 7-segment display module
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tm1637.h"

/* Define --------------------------------------------------------------------*/
#define TM1637_CYCLES_PER_1US 72

/* Private variables ---------------------------------------------------------*/
const char TM1637_SEGMENT_MAP[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
    0x00
};

/* Private function prototypes -----------------------------------------------*/
void tm1637_ClkHigh(TM1637_HandleTypeDef* hdisp);
void tm1637_ClkLow(TM1637_HandleTypeDef* hdisp);
void tm1637_DioHigh(TM1637_HandleTypeDef* hdisp);
void tm1637_DioLow(TM1637_HandleTypeDef* hdisp);
void tm1637_Start(TM1637_HandleTypeDef* hdisp);
void tm1637_Stop(TM1637_HandleTypeDef* hdisp);
void tm1637_ReadResult(TM1637_HandleTypeDef* hdisp);
void tm1637_WriteByte(TM1637_HandleTypeDef* hdisp, unsigned char b);
void tm1637_DelayUsec(unsigned int i);

/* Public function -----------------------------------------------------------*/

/**
 * @brief TODO
 * @param[in] hdisp : TODO
 * @param[in] dec   : TODO
 * @return None
 */
void TM1637_printDecUInt(TM1637_HandleTypeDef* hdisp, uint16_t dec)
{
  unsigned char digitArr[4];
  for (int i = 0; i < 4; ++i)
  {
    digitArr[i] = TM1637_SEGMENT_MAP[dec % 10];
    if (i == 2 && hdisp->Separator == TM1637_SEP_COLON)
    {
      digitArr[i] |= 1 << 7;
    }
    dec /= 10;
  }

  tm1637_Start(hdisp);
  tm1637_WriteByte(hdisp, 0x40);
  tm1637_ReadResult(hdisp);
  tm1637_Stop(hdisp);

  tm1637_Start(hdisp);
  tm1637_WriteByte(hdisp, 0xc0);
  tm1637_ReadResult(hdisp);

  for (int i = 0; i < 4; ++i) {
    tm1637_WriteByte(hdisp, digitArr[3 - i]);
    tm1637_ReadResult(hdisp);
  }
  tm1637_Stop(hdisp);
}


/**
 * @brief TODO Valid brightness values: 0 - 8. 0 = display off.
 * @param[in] hdisp      : TODO
 * @param[in] brightness : TODO
 * @return None
 */
void TM1637_SetBrightness(TM1637_HandleTypeDef* hdisp, uint8_t brightness)
{
  // Brightness command:
  // 1000 0XXX = display off
  // 1000 1BBB = display on, brightness 0-7
  // X = don't care
  // B = brightness
  tm1637_Start(hdisp);
  tm1637_WriteByte(hdisp, 0x87 + brightness);
  tm1637_ReadResult(hdisp);
  tm1637_Stop(hdisp);
}

/* Private function ----------------------------------------------------------*/
void tm1637_ClkHigh(TM1637_HandleTypeDef* hdisp)
{
  HAL_GPIO_WritePin(hdisp->CLK_Port, hdisp->CLK_Pin, GPIO_PIN_SET);
}

void tm1637_ClkLow(TM1637_HandleTypeDef* hdisp)
{
  HAL_GPIO_WritePin(hdisp->CLK_Port, hdisp->CLK_Pin, GPIO_PIN_RESET);
}

void tm1637_DioHigh(TM1637_HandleTypeDef* hdisp)
{
  HAL_GPIO_WritePin(hdisp->DIO_Port, hdisp->DIO_Pin, GPIO_PIN_SET);
}

void tm1637_DioLow(TM1637_HandleTypeDef* hdisp)
{
  HAL_GPIO_WritePin(hdisp->DIO_Port, hdisp->DIO_Pin, GPIO_PIN_RESET);
}

void tm1637_Start(TM1637_HandleTypeDef* hdisp)
{
    tm1637_ClkHigh(hdisp);
    tm1637_DioHigh(hdisp);
    tm1637_DelayUsec(2);
    tm1637_DioLow(hdisp);
}

void tm1637_Stop(TM1637_HandleTypeDef* hdisp)
{
    tm1637_ClkLow(hdisp);
    tm1637_DelayUsec(2);
    tm1637_DioLow(hdisp);
    tm1637_DelayUsec(2);
    tm1637_ClkHigh(hdisp);
    tm1637_DelayUsec(2);
    tm1637_DioHigh(hdisp);
}

void tm1637_ReadResult(TM1637_HandleTypeDef* hdisp)
{
    tm1637_ClkLow(hdisp);
    tm1637_DelayUsec(5);
    // while (dio); // We're cheating here and not actually reading back the response.
    tm1637_ClkHigh(hdisp);
    tm1637_DelayUsec(2);
    tm1637_ClkLow(hdisp);
}

void tm1637_WriteByte(TM1637_HandleTypeDef* hdisp, unsigned char b)
{
  for (int i = 0; i < 8; ++i)
  {
    tm1637_ClkLow(hdisp);
    if (b & 0x01)
    {
      tm1637_DioHigh(hdisp);
    }
    else
    {
      tm1637_DioLow(hdisp);
    }
    tm1637_DelayUsec(3);
    b >>= 1;
    tm1637_ClkHigh(hdisp);
    tm1637_DelayUsec(3);
  }
}

void tm1637_DelayUsec(unsigned int i)
{
  for (; i>0; i--)
  {
    for (int j = 0; j < TM1637_CYCLES_PER_1US; ++j)
    {
      __asm__ __volatile__("nop\n\t":::"memory");
    }
  }
}
