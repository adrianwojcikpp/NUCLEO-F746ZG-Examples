/**
 ******************************************************************************
 * @file    tm1637_config.c
 * @author  AW		Adrian.Wojcik@put.poznan.pl
 * @version 1.0
 * @date    08 Sep 2022
 * @brief   TM1637 driver: 4-digit 7-segment display module
 *          CONFIGURATION FILE
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tm1637.h"
#include "main.h"

/* Public variables ----------------------------------------------------------*/
TM1637_HandleTypeDef htm1637_disp1 = {
  .CLK_Port = DISP1_CLK_GPIO_Port, .CLK_Pin = DISP1_CLK_Pin,
  .DIO_Port = DISP1_DIO_GPIO_Port, .DIO_Pin = DISP1_DIO_Pin,
  .Separator = TM1637_SEP_NONE
};

TM1637_HandleTypeDef htm1637_disp2 = {
  .CLK_Port = DISP2_CLK_GPIO_Port, .CLK_Pin = DISP2_CLK_Pin,
  .DIO_Port = DISP2_DIO_GPIO_Port, .DIO_Pin = DISP2_DIO_Pin,
  .Separator = TM1637_SEP_NONE
};

TM1637_HandleTypeDef htm1637_disp3 = {
  .CLK_Port = DISP3_CLK_GPIO_Port, .CLK_Pin = DISP3_CLK_Pin,
  .DIO_Port = DISP3_DIO_GPIO_Port, .DIO_Pin = DISP3_DIO_Pin,
  .Separator = TM1637_SEP_NONE
};
