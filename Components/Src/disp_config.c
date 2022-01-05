/**
  ******************************************************************************
  * @file    disp_config.c
  * @author  AW
  * @version V1.0
  * @date    4-Nov-2020
  * @brief   Simple 4 x 8 segment LED display driver library configuration file.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "disp.h"
#include "disp_config.h"
#include "main.h"
#include "tim.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
DISP_HandleTypeDef hdisp1 = {
  // Display GPIO ports and pins
  .Segment_Ports = {
    DISP_A_GPIO_Port, DISP_B_GPIO_Port, DISP_C_GPIO_Port, 
    DISP_D_GPIO_Port, DISP_E_GPIO_Port, DISP_F_GPIO_Port, 
    DISP_G_GPIO_Port
  },

  .Segment_Pins = {
    DISP_A_Pin, DISP_B_Pin, DISP_C_Pin, DISP_D_Pin, 
    DISP_E_Pin, DISP_F_Pin, DISP_G_Pin
  },
  
  .Digit_Ports = {
    DISP_1_GPIO_Port, DISP_2_GPIO_Port,
    DISP_3_GPIO_Port, DISP_4_GPIO_Port
  },
  
  .Digit_Pins = {
    DISP_1_Pin, DISP_2_Pin, DISP_3_Pin, DISP_4_Pin,
  },
  
  .DecimalPoint_Port = DISP_H_GPIO_Port,
  .DecimalPoint_Pin = DISP_H_Pin,
  
  .Timer = &htim7,

  // Display data
  .DataArray = {0, 0, 0, 0},
  .DecimalPoint = DISP_DP_1
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
