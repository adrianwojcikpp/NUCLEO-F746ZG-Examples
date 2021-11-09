/**
  ******************************************************************************
  * @file    bh1750_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    04-Nov-2020
  * @brief   Simple driver for digital light sensor with I2C: BH1750
  *          configuration file.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "bh1750.h"
#include "bh1750_config.h"
#include "main.h"
#include "i2c.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
BH1750_HandleTypeDef hbh1750_1 = {
  .I2C = &hi2c1, .Address = BH1750_ADDRESS_L, .Timeout = 0xffff
};

BH1750_HandleTypeDef hbh1750_2 = {
  .I2C = &hi2c1, .Address = BH1750_ADDRESS_H, .Timeout = 0xffff
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/
