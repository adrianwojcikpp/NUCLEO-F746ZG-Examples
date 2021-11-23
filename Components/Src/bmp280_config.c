/**
  ******************************************************************************
  * @file    bmp280_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    15-Nov-2020
  * @brief   Configuration file for BMP280 sensor driver library;
  *          SPI routines implementation.
  * @ref     https://github.com/BoschSensortec/BMP280_driver
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "bmp280.h"
#include "bmp280_defs.h"
#include "bmp280_config.h"
#include "main.h"
#include "spi.h"
#include <string.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
BMP280_CS_PortType BMP280_CS_Ports[BMP280_NUM_OF_SENSORS] = {
  BMP280_CS1_GPIO_Port, BMP280_CS2_GPIO_Port
};

BMP280_CS_PinType BMP280_CS_Pins[BMP280_NUM_OF_SENSORS] = {
  BMP280_CS1_Pin,       BMP280_CS2_Pin
};

/* Public variables ----------------------------------------------------------*/
struct bmp280_dev hbmp280_1 = {
  .dev_id = 0,
  .intf = BMP280_SPI_INTF,
  .read = bmp280_spi_reg_read, .write = bmp280_spi_reg_write,
  .delay_ms = HAL_Delay
};

struct bmp280_dev hbmp280_2 = {
  .dev_id = 1,
  .intf = BMP280_SPI_INTF,
  .read = bmp280_spi_reg_read, .write = bmp280_spi_reg_write,
  .delay_ms = HAL_Delay
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/

/*!
 *  @brief BMP280 initialization function.
 *  @note Enables both pressure and temperature measurement with no oversampling.
 *        Disables internal digital filters. Sets measurement frequency to 4 Hz.
 *        Uses blocking mode SPI transmitting and receiving routine.
 *  @param[in] bmp BMP280 device structure
 *  @return Status of execution
 *  @retval 0  -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t BMP280_Init(struct bmp280_dev* bmp)
{
  int8_t rslt;
  struct bmp280_config conf;

  rslt = bmp280_init(bmp);

  /* Always read the current settings before writing, especially when all the configuration is not modified  */
  rslt = bmp280_get_config(&conf, bmp);

  /* configuring the temperature oversampling, filter coefficient and output data rate */
  /* Overwrite the desired settings */
  conf.filter = BMP280_FILTER_OFF;

  /* Temperature oversampling set at 1x */
  conf.os_temp = BMP280_OS_1X;

  /* Temperature oversampling set at 1x */
  conf.os_pres = BMP280_OS_1X;

  /* Setting the output data rate as 4 Hz (250 ms) */
  conf.odr = BMP280_ODR_250_MS;

  rslt = bmp280_set_config(&conf, bmp);

  /* Always set the power mode after setting the configuration */
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, bmp);

  return rslt;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *  @note Uses blocking mode SPI transmitting routine.
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length   : No of bytes to write.
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t bmp280_spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  /* Implement the SPI write routine according to the target machine. */
  HAL_StatusTypeDef status = HAL_OK;
  int8_t iError = 0;

#ifdef DEBUG
  uint8_t data[BMP280_SPI_BUFFER_LEN] = {0,};
  memcpy(data, reg_data, length);
#endif

  /* Software slave selection procedure */
  HAL_GPIO_WritePin(BMP280_CS_Ports[cs], BMP280_CS_Pins[cs], GPIO_PIN_RESET);

  /* Data exchange */
  status  = HAL_SPI_Transmit(BMP280_SPI, &reg_addr, BMP280_REG_ADDR_LEN, BMP280_TIMEOUT);
  status += HAL_SPI_Transmit(BMP280_SPI,  reg_data, length,            BMP280_TIMEOUT);

  /* Disable all slaves */
  for(uint8_t i = 0; i < BMP280_NUM_OF_SENSORS; i++)
    HAL_GPIO_WritePin(BMP280_CS_Ports[i], BMP280_CS_Pins[i], GPIO_PIN_SET);

  // The BMP2xx API calls for 0 return value as a success, and -1 returned as failure
  if (status != HAL_OK)
    iError = -1;

  return iError;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *  @note Uses blocking mode SPI receiving routine.
 *  @param[in]  cs       : Chip select to enable the sensor
 *  @param[in]  reg_addr : Register address
 *  @param[out] reg_data : Pointer to the data buffer to store the read data
 *  @param[in]  length   : No of bytes to read
 *  @return Status of execution
 *  @retval 0  -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t bmp280_spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  /* Implement the SPI read routine according to the target machine. */
  HAL_StatusTypeDef status = HAL_OK;
  int8_t iError = 0;

#ifdef DEBUG
  uint8_t data[BMP280_SPI_BUFFER_LEN] = {0,};
  memcpy(data, reg_data, length);
#endif

  /* Software slave selection procedure */
  HAL_GPIO_WritePin(BMP280_CS_Ports[cs], BMP280_CS_Pins[cs], GPIO_PIN_RESET);

  /* Data exchange */
  status  = HAL_SPI_Transmit(BMP280_SPI, &reg_addr, BMP280_REG_ADDR_LEN, BMP280_TIMEOUT);
  status += HAL_SPI_Receive( BMP280_SPI,  reg_data, length,            BMP280_TIMEOUT);

  /* Disable all slaves */
  for(uint8_t i = 0; i < BMP280_NUM_OF_SENSORS; i++)
    HAL_GPIO_WritePin(BMP280_CS_Ports[i], BMP280_CS_Pins[i], GPIO_PIN_SET);

  // The BMP2xx API calls for 0 return value as a success, and -1 returned as failure
  if (status != HAL_OK)
    iError = -1;

  return iError;
}

/*!
 *  @brief This internal API is used to get compensated pressure and temperature data.
 *  @param[in]  dev   : BMP280 device structure
 *  @param[out] press : Pressure measurement [hPa]
 *  @param[out] temp  : Temperature measurement [degC]
 *
 *  @return Status of execution
 *
 *  @retval 0 -> Success.
 *  @retval <0 -> Failure.
 *
 */
int8_t BMP280_ReadData(struct bmp280_dev *dev, float* press, float* temp)
{
  int8_t rslt = BMP280_OK;
  int32_t temp_int;
  uint32_t press_int;
  struct bmp280_uncomp_data bmp280_data;
  rslt = bmp280_get_uncomp_data(&bmp280_data, &hbmp280_1);
  rslt = bmp280_get_comp_temp_32bit(&temp_int,  bmp280_data.uncomp_temp,  dev);
  rslt = bmp280_get_comp_pres_32bit(&press_int, bmp280_data.uncomp_press, dev);
  *temp = (float)temp_int / 100.0f;
  *press = (float)press_int / 100.0f;
  return rslt;
}

/*!
 *  @brief This internal API is used to get compensated temperature data.
 *  @param[in]  dev   : BMP280 device structure
 *
 *  @return Temperature measurement [degC]
 */
float BMP280_ReadTemperature_degC(struct bmp280_dev *dev)
{
  int32_t temp_int;
  struct bmp280_uncomp_data bmp280_data;
  bmp280_get_uncomp_data(&bmp280_data, &hbmp280_1);
  bmp280_get_comp_temp_32bit(&temp_int,  bmp280_data.uncomp_temp,  dev);
  return (float)temp_int / 100.0f;
}

/*!
 *  @brief This internal API is used to get compensated pressure data.
 *  @param[in]  dev   : BMP280 device structure
 *
 *  @return Pressure measurement [hPa]
 */
float BMP280_ReadPressure_hPa(struct bmp280_dev *dev)
{
  uint32_t press_int;
  struct bmp280_uncomp_data bmp280_data;
  bmp280_get_uncomp_data(&bmp280_data, &hbmp280_1);
  bmp280_get_comp_pres_32bit(&press_int, bmp280_data.uncomp_press, dev);
  return (float)press_int / 100.0f;
}
