/**
  ******************************************************************************
  * @file    bmp2_config.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 2.0
  * @date    20-Nov-2021
  * @brief   Configuration file for BMP280 sensor driver library;
  *          SPI routines implementation.
  * @ref     https://github.com/BoschSensortec/BMP2-Sensor-API
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bmp2.h"
#include "bmp2_config.h"
#include "main.h"
#include "spi.h"
#include <string.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
BMP2_CS_IndexType BMP2_CS_Indexes[BMP2_NUM_OF_SENSORS] = {
  0,                    1,
};

BMP2_CS_PortType  BMP2_CS_Ports[BMP2_NUM_OF_SENSORS] = {
  BMP280_CS1_GPIO_Port, BMP280_CS2_GPIO_Port
};

BMP2_CS_PinType   BMP2_CS_Pins[BMP2_NUM_OF_SENSORS] = {
  BMP280_CS1_Pin,       BMP280_CS2_Pin
};

/* Public variables ----------------------------------------------------------*/
struct bmp2_dev hbmp2_1 = {
  .intf_ptr = (void*) &BMP2_CS_Indexes[0],
  .intf = BMP2_SPI_INTF,
  .read = bmp2_spi_read, .write = bmp2_spi_write,
  .delay_us = bmp2_delay_us
};

struct bmp2_dev hbmp2_2 = {
  .intf_ptr = (void*) &BMP2_CS_Indexes[1],
  .intf = BMP2_SPI_INTF,
  .read = bmp2_spi_read, .write = bmp2_spi_write,
  .delay_us = bmp2_delay_us
};

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/

/*!
 *  @brief BMP2xx initialization function.
 *  @note Enables both pressure and temperature measurement with no oversampling.
 *        Disables internal digital filters. Sets measurement frequency to 4 Hz.
 *        Uses blocking mode SPI transmitting and receiving routine.
 *  @param[in] dev : BMP2xx device structure
 *
 *  @return Status of execution
 *
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 *
 */
int8_t BMP2_Init(struct bmp2_dev* dev)
{
  int8_t rslt;
  uint32_t meas_time;
  struct bmp2_config conf;

  rslt = bmp2_init(dev);

  /* Always read the current settings before writing, especially when all the configuration is not modified */
  rslt = bmp2_get_config(&conf, dev);

  /* Configuring the over-sampling mode, filter coefficient and output data rate */
  /* Overwrite the desired settings */
  conf.filter = BMP2_FILTER_OFF;
  /* Over-sampling mode is set as ultra low resolution i.e., os_pres = 1x and os_temp = 1x */
  conf.os_mode = BMP2_OS_MODE_ULTRA_LOW_POWER;
  /* Setting the output data rate */
  conf.odr = BMP2_ODR_250_MS;

  rslt = bmp2_set_config(&conf, dev);

  /* Set normal power mode */
  rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &conf, dev);

  /* Calculate measurement time in microseconds */
  rslt = bmp2_compute_meas_time(&meas_time, &conf, dev);

  return rslt;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr   : Register address.
 *  @param[out] reg_data  : Pointer to the data buffer to store the read data.
 *  @param[in] length     : No of bytes to read.
 *  @param[in] intf_ptr   : Interface pointer
 *
 *  @return Status of execution
 *
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 *
 */
BMP2_INTF_RET_TYPE bmp2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  /* Implement the SPI read routine according to the target machine. */
  HAL_StatusTypeDef status = HAL_OK;
  int8_t iError = BMP2_INTF_RET_SUCCESS;
  uint8_t cs = *(uint8_t*)intf_ptr;

  /* Software slave selection procedure */
  HAL_GPIO_WritePin(BMP2_CS_Ports[cs], BMP2_CS_Pins[cs], GPIO_PIN_RESET);

  /* Data exchange */
  status  = HAL_SPI_Transmit(BMP2_SPI, &reg_addr, BMP2_REG_ADDR_LEN, BMP2_TIMEOUT);
  status += HAL_SPI_Receive( BMP2_SPI,  reg_data, length,            BMP2_TIMEOUT);

  /* Disable all slaves */
  for(uint8_t i = 0; i < BMP2_NUM_OF_SENSORS; i++)
    HAL_GPIO_WritePin(BMP2_CS_Ports[i], BMP2_CS_Pins[i], GPIO_PIN_SET);

#ifdef DEBUG
  uint8_t data[BMP2_SPI_BUFFER_LEN] = {0,};
  memcpy(data, reg_data, length);
#endif

  // The BMP2xx API calls for 0 return value as a success, and -1 returned as failure
  if (status != HAL_OK)
    iError = -1;

  return iError;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] reg_addr   : Register address.
 *  @param[in] reg_data   : Pointer to the data buffer whose data has to be written.
 *  @param[in] length     : No of bytes to write.
 *  @param[in] intf_ptr   : Interface pointer
 *
 *  @return Status of execution
 *
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 *
 */
BMP2_INTF_RET_TYPE bmp2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  /* Implement the SPI write routine according to the target machine. */
  HAL_StatusTypeDef status = HAL_OK;
  int8_t iError = BMP2_INTF_RET_SUCCESS;
  uint8_t cs = *(uint8_t*)intf_ptr;

#ifdef DEBUG
  uint8_t data[BMP2_SPI_BUFFER_LEN] = {0,};
  memcpy(data, reg_data, length);
#endif

  /* Software slave selection procedure */
  HAL_GPIO_WritePin(BMP2_CS_Ports[cs], BMP2_CS_Pins[cs], GPIO_PIN_RESET);

  /* Data exchange */
  status  = HAL_SPI_Transmit(BMP2_SPI, &reg_addr, BMP2_REG_ADDR_LEN, BMP2_TIMEOUT);
  status += HAL_SPI_Transmit(BMP2_SPI,  reg_data, length,            BMP2_TIMEOUT);

  /* Disable all slaves */
  for(uint8_t i = 0; i < BMP2_NUM_OF_SENSORS; i++)
    HAL_GPIO_WritePin(BMP2_CS_Ports[i], BMP2_CS_Pins[i], GPIO_PIN_SET);

  // The BMP2xx API calls for 0 return value as a success, and -1 returned as failure
  if (status != HAL_OK)
    iError = -1;

  return iError;
}

/*!
 *  @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 *  APIs.
 *
 *  @param[in] period_us  : The required wait time in microsecond.
 *  @param[in] intf_ptr   : Interface pointer
 *
 *  @return void.
 */
void bmp2_delay_us(uint32_t period_us, void *intf_ptr)
{
  UNUSED(intf_ptr);
  HAL_Delay(period_us / 1000uL);
}

/*!
 *  @brief This internal API is used to get compensated pressure and temperature data.
 *  @param[in]  dev   : BMP2xx device structure
 *  @param[out] press : Pressure measurement [hPa]
 *  @param[out] temp  : Temperature measurement [degC]
 *
 *  @return Status of execution
 *
 *  @retval 0 -> Success.
 *  @retval <0 -> Failure.
 *
 */
int8_t BMP2_ReadData(struct bmp2_dev *dev, double* press, double* temp)
{
    int8_t rslt = BMP2_E_NULL_PTR;
    struct bmp2_status status;
    struct bmp2_data comp_data;
    int8_t try = 10;

    do {
      /* Read sensor status */
      rslt = bmp2_get_status(&status, dev);
      /* Read compensated data */
      rslt = bmp2_get_sensor_data(&comp_data, dev);
      *temp = comp_data.temperature;
      *press = comp_data.pressure / 100.0;
      try--;
    } while (status.measuring != BMP2_MEAS_DONE && try > 0);

    return rslt;
}

/*!
 *  @brief This internal API is used to get compensated temperature data.
 *  @param[in]  dev   : BMP2xx device structure
 *
 *  @return Temperature measurement [degC]
 */
double BMP2_ReadTemperature_degC(struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_E_NULL_PTR;
    struct bmp2_status status;
    struct bmp2_data comp_data;
    double temp = -1.0;
    int8_t try = 10;

    do {
      /* Read sensor status */
      rslt = bmp2_get_status(&status, dev);
      /* Read compensated data */
      rslt = bmp2_get_sensor_data(&comp_data, dev);
      temp = comp_data.temperature;
      try--;
    } while (status.measuring != BMP2_MEAS_DONE && try > 0);

    return temp;
}


/*!
 *  @brief This internal API is used to get compensated pressure data.
 *  @param[in]  dev   : BMP2xx device structure
 *
 *  @return Pressure measurement [hPa]
 */
double BMP2_ReadPressure_hPa(struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_E_NULL_PTR;
    struct bmp2_status status;
    struct bmp2_data comp_data;
    double press = -1.0;
    int8_t try = 10;

    do {
      /* Read sensor status */
      rslt = bmp2_get_status(&status, dev);
      /* Read compensated data */
      rslt = bmp2_get_sensor_data(&comp_data, dev);
      press = comp_data.pressure / 100.0;
      try--;
    } while (status.measuring != BMP2_MEAS_DONE && try > 0);

    return press;
}
