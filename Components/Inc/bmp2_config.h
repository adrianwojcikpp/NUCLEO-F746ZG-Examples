/**
  ******************************************************************************
  * @file    bmp2_config.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 2.0
  * @date    20-Nov-2021
  * @brief   Configuration file for BMP2xx sensor driver library;
  *          SPI routines implementation.
  * @ref     https://github.com/BoschSensortec/BMP2-Sensor-API
  *
  ******************************************************************************
  */
#ifndef INC_BMP2_CONFIG_H_
#define INC_BMP2_CONFIG_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "bmp2.h"

/* Typedef -------------------------------------------------------------------*/
#define BMP2_CS_IndexType uint8_t
#define BMP2_CS_PortType  GPIO_TypeDef*
#define BMP2_CS_PinType   uint16_t

/* Define --------------------------------------------------------------------*/
#define BMP2_SPI (&hspi4)

#define BMP2_SPI_BUFFER_LEN  28  //! @see BMP280 technical note p. 24
#define BMP2_DATA_INDEX       1  //! @see BMP280 technical note p. 31-32
#define BMP2_REG_ADDR_INDEX   0  //! @see BMP280 technical note p. 31-32
#define BMP2_REG_ADDR_LEN     1  //! @see BMP280 technical note p. 31-32

#define BMP2_TIMEOUT          5
#define BMP2_NUM_OF_SENSORS   2

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern struct bmp2_dev hbmp2_1;
extern struct bmp2_dev hbmp2_2;

/* Public function prototypes ------------------------------------------------*/

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
int8_t BMP2_Init(struct bmp2_dev* dev);

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
BMP2_INTF_RET_TYPE bmp2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

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
BMP2_INTF_RET_TYPE bmp2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 *  APIs.
 *
 *  @param[in] period_us  : The required wait time in microsecond.
 *  @param[in] intf_ptr   : Interface pointer
 *
 *  @return void.
 */
void bmp2_delay_us(uint32_t period_us, void *intf_ptr);

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
int8_t BMP2_ReadData(struct bmp2_dev *dev, double* press, double* temp);

/*!
 *  @brief This internal API is used to get compensated temperature data.
 *  @param[in]  dev   : BMP2xx device structure
 *
 *  @return Temperature measurement [degC]
 */
double BMP2_ReadTemperature_degC(struct bmp2_dev *dev);

/*!
 *  @brief This internal API is used to get compensated pressure data.
 *  @param[in]  dev   : BMP2xx device structure
 *
 *  @return Pressure measurement [hPa]
 */
double BMP2_ReadPressure_hPa(struct bmp2_dev *dev);

#endif /* INC_BMP2_CONFIG_H_ */


