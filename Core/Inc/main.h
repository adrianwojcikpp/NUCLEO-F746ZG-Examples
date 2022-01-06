/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
 * @brief Common CMSIS unit tests routine.
 */
void CMSIS_UnitTests(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMP280_CS2_Pin GPIO_PIN_3
#define BMP280_CS2_GPIO_Port GPIOE
#define BMP280_CS1_Pin GPIO_PIN_4
#define BMP280_CS1_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define DISP_4_Pin GPIO_PIN_3
#define DISP_4_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define DISP_2_Pin GPIO_PIN_0
#define DISP_2_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define DISP_3_Pin GPIO_PIN_3
#define DISP_3_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define DISP_1_Pin GPIO_PIN_3
#define DISP_1_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_4
#define LCD_D5_GPIO_Port GPIOA
#define ENC_DT2_Pin GPIO_PIN_6
#define ENC_DT2_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_10
#define BTN3_GPIO_Port GPIOE
#define BTN3_EXTI_IRQn EXTI15_10_IRQn
#define BTN1_Pin GPIO_PIN_12
#define BTN1_GPIO_Port GPIOE
#define BTN1_EXTI_IRQn EXTI15_10_IRQn
#define BTN2_Pin GPIO_PIN_14
#define BTN2_GPIO_Port GPIOE
#define BTN2_EXTI_IRQn EXTI15_10_IRQn
#define LD2EX_Pin GPIO_PIN_15
#define LD2EX_GPIO_Port GPIOE
#define LD3EX_Pin GPIO_PIN_10
#define LD3EX_GPIO_Port GPIOB
#define LD1EX_Pin GPIO_PIN_11
#define LD1EX_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_15
#define LCD_RS_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define ENC_DT_Pin GPIO_PIN_11
#define ENC_DT_GPIO_Port GPIOD
#define LD_RGB_R_Pin GPIO_PIN_12
#define LD_RGB_R_GPIO_Port GPIOD
#define LD_RGB_G_Pin GPIO_PIN_13
#define LD_RGB_G_GPIO_Port GPIOD
#define LD_RGB_B_Pin GPIO_PIN_14
#define LD_RGB_B_GPIO_Port GPIOD
#define LD4EX_Pin GPIO_PIN_15
#define LD4EX_GPIO_Port GPIOD
#define DISP_E_Pin GPIO_PIN_2
#define DISP_E_GPIO_Port GPIOG
#define LAMP_TRIAC_Pin GPIO_PIN_3
#define LAMP_TRIAC_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define LCD_E_Pin GPIO_PIN_6
#define LCD_E_GPIO_Port GPIOC
#define ENC_CLK2_Pin GPIO_PIN_7
#define ENC_CLK2_GPIO_Port GPIOC
#define LCD_D7_Pin GPIO_PIN_8
#define LCD_D7_GPIO_Port GPIOC
#define LAMP_SYNC_Pin GPIO_PIN_9
#define LAMP_SYNC_GPIO_Port GPIOC
#define LAMP_SYNC_EXTI_IRQn EXTI9_5_IRQn
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DISP_A_Pin GPIO_PIN_10
#define DISP_A_GPIO_Port GPIOC
#define DISP_B_Pin GPIO_PIN_11
#define DISP_B_GPIO_Port GPIOC
#define DISP_C_Pin GPIO_PIN_12
#define DISP_C_GPIO_Port GPIOC
#define DISP_D_Pin GPIO_PIN_2
#define DISP_D_GPIO_Port GPIOD
#define DISP_H_Pin GPIO_PIN_5
#define DISP_H_GPIO_Port GPIOD
#define DISP_G_Pin GPIO_PIN_6
#define DISP_G_GPIO_Port GPIOD
#define DISP_F_Pin GPIO_PIN_7
#define DISP_F_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_5
#define LCD_D6_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define ENC_CLK_Pin GPIO_PIN_0
#define ENC_CLK_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
