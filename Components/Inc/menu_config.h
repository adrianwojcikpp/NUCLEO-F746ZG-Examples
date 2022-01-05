/**
  ******************************************************************************
  * @file    menu_config.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version V2.0
  * @date    12-Jan-2021
  * @brief   Simple LCD menu example configuration file: menu structure,
  *          content and behavior
  *
  ******************************************************************************
  */
#ifndef INC_MENU_CONFIG_H_
#define INC_MENU_CONFIG_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "menu.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/
#define MENU_TIM (&htim10);

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern Menu_TypeDef hmenu;

/* LED Red #1: on-board red LED (LD3) */
extern MenuItem_TypeDef menu_ledr1;

/* LED Red #2: breadboard red LED (LD3EX) */
extern MenuItem_TypeDef menu_ledr2;

/* LED Green #1: on-board green LED (LD1) */
extern MenuItem_TypeDef menu_ledg1;

/* LED Green #2: breadboard green LED (LD1EX) */
extern MenuItem_TypeDef menu_ledg2;

/* LED Blue #1: on-board blue LED (LD2) */
extern MenuItem_TypeDef menu_ledb1;

/* LED Blue #2: breadboard blue LED (LD2EX) */
extern MenuItem_TypeDef menu_ledb2;

/* LED RGB: channel R */
extern MenuItem_TypeDef menu_ledrgb_r;

/* LED RGB: channel G */
extern MenuItem_TypeDef menu_ledrgb_g;

/* LED RGB: channel B */
extern MenuItem_TypeDef menu_ledrgb_b;

/* Rotary encoder #1 */
extern MenuItem_TypeDef menu_enc1;

/* Analog input #1 */
extern MenuItem_TypeDef menu_ain1;

/* Analog input #2 */
extern MenuItem_TypeDef menu_ain2;

/* Temperature & pressure sensor: BMP280 */
extern MenuItem_TypeDef menu_bmp2;

/* Light sensor: BH1750 */
extern MenuItem_TypeDef menu_bh1750;

/* Analog output #2 */
extern MenuItem_TypeDef menu_aout2;

/* Public function prototypes ------------------------------------------------*/

#endif /* INC_MENU_CONFIG_H_ */
