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

/* Macro ---------------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern Menu_TypeDef hmenu;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief TODO
 * @param[in/out] hmenuitem :
 */
void MENU_Init(Menu_TypeDef* hmenu);

/**
 * @brief TODO
 * @param[in/out] hmenuitem :
 */
void MENU_ROUTINE(Menu_TypeDef* hmenu);

#endif /* INC_MENU_CONFIG_H_ */
