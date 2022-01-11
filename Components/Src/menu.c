/**
  ******************************************************************************
  * @file    menu.c
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version V1.0
  * @date    12-Jan-2021
  * @brief   Simple LCD menu example.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "menu.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
  const char __menu__padding[] = "                    "; // space x20
/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function ----------------------------------------------------------*/

/* Public function -----------------------------------------------------------*/

/**
 * @brief TODO
 * @param[out] hmenuitem :
 */
void MENU_ITEM_ClearDisplayBuffer(MenuItem_TypeDef* hmenuitem)
{
  sprintf(hmenuitem->DisplayStr, "%.*s", LCD_LINE_LEN, __menu__padding);
}

/**
 * @brief TODO
 * @param[in/out] hmenuitem :
 */
void MENU_ITEM_SetDisplayBuffer(MenuItem_TypeDef* hmenuitem, const char* str)
{
  int pad_len = LCD_LINE_LEN - hmenuitem->DisplayStrLen;
  sprintf(hmenuitem->DisplayStr, "%s%.*s", str, pad_len, __menu__padding);
}
