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

void MENU_Init(Menu_TypeDef* hmenu)
{
  LCD_SetCursor(hmenu->Display, 0, 0);
  LCD_printf(hmenu->Display, "%c", LCD_MENU_CURSOR_CHAR);
  LCD_SetCursor(hmenu->Display, 1, 0);
  LCD_printStr(hmenu->Display, " ");
  HAL_TIM_Base_Start_IT(hmenu->Timer);
}

void MENU_ROUTINE(Menu_TypeDef* hmenu)
{
  hmenu->Item->Routine(hmenu->Item);
  hmenu->Item->Next->Routine(hmenu->Item->Next);

  // #1 line - active item
  LCD_SetCursor(hmenu->Display, 0, 1);
  LCD_printStr(hmenu->Display, hmenu->Item->DisplayStr);

  // #1 line - next item
  LCD_SetCursor(hmenu->Display, 1, 1);
  LCD_printStr(hmenu->Display, hmenu->Item->Next->DisplayStr);

  // Serial port streaming
  HAL_UART_Transmit(hmenu->SerialPort, (uint8_t*)hmenu->Item->SerialPortStr, hmenu->Item->SerialPortStrLen, 10);
}
