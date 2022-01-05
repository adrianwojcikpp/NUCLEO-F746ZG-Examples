/**
  ******************************************************************************
  * @file    menu.h
  * @author  AW
  * @version V1.0
  * @date    12-Jan-2021
  * @brief   Simple LCD menu example.
  *
  ******************************************************************************
  */
#ifndef INC_MENU_H_
#define INC_MENU_H_

/* Config --------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "lcd.h"

/* Typedef -------------------------------------------------------------------*/
typedef struct _MenuItem MenuItem_TypeDef;

struct _MenuItem {
  char DisplayStr[LCD_LINE_BUF_LEN];
  uint16_t DisplayStrLen;
  MenuItem_TypeDef* Next;
  MenuItem_TypeDef* Prev;
  MenuItem_TypeDef* Child;
  MenuItem_TypeDef* Parent;
  void (*Routine)(MenuItem_TypeDef* hmenuitem);
  uint16_t RefreshRate;
};


#define MENU_TimerType TIM_HandleTypeDef*
#define MENU_DisplayType LCD_HandleTypeDef*

typedef struct {
  MenuItem_TypeDef* Item;
  MENU_DisplayType Display;
  MENU_TimerType Timer;
} Menu_TypeDef;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/
#define MENU_ITEM_CONTRUCTOR(__NAME__, __NEXT__, __PREV__, __TS__) __NAME__ = {\
  .Next = __NEXT__, .Prev = __PREV__, .Child = NULL, .Parent = NULL,  \
  .Routine = __##__NAME__##_routine, .DisplayStrLen = LCD_LINE_LEN,   \
  .RefreshRate = (10*__TS__) \
}

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
void MENU_ITEM_ClearDisplayBuffer(MenuItem_TypeDef* hmenuitem);
void MENU_ITEM_SetDisplayBuffer(MenuItem_TypeDef* hmenuitem, const char* str);
void MENU_Init(Menu_TypeDef* hmenu);
void MENU_ROUTINE(Menu_TypeDef* hmenu);

#endif /* INC_MENU_H_ */
