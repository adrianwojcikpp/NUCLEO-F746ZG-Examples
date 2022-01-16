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
#include "usart.h"

/* Typedef -------------------------------------------------------------------*/
typedef struct _MenuItem MenuItem_TypeDef;

struct _MenuItem {
  char DisplayStr[LCD_LINE_BUF_LEN];
  uint16_t DisplayStrLen;
  char SerialPortStr[SERIAL_PORT_MSG_BUF_LEN];
  uint16_t SerialPortStrLen;
  MenuItem_TypeDef* Next;
  MenuItem_TypeDef* Prev;
  MenuItem_TypeDef* Child;
  MenuItem_TypeDef* Parent;
  void (*Routine)(MenuItem_TypeDef* hmenuitem);
  uint16_t RefreshRate;
};

#define MENU_TimerType TIM_HandleTypeDef*
#define MENU_DisplayType LCD_HandleTypeDef*
#define MENU_SerialType UART_HandleTypeDef*

typedef struct {
  /* Menu structure */
  MenuItem_TypeDef* Item;
  /* Output devices */
  MENU_DisplayType Display;
  MENU_TimerType Timer;
  MENU_SerialType SerialPort;
} Menu_TypeDef;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/
#define MENU_ITEM_CONTRUCTOR(__NAME__, __TS__, __FUNC__) \
void __##__NAME__##_routine(MenuItem_TypeDef* hmenuitem) __FUNC__ \
MenuItem_TypeDef __NAME__ = {\
  .Next = &__NAME__, .Prev = &__NAME__, .Child = NULL, .Parent = NULL,  \
  .Routine = __##__NAME__##_routine, .DisplayStrLen = LCD_LINE_LEN,   \
  .RefreshRate = (10*__TS__) \
}

#define MENU_CALL_ROUTINE(__NAME__)  (__NAME__)->Routine((__NAME__))
/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

void MENU_ITEM_ClearDisplayBuffer(MenuItem_TypeDef* hmenuitem);

void MENU_ITEM_SetDisplayBuffer(MenuItem_TypeDef* hmenuitem, const char* str);

#endif /* INC_MENU_H_ */
