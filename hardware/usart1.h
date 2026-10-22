#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"

// 外部变量声明（供主循环使用）
extern volatile uint8_t usart1_cmd_ready;
extern char usart1_cmd_char;
extern float usart1_cmd_value;

// 函数声明
void USART1_Config(void);
void USART1_CheckAndParse(void);

#endif

