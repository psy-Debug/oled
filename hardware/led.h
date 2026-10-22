#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

// LED 初始化
void LED_Init(void);

// LED 控制宏
#define LED_ON()      GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define LED_OFF()     GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define LED_TOGGLE()  (GPIOA->ODR ^= GPIO_Pin_8) // 翻转电平

#endif

