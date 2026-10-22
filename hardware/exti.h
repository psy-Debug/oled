#ifndef __EXTI_H
#define __EXTI_H

#include "stm32f10x.h"
#include "bmi088.h"

// 外部声明
extern volatile uint8_t g_accel_data_ready;
extern volatile uint8_t g_gyro_data_ready;
extern volatile float g_yaw;           // 偏航角（弧度）

void EXTI_Init_Config(void);
uint8_t BMI088_GetLatestAccelData(BMI088_Data_t* data);
uint8_t BMI088_GetLatestGyroData(BMI088_Data_t* data);
uint16_t  gett2(void);
uint16_t  gett1(void);
#endif


