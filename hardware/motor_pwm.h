#ifndef __MOTOR_PWM_H
#define __MOTOR_PWM_H

#include "stm32f10x.h"

// 电机通道映射
#define MOTOR1_PWM_CHANNEL   TIM_Channel_1  // PA0
#define MOTOR2_PWM_CHANNEL   TIM_Channel_2  // PA1
#define MOTOR3_PWM_CHANNEL   TIM_Channel_3  // PA2
#define MOTOR4_PWM_CHANNEL   TIM_Channel_4  // PA3

// PWM参数
#define PWM_PERIOD           3599    // 周期 = 1000, PWM频率 = 72MHz/(7200)/(1000) = 10kHz
#define PWM_PRESCALER        0    // 预分频: 72MHz/7200 = 10kHz
#define PWM_DEAD_TIME        0       // 死区时间

// 电机输出占空比 [0,1] -> 比较值
//#define DUTY_TO_COMPARE(duty) ((uint16_t)((duty) * PWM_PERIOD))

// 初始化TIM2为PWM输出模式
void Motor_PWM_Init(void);

// 设置单个电机PWM占空比
void Motor_SetDuty(uint8_t motor_id, float duty);

// 设置所有电机PWM（批量）
void Motor_SetAllDuty(float duty1, float duty2, float duty3, float duty4);

// 停止所有电机
void Motor_StopAll(void);

// 安全停止（立即停止）
void Motor_EmergencyStop(void);

#endif
