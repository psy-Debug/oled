#include "motor_pwm.h"
#include "motor_mixer.h"

// 初始化TIM2为PWM输出模式
void Motor_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 2. 配置GPIO为复用推挽输出
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 3. 配置TIM2时基
    TIM_TimeBaseStruct.TIM_Period = PWM_PERIOD;                 // 自动重装载值
    TIM_TimeBaseStruct.TIM_Prescaler = PWM_PRESCALER;           // 预分频器
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;        // 时钟分割
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    // 向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
    
    // 4. 配置PWM输出通道
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;              // PWM模式1
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_Pulse = 0;                             // 初始占空比0
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;      // 高电平有效
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    
    // 初始化4个通道
    TIM_OC1Init(TIM2, &TIM_OCInitStruct);
    TIM_OC2Init(TIM2, &TIM_OCInitStruct);
    TIM_OC3Init(TIM2, &TIM_OCInitStruct);
    TIM_OC4Init(TIM2, &TIM_OCInitStruct);
    
    // 5. 使能预装载寄存器
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    // 6. 使能自动重装载预装载
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    
    // 7. 使能TIM2计数器
    TIM_Cmd(TIM2, ENABLE);
}

// 设置单个电机PWM占空比
void Motor_SetDuty(uint8_t motor_id, float duty)
{
    uint16_t compare;
    
    // 限制占空比范围
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    
    compare = (uint16_t)(duty * PWM_PERIOD);
    
    switch(motor_id) {
        case MOTOR_FRONT:   // M1
            TIM_SetCompare1(TIM2, compare);
            break;
        case MOTOR_RIGHT:   // M2
            TIM_SetCompare2(TIM2, compare);
            break;
        case MOTOR_BACK:    // M3
            TIM_SetCompare3(TIM2, compare);
            break;
        case MOTOR_LEFT:    // M4
            TIM_SetCompare4(TIM2, compare);
            break;
        default:
            break;
    }
}

// 设置所有电机PWM
void Motor_SetAllDuty(float duty1, float duty2, float duty3, float duty4)
{
    Motor_SetDuty(MOTOR_FRONT, duty1);
    Motor_SetDuty(MOTOR_RIGHT, duty2);
    Motor_SetDuty(MOTOR_BACK, duty3);
    Motor_SetDuty(MOTOR_LEFT, duty4);
}

// 停止所有电机
void Motor_StopAll(void)
{
    Motor_SetAllDuty(0, 0, 0, 0);
}

// 安全停止
void Motor_EmergencyStop(void)
{
    // 立即输出0
    TIM_SetCompare1(TIM2, 0);
    TIM_SetCompare2(TIM2, 0);
    TIM_SetCompare3(TIM2, 0);
    TIM_SetCompare4(TIM2, 0);
    
    // 可选：禁用输出
    // TIM_CtrlPWMOutputs(TIM2, DISABLE);
}
