/**
 * @file exti.c
 * @brief 外部中断配置 - 实现文件
 */
#include "bmi088.h"
#include "exti.h"
#include "OLED.h"
#include "delay1.h"
// BMI088 加速度计中断引脚 (INT1 -> PB0)
#define BMI088_ACC_INT_PORT      GPIOB
#define BMI088_ACC_INT_PIN       GPIO_Pin_0

// BMI088 陀螺仪中断引脚 (INT3 -> PB1)
#define BMI088_GYRO_INT_PORT     GPIOB
#define BMI088_GYRO_INT_PIN      GPIO_Pin_1
// 必须加 volatile ！！！否则进入主循环后编译器会直接跳过读取，导致断流
extern volatile uint8_t g_accel_data_ready;
extern volatile uint8_t g_gyro_data_ready;
// 独立的数据就绪标志
volatile uint8_t g_accel_data_ready = 0;
volatile uint8_t g_gyro_data_ready = 0;
volatile float g_yaw = 0.0f;           // 偏航角（弧度），主循环可直接读取
static uint32_t last_irq_time = 0;     // 上次中断时间（微秒）
// 独立的数据缓存
static BMI088_Data_t g_accel_data;
static BMI088_Data_t g_gyro_data;
static uint16_t t1=0;
static uint16_t t2=0;
/**
 * @brief 外部中断初始化
 * @note  配置 PB0 和 PB1 为上升沿触发
 */
void EXTI_Init_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 配置 PB0 (加速度计 INT1)
    GPIO_InitStruct.GPIO_Pin = BMI088_ACC_INT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;  // 下拉输入
    GPIO_Init(BMI088_ACC_INT_PORT, &GPIO_InitStruct);
    
    // 配置 PB1 (陀螺仪 INT3)
    GPIO_InitStruct.GPIO_Pin = BMI088_GYRO_INT_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;  // 下拉输入
    GPIO_Init(BMI088_GYRO_INT_PORT, &GPIO_InitStruct);
    
    // EXTI Line0 (PB0)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);
    
    // EXTI Line1 (PB1)
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);
    
    // NVIC 配置
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
 * @brief EXTI0 中断服务函数 (加速度计)
 */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        // 只读取加速度计数据
        BMI088_ReadAccelOnly(&g_accel_data);
        t1++;
        // 通知主循环加速度计有新数据
        g_accel_data_ready = 1;
        
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/**
 * @brief EXTI1 中断服务函数 (陀螺仪)
 */
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        // 只读取陀螺仪数据
			uint32_t now = GetUs();
        float dt = (now - last_irq_time) * 0.000001f;  // 微秒 → 秒
        last_irq_time = now;
        BMI088_ReadGyroOnly(&g_gyro_data);
        t2++;
        // 通知主循环陀螺仪有新数据
        g_gyro_data_ready = 1;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/**
 * @brief 获取最新加速度计数据
 * @param data 数据存储结构体指针
 * @return 1=有新数据, 0=无新数据
 */
uint8_t BMI088_GetLatestAccelData(BMI088_Data_t* data)
{
    if (g_accel_data_ready) {
        g_accel_data_ready = 0;
        data->accel_x = g_accel_data.accel_x;
        data->accel_y = g_accel_data.accel_y;
        data->accel_z = g_accel_data.accel_z;
        return 1;
    }
    return 0;
}

/**
 * @brief 获取最新陀螺仪数据
 * @param data 数据存储结构体指针
 * @return 1=有新数据, 0=无新数据
 */
uint8_t BMI088_GetLatestGyroData(BMI088_Data_t* data)
{
    if (g_gyro_data_ready) {
        g_gyro_data_ready = 0;
        data->gyro_x = g_gyro_data.gyro_x;
        data->gyro_y = g_gyro_data.gyro_y;
        data->gyro_z = g_gyro_data.gyro_z;
        return 1;
    }
    return 0;
}
uint16_t  gett1(void)
{return t1;}
uint16_t  gett2(void)
{return t2;}
