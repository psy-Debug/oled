/**
 * @file bmi088.h
 * @brief BMI088 加速度计+陀螺仪驱动 - 头文件
 * @note  使用 SPI 通信，阻塞式读取，ODR=1000Hz，单中断模式
 */

#ifndef __BMI088_H
#define __BMI088_H

#include "stm32f10x.h"
#include "bmi088.h"
/*============================================================================
 * 量程转换系数
 *============================================================================*/
#define BMI088_ACCEL_SCALE       0.000732422f   // ±24g量程: 24/32768
#define BMI088_GYRO_SCALE        0.061035f      // ±2000dps量程: 2000/32768


///*============================================================================
// * SPI 硬件配置
// *============================================================================*/
#define BMI088_SPI                    SPI1
#define BMI088_SPI_CLK                RCC_APB2Periph_SPI1

/* 片选引脚 - 加速度计 */
#define BMI088_CS_ACC_GPIO            GPIOA
#define BMI088_CS_ACC_PIN             GPIO_Pin_4
#define BMI088_CS_ACC_CLK             RCC_APB2Periph_GPIOA

/* 片选引脚 - 陀螺仪 */
#define BMI088_CS_GYRO_GPIO           GPIOA
#define BMI088_CS_GYRO_PIN            GPIO_Pin_15
#define BMI088_CS_GYRO_CLK            RCC_APB2Periph_GPIOA

/* SPI 引脚 */
#define BMI088_SCK_GPIO               GPIOA
#define BMI088_SCK_PIN                GPIO_Pin_5
#define BMI088_MISO_GPIO              GPIOA
#define BMI088_MISO_PIN               GPIO_Pin_6
#define BMI088_MOSI_GPIO              GPIOA
#define BMI088_MOSI_PIN               GPIO_Pin_7
#define BMI088_SPI_GPIO_CLK           RCC_APB2Periph_GPIOA

/* CS 操作宏 */
#define BMI088_CS_ACC_LOW()           GPIO_ResetBits(BMI088_CS_ACC_GPIO, BMI088_CS_ACC_PIN)
#define BMI088_CS_ACC_HIGH()          GPIO_SetBits(BMI088_CS_ACC_GPIO, BMI088_CS_ACC_PIN)
#define BMI088_CS_GYRO_LOW()          GPIO_ResetBits(BMI088_CS_GYRO_GPIO, BMI088_CS_GYRO_PIN)
#define BMI088_CS_GYRO_HIGH()         GPIO_SetBits(BMI088_CS_GYRO_GPIO, BMI088_CS_GYRO_PIN)

/*============================================================================
 * BMI088 寄存器地址
 *============================================================================*/
/* 加速度计寄存器 */
#define BMI088_ACC_CHIP_ID            0x00     // 芯片ID寄存器，应为0x1E
#define BMI088_ACC_X_LSB              0x12     // X轴数据低字节
#define BMI088_ACC_X_MSB              0x13     // X轴数据高字节
#define BMI088_ACC_Y_LSB              0x14     // Y轴数据低字节
#define BMI088_ACC_Y_MSB              0x15     // Y轴数据高字节
#define BMI088_ACC_Z_LSB              0x16     // Z轴数据低字节
#define BMI088_ACC_Z_MSB              0x17     // Z轴数据高字节
#define BMI088_ACC_PWR_CTRL           0x7D     // 电源控制寄存器
#define BMI088_ACC_PWR_CONF           0x7C     // 电源配置寄存器
#define BMI088_ACC_CONF               0x40     // 配置寄存器(ODR+带宽)
#define BMI088_ACC_RANGE              0x41     // 量程配置寄存器
#define BMI088_ACC_INT1_IO_CTRL       0x53     // INT1引脚配置
#define BMI088_ACC_INT_MAP_DATA       0x58     // 中断映射寄存器

/* 陀螺仪寄存器 */
#define BMI088_GYRO_CHIP_ID           0x00     // 芯片ID寄存器，应为0x0F
#define BMI088_GYRO_RATE_X_LSB        0x02     // X轴数据低字节
#define BMI088_GYRO_RATE_X_MSB        0x03     // X轴数据高字节
#define BMI088_GYRO_RATE_Y_LSB        0x04     // Y轴数据低字节
#define BMI088_GYRO_RATE_Y_MSB        0x05     // Y轴数据高字节
#define BMI088_GYRO_RATE_Z_LSB        0x06     // Z轴数据低字节
#define BMI088_GYRO_RATE_Z_MSB        0x07     // Z轴数据高字节
#define BMI088_GYRO_RANGE             0x0F     // 量程配置寄存器
#define BMI088_GYRO_BANDWIDTH         0x10     // 带宽/ODR配置寄存器
#define BMI088_GYRO_LPM1              0x11     // 低功耗模式1
#define BMI088_GYRO_SOFT_RESET        0x14     // 软复位寄存器
#define BMI088_GYRO_INT_CTRL          0x19     // 中断控制寄存器

/* SPI 通信标志 */
#define BMI088_SPI_READ_MASK          0x80     // 读操作标志位
#define BMI088_SPI_WRITE_MASK         0x7F     // 写操作标志位

/* 电源控制值 */
#define BMI088_ACC_PWR_CTRL_ENABLE    0x04     // 启用加速度计
#define BMI088_ACC_PWR_CONF_ACTIVE    0x00     // 激活模式

/*============================================================================
 * 数据结构
 *============================================================================*/
typedef struct {
    int16_t accel_x;     // 加速度X轴原始值
    int16_t accel_y;     // 加速度Y轴原始值
    int16_t accel_z;     // 加速度Z轴原始值
    int16_t gyro_x;      // 陀螺仪X轴原始值
    int16_t gyro_y;      // 陀螺仪Y轴原始值
    int16_t gyro_z;      // 陀螺仪Z轴原始值
} BMI088_Data_t;

/*============================================================================
 * 外部接口函数
 *============================================================================*/

/**
 * @brief BMI088 初始化
 * @return 1=成功, 0=失败
 */
uint8_t BMI088_Init(void);

/**
 * @brief 读取所有传感器数据（加速度+陀螺仪）
 * @param data 数据存储结构体指针
 */
void BMI088_ReadAll(BMI088_Data_t* data);

/**
 * @brief 获取量程转换系数
 * @return 加速度转换系数
 */
 
float BMI088_GetAccelScale(void);

/**
 * @brief 获取量程转换系数
 * @return 陀螺仪转换系数
 */
float BMI088_GetGyroScale(void);

/*============================================================================
 * SPI 底层函数（供中断服务函数使用）
 *============================================================================*/

/**
 * @brief SPI 读写一个字节（阻塞式）
 * @param tx_data 要发送的数据
 * @return 接收到的数据
 */
uint8_t BMI088_SPI_ReadWriteByte(uint8_t tx_data);


/**
 * @brief 只读取加速度计数据
 * @param data 数据存储结构体指针（只填充 accel_x/y/z）
 */
void BMI088_ReadAccelOnly(BMI088_Data_t* data);

/**
 * @brief 只读取陀螺仪数据
 * @param data 数据存储结构体指针（只填充 gyro_x/y/z）
 */
void BMI088_ReadGyroOnly(BMI088_Data_t* data);

#endif /* __BMI088_H */
uint8_t BMI088_ReadReg(uint16_t cs_pin, uint8_t reg);
void BMI088_WriteReg(uint16_t cs_pin, uint8_t reg, uint8_t data);

