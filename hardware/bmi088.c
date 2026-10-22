/**
 * @file bmi088.c
 * @brief BMI088 加速度计+陀螺仪驱动 - 实现文件
 * @note  使用 SPI1，阻塞式读取，无 DMA，无互斥锁
 */

#include "bmi088.h"
#include "delay1.h"      // 延迟函数


/*============================================================================
 * SPI 底层函数
 *============================================================================*/

/**
 * @brief SPI 读写一个字节（阻塞式，供初始化使用）
 * @param tx_data 要发送的数据
 * @return 接收到的数据
 * @note  SPI 是全双工，发送同时接收
 */
uint8_t BMI088_SPI_ReadWriteByte(uint8_t tx_data)
{
    // 等待发送缓冲区空
    while (SPI_I2S_GetFlagStatus(BMI088_SPI, SPI_I2S_FLAG_TXE) == RESET);
    
    // 发送数据
    SPI_I2S_SendData(BMI088_SPI, tx_data);
    
    // 等待接收完成
    while (SPI_I2S_GetFlagStatus(BMI088_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    
    // 返回接收数据
    return SPI_I2S_ReceiveData(BMI088_SPI);
}

/**
 * @brief 写 BMI088 寄存器
 * @param cs_pin 片选引脚（BMI088_CS_ACC_PIN 或 BMI088_CS_GYRO_PIN）
 * @param reg    寄存器地址
 * @param data   要写入的数据
 */
void BMI088_WriteReg(uint16_t cs_pin, uint8_t reg, uint8_t data)
{
    // 1. 片选控制 ...
    if (cs_pin == BMI088_CS_ACC_PIN) {
        BMI088_CS_ACC_LOW();
    } else if (cs_pin == BMI088_CS_GYRO_PIN) {
        BMI088_CS_GYRO_LOW();
    }
    
    // 插入短暂延时，确保 CS 建立时间 (t_CSB_setup) [citation:9]
    for(volatile int i = 0; i < 5; i++);
    
    // 2. 发送写命令和地址
    BMI088_SPI_ReadWriteByte(reg & BMI088_SPI_WRITE_MASK);
    // 发送数据
    BMI088_SPI_ReadWriteByte(data);
    
    // 3. 等待 SPI 传输彻底完成
    while (SPI_I2S_GetFlagStatus(BMI088_SPI, SPI_I2S_FLAG_BSY) == SET);
    
    // 拉高对应的片选
    if (cs_pin == BMI088_CS_ACC_PIN) {
        BMI088_CS_ACC_HIGH();
    } else if (cs_pin == BMI088_CS_GYRO_PIN) {
        BMI088_CS_GYRO_HIGH();
    }
    
    // ?? 致命修复：两次写操作之间必须等待至少 2μs [citation:1][citation:9]
    // 使用硬件定时器或简单的 NOP 循环实现精确延时
    for(volatile int i = 0; i < 20; i++); // 72MHz下约 0.28μs * 20 ≈ 5.6μs，足够
}

/**
 * @brief 读 BMI088 寄存器（修正版：正确处理加速度计的 Dummy 字节）
 * @param cs_pin 片选引脚
 * @param reg    寄存器地址
 * @return 读取到的数据
 */
uint8_t BMI088_ReadReg(uint16_t cs_pin, uint8_t reg)
{
    uint8_t data;
//    uint8_t i;
    
    // 根据传入的 cs_pin 拉低对应的片选
    if (cs_pin == BMI088_CS_ACC_PIN) {
        BMI088_CS_ACC_LOW();
    } else if (cs_pin == BMI088_CS_GYRO_PIN) {
        BMI088_CS_GYRO_LOW();
    }
    
    // 发送读命令
    BMI088_SPI_ReadWriteByte(reg | BMI088_SPI_READ_MASK);
    
    // ================== 关键修改 ==================
    // 加速度计需要连续读两次，丢弃第一次，取第二次
    if (cs_pin == BMI088_CS_ACC_PIN) {
        // 第1次：Dummy 字节，丢弃
        (void)BMI088_SPI_ReadWriteByte(0xFF);
        // 第2次：真实数据
        data = BMI088_SPI_ReadWriteByte(0xFF);
    } else {
        // 陀螺仪不需要 Dummy，直接读
        data = BMI088_SPI_ReadWriteByte(0xFF);
    }
    // ===========================================
    
    // 拉高对应的片选
    if (cs_pin == BMI088_CS_ACC_PIN) {
        BMI088_CS_ACC_HIGH();
    } else if (cs_pin == BMI088_CS_GYRO_PIN) {
        BMI088_CS_GYRO_HIGH();
    }
    
    return data;
}

/**
 * @brief 连续读多个寄存器
 * @param cs_pin 片选引脚
 * @param reg    起始寄存器地址
 * @param buf    数据缓冲区
 * @param len    读取长度
 * @note  加速度计批量读取时，第一个字节是 Dummy 必须丢弃
 *        所以调用此函数时 len 应该设为 7（包含 Dummy），然后从 buf[1] 开始取数据
 */
static void BMI088_ReadMulti(uint16_t cs_pin, uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t i;
    
    // 根据传入的 cs_pin 拉低对应的片选
    if (cs_pin == BMI088_CS_ACC_PIN) {
        BMI088_CS_ACC_LOW();
    } else if (cs_pin == BMI088_CS_GYRO_PIN) {
        BMI088_CS_GYRO_LOW();
    }
    
    BMI088_SPI_ReadWriteByte(reg | BMI088_SPI_READ_MASK);
    for (i = 0; i < len; i++) {
        buf[i] = BMI088_SPI_ReadWriteByte(0xFF);
    }
    
    // 拉高对应的片选
    if (cs_pin == BMI088_CS_ACC_PIN) {
        BMI088_CS_ACC_HIGH();
    } else if (cs_pin == BMI088_CS_GYRO_PIN) {
        BMI088_CS_GYRO_HIGH();
    }
}

/*============================================================================
 * GPIO 初始化
 *============================================================================*/
static void BMI088_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 使能 GPIO 时钟
    RCC_APB2PeriphClockCmd(BMI088_SPI_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BMI088_CS_ACC_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(BMI088_CS_GYRO_CLK, ENABLE);
    
    // 配置 CS 引脚为推挽输出
    GPIO_InitStruct.GPIO_Pin = BMI088_CS_ACC_PIN | BMI088_CS_GYRO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // CS 初始为高电平（未选中）
    GPIO_SetBits(GPIOA, BMI088_CS_ACC_PIN | BMI088_CS_GYRO_PIN);
    
    // 配置 SPI 引脚
    // SCK: 复用推挽输出
    GPIO_InitStruct.GPIO_Pin = BMI088_SCK_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BMI088_SCK_GPIO, &GPIO_InitStruct);
    
    // MOSI: 复用推挽输出
    GPIO_InitStruct.GPIO_Pin = BMI088_MOSI_PIN;
    GPIO_Init(BMI088_MOSI_GPIO, &GPIO_InitStruct);
    
    // MISO: 浮空输入
    GPIO_InitStruct.GPIO_Pin = BMI088_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BMI088_MISO_GPIO, &GPIO_InitStruct);
}

/*============================================================================
 * SPI 初始化
 *============================================================================*/
static void BMI088_SPI_Init(void)
{
    SPI_InitTypeDef SPI_InitStruct;
    
    // 使能 SPI 时钟
    RCC_APB2PeriphClockCmd(BMI088_SPI_CLK, ENABLE);
    // 复位 SPI 到默认状态
    SPI_I2S_DeInit(BMI088_SPI);
    
    // SPI 配置: 模式3, 主机, 8位数据, 2.25MHz
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;      // 模式3：时钟空闲高电平
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;     // 模式3：第二个边沿采样
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  // 72MHz/32=2.25MHz
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    
    SPI_Init(BMI088_SPI, &SPI_InitStruct);
    
    // 使能 SPI
    SPI_Cmd(BMI088_SPI, ENABLE);
}

/*============================================================================
 * 加速度计初始化
 *============================================================================*/
static uint8_t BMI088_Accel_Init(void)
{
    uint8_t chip_id;
    uint8_t retry;  
    
    // 读取芯片 ID
    chip_id = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x00);
    if (chip_id != 0x1E) {
        for (retry = 0; retry < 5; retry++) {
            chip_id = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x00);
            if (chip_id == 0x1E) break;
        }
        if (chip_id != 0x1E) return 0;
    }
   
    // 1. 软复位
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x7E, 0xB6);
    Delay(50);
   
    // 2. PWR_CONF = 0x00 (进入 Active 模式)
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x7C, 0x00);
    Delay(10); // !!! 稍微加长等待，确保状态机切换完毕
    
    // !!! 修改 2：调整顺序，在真正开启加速度测量前，先写入所有配置
    
    // ========== 真正的飞控最高性能配置 ==========
    // ACC_CONF (0x40): 配置采样率和滤波器
    // Bit 7: 1 (必须写1，开启高性能连续模式)
    // Bit 6-4: 010 (Normal 正常滤波器)
    // Bit 3-0: 1011 (800Hz ODR)
    // 0x80 | 0x20 | 0x0B = 0xAB
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x40, 0xAB);  // 写入 0xAB
    Delay(10);
    
    // 4. ACC_RANGE (0x41) = 0x03 (±24g)
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x41, 0x03);
    Delay(2);
    
    // 5. 中断配置
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x53, 0x0A); // INT1 推挽高有效
    Delay(2);
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x58, 0x04); // DRDY 映射 INT1
    Delay(2);
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x48, 0x00); // 关闭 FIFO
    Delay(2);

    // !!! 修改 3：配置全部写完后，最后一步开启测量引擎
    // 6. PWR_CTRL = 0x04 (开启加速度计)
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x7D, 0x04);
    Delay(50); // 必须等待 50ms 才能读出有效数据
    
    // === 此时你可以去读取 0x02 错误寄存器，0x04 错误必然消失 ===
    
    return 1;
}

/*============================================================================
 * 陀螺仪初始化
 *============================================================================*/
static uint8_t BMI088_Gyro_Init(void)
{
    uint8_t chip_id;
    uint8_t retry;
    
    Delay(50);
    
    // 读取芯片 ID
    chip_id = BMI088_ReadReg(BMI088_CS_GYRO_PIN, BMI088_GYRO_CHIP_ID);
    if (chip_id != 0x0F) {
        for (retry = 0; retry < 5; retry++) {
            Delay(10);
            chip_id = BMI088_ReadReg(BMI088_CS_GYRO_PIN, BMI088_GYRO_CHIP_ID);
            if (chip_id == 0x0F) break;
        }
        if (chip_id != 0x0F) return 0;
    }
    
    // 软复位
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, BMI088_GYRO_SOFT_RESET, 0xB6);
    Delay(30);
    
    // 量程: ±2000dps (飞控标配，防止翻滚时溢出)
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, BMI088_GYRO_RANGE, 0x00);
    Delay(20);
    
    // ========== 飞控核心配置 (已修正) ==========
    // GYRO_BANDWIDTH (0x10): 配置采样率和带宽
    // 写入 0x01: 2000Hz ODR, 230Hz 滤波带宽
    // (解释：不推荐使用 0x00(532Hz带宽)，因为对于飞控来说 532Hz 带宽会让电机噪声原封不动进入PID，导致电机发热。230Hz 是绝佳的平衡点)
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, BMI088_GYRO_BANDWIDTH, 0x01);
    Delay(20);
    
    // 退出低功耗模式，进入正常工作模式
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, BMI088_GYRO_LPM1, 0x00);
    Delay(10);
    
    // ========== 中断配置 (DRDY 映射到 INT3) ==========
    // 1. GYRO_INT_CTRL (0x15) = 0x80 (启用 Data Ready 新数据中断)
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, 0x15, 0x80);
    Delay(1);
    
    // 2. INT3_INT4_IO_CONF (0x16) = 0x01 (配置 INT3 引脚为推挽输出 + 高电平有效)
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, 0x16, 0x01);
    Delay(1);
    
    // 3. INT3_INT4_IO_MAP (0x18) = 0x01 (将 DRDY 中断信号映射到 INT3 引脚)
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, 0x18, 0x01);
    Delay(1);
    
    return 1;
}

/*============================================================================
 * 软复位函数
 *============================================================================*/
static void BMI088_SoftReset(void)
{
	  Delay(50);
    // 加速度计软复位
    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x7E, 0xB6);
    Delay(50);
    
    // 陀螺仪软复位
    BMI088_WriteReg(BMI088_CS_GYRO_PIN, BMI088_GYRO_SOFT_RESET, 0xB6);
    Delay(50);
}

/*============================================================================
 * 数据读取函数
 *============================================================================*/

/**
 * @brief 读取所有传感器数据（加速度+陀螺仪）
 * @param data 数据存储结构体指针
 * @note  阻塞式读取
 */
/**
 * @brief 只读取加速度计数据
 * @param data 数据存储结构体指针（只填充 accel_x/y/z）
 */
void BMI088_ReadAccelOnly(BMI088_Data_t* data)
{
    uint8_t buf[7];  // 加速度计需要7个字节（1个Dummy + 6个数据）
    
    // 读取加速度计 - 需要7个字节
    BMI088_ReadMulti(BMI088_CS_ACC_PIN, BMI088_ACC_X_LSB, buf, 7);
    // buf[0] = Dummy（丢弃）
    // buf[1] = X_L, buf[2] = X_H
    // buf[3] = Y_L, buf[4] = Y_H
    // buf[5] = Z_L, buf[6] = Z_H
    data->accel_x = (int16_t)((buf[2] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[4] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[6] << 8) | buf[5]);
}

/**
 * @brief 只读取陀螺仪数据
 * @param data 数据存储结构体指针（只填充 gyro_x/y/z）
 */
void BMI088_ReadGyroOnly(BMI088_Data_t* data)
{
    uint8_t buf[6];  // 陀螺仪只需要6个字节
    
    // 读取陀螺仪 - 陀螺仪不需要 Dummy，直接读6字节
    BMI088_ReadMulti(BMI088_CS_GYRO_PIN, BMI088_GYRO_RATE_X_LSB, buf, 6);
    data->gyro_x = (int16_t)((buf[1] << 8) | buf[0]);
    data->gyro_y = (int16_t)((buf[3] << 8) | buf[2]);
    data->gyro_z = (int16_t)((buf[5] << 8) | buf[4]);
}

/**
 * @brief 读取所有传感器数据（加速度+陀螺仪）
 * @param data 数据存储结构体指针
 * @note  阻塞式读取，两个传感器都读
 */
void BMI088_ReadAll(BMI088_Data_t* data)
{
    BMI088_ReadAccelOnly(data);
    BMI088_ReadGyroOnly(data);
}
/*============================================================================
 * 对外接口函数
 *============================================================================*/

/**
 * @brief 获取量程转换系数
 * @return 加速度转换系数 (g/LSB)
 */
float BMI088_GetAccelScale(void)
{
    return BMI088_ACCEL_SCALE;
}

/**
 * @brief 获取量程转换系数
 * @return 陀螺仪转换系数 (dps/LSB)
 */





float BMI088_GetGyroScale(void)
{
    return BMI088_GYRO_SCALE;
}

/**
 * @brief BMI088 完整初始化
 * @return 1=成功, 0=失败
 */
uint8_t BMI088_Init(void)
{
    // 禁用 JTAG 释放 PA15 作为普通 GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    // 硬件初始化
    BMI088_GPIO_Init();
    BMI088_SPI_Init();
    
    // 软复位
    BMI088_SoftReset();
		
    // 初始化加速度计
    if (BMI088_Accel_Init() == 0) {
        return 0;
    }
    
    // 初始化陀螺仪
    if (BMI088_Gyro_Init() == 0) {
        return 0;
    }
    
    return 1;
}

