#include "stm32f10x.h"
#include "OLED.h"
#include "delay1.h"
#include "bmi088.h"
#include "exti.h"
#include "attitude_quat.h"
#include "sensor_initial_alignment.h"
#include "motor_pwm.h"
#include "usart3.h"
#include "usart.h"
#include "motor_pwm.h"
#include "motor_mixer.h"
#include "flight_controller.h"
#include "usart1.h"
#include "led.h"
// 校准采样缓冲区
#define CALIB_SAMPLES 20
static float calib_accel_x[CALIB_SAMPLES];
static float calib_accel_y[CALIB_SAMPLES];
static float calib_accel_z[CALIB_SAMPLES];
static float filter_gx = 0, filter_gy = 0, filter_gz = 0;
// 校准后的数据
static float calib_ax, calib_ay, calib_az;
static float calib_gx, calib_gy, calib_gz;
extern RCControl_t rc_control;
// ========== 全局零偏变量 ==========
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
// 执行校准

uint8_t perform_gyro_calibration(void)
{
    BMI088_Data_t gyro;
    int count = 0;
    
    // 临时求和变量
    float sum_gx = 0;
    float sum_gy = 0;
    float sum_gz = 0;
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Calibrating Gyro", OLED_8X16);
    OLED_ShowString(0, 16, "Keep STILL !!!", OLED_8X16);
    OLED_Update();
    Delay(1500); // 留出时间让用户把手拿开，确保绝对静止
    
    // 严格模仿你的中断缓冲区读取逻辑
    while (count < 100) { // 采集100组数据求平均
        if (BMI088_GetLatestGyroData(&gyro)) {
            sum_gx += gyro.gyro_x * BMI088_GYRO_SCALE;
            sum_gy += gyro.gyro_y * BMI088_GYRO_SCALE;
            sum_gz += gyro.gyro_z * BMI088_GYRO_SCALE;
            count++;
            
            // 每20次更新一次显示
            if (count % 20 == 0) {
                OLED_ShowNum(0, 32, count, 3, OLED_8X16);
                OLED_ShowString(48, 32, "/100", OLED_8X16);
                OLED_Update();
            }
        }
    }
    
    // 计算 100 次的平均值，作为零偏
    gyro_bias_x = sum_gx / 100.0f;
    gyro_bias_y = sum_gy / 100.0f;
    gyro_bias_z = sum_gz / 100.0f;
    
    // 显示校准得到的零偏值（用于调试）
    OLED_Clear();
    OLED_ShowString(0, 0, "Gyro Calib OK!", OLED_8X16);
    OLED_ShowString(0, 16, "Bias Z (dps):", OLED_8X16);
    OLED_ShowFloatNum(0, 32, gyro_bias_z, 3, 2, OLED_8X16);
    OLED_Update();
    return 1;
}
uint8_t perform_calibration(void)
{
    BMI088_Data_t accel;
    int count = 0;
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Calibrating...", OLED_8X16);
    OLED_ShowString(0, 16, "Keep STILL!", OLED_8X16);
    OLED_Update();
    Delay(2000);
    
    while (count < CALIB_SAMPLES) {
        if (BMI088_GetLatestAccelData(&accel)) {
            calib_accel_x[count] = accel.accel_x * BMI088_ACCEL_SCALE;
            calib_accel_y[count] = accel.accel_y * BMI088_ACCEL_SCALE;
            calib_accel_z[count] = accel.accel_z * BMI088_ACCEL_SCALE;
            count++;
            
            if (count % 100 == 0) {
                OLED_ShowNum(0, 32, count, 3, OLED_8X16);
                OLED_ShowString(48, 32, "/100", OLED_8X16);
                OLED_Update();
            }
        }
    }
    
    // 执行校准
    if (SensorAlignment_Calibrate(CALIB_SAMPLES, 
                                   calib_accel_x, calib_accel_y, calib_accel_z,
                                   CALIB_SAMPLES) == 0) {
        OLED_ShowString(0, 48, "Calib Failed!", OLED_8X16);
        OLED_Update();
        return 0;
    }
    
    // 显示校准结果
    float init_roll, init_pitch;
    uint8_t calibrated;
    SensorAlignment_GetInfo(&init_roll, &init_pitch, &calibrated);
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Calib SUCCESS!", OLED_8X16);
    OLED_ShowString(0, 16, "Init Roll:", OLED_8X16);
    OLED_ShowFloatNum(80, 16, init_roll, 3, 1, OLED_8X16);
    OLED_ShowString(0, 32, "Init Pitch:", OLED_8X16);
    OLED_ShowFloatNum(80, 32, init_pitch, 3, 1, OLED_8X16);
    OLED_Update();
    
    return 1;
}

int main(void)
{
    BMI088_Data_t accel_data, gyro_data;
    float ax, ay, az;
    float gx, gy, gz;
    Attr angle;
//    uint32_t x1=0, x2=0, x3=0;
   // --- 新增控制相关变量 ---
//    FCConfig_t fc_params;       // PID 参数配置
    MotorOutput_t motors;       // 电机最终输出值 (0.0~1.0)
//    uint32_t log_timer = 0;     // 降频打印定时器 
		
  // 1. 基础硬件初始化
	  LED_Init(); 
		USART3_Init(115200);
		USART1_Config();
	  Motor_PWM_Init();
    Delay_Init();
	
    OLED_Init();
	
    OLED_Clear();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

    // ========== 新增：PA8 LED 初始化 ==========
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // =========================================

    OLED_ShowString(0, 0, "Init BMI088...", OLED_8X16);
    OLED_Update();		 
		Motor_SetAllDuty(0,0,0,0);
    // 2. 初始化 BMI088
    if (BMI088_Init() == 0) {
        OLED_Clear();
        OLED_ShowString(0, 0, "BMI088 ERR!", OLED_8X16);
        while(1);
    }
    // 初始化外部中断
    EXTI_Init_Config();
    // 等待传感器稳定
    Delay(500);
    // 清除可能的中断挂起
    EXTI_ClearITPendingBit(EXTI_Line0);
    EXTI_ClearITPendingBit(EXTI_Line1);
    // 丢弃前20组数据
    for (int i = 0; i < 20; i++) {
        BMI088_GetLatestAccelData(&accel_data);
        BMI088_GetLatestGyroData(&gyro_data);
        Delay(10);
    }
    // 重置数据就绪标志
    g_accel_data_ready = 0;
    g_gyro_data_ready = 0;
    
    // 执行传感器校准
    perform_calibration();
    
		perform_gyro_calibration();
    
		// 3. 飞控系统初始化 (集成点)
    // 传入控制周期 dt，建议与姿态解算频率同步
    // 这里先给一个 0.0005 (1ms)，实际运行中 fc_update 内部会使用你传入的参数
    fc_init(0.0005f, 0); // 传 0 使用默认内置 PID 参数
    
    // 初始化姿态解算
    att_init_quat(0.0005f, 0.92f);
    
    // 4. 模式手动选择 (演示用)
    // 你可以在这里临时决定模式：FC_MODE_IDLE(停止), FC_MODE_ARMED(怠速), FC_MODE_STABILIZE(自稳)
    fc_set_mode(FC_MODE_STOP); 

    OLED_ShowString(0, 0, "QUAT Ready!", OLED_8X16);
    OLED_Update();
		// ================= 关键修改点 =================
    uint32_t last_time = 0;           // 不要在这里调用 GetUs()！给 0 就行
    uint8_t is_first_frame = 1;       // 增加一个“第一帧”标志位
    uint32_t led_timer = 0;           // LED 计时器
    // ==============================================
		OLED_Clear();
    while(1)
    {
//            x1 = GetUs();

    // ========== 新增：PA8 LED 0.25s 闪烁逻辑 (非阻塞) ==========
    if (GetUs() - led_timer >= 250000) 
        {
            led_timer = GetUs();
            LED_TOGGLE(); 
        }
    
    // ========== 1. 解析串口命令（更新 rc_control） ==========
    if (usart1_cmd_ready == 1) {
        USART1_CheckAndParse();
        
//        OLED_ShowChar(0, 0, usart1_cmd_char, OLED_8X16);
//        OLED_ShowNum(0, 32, (int)usart1_cmd_value, 5, OLED_8X16);
        
       switch(usart1_cmd_char) {
    case 'A':  // 解锁（响应油门）
        fc_set_mode(FC_MODE_ARMED);
        break;
    case 'D':  // 怠速（电机待命）
        fc_set_mode(FC_MODE_IDLE);
        rc_control.throttle = 0;
        break;
    case 'S':  // 停止（完全停转）
        fc_set_mode(FC_MODE_STOP);
        rc_control.throttle = 0;
        break;
    case 'H':  // 自稳模式
        fc_set_mode(FC_MODE_STABILIZE);
        break;
    case 'T':  // 油门
        rc_control.throttle = usart1_cmd_value / 100.0f;
        break;
    default:
        break;
}
        
        usart1_cmd_ready = 0;
//        OLED_Update();  // 只在有命令时更新显示
    }
    
    // ========== 2. 读取传感器数据 ==========
    if (BMI088_GetLatestAccelData(&accel_data)) {
        ax = accel_data.accel_x * BMI088_ACCEL_SCALE;
        ay = accel_data.accel_y * BMI088_ACCEL_SCALE;
        az = accel_data.accel_z * BMI088_ACCEL_SCALE;
        SensorAlignment_ApplyAccel(ax, ay, az, &calib_ax, &calib_ay, &calib_az);
    }
    
    // ========== 3. 读取陀螺仪并更新控制 ==========
    if (BMI088_GetLatestGyroData(&gyro_data)) {
        
        // 时间计算
        uint32_t now = GetUs();
        if (is_first_frame) {
            last_time = now;
            is_first_frame = 0;
            continue;
        }
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;
        if (dt > 0.05f) dt = 0.0005f;
        
        // 1. 读取原始数据并转换单位 (度/秒)
    float raw_gx = gyro_data.gyro_x * BMI088_GYRO_SCALE - gyro_bias_x;
    float raw_gy = gyro_data.gyro_y * BMI088_GYRO_SCALE - gyro_bias_y;
    float raw_gz = gyro_data.gyro_z * BMI088_GYRO_SCALE - gyro_bias_z;
    
    // 2. 极其关键的软件滤波 (一阶低通 LPF)
    // alpha 滤波系数，范围 0 到 1。
    // alpha 越小，滤波越强，但延迟越大。0.1 是一个非常经典的起始值。
    // 公式：新滤波值 = 旧滤波值*(1-alpha) + 原始新值*alpha
    float alpha = 0.05f; 
    
    filter_gx = filter_gx * (1.0f - alpha) + raw_gx * alpha;
    filter_gy = filter_gy * (1.0f - alpha) + raw_gy * alpha;
    filter_gz = filter_gz * (1.0f - alpha) + raw_gz * alpha;
    
    // 3. 将【滤波后】的数据用于姿态解算和 PID！
    SensorAlignment_ApplyGyro(filter_gx, filter_gy, filter_gz, &calib_gx, &calib_gy, &calib_gz);
    
    // 姿态解算 (四元数内部其实不需要太强滤波，但传滤波后的更稳)
		
		
    att_update_quat(calib_gx, calib_gy, calib_gz, calib_ax, calib_ay, calib_az, dt);
    
		angle = att_get_quat();  // ← 你漏了这一行！
		
		
    // 飞控控制 (PID 必须吃滤波后的角速度，尤其是 D 项！)
		
    Attitude_t current_att = {angle.roll, angle.pitch, 0.0f}; 
    motors = fc_update(current_att, calib_gx, calib_gy, calib_gz, dt); 
		My_USART_Printf(USART3, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", angle.roll, angle.pitch, angle.yaw,get_pitch(),get_roll(),calib_az);
        
        // 6. 输出电机
        Motor_SetDuty(MOTOR_FRONT, motors.motor[0]);
        Motor_SetDuty(MOTOR_RIGHT, motors.motor[1]);
        Motor_SetDuty(MOTOR_BACK,  motors.motor[2]);
        Motor_SetDuty(MOTOR_LEFT,  motors.motor[3]);
				
    }
    
    // 可选：显示姿态
    // OLED_ShowFloatNum(0, 16, angle.roll, 3, 1, OLED_8X16);
    // OLED_ShowFloatNum(0, 32, angle.pitch, 3, 1, OLED_8X16);
    // OLED_Update();
    
//    x2 = GetUs();
//    x3 = x2 - x1;
        // ========== 显示 ==========
//        OLED_ShowFloatNum(0, 16, calib_ax, 3, 2, OLED_8X16);
//        OLED_ShowFloatNum(0, 32, calib_ay, 3, 2, OLED_8X16);
//        OLED_ShowFloatNum(0, 48, calib_az, 3, 2, OLED_8X16);
//        OLED_ShowFloatNum(64, 16, calib_gx, 3, 2, OLED_8X16);
//        OLED_ShowFloatNum(64, 32, calib_gy, 3, 2, OLED_8X16);
//        OLED_ShowFloatNum(64, 48, calib_gz, 3, 2, OLED_8X16);
        
        // 显示姿态角
//        OLED_ShowFloatNum(0, 16, angle.roll, 3, 1, OLED_8X16);
//        OLED_ShowFloatNum(0, 32, angle.pitch, 3, 1, OLED_8X16);
//        OLED_ShowFloatNum(0, 48, angle.yaw, 3, 1, OLED_8X16);        
//        OLED_ShowNum(0, 0, x3, 10, OLED_8X16);
//				OLED_ShowNum(0, 16, gett1(), 8, OLED_8X16);
//				OLED_ShowNum(0, 32, gett2(), 8, OLED_8X16);
//        OLED_Update();     
    }
}




//#include "stm32f10x.h"                  // Device header
//#include "OLED.h"
//#include "delay1.h"
//#include "bmi088.h"
//#include "exti.h"
//#include "attitude.h"

//#define BMI088_ACC_ST       0x03   // 状态寄存器
//#define BMI088_ACC_CONF      0x40   // 配置寄存器（你用的）
//#define BMI088_ACC_ERR       0x02   // 错误寄存器
//void Test_BMI088_Hardware_Truth(void)
//{
//    uint8_t reg_a53, reg_a54, reg_g16, reg_a58;

//    OLED_Clear();
//    OLED_ShowString(0, 0, "Init BMI088...", OLED_8X16);
//    OLED_Update();

////    // 1. 唤醒加速度计并等待
////    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x7D, 0x04);
////    Delay(50); 
////    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x7C, 0x00);
////    Delay(50); 
////    // 2. 唤醒陀螺仪并等待
////    BMI088_WriteReg(BMI088_CS_GYRO_PIN, 0x11, 0x00);
////    Delay(50); 

//    // 3. 切断所有内部中断映射
//    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x58, 0x00);  
//    BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x59, 0x00);  
//    BMI088_WriteReg(BMI088_CS_GYRO_PIN, 0x18, 0x00); 

//    while(1)
//    {
//       // ==========================================
//        // 状态 A: 强行拉高 (目标输出 3.3V)
//        // ==========================================
//        // 加速度计 INT1：0x06 = 0b00000110
//        //   bit3=0(推挽), bit2=0(低电平有效 -> 空闲高), bit1=1(输出使能必须为1)
//        BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x53, 0x08);
//        // 加速度计 INT2：同上
//        BMI088_WriteReg(BMI088_CS_ACC_PIN, 0x54, 0x08);
//        // 陀螺仪 INT3/INT4：0x00 = 0b00000000
//        //   bit3=0(INT4推挽), bit2=0(INT4低有效->空闲高), bit1=0(INT3推挽), bit0=0(INT3低有效->空闲高)
//        BMI088_WriteReg(BMI088_CS_GYRO_PIN, 0x16, 0x00);
//        Delay(10);

//        // 实时读回验证！
//        reg_a53 = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x53);
//        reg_a54 = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x54);
//        reg_g16 = BMI088_ReadReg(BMI088_CS_GYRO_PIN, 0x16);
//        reg_a58 = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x58);

//        OLED_Clear();
//        OLED_ShowString(0, 0, "Target: 3.3V", OLED_8X16);
//        
//        OLED_ShowString(0, 16, "A53:", OLED_8X16); OLED_ShowHexNum(32, 16, reg_a53, 3, OLED_8X16);
//        OLED_ShowString(64,16, "A54:", OLED_8X16); OLED_ShowHexNum(96, 16, reg_a54, 3, OLED_8X16);
//        
//        OLED_ShowString(0, 32, "G16:", OLED_8X16); OLED_ShowHexNum(32, 32, reg_g16, 3, OLED_8X16);
//        OLED_ShowString(64,32, "A58:", OLED_8X16); OLED_ShowHexNum(96, 32, reg_a58, 3, OLED_8X16);
//        OLED_Update();
//    }
//}
//void OLED_ShowBMI088Registers(void)
//{
//    uint8_t st_reg, conf_reg, err_reg,INT;
//    
//    // 读取三个寄存器
//		INT=BMI088_ReadReg(BMI088_CS_ACC_PIN,0x1D);
//    st_reg = BMI088_ReadReg(BMI088_CS_ACC_PIN, BMI088_ACC_ST);      // 状态寄存器
//    conf_reg = BMI088_ReadReg(BMI088_CS_ACC_PIN, BMI088_ACC_CONF);   // 配置寄存器
//    err_reg = BMI088_ReadReg(BMI088_CS_ACC_PIN, BMI088_ACC_ERR);     // 错误寄存器
//    
//    // OLED 显示
//    OLED_ShowString(0, 0, "ST:", OLED_8X16);
//    OLED_ShowNum(32, 0, st_reg, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 16, "CONF:", OLED_8X16);
//    OLED_ShowNum(48, 16, conf_reg, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 32, "ERR:", OLED_8X16);
//    OLED_ShowNum(48, 32, err_reg, 3, OLED_8X16);
//	
//	  OLED_ShowNum(64, 0, INT, 4, OLED_8X16);
//}
//void QuickDiagnose(void)
//{
//    uint8_t pwr_ctrl, pwr_conf, range;
//    
//    pwr_ctrl = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x7D);
//    pwr_conf = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x7C);
//    range    = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x41);
//    
//    OLED_ShowString(0, 0, "PWR_CTRL:", OLED_8X16);
//    OLED_ShowNum(96, 0, pwr_ctrl, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 16, "PWR_CONF:", OLED_8X16);
//    OLED_ShowNum(96, 16, pwr_conf, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 32, "RANGE:", OLED_8X16);
//    OLED_ShowNum(96, 32, range, 3, OLED_8X16);
//}
//void CheckWhySTisZero(void)
//{
//    uint8_t err_reg, conf_reg, int_status;
//    
//    err_reg = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x02);   // ERR_REG
//    conf_reg = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x40);  // ACC_CONF (确认)
//    int_status = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x1D); // INT_STATUS_0
//    
//    OLED_ShowString(0, 0, "ERR:", OLED_8X16);
//    OLED_ShowNum(48, 0, err_reg, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 16, "CONF:", OLED_8X16);
//    OLED_ShowNum(48, 16, conf_reg, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 32, "INT_STAT:", OLED_8X16);
//    OLED_ShowNum(96, 32, int_status, 3, OLED_8X16);
//}
//void BMI088_CheckInterrupt(void)
//{
//    uint8_t int_io_ctrl, int_map, int_status;
//    
//    // 读取中断相关寄存器
//    int_io_ctrl = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x53);  // INT1_IO_CTRL
//    int_map    = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x58);  // INT_MAP_DATA
//    int_status = BMI088_ReadReg(BMI088_CS_ACC_PIN, 0x1D);  // INT_STATUS_0 (注意是0x1D不是0x1B)
//    
//    OLED_ShowString(0, 0, "IO_CTRL:", OLED_8X16);
//    OLED_ShowNum(80, 0, int_io_ctrl, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 16, "INT_MAP:", OLED_8X16);
//    OLED_ShowNum(80, 16, int_map, 3, OLED_8X16);
//    
//    OLED_ShowString(0, 32, "INT_STAT:", OLED_8X16);
//    OLED_ShowNum(90, 32, int_status, 3, OLED_8X16);
//}
//int main()
//{
//	Delay_Init();
//	OLED_Init();
//	OLED_Clear();
//	OLED_ShowString(0, 0, "hello", OLED_8X16);
//	OLED_Update();
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	BMI088_Init();
//	att_init(0.0005f, 0.95f);
////	Test_BMI088_Hardware_Truth();
//	EXTI_Init_Config();
//	BMI088_Data_t accel_data, gyro_data;
//  float ax, ay, az;     // 加速度 (g)
//  float gx, gy, gz;     // 角速度 (dps)
//	Attr angle;
//	uint16_t x=0;
//	uint32_t x1=0,x2=0,x3=0;
//	while(1)
//	{
//		CheckWhySTisZero();
////		BMI088_CheckInterrupt();
////		QuickDiagnose();
////		OLED_ShowBMI088Registers();
////    x1=GetUs();
////		OLED_ShowFloatNum(0,16,ax,3,2,OLED_8X16);
////		OLED_ShowFloatNum(0,32,ay,3,2,OLED_8X16);
////		OLED_ShowFloatNum(0,48,az,3,2,OLED_8X16);
////		OLED_ShowFloatNum(64,16,gx,3,2,OLED_8X16);
////		OLED_ShowFloatNum(64,32,gy,3,2,OLED_8X16);
////		OLED_ShowFloatNum(64,48,gz,3,2,OLED_8X16);
////		OLED_ShowNum(0,0,x3,10,OLED_8X16);
////		

////		if (BMI088_GetLatestAccelData(&accel_data)) {
////            // 转换为物理单位
////            ax = accel_data.accel_x * BMI088_ACCEL_SCALE;
////            ay = accel_data.accel_y * BMI088_ACCEL_SCALE;
////            az = accel_data.accel_z * BMI088_ACCEL_SCALE;
////        }
////		if (BMI088_GetLatestGyroData(&gyro_data)) {
////            // 转换为物理单位
////						x1=GetUs();
////            gx = gyro_data.gyro_x * BMI088_GYRO_SCALE;
////            gy = gyro_data.gyro_y * BMI088_GYRO_SCALE;
////            gz = gyro_data.gyro_z * BMI088_GYRO_SCALE;
////            
////			if (BMI088_GetLatestAccelData(&accel_data)) {
////            // 转换为物理单位
////            ax = accel_data.accel_x * BMI088_ACCEL_SCALE;
////            ay = accel_data.accel_y * BMI088_ACCEL_SCALE;
////            az = accel_data.accel_z * BMI088_ACCEL_SCALE;
////        }
////			// 姿态更新
////            att_update(gx, gy, gz, ax, ay, az);
////            // 获取姿态（需要时再调，不要每帧都显示）
////            angle = att_get();

////        }
////		OLED_ShowFloatNum(0,32,angle.pitch,3,2,OLED_8X16);
////		OLED_ShowFloatNum(0,48,g_yaw * 57.29578f,3,2,OLED_8X16);
////		OLED_ShowFloatNum(0,16,angle.roll,3,2,OLED_8X16);
//		OLED_Update();
////		x2=GetUs();
////		x3=x2-x1;
//	}
//}

