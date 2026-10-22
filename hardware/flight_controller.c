#include "flight_controller.h"
#include <math.h>

/*============================================================================
 * 静态变量（飞控状态存储）
 *============================================================================*/

static FCConfig_t fc_config;           // 飞控配置（PID参数、限幅等）
RCControl_t rc_control = {0};   // 遥控器输入（油门、横滚、俯仰、偏航）
static FCMode_t current_mode = FC_MODE_STOP;  // 当前模式：空闲/解锁/自稳
static float control_dt = 0.0005f;       // 控制周期（秒），默认10ms
static uint8_t arm_request = 0;        // 解锁请求标志
static uint8_t disarm_request = 0;     // 上锁请求标志

static float target_roll = 0;          // 目标横滚角（度）
static float target_pitch = 0;         // 目标俯仰角（度）
static float target_yaw_rate = 0;      // 目标偏航角速度（度/秒）
static float pitch=0;
static float roll=0;
static float yaw=0;

/*============================================================================
 * 飞控初始化
 *============================================================================*/

/**
 * @brief 初始化飞控
 * @param dt 控制周期（秒），如 0.01 表示10ms
 * @param config 配置指针，传入0则使用默认配置
 */
void fc_init(float dt, FCConfig_t* config)
{
    control_dt = dt;
    
    if (config != 0) {
        fc_config = *config;
    } else {
        // ========== 经过严格量纲缩放后的参数 ==========     

// 1. 增大外环 P（让飞机对“倾斜”更敏感，更想赶紧平过来）
        // 之前是 2.0，现在改到 4.0 到 5.0。
        fc_config.angle_p_gain = 24.0f;      

        // 2. 调整内环 PID
        // P: 稍微增大一点 (0.005 -> 0.008)，让它对目标速度响应更有力。
        // I: 稍微增大一点 (0.001 -> 0.002)，让它在被你强行按住时，能更快地“憋”出反抗力。
        // D: 【直接设为 0！】有刷小四轴加上软件滤波后，PI 控制就足够丝滑了，关掉 D 可以消除手拿时的“死硬/抽搐”感。
        // 限幅：可以适当放宽到 0.4 (允许 40% 的油门差)。
        pid_init(&fc_config.pid_roll,  0.003f, 0.0015f, 0.0f, dt, -0.15f, 0.15f, 0.15f);
        pid_init(&fc_config.pid_pitch, 0.003f, 0.0015f, 0.0f, dt, -0.15f, 0.15f, 0.15f);
        pid_init(&fc_config.pid_yaw,   0.003f, 0.001f, 0.0f, dt, -0.15f, 0.15f, 0.1f);

        fc_config.max_angle = 30.0f;        // 最大倾斜限制 30 度
        fc_config.max_yaw_rate = 100.0f;    
    }
}
/**
 * @brief 获取飞控配置指针
 * @return 配置结构体指针
 */
FCConfig_t* fc_get_config(void)
{
    return &fc_config;
}


/*============================================================================
 * 模式管理
 *============================================================================*/

/**
 * @brief 设置飞控模式
 * @param mode 目标模式
 * 
 * 模式切换规则：
 * - IDLE → ARMED：需要满足条件（油门最低+偏航打右）
 * - ARMED → STABILIZE：直接切换
 * - 任意 → IDLE：需要上锁条件
 */
void fc_set_mode(FCMode_t mode)
{
    // 允许任意模式切换
    current_mode = mode;
    
    // 切换模式时重置PID状态
    pid_reset(&fc_config.pid_roll);
    pid_reset(&fc_config.pid_pitch);
    pid_reset(&fc_config.pid_yaw);
    
    // 退出时油门归零（安全）
    rc_control.throttle = 0;
    rc_control.roll = 0;
    rc_control.pitch = 0;
    rc_control.yaw = 0;
}

/**
 * @brief 获取当前飞控模式
 * @return 当前模式
 */
FCMode_t fc_get_mode(void)
{
    return current_mode;
}

/**
 * @brief 检查是否已解锁（电机允许转动）
 * @return 1=已解锁，0=未解锁
 */
uint8_t fc_is_armed(void)
{
    return (current_mode != FC_MODE_IDLE);
}

/**
 * @brief 请求解锁（需要遥控器配合）
 * @note 实际解锁还需满足油门最低且偏航打右
 */
void fc_arm(void)
{
    if (current_mode == FC_MODE_IDLE && rc_control.throttle < 0.05f) {
        arm_request = 1;
    }
}

/**
 * @brief 请求上锁（需要遥控器配合）
 * @note 实际解锁还需满足油门最低且偏航打左
 */
void fc_disarm(void)
{
    disarm_request = 1;
}


/*============================================================================
 * 遥控器输入处理
 *============================================================================*/

/**
 * @brief 更新遥控器输入
 * @param rc 遥控器数据结构体
 * 
 * 功能：
 * 1. 限制输入范围（油门0-1，摇杆-1到1）
 * 2. 处理解锁/上锁逻辑（油门最低+偏航打右解锁，偏航打左上锁）
 */
void fc_update_rc(RCControl_t rc)
{
    rc_control = rc;
    
    // ========== 输入限幅 ==========
    if (rc_control.throttle < 0) rc_control.throttle = 0;
    if (rc_control.throttle > 1) rc_control.throttle = 1;
    
    if (rc_control.roll > 1) rc_control.roll = 1;
    if (rc_control.roll < -1) rc_control.roll = -1;
    if (rc_control.pitch > 1) rc_control.pitch = 1;
    if (rc_control.pitch < -1) rc_control.pitch = -1;
    if (rc_control.yaw > 1) rc_control.yaw = 1;
    if (rc_control.yaw < -1) rc_control.yaw = -1;
    
    // ========== 解锁逻辑 ==========
    // 条件：有解锁请求 +（解锁开关打开 OR 摇杆右下角）
    if (arm_request) {
        if (rc_control.arm_switch || (rc_control.yaw > 0.8f && rc_control.throttle < 0.1f)) {
            fc_set_mode(FC_MODE_ARMED);
            arm_request = 0;
        }
    }
    
    // ========== 上锁逻辑 ==========
    // 条件：有上锁请求 OR（摇杆左下角）
    if (disarm_request || (rc_control.yaw < -0.8f && rc_control.throttle < 0.1f)) {
        fc_set_mode(FC_MODE_IDLE);
        disarm_request = 0;
    }
}


/*============================================================================
 * 飞控主循环（核心控制）
 *============================================================================*/

/**
 * @brief 飞控主更新函数（在每个控制周期调用）
 * @param current_attitude 当前姿态角（度），从姿态解算获取
 * @param gyro_x, gyro_y, gyro_z 当前角速度（度/秒），从陀螺仪获取
 * @return MotorOutput_t 电机输出值（范围0-1）
 * 
 * 控制架构（串级PID）：
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    外环：角度环（P控制器）                    │
 * │  目标角速度 = (目标角度 - 当前角度) × P_GAIN                 │
 * └─────────────────────────────────────────────────────────────┘
 *                              │
 *                              ▼
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    内环：角速度环（PID控制器）               │
 * │  电机控制量 = PID(目标角速度, 当前角速度)                    │
 * └─────────────────────────────────────────────────────────────┘
 *                              │
 *                              ▼
 * ┌─────────────────────────────────────────────────────────────┐
 * │                        混控器                               │
 * │  电机输出 = 油门 + 横滚控制 + 俯仰控制 ± 偏航控制           │
 * └─────────────────────────────────────────────────────────────┘
 */
MotorOutput_t fc_update(Attitude_t current_attitude, float gyro_x, float gyro_y, float gyro_z, float dt)
{
    MotorOutput_t motor_output;
    ControlInput_t control_input;
    
    switch(current_mode) {
        
        case FC_MODE_STOP:  // 停止：完全停转
				{control_input.throttle = 0;
            control_input.roll = 0;
            control_input.pitch = 0;
            control_input.yaw = 0;
            // 直接输出0，跳过混控器
            motor_output.motor[MOTOR_FRONT] = 0;
            motor_output.motor[MOTOR_RIGHT] = 0;
            motor_output.motor[MOTOR_BACK] = 0;
            motor_output.motor[MOTOR_LEFT] = 0;
            return motor_output;  // 直接返回
				}
        case FC_MODE_IDLE:  // 怠速：固定怠速值
				{control_input.throttle = 0;
            control_input.roll = 0;
            control_input.pitch = 0;
            control_input.yaw = 0;
            motor_output = mixer_compute_x(control_input);
            // 强制设置怠速
            for (int i = 0; i < MOTOR_COUNT; i++) {
                motor_output.motor[i] = 0.05f;  // 5%怠速
            }
            return motor_output;
					}
        case FC_MODE_ARMED:  // 解锁：响应油门，无PID
            control_input.throttle = rc_control.throttle;
            control_input.roll = rc_control.roll;
            control_input.pitch = rc_control.pitch;
            control_input.yaw = rc_control.yaw;
            break;
            
        case FC_MODE_STABILIZE:  // 自稳：PID控制
				{target_roll = rc_control.roll * fc_config.max_angle;
            target_pitch = rc_control.pitch * fc_config.max_angle;
            
            float roll_error = target_roll - current_attitude.roll;
            float pitch_error = target_pitch - current_attitude.pitch;
            
            float roll_rate_target = roll_error * fc_config.angle_p_gain;
            float pitch_rate_target = pitch_error * fc_config.angle_p_gain;
            
            float max_rate = fc_config.max_angle * fc_config.angle_p_gain;
            if (roll_rate_target > max_rate) roll_rate_target = max_rate;
            if (roll_rate_target < -max_rate) roll_rate_target = -max_rate;
            if (pitch_rate_target > max_rate) pitch_rate_target = max_rate;
            if (pitch_rate_target < -max_rate) pitch_rate_target = -max_rate;
            
            target_yaw_rate = 0* fc_config.max_yaw_rate;
            
            control_input.roll = pid_update(&fc_config.pid_roll, roll_rate_target, gyro_x, dt);
						control_input.pitch = pid_update(&fc_config.pid_pitch, pitch_rate_target, gyro_y, dt);
						control_input.yaw = pid_update(&fc_config.pid_yaw, target_yaw_rate, gyro_z, dt);
            control_input.throttle = rc_control.throttle;
						pitch=control_input.pitch;
            roll=control_input.roll;
						yaw=control_input.yaw;
            if (control_input.roll > 1) control_input.roll = 1;
            if (control_input.roll < -1) control_input.roll = -1;
            if (control_input.pitch > 1) control_input.pitch = 1;
            if (control_input.pitch < -1) control_input.pitch = -1;
            if (control_input.yaw > 1) control_input.yaw = 1;
            if (control_input.yaw < -1) control_input.yaw = -1;
            break;
					}
        default:
				{control_input.throttle = 0;
            control_input.roll = 0;
            control_input.pitch = 0;
            control_input.yaw = 0;
            break;}
    }
    
    motor_output = mixer_compute_x(control_input);
    mixer_apply_safety(&motor_output, control_input.throttle);
    
    return motor_output;
}
float get_pitch(void)
{return pitch;}
float get_roll(void)
{return roll;}
float get_yawquat(void)
{
return yaw;}
