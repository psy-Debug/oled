#ifndef __FLIGHT_CONTROLLER_H
#define __FLIGHT_CONTROLLER_H

#include "pid.h"
#include "motor_mixer.h"

// 姿态结构体（如果attitude.h中没有定义，在这里定义）
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

// 飞控模式
typedef enum {
    FC_MODE_STOP = 0,      // 停止：电机完全停转
    FC_MODE_IDLE,          // 怠速：电机怠速，不响应油门
    FC_MODE_ARMED,         // 解锁：响应油门，无PID
    FC_MODE_STABILIZE      // 自稳：PID控制
} FCMode_t;
// 遥控器输入
typedef struct {
    float throttle;         // 油门 [0,1]
    float roll;             // 横滚 [-1,1]
    float pitch;            // 俯仰 [-1,1]
    float yaw;              // 偏航 [-1,1]
    uint8_t arm_switch;     // 解锁开关（0/1）
} RCControl_t;

// 飞控配置
typedef struct {
    PID_t pid_roll;
    PID_t pid_pitch;
    PID_t pid_yaw;
    
    float max_angle;        // 最大倾斜角度（度）
    float max_yaw_rate;     // 最大偏航角速度（dps）
    float angle_p_gain;     // 角度控制到角速度的比例系数
    
    float rate_p_gain;      // 角速度环P增益
    float rate_i_gain;      // 角速度环I增益
    float rate_d_gain;      // 角速度环D增益
} FCConfig_t;

// 初始化飞控（config传入0则使用默认配置）
void fc_init(float dt, FCConfig_t* config);

// 获取当前配置
FCConfig_t* fc_get_config(void);

// 设置飞控模式
void fc_set_mode(FCMode_t mode);

// 获取当前模式
FCMode_t fc_get_mode(void);

// 解锁/上锁
void fc_arm(void);
void fc_disarm(void);

// 更新遥控器输入
void fc_update_rc(RCControl_t rc);

// 飞控主循环
MotorOutput_t fc_update(Attitude_t current_attitude, float gyro_x, float gyro_y, float gyro_z, float dt);

// 检查是否已解锁
uint8_t fc_is_armed(void);
float get_pitch(void);
float get_roll(void);
float get_yawquat(void);
#endif

