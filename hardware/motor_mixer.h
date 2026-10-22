#ifndef __MOTOR_MIXER_H
#define __MOTOR_MIXER_H

#include <stdint.h>

// 电机编号
typedef enum {
    MOTOR_FRONT = 0,    // 前电机 (M1)
    MOTOR_RIGHT,        // 右电机 (M2)
    MOTOR_BACK,         // 后电机 (M3)
    MOTOR_LEFT,         // 左电机 (M4)
    MOTOR_COUNT
} MotorID_t;

// 混控器控制输入
typedef struct {
    float throttle;     // 油门 [0, 1]
    float roll;         // 横滚控制 [-1, 1]
    float pitch;        // 俯仰控制 [-1, 1]
    float yaw;          // 偏航控制 [-1, 1]
} ControlInput_t;

// 电机输出（PWM占空比）
typedef struct {
    float motor[MOTOR_COUNT];   // 各电机输出 [0, 1]
} MotorOutput_t;

// 混控器配置
typedef struct {
    float throttle_idle;        // 怠速油门（默认0.05）
    float min_throttle;         // 最小允许油门
    float max_throttle;         // 最大允许油门
} MixerConfig_t;

// 初始化混控器（config为0时使用默认配置）
void mixer_init(MixerConfig_t *config);

// 设置混控参数
void mixer_set_config(MixerConfig_t config);

// 获取当前配置
MixerConfig_t mixer_get_config(void);

// X型四轴混控计算
MotorOutput_t mixer_compute_x(ControlInput_t input);

// +型四轴混控计算（备用）
MotorOutput_t mixer_compute_plus(ControlInput_t input);

// 电机输出安全限幅
void mixer_apply_safety(MotorOutput_t *output, float throttle);

#endif


