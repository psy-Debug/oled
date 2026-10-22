#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    
    float setpoint;         // 目标值
    float feedback;         // 反馈值
    float error;            // 当前误差
    float prev_error;       // 上一次误差
    float integral;         // 积分项
    float derivative;       // 微分项
    
    float output;           // PID输出
    
    float out_min;          // 输出最小值限制
    float out_max;          // 输出最大值限制
    float integral_limit;   // 积分限幅
    
    float dt;               // 采样周期（秒）
} PID_t;

// 初始化PID
void pid_init(PID_t *pid, float kp, float ki, float kd, float dt, 
              float out_min, float out_max, float integral_limit);

// 重置PID状态
void pid_reset(PID_t *pid);

// 更新PID输出（标准形式）
float pid_update(PID_t *pid, float setpoint, float feedback, float dt);

// 更新PID输出（带微分低通滤波）
float pid_update_filtered(PID_t *pid, float setpoint, float feedback, float filter_cutoff_hz);

// 设置PID参数（动态调参）
void pid_set_gains(PID_t *pid, float kp, float ki, float kd);

#endif

