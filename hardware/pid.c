#include "pid.h"
#include <math.h>

void pid_init(PID_t *pid, float kp, float ki, float kd, float dt,
              float out_min, float out_max, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_limit = integral_limit;
    pid_reset(pid);
}

void pid_reset(PID_t *pid)
{
    pid->setpoint = 0;
    pid->feedback = 0;
    pid->error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}

void pid_set_gains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// 修改后的 pid_update 增加 dt 参数
float pid_update(PID_t *pid, float setpoint, float feedback, float dt)
{
    pid->setpoint = setpoint;
    pid->feedback = feedback;
    pid->error = setpoint - feedback;
    
    // P 项
    float p_term = pid->kp * pid->error;
    
    // I 项 (使用传入的真实 dt)
    pid->integral += pid->ki * pid->error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    // D 项 (使用传入的真实 dt)
    // 注意：防止 dt 为 0 导致除以 0 崩溃
    if (dt > 0) {
        pid->derivative = pid->kd * (pid->error - pid->prev_error) / dt;
    } else {
        pid->derivative = 0;
    }
    pid->prev_error = pid->error;
    
    pid->output = p_term + pid->integral + pid->derivative;
    
    // 限幅
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;
    
    return pid->output;
}

float pid_update_filtered(PID_t *pid, float setpoint, float feedback, float filter_cutoff_hz)
{
    static float filtered_derivative = 0;
    float rc = 1.0f / (2.0f * 3.14159f * filter_cutoff_hz);
    float alpha = pid->dt / (rc + pid->dt);
    
    pid->setpoint = setpoint;
    pid->feedback = feedback;
    
    pid->error = setpoint - feedback;
    
    float p_term = pid->kp * pid->error;
    
    pid->integral += pid->ki * pid->error * pid->dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    float raw_derivative = (pid->error - pid->prev_error) / pid->dt;
    filtered_derivative = alpha * raw_derivative + (1 - alpha) * filtered_derivative;
    pid->derivative = pid->kd * filtered_derivative;
    pid->prev_error = pid->error;
    
    pid->output = p_term + pid->integral + pid->derivative;
    
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;
    
    return pid->output;
}
