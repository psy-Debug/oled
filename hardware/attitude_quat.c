#include "attitude_quat.h"
#include <math.h>
#include "delay1.h"
static float last_update_time = 0;
static float dt;              // 采样周期（秒）
static float alpha;           // 互补滤波系数
static Quaternion_t q;        // 当前姿态四元数
static Attr angle;            // 当前姿态角（度）
static uint8_t initialized;   // 是否已初始化

// 角度转弧度
#define DEG_TO_RAD 0.01745329252f
#define RAD_TO_DEG 57.295779513f

void att_init_quat(float sample_dt, float sample_alpha)
{
    dt = sample_dt;
    alpha = sample_alpha;
    initialized = 0;
    
    // 初始姿态：水平（单位四元数）
    q.w = 1.0f;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    
    angle.roll = 0;
    angle.pitch = 0;
    angle.yaw = 0;
}

void att_update_quat(float gx, float gy, float gz, float ax, float ay, float az, float delta_t)
{

    Quaternion_t delta;
    Quaternion_t q_gyro;
    Quaternion_t q_accel;
    Quaternion_t q_fusion;

    float gx_rad;
    float gy_rad;
    float gz_rad;

    gx_rad=gx*DEG_TO_RAD;
    gy_rad=gy*DEG_TO_RAD;
    gz_rad=gz*DEG_TO_RAD;

    /*
        陀螺积分
    */

    delta=Quat_FromGyro(gx_rad,gy_rad,gz_rad,delta_t);

    q_gyro=Quat_Multiply(q,delta);

    q_gyro=Quat_Normalize(q_gyro);

    /*
        加速度姿态
    */

    q_accel=
    Quat_FromAccel(ax,ay,az);

    /*
        第一次初始化
    */

    if(!initialized)
    {
        q=q_accel;

        initialized=1;

        return;
    }

    /*
        保留gyro的yaw
    */

    Euler_t gyro_euler=Quat_ToEuler(q_gyro);

    Euler_t accel_euler=Quat_ToEuler(q_accel);

    q_accel=Quat_FromEuler(accel_euler.roll, accel_euler.pitch,gyro_euler.yaw);

    /*
        融合
    */

    q_fusion=Quat_Slerp(q_gyro, q_accel, alpha);

    q=Quat_Normalize(q_fusion );

    /*
        保证符号连续
    */

    if(q.w<0)
    {
        q.w=-q.w;
        q.x=-q.x;
        q.y=-q.y;
        q.z=-q.z;
    }

    /*
        输出欧拉角
    */

    Euler_t euler=Quat_ToEuler(q);

    angle.roll=euler.roll* RAD_TO_DEG;

    angle.pitch=euler.pitch*  RAD_TO_DEG;

    angle.yaw= euler.yaw*RAD_TO_DEG;
}

Attr att_get_quat(void)
{
    return angle;
}

Quaternion_t att_get_quat_quat(void)
{
    return q;
}

void att_reset_quat(void)
{
    q.w = 1.0f;
    q.x = 0;
    q.y = 0;
    q.z = 0;
    angle.roll = 0;
    angle.pitch = 0;
    angle.yaw = 0;
    initialized = 0;
}

