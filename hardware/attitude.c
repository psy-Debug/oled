// attitude.c
#include "attitude.h"
#include <math.h>

static float dt;           // 采样周期（秒）
static float alpha;        // 融合系数
static Attr angle;         // 当前姿态角（度）
static float last_roll;    // 上一次横滚角（弧度）
static float last_pitch;   // 上一次俯仰角（弧度）

// 角度转弧度
#define DEG_TO_RAD (3.1415926535f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.1415926535f)

// 初始化
void att_init(float sample_dt, float sample_alpha) {
    dt = sample_dt;
    alpha = sample_alpha;
    angle.roll = 0;
    angle.pitch = 0;
    angle.yaw = 0;
    last_roll = 0;
    last_pitch = 0;
}

// 核心融合算法（修正版）
void att_update(float gx, float gy, float gz, float ax, float ay, float az) {
    float roll_rad = angle.roll * DEG_TO_RAD;
    float pitch_rad = angle.pitch * DEG_TO_RAD;
    
    // ========== 第1步：将机体角速度转换到世界坐标系 ==========
    // 公式来源：欧拉角微分方程（ZYX顺序）
    // 注意：当 cos(pitch) 接近 0 时会有奇点，但简单场景可用
    
    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_pitch = cosf(pitch_rad);
    
    // 防止除以0
    float safe_cos_pitch = cos_pitch;
    if (safe_cos_pitch < 0.001f && safe_cos_pitch > -0.001f) {
        safe_cos_pitch = (safe_cos_pitch >= 0) ? 0.001f : -0.001f;
    }
    float tan_pitch = sin_pitch / safe_cos_pitch;
    
    // 世界坐标系下的角速度变化率（弧度/秒）
    float world_roll_rate = gx + gy * sin_roll * tan_pitch + gz * cos_roll * tan_pitch;
    float world_pitch_rate = gy * cos_roll - gz * sin_roll;
    float world_yaw_rate = (gy * sin_roll + gz * cos_roll) / safe_cos_pitch;
    
    // ========== 第2步：陀螺仪积分（预测） ==========
    float roll_pred = angle.roll + world_roll_rate * dt * RAD_TO_DEG;
    float pitch_pred = angle.pitch + world_pitch_rate * dt * RAD_TO_DEG;
    float yaw_pred = angle.yaw + world_yaw_rate * dt * RAD_TO_DEG;
    
    // ========== 第3步：从加速度计算角度（观测） ==========
    // 静止时，加速度计测的是重力方向
    float xy_norm = sqrtf(ay*ay + az*az);
    if (xy_norm < 0.001f) xy_norm = 0.001f;
    
    float pitch_obs = atan2f(-ax, xy_norm) * RAD_TO_DEG;
    float roll_obs = atan2f(ay, az) * RAD_TO_DEG;
    
    // ========== 第4步：互补滤波融合 ==========
    angle.roll = alpha * roll_pred + (1.0f - alpha) * roll_obs;
    angle.pitch = alpha * pitch_pred + (1.0f - alpha) * pitch_obs;
    angle.yaw = yaw_pred;  // 偏航只能靠陀螺仪，会漂移
    
    // 保存弧度值供下次使用
    last_roll = angle.roll * DEG_TO_RAD;
    last_pitch = angle.pitch * DEG_TO_RAD;
    
    // 将角度限制在 [-180, 180] 范围内
    if (angle.yaw > 180.0f) angle.yaw -= 360.0f;
    if (angle.yaw < -180.0f) angle.yaw += 360.0f;
}

// 获取当前姿态
Attr att_get(void) {
    return angle;
}
float get_yaw(void)
{
	return angle.yaw;
}


