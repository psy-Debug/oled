#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "sensor_initial_alignment.h"

// 姿态角结构体（单位：度）
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attr;

// 初始化互补滤波
void att_init_quat(float dt, float alpha);

// 更新姿态（四元数版本）
void att_update_quat(float gx, float gy, float gz, float ax, float ay, float az, float dt);

// 获取当前姿态角（度）
Attr att_get_quat(void);

// 获取当前姿态四元数
Quaternion_t att_get_quat_quat(void);

// 重置姿态
void att_reset_quat(void);

#endif


