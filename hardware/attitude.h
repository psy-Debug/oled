// attitude.h
#ifndef ATTITUDE_H
#define ATTITUDE_H

// 姿态角结构体（单位：度）
typedef struct {
    float roll;   // 横滚
    float pitch;  // 俯仰
    float yaw;    // 偏航
} Attr;

// 初始化互补滤波
// dt: 采样周期（秒）
// alpha: 融合系数（0.95表示95%相信陀螺仪）
void att_init(float dt, float alpha);

// 更新姿态（每读一次传感器就调用一次）
// gx,gy,gz: 角速度（弧度/秒）—— 注意：是机体坐标系下的角速度！
// ax,ay,az: 加速度（单位g）
void att_update(float gx, float gy, float gz, float ax, float ay, float az);

// 获取当前姿态角（度）
Attr att_get(void);

#endif


