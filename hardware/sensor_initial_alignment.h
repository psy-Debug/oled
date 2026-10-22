#ifndef __SENSOR_ALIGNMENT_H
#define __SENSOR_ALIGNMENT_H

#include <stdint.h>

// 四元数结构体 (w, x, y, z)
// w = cos(θ/2), (x,y,z) = 旋转轴方向 × sin(θ/2)
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion_t;

// 欧拉角 (弧度)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Euler_t;

// 校准结果
typedef struct {
    Quaternion_t align_quat;   // 传感器→机体的旋转四元数
    float init_pitch;           // 初始俯仰角（弧度）
    float init_roll;            // 初始横滚角（弧度）
    uint8_t calibrated;         // 是否已校准
} AlignmentResult_t;

// 全局校准参数（外部可读）
extern AlignmentResult_t g_alignment;

// ========== 四元数基础运算函数声明 ==========

// 四元数乘法: q = q1 * q2
// 几何意义：先做q2旋转，再做q1旋转
Quaternion_t Quat_Multiply(Quaternion_t q1, Quaternion_t q2);

// 四元数共轭（对于单位四元数等于逆）
// 几何意义：反向旋转
Quaternion_t Quat_Conjugate(Quaternion_t q);

// 四元数归一化（保持模长为1）
Quaternion_t Quat_Normalize(Quaternion_t q);

// 四元数旋转向量
// 几何意义：用四元数q旋转向量(vx,vy,vz)
void Quat_RotateVector(Quaternion_t q, float* vx, float* vy, float* vz);

// 从欧拉角创建四元数（ZYX顺序：先偏航，再俯仰，再横滚）
Quaternion_t Quat_FromEuler(float roll, float pitch, float yaw);

// 从四元数获取欧拉角（ZYX顺序，单位：弧度）
Euler_t Quat_ToEuler(Quaternion_t q);

// 从轴角构造四元数
// 几何意义：绕单位轴(ux,uy,uz)旋转angle弧度
Quaternion_t Quat_FromAxisAngle(float ux, float uy, float uz, float angle);

// 从陀螺仪数据构造增量四元数（小角度近似）
// 几何意义：在dt时间内，绕角速度向量(gx,gy,gz)方向旋转|ω|*dt弧度
Quaternion_t Quat_FromGyro(float gx, float gy, float gz, float dt);

// 从加速度计数据构造姿态四元数
// 几何意义：静止时，重力方向决定水平姿态（无法得到偏航）
Quaternion_t Quat_FromAccel(float ax, float ay, float az);

// 四元数球面线性插值（用于互补滤波）
// 几何意义：在q1和q2之间按比例t平滑过渡
Quaternion_t Quat_Slerp(Quaternion_t q1, Quaternion_t q2, float t);

// ========== 校准相关函数声明 ==========

// 执行校准（需要在静止状态下调用）
// 采样samples次加速度数据，计算安装偏差
uint8_t SensorAlignment_Calibrate(int samples, 
                                   float* ax_array, float* ay_array, float* az_array,
                                   int len);

// 应用校准到传感器数据（将传感器坐标系转到机体坐标系）
void SensorAlignment_Apply(float sx, float sy, float sz, 
                           float* bx, float* by, float* bz);

// 应用校准到加速度数据
void SensorAlignment_ApplyAccel(float ax, float ay, float az,
                                 float* out_x, float* out_y, float* out_z);

// 应用校准到陀螺仪数据
void SensorAlignment_ApplyGyro(float gx, float gy, float gz,
                                float* out_x, float* out_y, float* out_z);

// 获取校准信息（用于调试）
void SensorAlignment_GetInfo(float* init_roll_deg, float* init_pitch_deg, uint8_t* calibrated);

// 重置校准（清除校准数据）
void SensorAlignment_Reset(void);

#endif


