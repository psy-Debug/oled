#include "sensor_initial_alignment.h"
#include <math.h>

// 全局校准参数定义
AlignmentResult_t g_alignment = {
    .align_quat = {1.0f, 0, 0, 0},
    .init_pitch = 0,
    .init_roll = 0,
    .calibrated = 0
};

// ========== 四元数基础运算函数实现 ==========

// 四元数乘法: q = q1 * q2
Quaternion_t Quat_Multiply(Quaternion_t q1, Quaternion_t q2)
{
    Quaternion_t result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// 四元数共轭（对于单位四元数等于逆）
Quaternion_t Quat_Conjugate(Quaternion_t q)
{
    Quaternion_t result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

// 四元数归一化
Quaternion_t Quat_Normalize(Quaternion_t q)
{
    float norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    Quaternion_t result;
    
    if (norm < 1e-6f) {
        result.w = 1.0f;
        result.x = 0;
        result.y = 0;
        result.z = 0;
        return result;
    }
    
    norm = 1.0f / sqrtf(norm);
    result.w = q.w * norm;
    result.x = q.x * norm;
    result.y = q.y * norm;
    result.z = q.z * norm;
    return result;
}

// 四元数旋转向量
void Quat_RotateVector(Quaternion_t q, float* vx, float* vy, float* vz)
{
    float x = *vx, y = *vy, z = *vz;
    float w = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    // 优化版：直接展开旋转矩阵，比两次四元数乘法更快
    float w2 = w * w;
    float x2 = qx * qx;
    float y2 = qy * qy;
    float z2 = qz * qz;
    float wx = w * qx;
    float wy = w * qy;
    float wz = w * qz;
    float xy = qx * qy;
    float xz = qx * qz;
    float yz = qy * qz;
    
    *vx = (w2 + x2 - y2 - z2) * x + 2.0f * (xy - wz) * y + 2.0f * (xz + wy) * z;
    *vy = 2.0f * (xy + wz) * x + (w2 - x2 + y2 - z2) * y + 2.0f * (yz - wx) * z;
    *vz = 2.0f * (xz - wy) * x + 2.0f * (yz + wx) * y + (w2 - x2 - y2 + z2) * z;
}

// 从欧拉角创建四元数（ZYX顺序：先偏航，再俯仰，再横滚）
Quaternion_t Quat_FromEuler(float roll, float pitch, float yaw)
{
    Quaternion_t q;
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

// 从四元数获取欧拉角（ZYX顺序，单位：弧度）
Euler_t Quat_ToEuler(Quaternion_t q)
{
    Euler_t euler;
    Quaternion_t norm_q = Quat_Normalize(q);
    
    float q0 = norm_q.w;
    float q1 = norm_q.x;
    float q2 = norm_q.y;
    float q3 = norm_q.z;
    
    // 防止数值误差导致asin参数超出[-1,1]
    float sin_pitch = -2.0f * (q1 * q3 - q0 * q2);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    
    euler.pitch = asinf(sin_pitch);
    euler.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    euler.yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    
    return euler;
}

// 从轴角构造四元数
Quaternion_t Quat_FromAxisAngle(float ux, float uy, float uz, float angle)
{
    Quaternion_t q;
    float half_angle = angle * 0.5f;
    float sin_half = sinf(half_angle);
    float cos_half = cosf(half_angle);
    
    // 确保轴是单位向量
    float norm = sqrtf(ux*ux + uy*uy + uz*uz);
    if (norm > 1e-6f) {
        ux /= norm;
        uy /= norm;
        uz /= norm;
    }
    
    q.w = cos_half;
    q.x = ux * sin_half;
    q.y = uy * sin_half;
    q.z = uz * sin_half;
    
    return Quat_Normalize(q);
}

// 从陀螺仪数据构造增量四元数（小角度近似，dt很小时）
Quaternion_t Quat_FromGyro(float gx, float gy, float gz, float dt)
{
    Quaternion_t delta;

    // gx gy gz单位：rad/s

    float omega = sqrtf(
                    gx*gx+
                    gy*gy+
                    gz*gz);

    if(omega<1e-6f)
    {
        delta.w=1.0f;
        delta.x=0;
        delta.y=0;
        delta.z=0;

        return delta;
    }

    // 总旋转角度 = 角速度 × 时间
    float angle=omega*dt;

    // 单位旋转轴
    float ux=gx/omega;
    float uy=gy/omega;
    float uz=gz/omega;

    float half=angle*0.5f;

    float sin_half=sinf(half);

    delta.w=cosf(half);

    delta.x=ux*sin_half;
    delta.y=uy*sin_half;
    delta.z=uz*sin_half;

    return Quat_Normalize(delta);
}

// 从加速度计数据构造姿态四元数
// 几何意义：静止时，重力方向决定水平姿态（无法得到偏航）
Quaternion_t Quat_FromAccel(float ax,float ay,float az)
{
    Quaternion_t q;

    float norm=
        sqrtf(
        ax*ax+
        ay*ay+
        az*az);

    if(norm<1e-6f)
    {
        q.w=1;
        q.x=0;
        q.y=0;
        q.z=0;

        return q;
    }

    ax/=norm;
    ay/=norm;
    az/=norm;

    float roll=
        atan2f(
            ay,
            az
            );

    float pitch=
        atan2f(
            -ax,
            sqrtf(
                ay*ay+
                az*az
                )
            );

    q=
    Quat_FromEuler(
            roll,
            pitch,
            0
            );

    return Quat_Normalize(q);
}

// 四元数球面线性插值
Quaternion_t Quat_Slerp(Quaternion_t q1, Quaternion_t q2, float t)
{
    Quaternion_t result;
    
    // 计算两个四元数的点积
    float dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
    
    // 如果点积为负，取反一个，保证最短路径
    if (dot < 0) {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        dot = -dot;
    }
    
    // 如果夹角很小，用线性插值
    if (dot > 0.9995f) {
        result.w = q1.w + (q2.w - q1.w) * t;
        result.x = q1.x + (q2.x - q1.x) * t;
        result.y = q1.y + (q2.y - q1.y) * t;
        result.z = q1.z + (q2.z - q1.z) * t;
        return Quat_Normalize(result);
    }
    
    // 球面线性插值
    float omega = acosf(dot);
    float sin_omega = sinf(omega);
    float s1 = sinf((1 - t) * omega) / sin_omega;
    float s2 = sinf(t * omega) / sin_omega;
    
    result.w = q1.w * s1 + q2.w * s2;
    result.x = q1.x * s1 + q2.x * s2;
    result.y = q1.y * s1 + q2.y * s2;
    result.z = q1.z * s1 + q2.z * s2;
    
    return Quat_Normalize(result);
}

// ========== 校准相关函数实现 ==========

// 执行校准
uint8_t SensorAlignment_Calibrate(int samples, 
                                   float* ax_array, float* ay_array, float* az_array,
                                   int len)
{
    int i;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    
    if (samples > len) samples = len;
    if (samples < 10) return 0;
    
    // 求平均，消除噪声
    for (i = 0; i < samples; i++) {
        sum_ax += ax_array[i];
        sum_ay += ay_array[i];
        sum_az += az_array[i];
    }
    
    float mean_ax = sum_ax / samples;
    float mean_ay = sum_ay / samples;
    float mean_az = sum_az / samples;
    
    // 计算标准差，判断是否静止
    float var_sum = 0;
    for (i = 0; i < samples; i++) {
        float dx = ax_array[i] - mean_ax;
        float dy = ay_array[i] - mean_ay;
        float dz = az_array[i] - mean_az;
        var_sum += dx*dx + dy*dy + dz*dz;
    }
    float std_dev = sqrtf(var_sum / samples);
    
    // 如果运动过大，校准失败
    if (std_dev > 0.03f) {
        return 0;
    }
    
    // 从平均加速度计算传感器姿态（传感器相对于水平的姿态）
    Quaternion_t sensor_attitude = Quat_FromAccel(mean_ax, mean_ay, mean_az);
    
    // 目标姿态：水平（单位四元数）
    Quaternion_t target_attitude = {1.0f, 0, 0, 0};
    
    // 计算校准四元数：将传感器姿态转到目标姿态
    // align_quat = target × sensor?1
    Quaternion_t sensor_inv = Quat_Conjugate(sensor_attitude);
    g_alignment.align_quat = Quat_Multiply(target_attitude, sensor_inv);
    g_alignment.align_quat = Quat_Normalize(g_alignment.align_quat);
    g_alignment.align_quat = Quat_Conjugate(g_alignment.align_quat);
    // 记录初始欧拉角（用于调试）
    Euler_t init_euler = Quat_ToEuler(sensor_attitude);
    g_alignment.init_roll = init_euler.roll;
    g_alignment.init_pitch = init_euler.pitch;
    g_alignment.calibrated = 1;
    
    return 1;
}

// 应用校准到传感器数据
void SensorAlignment_Apply(float sx, float sy, float sz, 
                           float* bx, float* by, float* bz)
{
    if (!g_alignment.calibrated) {
        *bx = sx;
        *by = sy;
        *bz = sz;
        return;
    }
    
    *bx = sx;
    *by = sy;
    *bz = sz;
    Quat_RotateVector(g_alignment.align_quat, bx, by, bz);
}

// 应用校准到加速度数据
void SensorAlignment_ApplyAccel(float ax, float ay, float az,
                                 float* out_x, float* out_y, float* out_z)
{
    SensorAlignment_Apply(ax, ay, az, out_x, out_y, out_z);
}

// 应用校准到陀螺仪数据
void SensorAlignment_ApplyGyro(float gx, float gy, float gz,
                                float* out_x, float* out_y, float* out_z)
{
    SensorAlignment_Apply(gx, gy, gz, out_x, out_y, out_z);
}

// 获取校准信息
void SensorAlignment_GetInfo(float* init_roll_deg, float* init_pitch_deg, uint8_t* calibrated)
{
    if (init_roll_deg != 0) {
        *init_roll_deg = g_alignment.init_roll * 57.29578f;
    }
    if (init_pitch_deg != 0) {
        *init_pitch_deg = g_alignment.init_pitch * 57.29578f;
    }
    if (calibrated != 0) {
        *calibrated = g_alignment.calibrated;
    }
}

// 重置校准
void SensorAlignment_Reset(void)
{
    g_alignment.align_quat.w = 1.0f;
    g_alignment.align_quat.x = 0;
    g_alignment.align_quat.y = 0;
    g_alignment.align_quat.z = 0;
    g_alignment.init_roll = 0;
    g_alignment.init_pitch = 0;
    g_alignment.calibrated = 0;
}

