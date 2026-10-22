#include "quat.h"

// 从两个向量计算旋转四元数（将向量a转到向量b）
// 本质：找到a到b的最短路径旋转
Quat quat_from_two_vec(float ax, float ay, float az, float bx, float by, float bz) {
    Quat q;
    
    // 步骤1：计算旋转轴 = a × b（叉积）
    float cx = ay * bz - az * by;
    float cy = az * bx - ax * bz;
    float cz = ax * by - ay * bx;
    
    // 步骤2：计算旋转角度的cos值 = a·b（点积）
    float dot = ax*bx + ay*by + az*bz;
    
    // 步骤3：求旋转轴的模长
    float axis_len = sqrtf(cx*cx + cy*cy + cz*cz);
    
    // 步骤4：根据两向量关系分情况处理
    if (axis_len < 1e-6f) {
        // 情况1：两向量平行
        if (dot > 0) {
            // 同向 → 不旋转
            q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
        } else {
            // 反向 → 旋转180度（绕X轴）
            q.w = 0.0f; q.x = 1.0f; q.y = 0.0f; q.z = 0.0f;
        }
    } else {
        // 情况2：两向量不平行 → 正常计算旋转
        
        // 归一化旋转轴
        cx /= axis_len;
        cy /= axis_len;
        cz /= axis_len;
        
        // 从点积得到旋转角度（弧度）
        float angle = acosf(dot);
        float half_angle = angle * 0.5f;
        float sin_half = sinf(half_angle);
        
        // 四元数 = [cos(半角), sin(半角)*轴]
        q.w = cosf(half_angle);
        q.x = cx * sin_half;
        q.y = cy * sin_half;
        q.z = cz * sin_half;
    }
    
    return q;
}

// 用四元数旋转一个向量
// 公式: v' = q * v * q^(-1)
// 这是四元数旋转的标准公式
void quat_rotate_vec(Quat q, float* v_in, float* v_out) {
    float x = v_in[0], y = v_in[1], z = v_in[2];
    float w = q.w, qx = q.x, qy = q.y, qz = q.z;
    
    // 预计算一些常用项（优化性能）
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
    
    // 旋转后的新向量 = 旋转矩阵 × 原向量
    v_out[0] = (w2 + x2 - y2 - z2) * x + 2.0f * (xy - wz) * y + 2.0f * (xz + wy) * z;
    v_out[1] = 2.0f * (xy + wz) * x + (w2 - x2 + y2 - z2) * y + 2.0f * (yz - wx) * z;
    v_out[2] = 2.0f * (xz - wy) * x + 2.0f * (yz + wx) * y + (w2 - x2 - y2 + z2) * z;
}


