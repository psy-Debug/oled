#ifndef QUAT_H
#define QUAT_H

#include <math.h>

// 四元数结构体
typedef struct {
    float w, x, y, z;
} Quat;

// 从两个向量计算旋转四元数（将向量a转到向量b）
// 输入: a, b 必须是单位向量
Quat quat_from_two_vec(float ax, float ay, float az, float bx, float by, float bz);

// 用四元数旋转一个向量
// 输入: q 旋转四元数, v_in 原向量, v_out 旋转后的向量
void quat_rotate_vec(Quat q, float* v_in, float* v_out);

#endif


