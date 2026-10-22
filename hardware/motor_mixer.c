#include "motor_mixer.h"

static MixerConfig_t mixer_config = {
    .throttle_idle = 0.05f,
    .min_throttle = 0.05f,
    .max_throttle = 0.95f
};

void mixer_init(MixerConfig_t *config)
{
    if (config != 0) {  // 修改：NULL -> 0
        mixer_config = *config;
    }
}

void mixer_set_config(MixerConfig_t config)
{
    mixer_config = config;
}

MixerConfig_t mixer_get_config(void)
{
    return mixer_config;
}

MotorOutput_t mixer_compute_x(ControlInput_t input)
{
    MotorOutput_t output;
    
    float throttle = input.throttle;
    if (throttle < 0) throttle = 0;
    if (throttle > 1) throttle = 1;
    
    float roll = input.roll;
    if (roll > 1) roll = 1;
    if (roll < -1) roll = -1;
    
    float pitch = input.pitch;
    if (pitch > 1) pitch = 1;
    if (pitch < -1) pitch = -1;
    
    float yaw = input.yaw;
    if (yaw > 1) yaw = 1;
    if (yaw < -1) yaw = -1;
    
    // X型混控矩阵
    output.motor[MOTOR_FRONT] =throttle + roll - pitch;//+yaw ;//左前逆时针
    output.motor[MOTOR_RIGHT] =throttle + roll + pitch;// -yaw;//左后
    output.motor[MOTOR_BACK]  =throttle - roll + pitch;//+yaw;//右后
    output.motor[MOTOR_LEFT]  =throttle - roll - pitch ;//-yaw;//右前顺时针
    
    return output;
}

MotorOutput_t mixer_compute_plus(ControlInput_t input)
{
    MotorOutput_t output;
    
    float throttle = input.throttle;
    if (throttle < 0) throttle = 0;
    if (throttle > 1) throttle = 1;
    
    float roll = input.roll;
    float pitch = input.pitch;
    float yaw = input.yaw;
    
    // +型布局
    output.motor[MOTOR_FRONT] = throttle + pitch + yaw;
    output.motor[MOTOR_RIGHT] = throttle + roll - yaw;
    output.motor[MOTOR_BACK]  = throttle - pitch + yaw;
    output.motor[MOTOR_LEFT]  = throttle - roll - yaw;
    
    return output;
}

void mixer_apply_safety(MotorOutput_t *output, float throttle)
{
    float max_motor = -9999;
    int i;
    
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (output->motor[i] > max_motor) max_motor = output->motor[i];
    }
    
    // 防饱和处理
    if (max_motor > 1.0f) {
        float scale = 1.0f / max_motor;
        for (i = 0; i < MOTOR_COUNT; i++) {
            output->motor[i] *= scale;
        }
    }
    
    // 怠速处理
    if (throttle > mixer_config.throttle_idle) {
        for (i = 0; i < MOTOR_COUNT; i++) {
            if (output->motor[i] < mixer_config.min_throttle) {
                output->motor[i] = mixer_config.min_throttle;
            }
        }
    } else {
        for (i = 0; i < MOTOR_COUNT; i++) {
            output->motor[i] = mixer_config.throttle_idle;
        }
    }
    
    // 最终限幅
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (output->motor[i] > mixer_config.max_throttle) {
            output->motor[i] = mixer_config.max_throttle;
        }
        if (output->motor[i] < 0) {
            output->motor[i] = 0;
        }
    }
}
