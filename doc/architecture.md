# 架构文档 — OLED Flight Controller（四旋翼飞控）

> STM32F103 四元数姿态解算 + 串级 PID 飞控框架 | 版本 1.0

---

## 一、整体架构

```
┌────────────────────────────────────────────────────────────┐
│  main.c（应用层 / 主循环）                                   │
│  初始化 → 校准 → while(1){ 指令 → 姿态解算 → 控制 → 输出 }   │
│  只依赖: hardware/ 各模块头文件                              │
├────────────────────────────────────────────────────────────┤
│  hardware/（功能模块层，扁平结构）                            │
│                                                            │
│  ┌─ 感知链 ──────────────────────────────────────────┐     │
│  │ bmi088      SPI 读取加速度计+陀螺仪原始数据        │     │
│  │ exti        PB0/PB1 数据就绪中断 → 标志位通知      │     │
│  │ sensor_initial_alignment  安装校准 + 四元数数学库  │     │
│  │ attitude_quat  四元数互补滤波 → roll/pitch/yaw     │     │
│  └───────────────────────────────────────────────────┘     │
│  ┌─ 控制链 ──────────────────────────────────────────┐     │
│  │ flight_controller  状态机 + 串级PID（角度环+速率环）│     │
│  │ pid         通用 PID（积分限幅 + 输出限幅）         │     │
│  │ motor_mixer X 型混控：控制量 → 四电机占空比         │     │
│  │ motor_pwm   TIM2 四路 PWM → 电调                   │     │
│  └───────────────────────────────────────────────────┘     │
│  ┌─ 交互链 ──────────────────────────────────────────┐     │
│  │ usart1  遥控指令（A/D/S/H/T）                      │     │
│  │ usart3  遥测输出（姿态角 + PID 输出，115200）       │     │
│  │ OLED    校准提示 / 调试显示                         │     │
│  │ led     PA8 心跳灯                                  │     │
│  └───────────────────────────────────────────────────┘     │
├────────────────────────────────────────────────────────────┤
│  lib (Keil 工程自带: CMSIS + STM32F10x 标准外设库)           │
└────────────────────────────────────────────────────────────┘
```

### 数据流（单向 ↓）

```
BMI088 (SPI 寄存器)
   │ 中断置标志
   ▼
exti 标志位 ──→ main 轮询消费
   │
   ▼
校准(零偏+安装) ──→ 低通滤波 ──→ 四元数互补滤波
   │                              │
   ▼                              ▼
角速度 calib_gx/gy/gz          姿态角 roll/pitch/yaw
   │                              │
   └──────→ 串级 PID ──→ 混控 ──→ PWM ──→ 电机
```

---

## 二、模块文件清单

### 感知链

| 模块 | 文件 | 职责 |
|------|------|------|
| BMI088 驱动 | `hardware/bmi088.c .h` | SPI1 寄存器读写、量程换算（±24g / ±2000dps） |
| 中断 | `hardware/exti.c .h` | EXTI0(PB0)/EXTI1(PB1) 中断服务，数据就绪标志 |
| 安装校准 | `hardware/sensor_initial_alignment.c .h` | 加速度计校准、坐标系旋转对齐、四元数基础运算 |
| 姿态解算 | `hardware/attitude_quat.c .h` | 四元数互补滤波（α=0.92），输出欧拉角 |
| 四元数 | `hardware/quat.c .h` | 四元数乘法/归一化/转换 |

### 控制链

| 模块 | 文件 | 职责 |
|------|------|------|
| 飞控核心 | `hardware/flight_controller.c .h` | 模式状态机、串级 PID、目标角速度生成 |
| PID | `hardware/pid.c .h` | 标准位置式 PID（积分限幅、输出限幅） |
| 混控器 | `hardware/motor_mixer.c .h` | X 型布局混控、安全限幅 |
| PWM 输出 | `hardware/motor_pwm.c .h` | TIM2 CH1-4 (PA0-PA3)，10kHz PWM |
| 快速数学 | `hardware/qmath.c .h` | 查表三角函数 |

### 交互链

| 模块 | 文件 | 职责 |
|------|------|------|
| 指令串口 | `hardware/usart1.c .h` | 接收 `A/D/S/H/T` 指令 |
| 遥测串口 | `hardware/usart3.c .h` | printf 输出调试数据 |
| 显示 | `hardware/OLED.c .h` + `OLED_Data.c` + `OLED_Font.h` | 校准流程提示 |
| LED | `hardware/led.c .h` | PA8 心跳 |

---

## 三、核心数据结构

### 3.1 飞控模式（`flight_controller.h`）

```c
typedef enum {
    FC_MODE_STOP = 0,   // 停止：电机全停
    FC_MODE_IDLE,       // 怠速：固定 5% 油门
    FC_MODE_ARMED,      // 解锁：杆量直接混控（无 PID）
    FC_MODE_STABILIZE   // 自稳：串级 PID
} FCMode_t;

typedef struct {
    float throttle;     // 油门 [0,1]
    float roll;         // 横滚 [-1,1]
    float pitch;        // 俯仰 [-1,1]
    float yaw;          // 偏航 [-1,1]
    uint8_t arm_switch; // 解锁开关
} RCControl_t;

typedef struct {
    PID_t pid_roll, pid_pitch, pid_yaw;
    float max_angle;       // 最大倾斜角 30°
    float max_yaw_rate;    // 最大偏航角速度 100 dps
    float angle_p_gain;    // 角度环 P = 24.0
} FCConfig_t;
```

### 3.2 混控器（`motor_mixer.h`）

```c
typedef enum { MOTOR_FRONT, MOTOR_RIGHT, MOTOR_BACK, MOTOR_LEFT } MotorID_t;

typedef struct {
    float throttle;  // [0,1]
    float roll;      // [-1,1]
    float pitch;     // [-1,1]
    float yaw;       // [-1,1]
} ControlInput_t;

typedef struct {
    float motor[4];  // 四电机占空比 [0,1]
} MotorOutput_t;

MotorOutput_t mixer_compute_x(ControlInput_t input);  // X 型混控
```

### 3.3 姿态（`attitude_quat.h` / `sensor_initial_alignment.h`）

```c
typedef struct { float roll, pitch, yaw; } Attr;          // 欧拉角（度）
typedef struct { float w, x, y, z; } Quaternion_t;        // 四元数

void att_init_quat(float dt, float alpha);                // α=0.92
void att_update_quat(float gx, float gy, float gz,
                     float ax, float ay, float az, float dt);
Attr att_get_quat(void);
```

---

## 四、串级 PID 详解

```
遥控杆量 (roll/pitch [-1,1])
    │ × max_angle(30°)
    ▼
目标角度 ──→ [角度环 P=24.0] ──→ 目标角速度（限幅 ±720 dps）
                │
                ▼
目标角速度 ──→ [速率环 PI] ──→ 控制量（限幅 ±0.15）
                │      P=0.003  I=0.0015  D=0
                ▼
        混控器 X 布局
                │
                ▼
        四路 PWM 占空比
```

**为什么外环只用 P**：姿态角变化较慢，P 足够建立角速度指令；积分放内环消除持续扰动，D 项因陀螺仪噪声弃用（已靠 0.05 低通滤波抑制）。

---

## 五、传感器数据处理链

```
BMI088 原始值
   │ × BMI088_ACCEL_SCALE (24/32768)  或  × BMI088_GYRO_SCALE (2000/32768)
   ▼
物理单位 (g / dps)
   │ 陀螺仪：减去零偏 gyro_bias（上电 100 样本校准）
   ▼
一阶低通滤波  filter = filter×(1-0.05) + raw×0.05
   │
   ▼
SensorAlignment_ApplyGyro/Accel（安装坐标系 → 机体坐标系）
   │
   ▼
att_update_quat() 四元数互补滤波（陀螺积分 + 加速度计校正 α=0.92）
   │
   ▼
roll / pitch / yaw
```

---

## 六、校准流程

### 6.1 加速度计安装校准 `perform_calibration()`

1. OLED 提示 "Calibrating... Keep STILL!"
2. 静止采集 20 组加速度数据
3. `SensorAlignment_Calibrate()` 计算安装姿态四元数
4. 显示初始 roll/pitch（用于验证安装角度）

### 6.2 陀螺仪零偏校准 `perform_gyro_calibration()`

1. OLED 提示 "Calibrating Gyro Keep STILL !!!"
2. 采集 100 组陀螺仪数据取平均 → `gyro_bias_x/y/z`
3. 运行时每帧减去零偏

---

## 七、中断与主循环协作

| 角色 | 位置 | 内容 |
|------|------|------|
| 生产者 | `EXTI0_IRQHandler` | SPI 读加速度计 → `g_accel_data_ready=1` |
| 生产者 | `EXTI1_IRQHandler` | SPI 读陀螺仪 → `g_gyro_data_ready=1` |
| 生产者 | `USART1` 中断 | 指令帧 → `usart1_cmd_ready=1` |
| 消费者 | `main()` 主循环 | 轮询三个标志位，做校准/滤波/解算/PID/输出 |

**原则**：中断只搬运数据，所有计算在主循环完成。

---

## 八、USART1 指令协议

| 字符 | 动作 | 附加参数 |
|------|------|----------|
| `A` | 解锁 → `FC_MODE_ARMED` | — |
| `D` | 怠速 → `FC_MODE_IDLE` | 油门清零 |
| `S` | 停止 → `FC_MODE_STOP` | 油门清零 |
| `H` | 自稳 → `FC_MODE_STABILIZE` | — |
| `T` | 设置油门 | 值/100 → [0,1] |

解析函数：`USART1_CheckAndParse()` → `usart1_cmd_char` + `usart1_cmd_value`

---

## 九、已知限制与改进方向

| 项目 | 现状 | 改进方向 |
|------|------|----------|
| 偏航控制 | `target_yaw_rate = 0`（未接入杆量） | 接入 yaw 杆量 |
| 遥控输入 | 串口字符指令 | 接入无线遥控器/接收机 |
| D 项 | 弃用 | 加回 D 并配合 D 项滤波 |
| 姿态解算 | 互补滤波 | 可升级 Mahony/EKF |
| 电调协议 | PWM | 可升级 DShot |
