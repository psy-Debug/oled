# OLED Flight Controller — STM32F103 四旋翼飞控

> 四元数姿态解算 + 串级 PID 控制框架 | Cortex-M3 裸机

---

## 快速开始

1. **Keil 打开项目**：`test.uvprojx`
2. **include paths**：`hardware` 目录
3. **编译** → **烧录** → 上电执行校准 → 串口发指令起飞

---

## 硬件资源一览

| 外设 | 引脚 | 用途 |
|------|------|------|
| BMI088 (SPI1) | PA4(CS_ACC) PA15(CS_GYRO) PA5/PA6/PA7 | 加速度计 + 陀螺仪 |
| 加速度计 INT1 | PB0 (EXTI0) | 数据就绪中断 |
| 陀螺仪 INT3 | PB1 (EXTI1) | 数据就绪中断 |
| 电机 PWM (TIM2) | PA0/PA1/PA2/PA3 | 四路电调 10kHz PWM |
| OLED | 软 I2C | 状态显示 |
| USART1 | — | 遥控指令输入 |
| USART3 | — | 遥测输出 115200 |
| LED | PA8 | 心跳灯 0.25s 闪烁 |

---

## 目录结构

```
oled/
├── main.c                  ← 应用入口（初始化 + 主循环）
├── hardware/
│   ├── bmi088.c/h          ← BMI088 SPI 驱动
│   ├── exti.c/h            ← 外部中断（数据就绪 + 中断服务）
│   ├── sensor_initial_alignment.c/h ← 安装校准 + 四元数数学库
│   ├── attitude_quat.c/h   ← 四元数互补滤波姿态解算
│   ├── quat.c/h            ← 四元数基础运算
│   ├── flight_controller.c/h ← 飞控状态机 + 串级 PID
│   ├── motor_mixer.c/h     ← X 型电机混控
│   ├── motor_pwm.c/h       ← TIM2 PWM 输出
│   ├── pid.c/h             ← 通用 PID 算法
│   ├── qmath.c/h           ← 快速数学库
│   ├── OLED.c/h (+Data/Font) ← OLED 显示
│   ├── usart1.c/h          ← 遥控指令接收
│   ├── usart3.c/h          ← 遥测输出
│   └── led.c/h             ← 状态灯
├── interrupt_pin_mapping.txt ← BMI088 中断寄存器手册
└── test.uvprojx            ← Keil 工程
```

---

## 控制架构（串级 PID）

```
遥控杆量 / 指令
      │
      ▼
┌─────────────────────────────────────┐
│  外环：角度环（P）                    │
│  目标角速度 = (目标角 - 当前角) × 24.0 │
└─────────────────────────────────────┘
      │ 限幅
      ▼
┌─────────────────────────────────────┐
│  内环：角速度环（PI）                 │
│  P=0.003  I=0.0015  D=0（限幅±0.15）│
└─────────────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────┐
│  混控器 mixer_compute_x（X 型布局）   │
│  油门 + 横滚/俯仰/偏航 → 4 电机占空比 │
└─────────────────────────────────────┘
      │ [0,1]
      ▼
   TIM2 PWM (10kHz) → 电调
```

---

## 飞行模式

| 模式 | 指令 | 行为 |
|------|------|------|
| `FC_MODE_STOP` | `S` | 电机全停 |
| `FC_MODE_IDLE` | `D` | 四电机固定 5% 怠速 |
| `FC_MODE_ARMED` | `A` | 油门+姿态杆量直接混控输出（无 PID） |
| `FC_MODE_STABILIZE` | `H` | 串级 PID 自稳 |

**USART1 指令协议**：

| 字符 | 含义 |
|------|------|
| `A` | 解锁（ARMED） |
| `D` | 怠速（IDLE，油门清零） |
| `S` | 停止（STOP，全停） |
| `H` | 自稳（STABILIZE） |
| `T` + 数值 | 油门（值/100 → 0~1） |

---

## 主循环数据流

```
while(1)
 ├── 0.25s LED 心跳
 ├── 1. USART1 指令解析 → 切换模式/油门
 ├── 2. 加速度计数据就绪？ → 应用安装校准 → calib_ax/ay/az
 ├── 3. 陀螺仪数据就绪？
 │       ├── 计算真实 dt
 │       ├── 去零偏 (gyro_bias)
 │       ├── 一阶低通滤波 (alpha=0.05)
 │       ├── 应用安装校准 → calib_gx/gy/gz
 │       ├── att_update_quat() 四元数互补滤波 → 姿态角
 │       └── fc_update() 串级 PID → 混控 → Motor_SetDuty ×4
 └── 4. USART3 遥测：roll,pitch,yaw,pid输出
```

---

## 中断向量表

| ISR | 文件 | 功能 |
|-----|------|------|
| `EXTI0_IRQHandler` | `exti.c` | 加速度计 DRDY (PB0) → SPI 读数据 → `g_accel_data_ready=1` |
| `EXTI1_IRQHandler` | `exti.c` | 陀螺仪 DRDY (PB1) → SPI 读数据 → `g_gyro_data_ready=1` |
| `USART1_IRQHandler` | `usart1.c` | 遥控指令接收 → `usart1_cmd_ready=1` |

主循环轮询标志位消费数据，不在中断里做控制计算。

---

## 上电流程

```
上电
 ├── LED/串口/电机PWM/OLED 初始化
 ├── BMI088_Init()（失败→死循环提示 "BMI088 ERR!"）
 ├── EXTI 中断配置（PB0/PB1）
 ├── 预热：丢弃前 20 帧数据
 ├── perform_calibration()       ← 加速度计安装校准（20 样本）
 ├── perform_gyro_calibration()  ← 陀螺仪零偏校准（100 样本取平均）
 ├── fc_init(0.0005)             ← 默认 PID 参数
 ├── att_init_quat(0.0005, 0.92) ← 互补滤波系数
 └── fc_set_mode(FC_MODE_STOP)
```

---

## 关键参数

| 参数 | 值 | 位置 |
|------|----|------|
| 外环角度 P | 24.0 | `flight_controller.c` `fc_init()` |
| 内环速率 PID | P=0.003 I=0.0015 D=0 | 同上 |
| 内环输出限幅 | ±0.15 | 同上 |
| 最大倾斜角 | 30° | `max_angle` |
| 陀螺低通滤波 | α=0.05 | `main.c` |
| 互补滤波系数 | α=0.92 | `att_init_quat` |
| PWM 频率 | 10kHz (TIM2, 7200 分频) | `motor_pwm.h` |
| 加速度计量程 | ±24g (0.000732g/LSB) | `bmi088.h` |
| 陀螺仪量程 | ±2000dps (0.061°/s/LSB) | `bmi088.h` |
