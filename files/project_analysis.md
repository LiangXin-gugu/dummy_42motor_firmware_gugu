# dummy-42motor-fw 工程分析：功能、运行时序与指令 Pipeline

本文档分析 `dummy-42motor-fw` 步进电机固件的功能定位、运行时序、控制逻辑和指令接收执行的完整 pipeline，并标注对应代码位置。

## 1. 工程概览

`dummy-42motor-fw` 是基于 **STM32F103CB** 的 42 系列步进电机 FOC 闭环控制固件。它是 DummyRobot 六轴机械臂中每个关节电机的从端控制器，接收主控（`dummy-ref-core-fw`）通过 CAN 总线下发的指令，驱动 TB67H450 H 桥驱动器控制步进电机，并通过 MT6816 磁编码器实现位置闭环。

### 硬件配置

| 组件 | 芯片/型号 | 接口 | 用途 |
| --- | --- | --- | --- |
| MCU | STM32F103CB | — | 主控芯片，Cortex-M3，72MHz |
| 电机驱动 | TB67H450 | GPIO + PWM + DAC | H 桥驱动，FOC 电流矢量输出 |
| 编码器 | MT6816 | SPI | 单圈绝对磁编码器，14 位分辨率 |
| 通信 | CAN1 | PB8/PB9 | 与主控通信，接收运动指令 |
| 调试 | USART1 | PB6/PB7 | UART 调试口，115200 波特率 |
| 拨码开关 | 3-bit | PA8/PA9/PA10 | 设置 CAN 节点 ID (0~7) |
| 按钮 | 2 个 | GPIO | Button1：模式切换/校准触发；Button2：清零/清堵转 |
| LED | 状态灯 | GPIO | 显示电机运行状态 |

### 软件分层架构

```text
UserApp/
  ├── main.cpp             ← 入口函数、定时器回调、按钮事件
  ├── configurations.h     ← BoardConfig_t 配置结构体定义
  ├── common_inc.h         ← 统一头文件
  └── protocols/
      ├── interface_can.cpp  ← CAN 指令解析与响应
      └── interface_uart.cpp ← UART 指令解析

Ctrl/
  ├── Motor/
  │   ├── motor.h / motor.cpp          ← 电机闭环控制核心（Controller 类）
  │   └── motion_planner.h / .cpp      ← 运动规划器（位置/速度/电流/轨迹跟踪器）
  ├── Driver/
  │   ├── driver_base.h                ← 驱动器抽象基类
  │   └── tb67h450_base.h / .cpp       ← TB67H450 驱动实现
  └── Sensor/Encoder/
      ├── encoder_base.h               ← 编码器抽象基类
      ├── mt6816_base.h / .cpp         ← MT6816 编码器实现
      └── encoder_calibrator_base.h / .cpp  ← 编码器校准器

Port/                        ← 平台适配层（STM32 具体实现）
  ├── tb67h450_stm32.*      ← TB67H450 STM32 平台实现
  ├── mt6816_stm32.*        ← MT6816 STM32 平台实现
  ├── encoder_calibrator_stm32.*
  ├── button_stm32.* / led_stm32.*
  └── Platform/
      ├── Memory/            ← EEPROM 仿真（Flash 持久化）
      ├── Utils/             ← 硬件工具函数
      └── retarget.c         ← printf 重定向到 UART

Core/                        ← HAL 层（CubeMX 生成）
  ├── Src/main.c             ← main() 入口，外设初始化
  ├── Src/can.c              ← CAN 初始化与接收回调
  ├── Src/usart.c            ← UART 初始化与 DMA 接收
  ├── Src/stm32f1xx_it.c     ← 中断向量，转发 TIM1/TIM4/CAN/UART 中断
  └── Src/tim.c              ← 定时器初始化
```

## 2. 启动流程

### 2.1 硬件初始化

入口在 `Core/Src/main.c`：

```text
main()
  ├── HAL_Init()
  ├── SystemClock_Config()        ← HSE 8MHz × PLL6 = 48MHz
  ├── 外设初始化:
  │   ├── MX_GPIO_Init()
  │   ├── MX_DMA_Init()
  │   ├── MX_ADC1_Init()
  │   ├── MX_USART1_UART_Init()   ← UART1 初始化 + DMA 接收 + IDLE 中断
  │   ├── MX_CAN_Init()           ← CAN 初始化 + 过滤器 + 中断使能
  │   ├── MX_SPI1_Init()
  │   ├── MX_TIM2_Init() / MX_TIM3_Init() / MX_TIM4_Init() / MX_TIM1_Init()
  │   └── (调用) Main()
  └── while(1) {}  ← 不会到达
```

定义位置：`dummy-42motor-fw/Core/Src/main.c`

### 2.2 Main() 初始化流程

`Main()` 是 C++ 用户代码入口：

```text
Main()
  ├── 读取拨码开关 → 计算 nodeID (0~7)
  ├── 从 EEPROM 加载 boardConfig
  │   └── 若 configStatus != CONFIG_OK，写入默认配置
  ├── boardConfig.canNodeId = nodeID (拨码开关优先)
  ├── 将 boardConfig 各参数注入 motor.config
  ├── motor.AttachDriver(&tb67H450)
  ├── motor.AttachEncoder(&mt6816)
  ├── motor.controller->Init()
  ├── motor.driver->Init()
  ├── motor.encoder->Init()
  ├── button1/button2 绑定事件回调
  ├── HAL_Delay(100)
  ├── HAL_TIM_Base_Start_IT(&htim1)   ← 启动 100Hz 定时器
  ├── HAL_TIM_Base_Start_IT(&htim4)   ← 启动 20kHz 定时器
  └── for(;;) 主循环:
      ├── encoderCalibrator.TickMainLoop()  ← 校准计算（非实时）
      └── 检查 configStatus → 写 EEPROM 或重启
```

定义位置：`dummy-42motor-fw/UserApp/main.cpp`

## 3. 运行时序

系统不使用 RTOS，纯裸机前后台架构。由两个硬件定时器中断驱动核心逻辑。

### 3.1 定时器中断时序

| 中断源 | 频率 | 周期 | 回调函数 | 位置 | 职责 |
| --- | --- | --- | --- | --- | --- |
| TIM1_UP_IRQn | 100Hz | 10ms | `Tim1Callback100Hz()` | main.cpp | 低速任务：按钮去抖、状态 LED、温度采样 |
| TIM4_IRQHandler | 20kHz | 50µs | `Tim4Callback20kHz()` | main.cpp | 高速任务：编码器校准 或 电机闭环控制 |
| CAN RX FIFO0 | 事件触发 | — | `HAL_CAN_RxFifo0MsgPendingCallback()` | can.c | 接收并处理 CAN 指令 |
| USART1 IDLE | 事件触发 | — | `USART1_IRQHandler()` → `OnRecvEnd()` | stm32f1xx_it.c | 接收并处理 UART 指令 |

中断转发关系（`stm32f1xx_it.c`）：

```text
TIM1_UP_IRQHandler() → Tim1Callback100Hz() → return  (绕过 HAL_TIM_IRQHandler)
TIM4_IRQHandler()   → Tim4Callback20kHz() → return  (绕过 HAL_TIM_IRQHandler)
USART1_IRQHandler() → IDLE 检测 → OnRecvEnd() → HAL_UART_IRQHandler()
CAN RX0 IRQ         → HAL_CAN_IRQHandler() → HAL_CAN_RxFifo0MsgPendingCallback()
```

定义位置：`dummy-42motor-fw/Core/Src/stm32f1xx_it.c`

### 3.2 TIM1 100Hz 回调

```cpp
void Tim1Callback100Hz()
{
    button1.Tick(10);        // 按钮去抖，10ms 间隔
    button2.Tick(10);
    statusLed.Tick(10, motor.controller->state);  // 状态 LED 刷新

    // 温度采样，每 100 次 = 1 秒采集一次
    if (boardConfig.enableTempWatch)
    {
        count++;
        if (count >= 100)
        {
            boardConfig.motor_temperature = AdcGetChipTemperature();
            count = 0;
        }
    }
}
```

定义位置：`dummy-42motor-fw/UserApp/main.cpp`

### 3.3 TIM4 20kHz 回调（核心控制循环）

```cpp
void Tim4Callback20kHz()
{
    if (encoderCalibrator.isTriggered)
        encoderCalibrator.Tick20kHz();   // 编码器校准模式
    else
        motor.Tick20kHz();               // 正常闭环控制模式
}
```

两者互斥：校准期间不执行电机控制，校准完成后重启。

定义位置：`dummy-42motor-fw/UserApp/main.cpp`

### 3.4 主循环

```cpp
for (;;)
{
    encoderCalibrator.TickMainLoop();  // 校准数据处理与 Flash 写入

    if (boardConfig.configStatus == CONFIG_COMMIT)
    {
        boardConfig.configStatus = CONFIG_OK;
        eeprom.put(0, boardConfig);    // 配置变更后写入 EEPROM
    }
    else if (boardConfig.configStatus == CONFIG_RESTORE)
    {
        eeprom.put(0, boardConfig);     // 恢复默认配置后重启
        HAL_NVIC_SystemReset();
    }
}
```

定义位置：`dummy-42motor-fw/UserApp/main.cpp`

## 4. 电机闭环控制逻辑

### 4.1 Tick20kHz 主流程

`motor.Tick20kHz()` 每次被调用时执行：

```text
Tick20kHz()
  ├── 1. encoder->UpdateAngle()          ← SPI 读取 MT6816 角度
  └── 2. CloseLoopControlTick()         ← 闭环控制核心
```

定义位置：`dummy-42motor-fw/Ctrl/Motor/motor.cpp`

### 4.2 CloseLoopControlTick 详细流程

```text
CloseLoopControlTick()
  │
  ├── 首次调用: 用编码器角度初始化 realPosition
  │
  ├── 更新数据:
  │   ├── 读取编码器 rectifiedAngle → realLapPosition
  │   ├── 计算 deltaLapPosition (处理跨零)
  │   └── 累加 realPosition += deltaLapPosition  (多圈位置)
  │
  ├── 估计数据:
  │   ├── 速度估计 estVelocity (积分滤波器)
  │   ├── 超前角补偿 estLeadPosition (速度→相位补偿)
  │   ├── 估计位置 estPosition = realPosition + estLeadPosition
  │   └── 误差 estError = softPosition - estPosition
  │
  ├── 控制环输出 (根据 modeRunning):
  │   ├── MODE_STOP / 堵转 / 失能 / 未校验 → driver->Sleep()
  │   ├── MODE_COMMAND_POSITION / Trajectory / STEP_DIR → CalcDceToOutput()
  │   ├── MODE_COMMAND_VELOCITY → CalcPidToOutput()
  │   └── MODE_COMMAND_CURRENT → CalcCurrentToOutput()
  │
  ├── 模式切换处理: requestMode → modeRunning
  │
  ├── 目标值限幅: goalVelocity / goalCurrent 不超过额定值
  │
  ├── 运动规划:
  │   ├── 新曲线检测 (softNewCurve / 模式切换 / disable→enable)
  │   └── 按模式调用对应 Tracker.NewTask()
  │
  ├── 软目标更新:
  │   ├── POSITION → positionTracker.CalcSoftGoal(goalPosition)
  │   ├── VELOCITY → velocityTracker.CalcSoftGoal(goalVelocity)
  │   ├── CURRENT → currentTracker.CalcSoftGoal(goalCurrent)
  │   ├── Trajectory → trajectoryTracker.CalcSoftGoal(goalPos, goalVel)
  │   └── STEP_DIR → positionInterpolator.CalcSoftGoal(goalPosition)
  │
  └── 状态检测与更新:
      ├── 堵转检测 (stallProtectSwitch, 电流满载+速度接近0持续1秒)
      ├── 过载检测 (非电流模式下电流满载持续1秒)
      └── 状态更新: NO_CALIB / STOP / STALL / OVERLOAD / FINISH / RUNNING
```

定义位置：`dummy-42motor-fw/Ctrl/Motor/motor.cpp`

### 4.3 控制算法

系统有三种控制算法，根据模式选择：

#### DCE（双环控制估计器）— 位置/轨迹/步进模式

```cpp
void CalcDceToOutput(int32_t _location, int32_t _speed)
{
    // 位置误差 + 速度误差 → DCE PID → 输出电流
    config->dce.pError = _location - estPosition;  // 位置误差
    config->dce.vError = (_speed - estVelocity) >> 7;  // 速度误差
    // kp * pError + ki * 积分(pError) + kv * 积分(vError) + kd * vError
    config->dce.output = (outputKp + outputKi + outputKd) >> 10;
    CalcCurrentToOutput(config->dce.output);
}
```

#### PID — 速度模式

```cpp
void CalcPidToOutput(int32_t _speed)
{
    config->pid.vError = _speed - estVelocity;  // 速度误差
    // kp * vError + ki * 积分(vError) + kd * d(vError)
    config->pid.output = (outputKp + outputKi + outputKd) >> 10;
    CalcCurrentToOutput(config->pid.output);
}
```

#### 直接电流 — 电流模式

```cpp
void CalcCurrentToOutput(int32_t current)
{
    focCurrent = current;
    // 正向电流 → 超前 90° (一个 SOFT_DIVIDE_NUM)；反向 → 滞后 90°
    focPosition = estPosition + (current > 0 ? SOFT_DIVIDE_NUM : -SOFT_DIVIDE_NUM);
    driver->SetFocCurrentVector(focPosition, focCurrent);
}
```

定义位置：`dummy-42motor-fw/Ctrl/Motor/motor.cpp`

### 4.4 运动规划器

`MotionPlanner` 包含 4 个跟踪器，对应不同模式的轨迹规划：

| 跟踪器 | 适用模式 | 功能 |
| --- | --- | --- |
| `PositionTracker` | COMMAND_POSITION / PWM_POSITION | 梯形速度曲线位置跟踪 |
| `VelocityTracker` | COMMAND_VELOCITY / PWM_VELOCITY | 加速度限速的速度跟踪 |
| `CurrentTracker` | COMMAND_CURRENT / PWM_CURRENT | 电流斜率限制 |
| `TrajectoryTracker` | COMMAND_Trajectory | 位置+速度双目标轨迹跟踪 |
| `PositionInterpolator` | STEP_DIR | 步进/方向信号插值 |

定义位置：`dummy-42motor-fw/Ctrl/Motor/motion_planner.h`

### 4.5 关键常量

```text
MOTOR_ONE_CIRCLE_HARD_STEPS = 200        (1.8° 步进电机一圈的整步数)
SOFT_DIVIDE_NUM = 256                     (微步细分数)
MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = 51200  (一圈的微步总数 = 200 × 256)
CONTROL_FREQUENCY = 20000 Hz              (控制环频率)
CONTROL_PERIOD = 50 µs                   (控制环周期)
```

定义位置：`dummy-42motor-fw/Ctrl/Motor/motor.h`、`motion_planner.h`

## 5. 控制模式

```cpp
typedef enum
{
    MODE_STOP,                 // 电机停止，驱动器 Sleep
    MODE_COMMAND_POSITION,     // 位置控制 (DCE)
    MODE_COMMAND_VELOCITY,     // 速度控制 (PID)
    MODE_COMMAND_CURRENT,      // 电流控制 (直接)
    MODE_COMMAND_Trajectory,   // 轨迹跟踪 (DCE + 位置速度双目标)
    MODE_PWM_POSITION,         // PWM 位置控制 (同 COMMAND_POSITION)
    MODE_PWM_VELOCITY,         // PWM 速度控制
    MODE_PWM_CURRENT,          // PWM 电流控制
    MODE_STEP_DIR,            // 步进/方向外部控制
} Mode_t;
```

定义位置：`dummy-42motor-fw/Ctrl/Motor/motor.h`

### 状态机

```cpp
typedef enum
{
    STATE_STOP,      // 停止
    STATE_FINISH,    // 到达目标
    STATE_RUNNING,   // 运动中
    STATE_OVERLOAD,  // 过载
    STATE_STALL,     // 堵转
    STATE_NO_CALIB   // 未校准
} State_t;
```

## 6. CAN 指令 Pipeline

### 6.1 CAN ID 编码格式

```text
StdId = (nodeID << 7) | cmd
         4 bits ID    7 bits command

nodeID: 0 = 广播（所有节点响应），1~7 = 对应拨码开关设置的节点
cmd:    0x00~0x7F，见下表
```

### 6.2 CAN 接收流程

```text
CAN 总线收到帧
  └── HAL_CAN_RxFifo0MsgPendingCallback()     [can.c]
        ├── HAL_CAN_GetRxMessage() → 获取 RxHeader + RxData[8]
        ├── 解析: id = StdId >> 7, cmd = StdId & 0x7F
        ├── 判断: id == 0 (广播) 或 id == boardConfig.canNodeId (自身)
        └── OnCanCmd(cmd, RxData, DLC)         [interface_can.cpp]
```

定义位置：`dummy-42motor-fw/Core/Src/can.c`、`dummy-42motor-fw/UserApp/protocols/interface_can.cpp`

### 6.3 CAN 指令表

#### 即时指令（0x00~0x0F，不写 EEPROM）

| cmd | 功能 | 数据格式 | 触发流程 |
| --- | --- | --- | --- |
| 0x01 | 使能/失能电机 | data[0..3]=uint32 (1=使能速度模式, 0=停止) | `requestMode = MODE_COMMAND_VELOCITY` 或 `MODE_STOP` |
| 0x02 | 触发编码器校准 | 无 | `encoderCalibrator.isTriggered = true` |
| 0x03 | 设置电流目标 | data[0..3]=float (A) | 切换 `MODE_COMMAND_CURRENT`，`SetCurrentSetPoint(A × 1000)` |
| 0x04 | 设置速度目标 | data[0..3]=float (r/s) | 切换 `MODE_COMMAND_VELOCITY`，`SetVelocitySetPoint(r/s × 51200)` |
| 0x05 | 设置位置目标 | data[0..3]=float (圈), data[4]=是否需要 ACK | 切换 `MODE_COMMAND_POSITION`，`SetPositionSetPoint(圈 × 51200)` |
| 0x06 | 带时间限速设位 | data[0..3]=float (圈), data[4..7]=float (时间s) | `SetPositionSetPointWithTime()`，计算可达最大速度 |
| 0x07 | 带速度限速设位 | data[0..3]=float (圈), data[4..7]=float (速度 r/s) | 设置 `ratedVelocity`，`SetPositionSetPoint()`，**总是返回 ACK** |

对于 0x05/0x06/0x07，如果 `data[4]` 非零（或 0x07 总是），会通过 CAN 回复当前位置和完成状态：

```cpp
tmpF = motor.controller->GetPosition();  // 返回圈数 (float)
// 回复帧: StdId = (nodeID << 7) | 0x23, data[0..3]=float 位置, data[4]=是否完成
```

#### 配置指令（0x10~0x1F，写 EEPROM）

| cmd | 功能 | 数据格式 | 存储字段 |
| --- | --- | --- | --- |
| 0x11 | 设置节点 ID | data[0..3]=uint32 | `boardConfig.canNodeId` |
| 0x12 | 设置电流限制 | data[0..3]=float (A) | `ratedCurrent = A × 1000`, `boardConfig.currentLimit` |
| 0x13 | 设置速度限制 | data[0..3]=float (r/s) | `ratedVelocity = r/s × 51200`, `boardConfig.velocityLimit` |
| 0x14 | 设置加速度 | data[0..3]=float (r/s²) | `ratedVelocityAcc`, `boardConfig.velocityAcc` |
| 0x15 | 应用当前位置为 Home | 无 | `ApplyPosAsHomeOffset()` → `boardConfig.encoderHomeOffset` |
| 0x16 | 设置上电自启 | data[0..3]=uint32 (0/1) | `boardConfig.enableMotorOnBoot` |
| 0x17 | 设置 DCE Kp | data[0..3]=int32 | `boardConfig.dce_kp` |
| 0x18 | 设置 DCE Kv | data[0..3]=int32 | `boardConfig.dce_kv` |
| 0x19 | 设置 DCE Ki | data[0..3]=int32 | `boardConfig.dce_ki` |
| 0x1A | 设置 DCE Kd | data[0..3]=int32 | `boardConfig.dce_kd` |
| 0x1B | 使能堵转保护 | data[0..3]=uint32 (0/1) | `boardConfig.enableStallProtect` |

`data[4]` 非零时设置 `configStatus = CONFIG_COMMIT`，主循环会写入 EEPROM。

#### 查询指令（0x20~0x2F）

| cmd | 功能 | 回复帧 StdId | 回复数据 |
| --- | --- | --- | --- |
| 0x21 | 查询电流 | `(nodeID << 7) \| 0x21` | data[0..3]=float 电流(A), data[4]=是否完成 |
| 0x22 | 查询速度 | `(nodeID << 7) \| 0x22` | data[0..3]=float 速度(r/s), data[4]=是否完成 |
| 0x23 | 查询位置 | `(nodeID << 7) \| 0x23` | data[0..3]=float 位置(圈), data[4]=是否完成 |
| 0x24 | 查询 Home 偏移 | `(nodeID << 7) \| 0x24` | data[0..3]=int32 encoderHomeOffset |
| 0x25 | 查询温度 | `(nodeID << 7) \| 0x25` | data[0..3]=float 温度 |

#### 特殊指令

| cmd | 功能 |
| --- | --- |
| 0x7D | 使能温度监测 |
| 0x7E | 擦除配置（恢复默认，`configStatus = CONFIG_RESTORE`，重启） |
| 0x7F | 重启 (`HAL_NVIC_SystemReset()`) |

定义位置：`dummy-42motor-fw/UserApp/protocols/interface_can.cpp`

### 6.4 位置查询与完成状态

主控查询位置（CAN 0x23）时，回复帧格式：

```text
回复 StdId = (nodeID << 7) | 0x23
data[0..3] = float GetPosition() = (realPosition - encoderHomeOffset) / 51200  (圈数)
data[4]    = (state == STATE_FINISH) ? 1 : 0
```

主控通过 `data[4]` 判断电机是否到达目标位置。

## 7. UART 指令 Pipeline

### 7.1 UART 接收流程

```text
USART1 DMA 接收 (128 字节缓冲区)
  └── USART1_IRQHandler() [stm32f1xx_it.c]
        ├── 检测 IDLE 标志 (一帧数据结束)
        ├── HAL_UART_DMAStop() → 停止 DMA
        ├── rxLen = BUFFER_SIZE - DMA剩余计数
        ├── OnRecvEnd(rx_buffer, rxLen) → OnUartCmd() [interface_uart.cpp]
        └── 重新启动 DMA 接收
```

定义位置：`dummy-42motor-fw/Core/Src/stm32f1xx_it.c`、`dummy-42motor-fw/Core/Src/usart.c`

`OnRecvEnd` 在 `usart.c` 中通过 `Uart_SetRxCpltCallBack(OnUartCmd)` 绑定。

### 7.2 UART 指令表

UART 指令为简单文本格式，首字符决定命令类型：

| 格式 | 功能 | 触发流程 |
| --- | --- | --- |
| `c<value>` | 设置电流 | 切换 `MODE_COMMAND_CURRENT`，`SetCurrentSetPoint(value × 1000)` |
| `v<value>` | 设置速度 | 切换 `MODE_COMMAND_VELOCITY`，`SetVelocitySetPoint(value × 51200)` |
| `p<value>` | 设置位置 | 切换 `MODE_COMMAND_POSITION`，`SetPositionSetPoint(value × 51200)` |

示例：`p1.5` 表示设置目标位置为 1.5 圈。

`value` 单位说明：
- `c`：安培 (A)，内部乘 1000 转为 mA
- `v`：圈/秒 (r/s)，内部乘 51200 转为微步/秒
- `p`：圈数 (turns)，内部乘 51200 转为微步

定义位置：`dummy-42motor-fw/UserApp/protocols/interface_uart.cpp`

### 7.3 printf 输出

`printf` 通过 retarget.c 重定向到 USART1：

```cpp
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(gHuart, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}
```

注意：UART 的 `_write` 使用**阻塞式** `HAL_UART_Transmit`（非 DMA），与 CAN 中断处理可能冲突。

定义位置：`dummy-42motor-fw/Port/Platform/retarget.c`

## 8. 编码器校准流程

### 8.1 触发方式

| 触发源 | 条件 |
| --- | --- |
| Button1 长按 | `OnButton1Event(LONG_PRESS)` → `encoderCalibrator.isTriggered = true` |
| CAN 命令 0x02 | `OnCanCmd(0x02)` → `encoderCalibrator.isTriggered = true` |
| 上电时双键按下 | `button1.IsPressed() && button2.IsPressed()` |

### 8.2 校准状态机

校准在 `Tick20kHz()` 中执行（替代电机控制），状态流转：

```text
CALI_DISABLE
  └── isTriggered → CALI_FORWARD_PREPARE

CALI_FORWARD_PREPARE
  └── 加速旋转一圈 → CALI_FORWARD_MEASURE

CALI_FORWARD_MEASURE
  └── 正向匀速旋转一圈，每个硬步采 16 个点取平均
  └── 到达终点 → CALI_BACKWARD_RETURN

CALI_BACKWARD_RETURN
  └── 继续前进 20 个 SOFT_DIVIDE_NUM 间距（消除间隙）→ CALI_BACKWARD_GAP_DISMISS

CALI_BACKWARD_GAP_DISMISS
  └── 反向退回消除间隙的 20 个间距 → CALI_BACKWARD_MEASURE

CALI_BACKWARD_MEASURE
  └── 反向匀速旋转一圈，采样
  └── 到达起点 → CALI_CALCULATING

CALI_CALCULATING
  └── TickMainLoop() 中执行:
      ├── CalibrationDataCheck() → 验证数据连续性和方向
      ├── 生成 16384 (51200/200×256→实际 51200) 项查表数据
      ├── WriteFlash16bitsAppend() → 写入 Flash 校准表
      └── HAL_NVIC_SystemReset() → 重启
```

定义位置：`dummy-42motor-fw/Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp`

### 8.3 校准表的作用

校准表 `quickCaliDataPtr[16384]` 存储在 Flash 中，MT6816 读取角度时：

```cpp
// mt6816_base.cpp
angleData.rectifiedAngle = quickCaliDataPtr[angleData.rawAngle];
```

将编码器原始角度（0~16383）映射为校准后的电角度，消除安装偏心和齿槽效应。

## 9. 按钮事件

| 按钮 | 事件 | 动作 |
| --- | --- | --- |
| Button1 | CLICK | 切换电机 STOP ↔ 运行模式 |
| Button1 | LONG_PRESS | 触发编码器校准 |
| Button2 | CLICK | 清除堵转标志 (`ClearStallFlag()`) |
| Button2 | LONG_PRESS | 根据当前模式将目标值清零 |

定义位置：`dummy-42motor-fw/UserApp/main.cpp`

## 10. EEPROM 配置持久化

### 10.1 配置结构体

```cpp
typedef struct Config_t
{
    configStatus_t configStatus;   // 配置状态标志
    uint32_t canNodeId;            // CAN 节点 ID
    int32_t encoderHomeOffset;     // 编码器零点偏移
    uint32_t defaultMode;          // 默认运行模式
    int32_t currentLimit;          // 电流限制 (mA)
    int32_t velocityLimit;         // 速度限制 (微步/秒)
    int32_t velocityAcc;           // 加速度 (微步/秒²)
    int32_t calibrationCurrent;    // 校准电流 (mA)
    int32_t dce_kp, dce_kv, dce_ki, dce_kd;  // DCE 参数
    float motor_temperature;       // 电机温度
    bool enableMotorOnBoot;        // 上电自启
    bool enableStallProtect;       // 堵转保护
    bool enableTempWatch;          // 温度监测
} BoardConfig_t;
```

定义位置：`dummy-42motor-fw/UserApp/configurations.h`

### 10.2 配置状态流转

```text
CONFIG_RESTORE (0) → 恢复默认值，写 EEPROM，重启
CONFIG_OK (1)      → 正常运行
CONFIG_COMMIT (2)  → 配置已修改，主循环写入 EEPROM
```

写入优先级：EEPROM > Motor.h 默认值。

## 11. 完整指令执行路径示例

### 示例 1：CAN 设置位置（主控最常用路径）

主控发送 CAN 帧：`StdId = (nodeID << 7) | 0x07, data = [float 位置, float 速度]`

```text
CAN 总线接收
  └── HAL_CAN_RxFifo0MsgPendingCallback()     [can.c]
        ├── 解析 id 和 cmd
        └── OnCanCmd(0x07, RxData, 8)         [interface_can.cpp]
              ├── 切换 MODE_COMMAND_POSITION
              ├── 设置 ratedVelocity = 速度 × 51200
              ├── SetPositionSetPoint(位置 × 51200)
              │     └── goalPosition = _pos + encoderHomeOffset
              ├── 立即回复 ACK:
              │   tmpF = GetPosition()
              │   CAN_Send: StdId=(nodeID<<7)|0x23, data=[float, finish_flag]
              │
              └── (20kHz 中断中)
                    Tim4Callback20kHz()       [main.cpp]
                      └── motor.Tick20kHz()   [motor.cpp]
                            ├── encoder->UpdateAngle() → 读 MT6816
                            └── CloseLoopControlTick()
                                  ├── 更新 realPosition
                                  ├── 估计 estVelocity / estPosition
                                  ├── CalcDceToOutput(softPosition, softVelocity)
                                  │   └── DCE PID → 输出电流
                                  │       └── CalcCurrentToOutput()
                                  │           └── driver->SetFocCurrentVector(focPosition, focCurrent)
                                  │               └── TB67H450 输出 DAC + PWM
                                  ├── positionTracker.CalcSoftGoal(goalPosition)
                                  │   └── 梯形速度曲线规划 → softPosition, softVelocity
                                  └── 状态更新: RUNNING → FINISH (到达目标)
```

### 示例 2：UART 设置速度

上位机发送：`v10\r\n`（10 转/秒）

```text
USART1 DMA 接收
  └── USART1_IRQHandler() IDLE 检测       [stm32f1xx_it.c]
        └── OnRecvEnd(rx_buffer, rxLen)   [usart.c]
              └── OnUartCmd("v10", 3)      [interface_uart.cpp]
                    ├── 解析: token="10", vel=10.0
                    ├── 切换 MODE_COMMAND_VELOCITY
                    └── SetVelocitySetPoint(10 × 51200 = 512000)
                          └── goalVelocity = 512000 (限制在 ratedVelocity 内)
```

## 12. 与主控的交互关系

```text
dummy-ref-core-fw (主控)                    dummy-42motor-fw (电机端)
        │                                              │
        │  CAN 0x01: 使能电机                           │
        │──────────────────────────────────────────────>│ requestMode = MODE_COMMAND_VELOCITY
        │                                              │
        │  CAN 0x07: 设置位置+速度                      │
        │  StdId=(nodeID<<7)|0x07                       │
        │  data=[float 位置, float 速度]                 │
        │──────────────────────────────────────────────>│ SetPositionSetPoint()
        │                                              │ ratedVelocity = 速度 × 51200
        │  CAN 回复 0x23: 当前位置+完成状态             │
        │<──────────────────────────────────────────────│ CAN_Send: 位置 + finish_flag
        │                                              │
        │  (20kHz 闭环控制)                              │ Tim4Callback20kHz()
        │                                              │ → motor.Tick20kHz()
        │                                              │   → FOC 电流矢量输出
        │                                              │   → TB67H450 驱动电机
        │                                              │   → MT6816 位置反馈
        │                                              │
        │  CAN 0x23: 查询位置                           │
        │──────────────────────────────────────────────>│ OnCanCmd(0x23)
        │  CAN 回复: 位置 + finish                      │ GetPosition() → CAN_Send
        │<──────────────────────────────────────────────│
        │                                              │
        │  CAN 0x01: 失能                               │
        │  data=[0]                                    │
        │──────────────────────────────────────────────>│ requestMode = MODE_STOP
```

## 13. 关键文件索引

| 模块 | 文件路径 | 职责 |
| --- | --- | --- |
| C++ 入口 | `UserApp/main.cpp` | 全局对象定义、定时器回调、按钮事件、主循环 |
| 配置 | `UserApp/configurations.h` | `BoardConfig_t` 结构体、`configStatus_t` 枚举 |
| 通用头文件 | `UserApp/common_inc.h` | 统一 include，声明 `Main()`/`OnUartCmd()`/`OnCanCmd()` |
| CAN 协议 | `UserApp/protocols/interface_can.cpp` | CAN 指令解析、参数设置、查询回复 |
| UART 协议 | `UserApp/protocols/interface_uart.cpp` | UART 文本指令解析 |
| 电机控制 | `Ctrl/Motor/motor.h` / `motor.cpp` | `Motor` 类、`Controller` 内部类、闭环控制逻辑 |
| 运动规划 | `Ctrl/Motor/motion_planner.h` / `.cpp` | 4 个跟踪器：位置/速度/电流/轨迹 |
| 驱动基类 | `Ctrl/Driver/driver_base.h` | `DriverBase` 抽象接口 |
| TB67H450 驱动 | `Ctrl/Driver/tb67h450_base.h` / `.cpp` | FOC 电流矢量计算与 DAC/PWM 输出 |
| 编码器基类 | `Ctrl/Sensor/Encoder/encoder_base.h` | `EncoderBase` 抽象接口 |
| MT6816 编码器 | `Ctrl/Sensor/Encoder/mt6816_base.h` / `.cpp` | SPI 读取角度、校准表查表 |
| 编码器校准 | `Ctrl/Sensor/Encoder/encoder_calibrator_base.h` / `.cpp` | 正反向采样、数据校验、Flash 写表 |
| CAN 初始化 | `Core/Src/can.c` | CAN 初始化、过滤器、接收回调、`CAN_Send()` |
| UART 初始化 | `Core/Src/usart.c` | UART DMA 接收、IDLE 中断、`OnRecvEnd` 回调绑定 |
| 中断入口 | `Core/Src/stm32f1xx_it.c` | TIM1/TIM4/USART1/CAN 中断转发 |
| 硬件入口 | `Core/Src/main.c` | `main()`、时钟、外设初始化 |
| printf 重定向 | `Port/Platform/retarget.c` | `_write()` → `HAL_UART_Transmit` |
| EEPROM 仿真 | `Port/Platform/Memory/` | Flash 模拟 EEPROM 持久化 |

## 14. 关键设计要点

1. **裸机前后台架构**：无 RTOS，20kHz 定时器中断做高频控制，主循环做低频任务（EEPROM 写入、校准计算）。中断回调直接调用用户函数（绕过 `HAL_TIM_IRQHandler`）以降低延迟。

2. **互斥的校准/控制**：`encoderCalibrator.isTriggered` 标志使 20kHz 中断在电机控制和编码器校准之间二选一。校准期间电机控制完全暂停。

3. **CAN ID 编码**：高 4 位为节点 ID，低 7 位为命令码。节点 ID 0 为广播。支持最多 15 个节点（实际用拨码开关 0~7）。

4. **位置查询双回复**：0x05/0x06 在 `data[4]` 非零时立即回复当前状态；0x07 总是回复。主控通过 `data[4]` 的 finish 标志判断是否到达目标。

5. **FOC 简化实现**：不使用 Park/Clarke 变换，而是通过 `SetFocCurrentVector(directionInCount, current_mA)` 直接输出相位差 90° 的两路正弦电流，由 TB67H450 的 DAC + PWM 合成。

6. **多圈位置软件累加**：MT6816 只提供单圈绝对角度，`realPosition` 通过每次 `Tick20kHz()` 累加 `deltaLapPosition` 实现多圈跟踪，断电后丢失。

7. **Home Offset 单圈特性**：`ApplyPosAsHomeOffset()` 保存的是 `realPosition % 51200`（单圈余数），不是完整多圈位置。重启后用单圈角度 + EEPROM 中的 offset 重建位置。
