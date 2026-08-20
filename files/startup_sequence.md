# dummy-42motor-fw 上电启动时序与运行模型分析

> 本文档梳理固件从上电复位到稳态运行的完整时序，标注每一步对应的源码位置，
> 并说明最终系统启动了几个"线程"（执行流）以及各自职责。

---

## 1. 整体时序总览

```text
上电/复位
   │
   ▼
[阶段0] 硬件复位 + 启动汇编 (startup_stm32f103xb.s)
   │   设置 MSP → 拷贝 .data → 清零 .bss → SystemInit() → 跳转 main()
   ▼
[阶段1] C 运行时入口 main() (Core/Src/main.c)
   │   HAL_Init() → SystemClock_Config() → MX_xxx_Init() × 9
   │   ★ CAN / UART 接收中断在此阶段即已使能
   ▼
[阶段2] 应用入口 Main() (UserApp/main.cpp)
   │   读 EEPROM 配置 → 拨码开关取节点 ID → 挂载驱动/编码器
   │   → HAL_Delay(100) → HAL_TIM_Base_Start_IT(TIM1/TIM4)
   ▼
[阶段3] 稳态运行（主循环 + 4 个中断执行流并行）
       for(;;) { 校准器 Tick + 配置写回检查 }
```

---

## 2. 阶段 0：硬件复位与启动汇编

| 步骤 | 动作 | 代码位置 |
| --- | --- | --- |
| 0.1 | CPU 从向量表第 0 项加载 MSP（主堆栈指针） | `startup/startup_stm32f103xb.s` 向量表 |
| 0.2 | 跳转到 `Reset_Handler` | 同上 |
| 0.3 | 将 `.data` 段初值从 Flash 拷贝到 SRAM | `Reset_Handler` L64-L79 |
| 0.4 | 清零 `.bss` 段 | `Reset_Handler` L81 起 |
| 0.5 | 调用 `SystemInit()`（HAL 提供，配置向量表偏移等） | 同上 |
| 0.6 | 调用 `main()`，进入 C 世界 | 同上 |

---

## 3. 阶段 1：`main()` 硬件初始化（Core/Src/main.c）

代码位置：`Core/Src/main.c` L70-L104

```c
HAL_Init();                 // 初始化 HAL，启动 SysTick（1ms 时基，供 HAL_Delay 使用）
SystemClock_Config();       // 配置系统时钟
MX_GPIO_Init();             // GPIO + ID 拨码开关引脚
MX_DMA_Init();              // DMA 时钟与 NVIC（必须在 USART 之前，MspInit 会链接 DMA 通道）
MX_ADC1_Init();             // ADC：温度采样通道
MX_USART1_UART_Init();      // ★ UART 接收 + IDLE 中断在此启动
MX_CAN_Init();              // ★ CAN 接收中断在此启动
MX_SPI1_Init();             // SPI：MT6816 编码器接口
MX_TIM2_Init();
MX_TIM3_Init();
MX_TIM4_Init();             // 20kHz 定时器初始化（仅配置，尚未启动计数）
MX_TIM1_Init();             // 100Hz 定时器初始化（仅配置，尚未启动计数）
Main();                     // → 进入 UserApp/main.cpp 应用层
```

### 3.1 UART 接收中断的启动点 —— `MX_USART1_UART_Init()`

代码位置：`Core/Src/usart.c`

```c
/* L61 */ __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   // 使能 IDLE（总线空闲）中断
/* L62 */ RetargetInit(&huart1);                         // printf 重定向绑定
/* L63 */ Uart_SetRxCpltCallBack(OnUartCmd);             // 把 OnUartCmd 挂到 OnRecvEnd 函数指针
/* L65 */ HAL_UART_Receive_DMA(&huart1, rx_buffer, BUFFER_SIZE);  // 启动 DMA 循环接收
```

NVIC 使能：`HAL_UART_MspInit()` 中 `HAL_NVIC_EnableIRQ(USART1_IRQn)`（usart.c L135）。
至此，**上电即开始监听串口**，字节由 DMA1_Channel5 自动搬入 `rx_buffer`，一帧结束由 IDLE 中断通知。

### 3.2 CAN 接收中断的启动点 —— `MX_CAN_Init()`

代码位置：`Core/Src/can.c`

```c
/* L65-L81 */ 配置接收过滤器（IDMASK 全通，分配到 RX FIFO0）
/* L83 */     HAL_CAN_Start(&hcan);                       // 启动 CAN 外设
/* L85-L94 */ HAL_CAN_ActivateNotification(&hcan,
                  CAN_IT_RX_FIFO0_MSG_PENDING | ...);     // 使能 RX FIFO0 新报文中断
```

NVIC 使能：`HAL_CAN_MspInit()` 中 `HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn)`（can.c L141）。
至此，**上电即开始监听 CAN 总线**。

> 注意：TIM1/TIM4 虽然在此阶段完成了初始化且 NVIC 已使能（tim.c），
> 但 CubeMX 不自动生成 `HAL_TIM_Base_Start_IT()`，
> 定时器尚未开始计数，要等阶段 2 由应用层手动启动。

---

## 4. 阶段 2：应用入口 `Main()`（UserApp/main.cpp）

代码位置：`UserApp/main.cpp` L18-L114

| 步骤 | 动作 | 行号 |
| --- | --- | --- |
| 4.1 | 读取 MCU 序列号；读 PA8/PA9/PA10 三位拨码开关得到 CAN 节点 ID | L20-L28 |
| 4.2 | 从 EEPROM 读取 `boardConfig`；若校验失败则写入默认配置并回存 | L32-L55 |
| 4.3 | 将配置下发到 `motor`：运动参数（电流/速度/加速度限幅）、闭环 PID 参数、堵转保护开关 | L58-L70 |
| 4.4 | 挂载硬件对象：`motor.AttachDriver(&tb67H450)`、`motor.AttachEncoder(&mt6816)`，并依次 `Init()` 控制器/驱动芯片/编码器 | L79-L83 |
| 4.5 | 绑定按钮事件回调 `OnButton1Event` / `OnButton2Event` | L87-L88 |
| 4.6 | `HAL_Delay(100)` 等待外设稳定 | L92 |
| 4.7 | **启动两个定时中断**：`HAL_TIM_Base_Start_IT(&htim1)` (100Hz)、`HAL_TIM_Base_Start_IT(&htim4)` (20kHz) | L93-L94 |
| 4.8 | 若上电瞬间两键同时按下 → 触发编码器校准流程 `encoderCalibrator.isTriggered = true` | L96-L97 |
| 4.9 | 进入主循环 `for(;;)`（永不返回） | L100-L114 |

主循环体内容：

```c
for (;;)
{
    encoderCalibrator.TickMainLoop();          // 校准状态机推进（空闲时开销极小）

    if (boardConfig.configStatus == CONFIG_COMMIT)    // 收到"保存配置"请求
    { boardConfig.configStatus = CONFIG_OK; eeprom.put(0, boardConfig); }
    else if (boardConfig.configStatus == CONFIG_RESTORE)  // 收到"恢复出厂"请求
    { eeprom.put(0, boardConfig); HAL_NVIC_SystemReset(); }
}
```

---

## 5. 阶段 3：稳态运行 —— 启动了几个"线程"？

本工程是 **裸机（bare-metal）架构，没有使用 FreeRTOS 等实时操作系统，
不存在操作系统意义上的线程**。系统的并发由"1 个主循环 + 4 个中断执行流"构成，
功能上等价于 **5 个执行流（可理解为 5 个"伪线程"）**：

| # | 执行流 | 触发方式 | 频率 | 入口 | 职责 |
| --- | --- | --- | --- | --- | --- |
| 0 | **主循环（Main Loop）** | 顺序执行 | 无限循环、尽力快 | `Main()` 的 `for(;;)`（main.cpp L100） | 编码器校准状态机推进、配置写回/恢复出厂检查 |
| 1 | **TIM1 低速周期任务** | 定时中断 | 100Hz（10ms） | `TIM1_UP_IRQHandler()` → `Tim1Callback100Hz()`（main.cpp L121） | 按钮去抖与事件检测（`button1/2.Tick`）、状态 LED 刷新、温度采样（每秒一次，受开关控制） |
| 2 | **TIM4 高速周期任务** | 定时中断 | 20kHz（50µs） | `TIM4_IRQHandler()` → `Tim4Callback20kHz()`（main.cpp L149） | 二选一：校准中 → `encoderCalibrator.Tick20kHz()`；正常 → `motor.Tick20kHz()`（FOC 电流环 + 位置/速度闭环、SVPWM 输出） |
| 3 | **CAN 接收任务** | 事件中断 | 随报文到达 | `HAL_CAN_IRQHandler()` → `HAL_CAN_RxFifo0MsgPendingCallback()`（can.c L189） | 取 RX 报文 → 按 `StdId>>7` 过滤节点 ID → `OnCanCmd()` 分发协议指令（interface_can.cpp） |
| 4 | **UART 接收任务** | 事件中断 | 随数据帧结束 | `USART1_IRQHandler()` IDLE 分支（stm32f1xx_it.c L374） | 停 DMA → 计算接收长度 → `OnRecvEnd()`（即 `OnUartCmd`，interface_uart.cpp）解析文本指令 → 重启 DMA 接收 |

### 5.1 各执行流详解

**执行流 0：主循环（背景任务）**
- 唯一的"非中断"执行流，优先级最低（会被任何 ISR 打断）。
- 职责很轻：推进编码器校准状态机（校准是主循环与 20kHz 中断协作完成的），
  以及响应 `CONFIG_COMMIT`（保存配置到 Flash）与 `CONFIG_RESTORE`（恢复出厂并复位）。

**执行流 1：TIM1 @100Hz（人机交互层）**
- 按钮去抖计数、长短按事件产生（回调运行在中断上下文）。
- LED 按控制器状态闪烁；`enableTempWatch` 开启时每 100 次中断采一次芯片温度。

**执行流 2：TIM4 @20kHz（实时控制层，系统心脏）**
- 正常模式：`motor.Tick20kHz()` 完成编码器采样、FOC 电流环/位置环/速度环计算、PWM 占空比更新。
- 校准模式：`encoderCalibrator.Tick20kHz()` 控制电机步进励磁并采样编码器原始值。
- 两者互斥，由 `encoderCalibrator.isTriggered` 切换。

**执行流 3：CAN 接收（通信层）**
- 过滤器配置为全通 + 软件按节点 ID 过滤（`id == 0` 为广播）。
- 指令在中断上下文中直接解析执行（`OnCanCmd`），会打断主循环与 TIM1，但不打断 TIM4（取决于 NVIC 优先级配置）。

**执行流 4：UART 接收（通信层/调试）**
- DMA + IDLE 模式：字节自动入缓冲区，帧结束触发中断统一处理。
- `OnRecvEnd` 是 `usart.c` 中的函数指针，初始化时绑定为 `OnUartCmd`（文本指令解析）。

### 5.2 中断转发关系一览

```text
TIM1_UP_IRQHandler()  → Tim1Callback100Hz() → return   (绕过 HAL_TIM_IRQHandler，降低开销)
TIM4_IRQHandler()     → Tim4Callback20kHz() → return   (同上)
USART1_IRQHandler()   → IDLE 标志检测 → OnRecvEnd() → 重启 DMA
CAN RX0 IRQ           → HAL_CAN_IRQHandler() → HAL_CAN_RxFifo0MsgPendingCallback() → OnCanCmd()
SysTick               → HAL_IncTick()                  (1ms 时基，供 HAL_Delay/超时使用)
```

---

## 6. 上电时序甘特图

```text
时间轴 ──────────────────────────────────────────────────────────────────►

startup.s   ███  (拷贝 .data / 清零 .bss，微秒级)
main.c          ███████  (HAL_Init → 时钟 → 9 个 MX_Init)
  UART中断                │────── 已使能，DMA 开始监听 ──────────────────►
  CAN 中断                │────── 已使能，开始监听总线 ──────────────────►
Main()                        █████  (EEPROM → 挂载硬件 → Delay 100ms)
TIM1 100Hz                              │── 启动 ──► 每 10ms 一次
TIM4 20kHz                              │── 启动 ──► 每 50µs 一次
主循环 for(;;)                          │── 进入，永久运行 ─────────────►
```

---

## 7. 关键源文件索引

| 阶段 | 文件 | 职责 |
| --- | --- | --- |
| 复位启动 | `startup/startup_stm32f103xb.s` | 向量表、Reset_Handler、跳转 main |
| C 入口 | `Core/Src/main.c` | `main()`、时钟配置、外设初始化序列 |
| UART 初始化 | `Core/Src/usart.c` | IDLE 中断使能、DMA 接收启动、`OnRecvEnd` 绑定 |
| CAN 初始化 | `Core/Src/can.c` | 过滤器、`HAL_CAN_Start`、接收通知使能、RX 回调 |
| 中断入口 | `Core/Src/stm32f1xx_it.c` | TIM1/TIM4/USART1 中断转发 |
| 应用入口 | `UserApp/main.cpp` | 配置加载、电机初始化、定时中断启动、主循环、100Hz/20kHz 回调 |
| 协议解析 | `UserApp/protocols/interface_uart.cpp` / `interface_can.cpp` | `OnUartCmd` / `OnCanCmd` |

---

## 8. 结论

- **上电后没有创建任何 OS 线程**（裸机工程，无 RTOS）。
- 系统最终稳定运行 **1 个主循环 + 4 个中断执行流，共 5 个执行流**：
  主循环管"校准与配置"，TIM1 管"人机交互（100Hz）"，TIM4 管"电机实时控制（20kHz）"，
  CAN/UART 中断管"指令接收与协议分发"。
- 通信中断（CAN/UART）在外设初始化阶段即已使能，**先于电机控制中断启动**；
  电机控制中断（TIM1/TIM4）由 `Main()` 在配置加载、硬件挂载完成后手动启动，
  以保证闭环控制开始时所有参数已就绪。
