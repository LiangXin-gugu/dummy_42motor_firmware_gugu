# MotionPlanner 运动规划层原理说明

> 关联源码：`dummy-42motor-fw/Ctrl/Motor/motion_planner.h/.cpp`、`motor.cpp`、`motor.h`
> 前置文档：`close_loop_control_tick.md`（CloseLoopControlTick 主流水线）

---

## 1. 为什么需要 MotionPlanner

### 1.1 问题：把上位机目标位置直接喂给闭环会发生什么

设想跳过规划层，让 `CloseLoopControlTick` 中的 DCE 位置双闭环直接跟踪上位机下发的
`goalPosition`。以"从静止走到 10 圈外"为例，逐级推演：

**① DCE 立即饱和，闭环退化为"最大力矩硬推"**

`CalcDceToOutput()` 中位置误差被限幅（motor.cpp L384-386）：

```cpp
config->dce.pError = _location - estPosition;
if (config->dce.pError > (3200)) config->dce.pError = (3200);  // 限幅 ±1/16 圈
if (config->dce.pError < (-3200)) config->dce.pError = (-3200);
```

10 圈的误差远超 ±3200，`pError` 恒等于 3200，输出电流瞬间钳位到 `ratedCurrent`，
`focPosition = estPosition ± 256`（满电流、恒超前/滞后 90° 电角）。
此时电机行为等价于**开环最大力矩加速**——控制环除维持换相外已无任何调节作用。

**② 没有减速计划 → 冲过头、来回振荡**

没有任何机制在接近目标前减速。转子带着全部动能冲过目标点，误差反号，电流饱和反向，
再被拉回……形成大幅振荡。DCE 的 `kv·vError` 项（此时 softVelocity = 0）虽能提供阻尼，
但它是为"跟踪平滑参考"设计的，远不足以吸收高速运动的动能。
结果是**超调、振荡，甚至触发堵转保护**。

**③ 物理上根本达不到突变参考**

- **加速度极限**：转子惯量 J 与最大力矩 T 决定最大加速度 `a_max = T/J`。
  参考曲线要求的加速度一旦超过它，跟随误差持续增大；
- **速度极限**：高速下受编码器超前角补偿范围（±430 计数）、绕组电感与反电动势限制；
- **保护误触发**：饱和电流 = `ratedCurrent` 且速度跟不上，正好命中阶段 9 的堵转判据
  （满电流 + 低速持续 1s），电机被误判堵转而断电。

**④ 机械层面的伤害**

电流阶跃 → 力矩阶跃 → 同步带/齿轮/丝杠承受冲击，产生噪声与磨损；
步进电机在特定速度区间存在**共振区**，不受控扫过会引起剧烈振动。
多轴机械臂中各关节各自"满速扑向目标"还会破坏轨迹同步，末端路径完全不可预测。

### 1.2 MotionPlanner 的对策

| 问题 | MotionPlanner 对策 |
|---|---|
| 参考突变、要求无穷大速度/加速度 | 梯形速度曲线：加速度恒为 `ratedVelocityAcc`，速度封顶 `ratedVelocity`，参考轨迹物理可实现 |
| 冲过头、无减速 | 每拍用刹车距离判据 `\|Δp\| ≤ v²/2a` 精确计算减速点，保证到达目标时速度恰好为零 |
| DCE 饱和退化 | 软目标始终贴近真实运动，`pError` 保持在 ±3200 线性区内，控制环工作在正常调节状态，电流平滑 |
| 速度前馈缺失 | 规划器同时输出 `softVelocity`，DCE 的 `kv·(softVelocity − estVelocity)` 成为前馈/阻尼项，大幅减小跟随误差 |
| 运动中改目标 | 反应式逐拍决策：目标反号先刹停再折返，任意时刻改目标都安全 |
| 模式切换/恢复冲击 | `NewTask(estPosition, estVelocity)` 从真实状态续接，无扰动切换 |
| 机械冲击与共振 | 恒加速度斜坡限制 jerk，受控地通过速度区间 |

### 1.3 本质：控制环带宽与参考轨迹

控制环是有带宽的：20kHz 采样、IIR 滤波、DCE 增益共同决定了它只能良好跟踪
**变化率在其能力之内**的信号。运动规划的工作，就是把上位机的"事件级指令"（走到哪）
翻译成控制环"逐拍可跟踪的参考序列"（每拍的 softPosition/softVelocity）。
这是运动控制的标准分层：

```text
上位机:   "位置 = 10 圈"              （任务层，事件驱动，可突变）
规划器:   梯形速度曲线（限幅 v、a）     （轨迹层，20kHz，保证可行性）
DCE 闭环: 跟踪每拍软目标               （伺服层，只负责消除小误差）
驱动器:   FOC 电流矢量输出             （执行层）
```

跳过轨迹层，等于让伺服层独自承担"把不可行参考变成可行运动"的任务——
它靠饱和与限幅做不到，只会以振荡、冲击、保护跳闸的形式失败。

> 类比：汽车导航不能因为目的地在 100 km 外，就让油门控制器直接以"100 km 外"作为
> 设定值；必须先有速度规划（起步加速、限速巡航、到点前刹车），油门/刹车闭环才有意义。

### 1.4 规划也可以放在上位机做

规划层并非必须驻留在电机固件中。本固件预留了 `MODE_COMMAND_Trajectory`（轨迹模式），
由上位机周期性下发 **(位置, 速度) 设定点对**，固件用 `TrajectoryTracker` 接收：

```text
上位机:   完整轨迹规划（S曲线/多轴同步/前瞻）→ 周期性下发 (p, v) 设定点
固件:     TrajectoryTracker：两点间 20kHz 插值 + 超时保护 → DCE 闭环跟踪
```

`TrajectoryTracker::CalcSoftGoal()`（motion_planner.cpp L398-450）的核心：

1. 每收到新设定点，用运动学公式反解恒定加速度，使电机恰好在目标点达到目标速度：
   `a = (v_goal² − v_now²) / (2·(p_goal − p_now))`；
2. 之后每拍按该加速度积分推进，生成连续软目标；
3. 超过 200ms（`updateTimeout`）未收到新点 → 自动以 `ratedVelocityAcc` 减速到 0（防上位机掉线）。

但注意：**即使规划全在上位机，固件仍保留插值层，不能原样直通 DCE**，原因：

1. **速率不匹配**：CAN/UART 设定点到达率仅几百 Hz~1kHz，控制环 20kHz，
   中间空白必须按 (p, v, a) 插值填充，否则参考呈阶梯跳变；
2. **通信抖动**：总线延迟不确定，插值层吸收到达时刻抖动；
3. **安全必须在固件侧**：超时减速、电流钳位、堵转/过载保护是最后防线，
   上位机会崩溃/断线，这些责任无法委托。

**选型建议**：单轴点到点定位用固件内 `MODE_COMMAND_POSITION`（最省心，断总线也能走完停稳）；
多轴机械臂连续轨迹用 `MODE_COMMAND_Trajectory`（轨迹由上位机运动学逆解统一生成，
固件各自规划无法保证末端路径正确）。

---

## 2. PositionTracker 详解（位置模式梯形速度规划）

`PositionTracker` 服务于 `MODE_COMMAND_POSITION` / `MODE_PWM_POSITION`，
每拍（20kHz）被 `motor.cpp` 阶段 7 调用，输入硬目标 `goalPosition`，
输出软目标 `go_location` / `go_velocity`。

### 2.1 内部状态与参数

```cpp
// 规划状态（规划器自己积分出的理想轨迹）
int32_t trackPosition;      // 规划位置
int32_t trackVelocity;      // 规划速度
int32_t positionIntegral;   // 位置定点积分余数
int32_t velocityIntegral;   // 速度定点积分余数

// 参数
int32_t velocityUpAcc;        // 加速度 = ratedVelocityAcc（计数/s²）
int32_t velocityDownAcc;      // 减速度 = ratedVelocityAcc
float   quickVelocityDownAcc; // 预计算常数 = 0.5 / velocityDownAcc（用于刹车距离）
int32_t speedLockingBrake;    // 锁定停速阈值 = ratedVelocityAcc / 1000
```

- `NewTask(real_location, real_speed)`：模式切换/新任务时把规划状态对齐到真实估计值
  （无扰动启动），并清零积分器；
- `quickVelocityDownAcc` 在 `SetVelocityAcc()` 中预计算，避免每拍除法。

### 2.2 定点积分器：纯整数的双重积分

```cpp
void CalcVelocityIntegral(int32_t value)      // value = 加速度 a
{
    velocityIntegral += value;
    trackVelocity += velocityIntegral / CONTROL_FREQUENCY;   // v += a / 20000
    velocityIntegral = velocityIntegral % CONTROL_FREQUENCY; // 余数保留
}
void CalcPositionIntegral(int32_t value)      // value = 速度 v
{
    positionIntegral += value;
    trackPosition += positionIntegral / CONTROL_FREQUENCY;   // p += v / 20000
    positionIntegral = positionIntegral % CONTROL_FREQUENCY;
}
```

原理 = **显式欧拉双重积分 + 定点余数累加**：

- 每拍速度增量 Δv = a·Δt = a / 20000；每拍位置增量 Δp = v / 20000；
- 除不尽的余数存回积分变量，下一拍继续累加，长期运行无漂移；
- 全程整数运算，仅刹车距离一处用 float（v² 最大约 (30×51200)² ≈ 2.3×10¹²，超出 int32）。

### 2.3 核心判据：刹车距离公式

匀减速从速度 v 减到 0 所需距离：

```
s_brake = v² / (2a)
```

代码中每拍计算 `need_down_location = trackVelocity² × quickVelocityDownAcc`，
与剩余距离 `|Δp|` 比较：

- `|Δp| > s_brake` → 还有余裕，继续加速（封顶 ratedVelocity）；
- `|Δp| ≤ s_brake` → 必须立刻减速，否则冲过头。

由于每拍重判，减速点随目标变化**实时自动调整**——这是"反应式在线梯形规划"，
无需预先算好整条曲线，天然支持运动中改目标、改限速。

### 2.4 决策状态机（逐分支）

设 `Δp = goalPosition − trackPosition`，`v = trackVelocity`：

```mermaid
flowchart TD
    A[每拍入口: Δp = goal − trackPosition] --> B{Δp == 0 ?}
    B -- 是 --> C{\|v\| ≤ speedLockingBrake ?}
    C -- 是 --> C1[清零 v 与积分器<br>锁定停车]
    C -- 否 --> C2[按速度符号减速<br>过零钳位]
    B -- 否 --> D{v == 0 ?}
    D -- 是 --> D1[按 Δp 符号起步加速]
    D -- 否 --> E{v 与 Δp 同向 ?}
    E -- 反向 --> F[先减速到 0<br>下一拍反向起步]
    E -- 同向 --> G{\|v\| ≤ ratedVelocity ?}
    G -- 超速 --> G1[先减速]
    G -- 是 --> H{\|Δp\| > v²/2a ?}
    H -- 是 --> H1[继续加速, 封顶 ratedVelocity]
    H -- 否 --> H2[开始减速, 钳位到 0]
    C1 --> Z
    C2 --> Z
    D1 --> Z
    F --> Z
    G1 --> Z
    H1 --> Z
    H2 --> Z[CalcPositionIntegral 推进位置<br>输出 go_location / go_velocity]
```

各分支对应代码（motion_planner.cpp L190-330）：

| 分支 | 条件 | 动作 | 目的 |
|---|---|---|---|
| ① 锁定停车 | Δp==0 且 \|v\|≤speedLockingBrake | 直接清零 v 与所有积分器 | 残余速度足够小时截断，避免定点余数造成目标点附近蠕动 |
| ② 冲线拉回 | Δp==0 但 v≠0 | 以 velocityDownAcc 减速，过零钳位 | 冲过目标后把规划速度拉回 0，靠控制环拉回位置 |
| ③ 起步 | Δp≠0 且 v==0 | 按 Δp 符号 ±velocityUpAcc 加速 | 从静止向目标起步 |
| ④ 同向巡航 | Δp、v 同号，\|v\|≤rated | 算刹车距离：余裕则加速（封顶 rated），否则减速 | 梯形曲线的加速/减速段切换 |
| ⑤ 超速回落 | 同向但 \|v\|>rated | 先减速 | 中途下调限速或从高速状态切入时的处理 |
| ⑥ 反向刹停 | Δp、v 异号 | 减速到 0，下一拍进入分支③反向起步 | 运动中目标改到身后：先刹停再折返，避免冲击 |

收尾动作（每拍必做）：

```cpp
CalcPositionIntegral(trackVelocity);  // 用本拍速度推进规划位置
go_location = trackPosition;
go_velocity = trackVelocity;
```

### 2.5 一次典型定位的完整过程

从静止走到远处某点（正向）：

```text
阶段     判据结果                             动作                     速度曲线
─────────────────────────────────────────────────────────────────────────
起步     Δp>0, v=0                          +velocityUpAcc 加速        /
加速段   |Δp| > v²/2a                       继续加速                  /
匀速段   已封顶且 |Δp| > v²/2a               保持 ratedVelocity        ─
减速点   |Δp| ≤ v²/2a 首次成立的那拍         -velocityDownAcc 减速     \
末段     Δp==0 且 |v| ≤ speedLockingBrake    截断清零，锁定停车         _
```

速度曲线为经典**梯形**（加速—匀速—减速），位置曲线为 S 形平滑过渡。
到达时 v 恰好归零，`motor.cpp` 阶段 10 据此判定
`softPosition == goalPosition && softVelocity == 0` → `STATE_FINISH`。

### 2.6 单拍计算步骤汇总（伪代码）

```text
每拍执行 CalcSoftGoal(goalPosition):
  1. Δp = goalPosition − trackPosition
  2. 按 2.4 状态机选定本拍加速度 acc ∈ {+velocityUpAcc, 0, −velocityDownAcc}
     （或锁定清零）
  3. CalcVelocityIntegral(acc)      → trackVelocity += acc/20000（余数保留）
  4. CalcPositionIntegral(trackVelocity) → trackPosition += v/20000（余数保留）
  5. 输出 go_location = trackPosition, go_velocity = trackVelocity
```

---

## 3. 设计要点小结

1. **反应式而非预规划**：不预先生成整条曲线，每拍独立决策，天然支持运动中改目标、改限速
   （`ratedVelocity` 被 CAN 指令实时修改也立即生效）。
2. **全定点实现**：只有刹车距离一处用 float；积分余数机制保证长时间运行无累积误差，
   适配无 FPU 场景与 50µs 中断预算。
3. **对称与防御**：正负方向逻辑完全镜像；对"超速进入""反向目标""冲线"等边界
   均有明确处理路径。
4. **与伺服层解耦**：规划器只生成"理想轨迹"，真实跟随误差由 DCE 闭环吸收；
   两者通过 softPosition/softVelocity 两个软目标衔接。
5. **分层可上移**：完整轨迹规划可移至上位机（MODE_COMMAND_Trajectory），
   固件保留插值与超时保护兜底——规划在哪一层，安全责任的安全底线始终在固件层。
