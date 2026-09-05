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

## 3. TrajectoryTracker 详解（轨迹模式在线插值与超时保护）

`TrajectoryTracker` 服务于 `MODE_COMMAND_Trajectory`（轨迹模式）。与 `PositionTracker`
「固件自己规划梯形曲线」不同，轨迹模式把**整条轨迹的规划权交给上位机**：上位机做完运动学
逆解后，周期性下发一串 **(目标位置, 目标速度)** 设定点对，固件只负责在相邻两个设定点之间做
**20kHz 恒定加速度插值**，并在通信中断时兜底减速停车。

每拍（20kHz）由 `motor.cpp` 调用（L189-190、L229-233）：

```cpp
// ① 模式切入时：用真实估计状态无扰动续接
case MODE_COMMAND_Trajectory:
    motionPlanner.trajectoryTracker.NewTask(controller->estPosition, controller->estVelocity);
// ② 每拍更新软目标
case MODE_COMMAND_Trajectory:
    motionPlanner.trajectoryTracker.CalcSoftGoal(controller->goalPosition, controller->goalVelocity);
    controller->softPosition = motionPlanner.trajectoryTracker.goPosition;
    controller->softVelocity = motionPlanner.trajectoryTracker.goVelocity;
```

输出的 `goPosition / goVelocity` 即喂给 DCE 双闭环的 `softPosition / softVelocity`
（motor.cpp L120-122 `CalcDceToOutput(softPosition, softVelocity)`）。

### 3.1 内部状态与参数

```cpp
// 输出
int32_t goPosition;   // 本拍软目标位置
int32_t goVelocity;   // 本拍软目标速度

// 规划状态（插值器自己积分出的理想轨迹）
int32_t positionNow;              // 当前规划位置
int32_t velocityNow;              // 当前规划速度
int32_t dynamicVelocityAcc;       // 本段恒定加速度（收到新点时反解一次，段内保持）
int32_t dynamicVelocityAccRemainder; // 速度定点积分余数
int32_t velovityNowRemainder;     // 位置定点积分余数（源码拼写如此）

// 新点检测与超时保护
int32_t recordPosition;      // 上一次收到的目标位置
int32_t recordVelocity;      // 上一次收到的目标速度
int32_t updateTime;          // 距上次收到新点的累计时间（µs）
int32_t updateTimeout = 200; // 设定点最大允许间隔（ms），由 Init(200) 设定
bool    overtimeFlag;        // 超时标志
int32_t velocityDownAcc;     // 超时减速用减速度 = ratedVelocityAcc
```

- `NewTask(real_location, real_speed)`：模式切入/新任务时把 `positionNow/velocityNow` 对齐到
  真实估计值，清零两个积分余数、`updateTime` 与 `overtimeFlag`，实现**无扰动续接**；
- `Init()`：`velocityDownAcc = ratedVelocityAcc`，`updateTimeout = 200ms`。

### 3.2 核心：两点边值反解恒定加速度

`CalcSoftGoal` 第一步判断「本拍是否收到了新设定点」——用 `recordPosition/recordVelocity`
与入参比较（motion_planner.cpp L400-410）：

```cpp
if (_goalVelocity != recordVelocity || _goalPosition != recordPosition) {
    updateTime = 0;
    recordVelocity = _goalVelocity;
    recordPosition = _goalPosition;
    dynamicVelocityAcc = (int32_t)((float)(_goalVelocity + velocityNow) *
                                   (float)(_goalVelocity - velocityNow) /
                                   (float)(2 * (_goalPosition - positionNow)));
    overtimeFlag = false;
}
```

关键就是这行加速度的求解。利用**不含时间的运动学公式** `v² = v₀² + 2a·Δs`，反解 a：

```text
              v_goal² − v_now²     (v_goal + v_now)·(v_goal − v_now)
    a  =  ─────────────────────  =  ─────────────────────────────────
              2·(p_goal − p_now)          2·(p_goal − p_now)
```

- 分子 `(_goalVelocity + velocityNow)·(_goalVelocity − velocityNow)` = `v_goal² − v_now²`
  （用平方差展开而非直接平方，且以 float 承载，避免 int32 溢出）；
- 分母 `2·(_goalPosition − positionNow)` = `2·Δs`；
- 解出的 `a` 保证：**从当前规划状态 (positionNow, velocityNow) 出发，以恒定加速度运动，
  恰好在到达 p_goal 时速度等于 v_goal**——精确匹配上位机给定的位置与速度两个边界条件。

这个 `dynamicVelocityAcc` **每收到一个新点才重算一次**，段内保持不变；下一拍即使没有新点，
也继续沿用它积分，直到走到该段末端或收到下一个点。

### 3.3 无新点时：计时 + 超时兜底

```cpp
} else {
    if (updateTime >= (updateTimeout * 1000))   // 200ms → 200000µs
        overtimeFlag = true;
    else
        updateTime += context->CONTROL_PERIOD;  // 每拍 +50µs
}
```

- 只要设定点没变化，就累加 `updateTime`（每拍 50µs，200ms 对应 4000 拍）；
- 一旦超过 `updateTimeout`（默认 200ms）仍无新点，判定上位机掉线/停发 → 置 `overtimeFlag`，
  转入安全减速。

### 3.4 逐拍积分：生成 goPosition / goVelocity

```cpp
if (overtimeFlag) {                     // 超时：减速到 0（安全兜底）
    if (velocityNow == 0)       dynamicVelocityAccRemainder = 0;
    else if (velocityNow > 0) { CalcVelocityIntegral(-velocityDownAcc);
                                if (velocityNow <= 0){ 余数=0; velocityNow=0; } }
    else                      { CalcVelocityIntegral(+velocityDownAcc);
                                if (velocityNow >= 0){ 余数=0; velocityNow=0; } }
} else {                                // 正常：按反解出的恒定加速度推进
    CalcVelocityIntegral(dynamicVelocityAcc);
}
CalcPositionIntegral(velocityNow);      // 用本拍速度推进位置

goPosition = positionNow;
goVelocity = velocityNow;
```

两个定点积分器与 `PositionTracker` 完全同构（显式欧拉 + 余数累加，纯整数、20kHz）：

```cpp
void CalcVelocityIntegral(int32_t a){   // v += a/20000，余数保留
    dynamicVelocityAccRemainder += a;
    velocityNow += dynamicVelocityAccRemainder / CONTROL_FREQUENCY;
    dynamicVelocityAccRemainder %= CONTROL_FREQUENCY;
}
void CalcPositionIntegral(int32_t v){   // p += v/20000，余数保留
    velovityNowRemainder += v;
    positionNow += velovityNowRemainder / CONTROL_FREQUENCY;
    velovityNowRemainder %= CONTROL_FREQUENCY;
}
```

- **正常段**：速度按恒定 `dynamicVelocityAcc` 线性变化，位置随之呈抛物线——相邻两设定点间
  是一段**恒加速度轨迹**（速度线性、位置二次）；
- **超时段**：忽略 `dynamicVelocityAcc`，改用 `velocityDownAcc`（= ratedVelocityAcc）按速度
  符号对称地把速度拉回 0，过零即钳位并清余数，最终停稳；
- 正常段每拍执行 `CalcPositionIntegral(velocityNow)` 推进位置，并把 `positionNow / velocityNow`
  赋给 `goPosition / goVelocity`；零速目标下若本拍位置越过 goalPosition，则改为锁存对齐到该点（见 §3.7）。

### 3.5 决策流程

```mermaid
flowchart TD
    A[每拍入口 CalcSoftGoal goalPos, goalVel] --> B{设定点变化?<br/>goal != record}
    B -- 是/收到新点 --> C[updateTime=0 记录新点 holdFlag=false<br/>反解 a=v²-v₀²/2Δs<br/>Δp=0 保护 + 钳到 ±maxAcc<br/>overtimeFlag=false]
    B -- 否/无新点 --> D{updateTime >= 200ms?}
    D -- 是 --> E[overtimeFlag=true]
    D -- 否 --> F[updateTime += 50µs]
    C --> HO{holdFlag?}
    E --> HO
    F --> HO
    HO -- 是/已锁存 --> HK[goPosition=p goVelocity=0<br/>return 精确保持]
    HO -- 否 --> G{overtimeFlag?}
    G -- 否 --> H[CalcVelocityIntegral dynamicVelocityAcc<br/>恒加速度推进速度]
    G -- 是 --> I[按 velocityNow 符号<br/>CalcVelocityIntegral ∓velocityDownAcc<br/>过零钳位到 0]
    H --> J{零速目标 且 本拍位置越过 p?}
    J -- 是 --> L[锁存 positionNow=p<br/>velocityNow=0 holdFlag=true]
    J -- 否 --> M[CalcPositionIntegral velocityNow<br/>推进位置]
    I --> M
    L --> K[goPosition=positionNow<br/>goVelocity=velocityNow]
    M --> K
```

### 3.6 与 PositionTracker 的本质区别

| 维度 | PositionTracker（位置模式） | TrajectoryTracker（轨迹模式） |
|---|---|---|
| 输入 | 仅目标位置 `goalPosition` | 目标位置 + 目标速度 `(goalPosition, goalVelocity)` |
| 曲线由谁规划 | **固件**：每拍用刹车距离 `\|Δp\|≤v²/2a` 反应式决策加/减速 | **上位机**：固件只在相邻设定点间做恒加速度插值 |
| 加速度来源 | 固定 ±ratedVelocityAcc，靠判据切换加/减速段 | 每个新点用 `a=(v²−v₀²)/2Δs` 反解（带 Δp=0 保护、钳到 ±ratedVelocityAcc），段内恒定 |
| 速度封顶 | 有（ratedVelocity） | 无，速度轮廓完全由上位机 (p,v) 对决定 |
| 停车方式 | 到点自动减速锁定（Δp=0 且 v≈0） | `v_goal=0` 时固件「位置越过」锁存精确停在 p；掉线时超时减速兜底 |
| 适用场景 | 单轴点到点定位，断总线也能走完停稳 | 多轴机械臂连续轨迹，末端路径由上位机逆解统一保证 |

一句话概括：**PositionTracker 是「给我终点，我自己规划怎么平稳走到并停下」；
TrajectoryTracker 是「你（上位机）已经把轨迹算好并逐点告诉我该到哪、该多快，我只负责把稀疏
设定点插值成 20kHz 连续软目标，并在你断线时安全刹车」。**

### 3.7 使用注意（潜在边界）

1. **Δp=0 的奇点（已在固件侧加固）**：若某新点 `_goalPosition == positionNow`，边值反解分母为 0。
   现在 `CalcSoftGoal` 做了除零保护：`Δp==0` 时不再相除，退化为「朝目标速度以 ±maxAcc 逼近」；
   `Δp≠0` 时先在 float 域把 `dynamicVelocityAcc` 钳到 `±maxAcc`（= ratedVelocityAcc）再转 int32，
   从根本上杜绝 inf / INT32 饱和飞车。上位机仍应避免「位置不变但速度突变」这类病态点。
2. **零速终点到达锁存（已在固件侧加固）**：当某设定点 `_goalVelocity==0`（轨迹收尾或中途暂停），
   规划态在**位置越过 goalPosition 的当拍**即锁存：`positionNow=goalPosition、velocityNow=0、holdFlag=true`，
   此后跳过积分、精确保持在 p，直到下一个新点释放（新点分支会置 `holdFlag=false`）。因而在额定加速度
   可实现的范围内，`(p,0)` 收尾能**精确停在 p**，不再过冲/回退，也不再依赖 200ms 超时。
   - 若要求的减速 `|v₀²/2Δp| > maxAcc`（比额定加速度还急），钳制后到 p 仍有残余速度，会在 p 处做一次
     **有界硬停**（力度受 DCE 限流约束）——位置精确、停止偏 firm。
   - 「已静止却收到远处 `(p,0)`」属病态命令：`approaching=false` 不触发锁存，保持原位（与旧行为一致）。
3. **updateTimeout 需匹配下发频率**：默认 200ms。上位机若下发间隔可能超过它会误触发减速；
   高频流式下发（如 ≥50Hz，即 20ms 一个点）时 200ms 有足够裕度。注意锁存/超时判据均以「设定点数值
   是否变化」为准——**忠实重发完全相同的 (p,v) 不刷新计时**，200ms 后仍会触发超时兜底。
4. **加速度上限（对原「无内部限速/限位」的修订）**：轨迹模式仍不校验 ratedVelocity 与软限位，位置/速度
   轮廓由上位机决定；但**段内加速度现已被 `maxAcc=ratedVelocityAcc` 封顶**（除零保护与钳制引入）。对平滑
   轨迹（段内加速度远小于额定值）无影响，仅对过激/退化命令生效。运动安全的最后防线仍是固件超时兜底。

---

## 4. 设计要点小结

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
