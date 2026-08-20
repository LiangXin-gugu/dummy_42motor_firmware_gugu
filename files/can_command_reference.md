# CAN 指令参考与配置持久化说明

> 关联源码：`UserApp/protocols/interface_can.cpp`（指令处理）、`Core/Src/can.c`（接收分发）、
> `UserApp/main.cpp`（上电加载/落盘）、`UserApp/configurations.h`（BoardConfig_t 定义）
> 前置文档：`project_analysis.md` §6、`startup_sequence.md` §5

---

## 1. 帧格式与分发流程

CAN 标准帧 11 位 StdId 被拆成两段（can.c L198-203）：

```text
StdId = (nodeID << 7) | cmd
        └── 高 4 位: 节点 ID (0 = 广播，所有节点响应；1~7 = 拨码开关设定值)
        └── 低 7 位: 命令码 cmd (0x00~0x7F)
data[0..7] = 参数（float / int32 / uint32，小端字节序）
```

```text
CAN 帧到达
  └── HAL_CAN_RxFifo0MsgPendingCallback()   [can.c]
        ├── HAL_CAN_GetRxMessage() 取 RxHeader + RxData[8]
        ├── id = StdId >> 7,  cmd = StdId & 0x7F
        ├── if (id == 0 广播 || id == boardConfig.canNodeId)
        └── OnCanCmd(cmd, RxData, DLC)      [interface_can.cpp]
```

> 注意：本固件的 `canNodeId` 实际由板上 3 位拨码开关决定（PA8/PA9/PA10，
> main.cpp L25-28），上电时覆盖 EEPROM 中保存的值（见 §4.3）。

命令按码段分为四组：

| 码段 | 类别 | 断电后 |
|---|---|---|
| `0x01~0x0F` | 即时运动指令 | 不保存，断电即失 |
| `0x10~0x1F` | 配置指令 | 可保存（视 commit 标志） |
| `0x20~0x2F` | 查询指令 | 只读，无状态 |
| `0x7d/0x7e/0x7f` | 特殊指令 | 见各条说明 |

---

## 2. 指令总表

### 2.1 即时运动指令（0x01~0x07，不写 EEPROM）

| cmd | 功能 | data 格式 | 行为 |
|---|---|---|---|
| 0x01 | 使能/失能电机 | data[0..3] = uint32：1=使能，0=停止 | 1 → `requestMode = MODE_COMMAND_VELOCITY`（速度目标为 0，即上电锁轴）；0 → `MODE_STOP`（驱动器睡眠） |
| 0x02 | 触发编码器校准 | 无 | `encoderCalibrator.isTriggered = true`，电机自动正反转一整圈采样，生成校准表写入 Flash 后复位 |
| 0x03 | 设置电流目标 | data[0..3] = float（安培） | 切换 `MODE_COMMAND_CURRENT`，目标 = A×1000（mA） |
| 0x04 | 设置速度目标 | data[0..3] = float（r/s） | 切换 `MODE_COMMAND_VELOCITY`；首次切入时把 `ratedVelocity` 恢复为 `velocityLimit`；目标 = r/s×51200 |
| 0x05 | 设置位置目标 | data[0..3] = float（圈）；data[4] = 1 时要求 ACK | 切换 `MODE_COMMAND_POSITION`，梯形规划走到目标；data[4]=1 时立即回复 0x23（当前位置 + 完成标志） |
| 0x06 | 按时间到位 | data[0..3] = float（圈）；data[4..7] = float（秒） | `SetPositionSetPointWithTime()`：反解能在指定时间内完成的最大速度并临时设为 `ratedVelocity`；超时不可达则用板级限速。data[4] 作 ACK 开关（与参数复用，实际传参时不发 ACK） |
| 0x07 | 带限速到位 | data[0..3] = float（圈）；data[4..7] = float（r/s） | 本次运动临时用指定限速（**不保存**），走到目标；**总是**回复 0x23 ACK |

回复帧格式（0x23 ACK）：`StdId = (nodeID<<7) | 0x23`，
data[0..3] = float 当前位置（圈），data[4] = 1 表示 `STATE_FINISH`。

### 2.2 配置指令（0x11~0x1B，可写 EEPROM）

所有配置指令的共性：

1. 立即修改运行时参数（`motor.config.*`），**马上生效**；
2. 同步到 `boardConfig` 结构体（内存中的配置副本）；
3. **只有 data[4] = 1（commit 标志）时才置 `configStatus = CONFIG_COMMIT`**，
   由主循环异步写入 Flash（0x15 例外，总是提交）。

| cmd | 功能 | data 格式 | 影响参数 | commit |
|---|---|---|---|---|
| 0x11 | 设置节点 ID | data[0..3] = uint32 | `boardConfig.canNodeId` | 条件 |
| 0x12 | 设置电流限制 | data[0..3] = float（A） | `ratedCurrent` = A×1000 | 条件 |
| 0x13 | 设置速度限制 | data[0..3] = float（r/s） | `ratedVelocity` = r/s×51200 | 条件 |
| 0x14 | 设置加速度 | data[0..3] = float（r/s²） | `ratedVelocityAcc`，并同步到 velocityTracker / positionTracker 的 `SetVelocityAcc()`（重算刹车距离常数） | 条件 |
| 0x15 | 应用当前位置为机械零点 | 无 | `encoderHomeOffset = realPosition % 51200` | **总是提交** |
| 0x16 | 设置上电自启 | data[0..3] = uint32（0/1） | `enableMotorOnBoot` | 条件 |
| 0x17 | 设置 DCE Kp | data[0..3] = int32 | `dce.kp` | 条件 |
| 0x18 | 设置 DCE Kv | data[0..3] = int32 | `dce.kv` | 条件 |
| 0x19 | 设置 DCE Ki | data[0..3] = int32 | `dce.ki` | 条件 |
| 0x1A | 设置 DCE Kd | data[0..3] = int32 | `dce.kd` | 条件 |
| 0x1B | 设置堵转保护开关 | data[0..3] = uint32（0/1） | `stallProtectSwitch` | 条件 |

### 2.3 查询指令（0x21~0x25，只读）

| cmd | 查询内容 | 回复 |
|---|---|---|
| 0x21 | 当前 FOC 电流 | StdId=(id<<7)\|0x21，data[0..3]=float(A)，data[4]=完成标志 |
| 0x22 | 当前速度 | StdId=(id<<7)\|0x22，data[0..3]=float(r/s) |
| 0x23 | 当前位置 | StdId=(id<<7)\|0x23，data[0..3]=float(圈，已减 home offset)，data[4]=完成标志 |
| 0x24 | 零点偏移 | StdId=(id<<7)\|0x24，data[0..3]=int32(encoderHomeOffset 原始计数) |
| 0x25 | 电机温度 | StdId=(id<<7)\|0x25，data[0..3]=float(温度) |

### 2.4 特殊指令

| cmd | 功能 | 行为 |
|---|---|---|
| 0x7d | 开启温度监视 | `enableTempWatch = true`（仅运行时；上电被强制复位为 false） |
| 0x7e | 恢复出厂设置 | `configStatus = CONFIG_RESTORE` → 主循环写入 Flash 后复位（见 §4.2） |
| 0x7f | 系统复位 | `HAL_NVIC_SystemReset()` 立即重启 |

---

## 3. 哪些值会被保存到硬件？

### 3.1 存储介质

`boardConfig` 通过 **EEPROM 模拟**（`eeprom.put(0, boardConfig)`）保存在
STM32F103 的**内部 Flash 页**中，属于非易失存储，**断电不丢失**。
另一块独立保存的数据是编码器校准表（32KB 查找表，校准流程单独写入 Flash 尾部）。

### 3.2 保存时机：commit 机制

配置指令**不会立即写 Flash**（Flash 写入慢且耗资源），而是打标记、由主循环异步落盘
（main.cpp L105-113）：

```cpp
for (;;)
{
    encoderCalibrator.TickMainLoop();

    if (boardConfig.configStatus == CONFIG_COMMIT)   // 有配置指令要求提交
    {
        boardConfig.configStatus = CONFIG_OK;
        eeprom.put(0, boardConfig);                  // 整块写入 Flash
    } else if (boardConfig.configStatus == CONFIG_RESTORE)
    {
        eeprom.put(0, boardConfig);                  // 写入无效状态标记
        HAL_NVIC_SystemReset();                      // 重启走恢复出厂流程
    }
}
```

因此对 0x11~0x1B（除 0x15）：

- **data[4] = 0**：只改运行时参数，断电丢失；
- **data[4] = 1**：标记 commit，主循环随后写入 Flash，断电保留；
- **0x15（设为零点）**：无条件 commit，总是保存。

可以把多条配置指令连续下发（前 N-1 条 data[4]=0，最后一条 data[4]=1），
只需一次 Flash 写入即可全部生效。

---

## 4. 断电重上电后的行为

### 4.1 上电加载流程（main.cpp L30-70）

```text
eeprom.get(0, boardConfig)
  ├── configStatus == CONFIG_OK ?
  │     ├── 是 → 使用 EEPROM 中保存的配置（优先级高于代码默认值）
  │     └── 否 → 使用代码内置默认配置，并回写 EEPROM
  │
  ├── 强制 enableTempWatch = false
  ├── canNodeId = 拨码开关读数（覆盖 EEPROM 保存值！）
  └── 逐项应用到 motor.config：
        encoderHomeOffset / ratedCurrent / ratedVelocity / ratedVelocityAcc
        / caliCurrent / dce.kp~kd / stallProtectSwitch
```

默认配置值（首次上电或恢复出厂后）：

| 参数 | 默认值 |
|---|---|
| encoderHomeOffset | 0 |
| defaultMode | MODE_COMMAND_POSITION |
| currentLimit | 1 A |
| velocityLimit | 30 r/s |
| velocityAcc | 100 r/s² |
| calibrationCurrent | 2 A |
| dce_kp / kv / ki / kd | 200 / 80 / 300 / 250 |
| enableMotorOnBoot | false |
| enableStallProtect | false |

### 4.2 恢复出厂（0x7e）的原理

`CONFIG_RESTORE = 0`（枚举首值）。0x7e 把 `configStatus` 置为 `CONFIG_RESTORE`
后主循环把整个 `boardConfig` 原样写入 Flash 并复位。重启后加载时发现
`configStatus != CONFIG_OK`，于是走"默认配置 + 回写"分支——实现恢复出厂。

### 4.3 持久化结论速查表

| 参数 | 设置命令 | commit 后断电保留？ | 备注 |
|---|---|---|---|
| 节点 ID | 0x11 | 形式上保存，**但上电被拨码开关覆盖** | 实际以硬件开关为准 |
| 电流限制 | 0x12 | ✅ | |
| 速度限制 | 0x13 | ✅ | |
| 加速度 | 0x14 | ✅ | |
| 机械零点 | 0x15 | ✅（总是保存） | 0x24 可查询 |
| 上电自启 | 0x16 | ✅ | |
| DCE kp/kv/ki/kd | 0x17~0x1A | ✅ | |
| 堵转保护开关 | 0x1B | ✅ | |
| 编码器校准表 | 0x02 触发校准 | ✅（独立 Flash 区） | 校准成功自动写入并重启 |
| 电流/速度/位置目标 | 0x03~0x07 | ❌ | 纯运行时 |
| 0x07 的临时限速 | 0x07 | ❌ | 仅本次运动有效 |
| 温度监视开关 | 0x7d | ❌ | 上电强制关闭 |

### 4.4 注意事项

1. **不 commit 的配置只活到下次复位**：调试时可先 data[4]=0 试参数，满意后再发一条
   data[4]=1 固化，避免频繁写 Flash（模拟 EEPROM 有擦写次数限制）。
2. **节点 ID 的特殊性**：0x11 保存的 canNodeId 上电即被拨码开关覆盖，
   想永久改节点 ID 需要改硬件开关。
3. **广播帧慎用**：nodeID=0 时所有节点都会执行并回复，多节点同时回复可能总线冲突。
4. **写 Flash 在主循环完成**：commit 后的 `eeprom.put()` 是阻塞操作，
   执行瞬间可能影响校准状态机推进，但不影响 TIM4 的 20kHz 控制中断。
