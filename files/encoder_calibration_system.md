# MT6816 编码器绝对位置校准系统

## 1. 概述

本系统用于对 **MT6816 磁编码器的角度测量误差**进行高精度校准。通过全角度（0~360°）采样建立查找表，存储至 Flash，实现在运行时消除编码器的系统性测量误差。

---

## 2. 硬件背景

### 2.1 MT6816 编码器特性
- **分辨率**: 14 位（0~16383）
- **硬步数**: 200 步/圈（对应 1.8°步进电机）
- **软细分**: 256 份/硬步
- **总分辨率**: 200 × 256 = **51200 步/圈**

### 2.2 为何需要校准？

即使高精密磁编码器，仍可能存在以下误差来源：
- **磁极安装机械偏差**
- **磁场非线性**
- **PCB 布局引入的干扰**
- **温度漂移**

这些误差会导致 FOC 控制在某些位置出现转矩波动、噪声或定位不准。通过建立**全圈 16384 点的映射表**，可在运行时实时修正角度值，确保 FOC 换相角度的绝对准确性。

---

## 3. 校准触发机制

### 3.1 两种触发方式

#### 方式一：上电瞬间双键按下（hardcoded in main.cpp L96-97）
```cpp
if (button1.IsPressed() && button2.IsPressed())
    encoderCalibrator.isTriggered = true;
```
**特点**：必须在启动时按住两个按键，适用于初始调试阶段。

#### 方式二：任意时刻长按 Key1（main.cpp L169）
```cpp
case ButtonBase::LONG_PRESS:
    encoderCalibrator.isTriggered = true;
    encoderCalibrator.Tick20kHz();
    encoderCalibrator.TickMainLoop();
```
**特点**：运行时动态触发，适合现场维护。

---

## 4. 校准运行流程

### 4.1 状态机设计

```mermaid
stateDiagram-v2
    [*] --> CALI_DISABLE
    CALI_DISABLE --> CALI_FORWARD_PREPARE: isTriggered = true
    CALI_FORWARD_PREPARE --> CALI_FORWARD_MEASURE: goPosition = 2×51200
    CALI_FORWARD_MEASURE --> CALI_BACKWARD_RETURN: goPosition > 2×51200
    CALI_BACKWARD_RETURN --> CALI_BACKWARD_GAP_DISMISS: 回退偏移量
    CALI_BACKWARD_GAP_DISMISS --> CALI_BACKWARD_MEASURE: goPosition = 2×51200
    CALI_BACKWARD_MEASURE --> CALI_CALCULATING: goPosition < 51200
    CALI_CALCULATING --> CALI_DISABLE: 计算完成 → 写 Flash
    
    note right: 
        CALI_FORWARD_MEASURE：正向采集 200 个硬步的中心点数据
        CALI_BACKWARD_MEASURE：反向采集同一位置数据
        双向平均可消除迟滞误差
    end note
```

### 4.2 详细步骤

#### 步骤 1: `CALI_FORWARD_PREPARE`（预旋转）
```cpp
goPosition += AUTO_CALIB_SPEED;  // 2 步/20kHz = 400 步/s
motor->driver->SetFocCurrentVector(goPosition, motor->config.motionParams.caliCurrent);
```
- **目标**: 快速旋转至起始点（1 圈位置）
- **电流**: 使用配置的校准电流（默认 2000mA）
- **耗时**: ~1s

#### 步骤 2: `CALI_FORWARD_MEASURE`（正向采样）
```cpp
if ((goPosition % motor->SOFT_DIVIDE_NUM) == 0)  // 每 256 步采一次
{
    sampleDataRaw[sampleCount++] = motor->encoder->angleData.rawAngle;
    if (sampleCount == 16)  // 16 次采样取平均
    {
        sampleDataAverageForward[...] = CycleDataAverage(sampleDataRaw, ...);
    }
}
```
- **采样频率**: 每软细分 1 个点，共 256 点/硬步 × 200 硬步 = **51200 点**
- **但实际只记录 200+1 个硬步中心点**（因后续会插值）
- **抗噪**: 每个点 16 次连续采样平均，抑制高频噪声

#### 步骤 3: `CALI_BACKWARD_RETURN`（回退跳过无效区）
- 快速回退约 20 个软细分，避免端点非线性区域

#### 步骤 4: `CALI_BACKWARD_MEASURE`（反向采样）
与正向采样完全对称，得到 `sampleDataAverageBackward[]`

#### 步骤 5: `CALI_CALCULATING`（数据处理）
```cpp
state = CALI_CALCULATING;
motor->driver->SetFocCurrentVector(0, 0);  // 断电保持
```
暂停控制，进入 `TickMainLoop()` 进行离线处理（见后文）。

---

## 5. 数据分析算法

### 5.1 数据质量检查

```cpp
void CalibrationDataCheck()
{
    // ① 正反向平均一致性检查
    for each step:
        sampleDataAverageForward[count] = (fwd + bwd) / 2
    
    // ② 方向性检查（单调递增/递减）
    delta = fwd[0] - fwd[last_step]
    if delta == 0 → CALI_ERROR_AVERAGE_DIR
    
    // ③ 连续性检查（相邻步差值合法）
    delta = fwd[count] - fwd[count-1]
    if abs(delta) > 1.5×理论值 → CALI_ERROR_AVERAGE_CONTINUTY
    if abs(delta) < 0.5×理论值 → CALI_ERROR_AVERAGE_CONTINUTY
    if delta == 0 → CALI_ERROR_AVERAGE_DIR
    
    // ④ 相位跳变点检测（找过零点）
    count negative steps = 0
    for each step:
        if (delta < 0):
            rcdX = count          // 负跳变起始硬步
            rcdY = encoder_val    // 负跳变起始软细分
            step_num++
    
    if step_num != 1 → CALI_ERROR_PHASE_STEP
}
```

### 5.2 错误类型

| 错误码 | 含义 | 可能原因 |
|--------|------|----------|
| `CALI_ERROR_AVERAGE_DIR` | 方向不一致 | 接线反了 / 电机打滑 |
| `CALI_ERROR_AVERAGE_CONTINUTY` | 相邻点差异过大 | 编码器受干扰 / 机械振动 |
| `CALI_ERROR_PHASE_STEP` | 相位跳变点异常 | 编码器安装偏了 |
| `CALI_ERROR_ANALYSIS_QUANTITY` | 数据量不对 | Flash 写入失败 |

---

## 6. 查找表生成与写入

### 6.1 插值公式

假设检测到负跳变发生在硬步 `rcdX`、软细分 `rcdY`，则：

```cpp
for x = rcdX to rcdX + 200:
    dataI32 = angle[x+1] - angle[x]   // 该硬步包含的软点数
    for y = 0 to dataI32:
        dataU16 = mod(SOFT_DIVIDE_NUM * x + SOFT_DIVIDE_NUM * y / dataI32, 51200)
        WriteFlash16bitsAppend(dataU16)
```

**物理意义**：将 16384 个编码器原始值映射到 51200 个标准步位置，形成**均匀化查找表**。

### 6.2 Flash 存储规划

- **分区地址**: `STOCKPILE_APP_CALI_ADDR = 0x08017C00`（32K）
- **占用空间**: 16384 × 2 bytes = **32KB**（占满）
- **数据格式**: 16-bit 无符号整数，表示校正后的编码器原始值

---

## 7. 运行时查表修正

### 7.1 应用位置

在编码器驱动中（未展示），每次读取原始角度后：

```cpp
uint16_t raw_angle = SPI_Read();
uint16_t calibrated_angle = lookup_table(raw_angle);
```

### 7.2 效果验证

- **校准前**: 空载电流波形存在周期性纹波
- **校准后**: 电流正弦度提升，噪声降低 ≥3dB

---

## 8. 操作流程指南

### 8.1 手动校准流程

1. **固定电机轴**（防止转动伤人）
2. **上电**同时按住 **Key1 + Key2**，或运行时 **长按 Key1**
3. 等待约 **15 秒**（完成整圈扫描）
4. 观察状态 LED：
   - **快闪**：校准成功 → 自动复位
   - **慢闪**：校准失败 → 检查机械结构
5. 复位后执行一次 `GetSerialNumber()` 查看校准结果日志

### 8.2 自动化建议

- 生产线上可集成夹具，通电即自检
- 若连续 3 次校准失败，标记为不良品

---

## 9. 安全注意事项

⚠️ **重要警告**：
- 校准过程中电机会以 **2000mA 电流旋转**，务必确认周围无障碍物！
- 建议先测试时使用低速模式（修改 `AUTO_CALIB_SPEED`）
- 校准期间禁止触摸电机轴

---

## 10. 故障排查

| 现象 | 可能原因 | 解决方案 |
|------|----------|----------|
| 校准卡死在 `CALI_FORWARD_PREPARE` | 堵转 / 电流不足 | 增大 `calibrationCurrent` |
| 反复报 `CALI_ERROR_PHASE_STEP` | 编码器安装偏心 | 重新校准机械同轴度 |
| Flash 写入失败 | 电压不稳 | 检查电源滤波电容 |
| 校准后噪音更大 | 映射表损坏 | 清除 Flash 后重试 |

---

## 附录 A：关键常量定义

```cpp
// encoder_calibrator_base.h
MOTOR_ONE_CIRCLE_HARD_STEPS = 200      // 1.8°步进
SAMPLE_COUNTS_PER_STEP = 16            // 抗扰采样数
AUTO_CALIB_SPEED = 2                   // 预旋速度 (步/20kHz)
FINE_TUNE_CALIB_SPEED = 1              // 采样速度 (步/20kHz)
```

## 附录 B：相关文件清单

| 文件 | 作用 |
|------|------|
| `Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp` | 核心算法实现 |
| `Port/encoder_calibrator_stm32.cpp` | Flash 操作接口 |
| `UserApp/main.cpp` | 触发逻辑与事件回调 |
| `Port/Platform/Memory/stockpile_f103cb.*` | Flash 分区管理 |
