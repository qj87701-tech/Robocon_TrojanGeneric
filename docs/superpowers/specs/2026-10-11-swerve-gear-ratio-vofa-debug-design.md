# 舵轮转向传动比与 VOFA 调试模块设计

## 1. 目标

本设计完成以下两项工作：

1. 将 DM3519 输出轴、17 齿主动齿轮、215 齿从动齿轮和舵轮转向机构的实际传动关系接入四舵轮底盘控制链路。
2. 将 USART9 VOFA 调试代码从 `tsk_config_and_callback` 中独立到 `tsk_debug`，统一封装四套 `DM + VESC` 轮组，当前默认观察左前轮 `DM ID 1 + VESC ID 1`。

硬件关系为：

```text
DM3519 输出轴
  → 17 齿主动齿轮
  → 215 齿从动齿轮
  → 舵轮转向机构
```

舵轮转向机构允许无限连续旋转。DM3519 不带减速箱，使用位置速度模式。

## 2. 本阶段不处理的内容

- 不解决整车断电后舵轮绝对朝向恢复问题。
- 不增加舵轮输出端绝对编码器、归零开关或回零状态机。
- 不补偿齿轮间隙、传动效率和弹性形变。
- 不调整 DM3519 控制增益。
- 不调整 VESC FOC、速度 PID、电流限制和 Observer 参数。
- 不增加 DM 掉线或连续角度未初始化时联动关闭 VESC 的逻辑。
- 不改变遥控离线、整车电机失能等现有安全行为。

## 3. 当前代码问题

当前 `mod_chassis_swerve` 输出的是舵轮机构目标角度，`tsk_chassis` 却直接将该角度传给 DM3519，未进行传动比换算；DM反馈角度也被直接当成舵轮机构角度。

当前关系实际缺少：

```text
传动比 = 215 / 17 = 12.6470588
```

外啮合还会使电机轴与舵轮机构旋转方向相反。

DM上位机实测表明：位置速度模式可以接受 `79.464 rad` 多圈目标，但常规CAN位置反馈会在 `-12.5 rad` 到 `+12.5 rad` 之间回绕。目标 `79.464 rad` 最终显示约 `4.464 rad`，符合：

```text
79.464 - 3 × 25 = 4.464 rad
```

因此必须区分：

- 位置速度模式的多圈目标角度；
- 周期为 `25 rad` 的原始反馈角度；
- STM32根据相邻反馈解包得到的连续电机轴角度；
- 经过17:215换算后的连续舵轮机构角度。

## 4. 分层原则

采用以下职责划分：

```text
遥控速度
  ↓
mod_chassis_swerve
  计算机构目标角度、轮速和行进轮反向选择
  ↓
tsk_chassis
  机构角度 × 215/17 → DM电机轴目标角度
  机构轮速 → VESC机械角速度
  ↓
DM3519 / VESC

DM回绕角度
  ↓
dvc_motor_dm
  按25rad周期解包为连续电机轴角度
  ↓
tsk_chassis
  连续电机轴角度 × 17/215 → 连续机构角度
  ↓
mod_chassis_swerve反馈 + tsk_debug诊断
```

`mod_chassis_swerve` 保持硬件无关，不感知DM型号、齿轮齿数、CAN ID和VOFA。

## 5. DM3519连续角度设计

### 5.1 数据定义

`Class_Motor_DM_Normal` 保留原始反馈角度，并增加连续角度状态：

```cpp
Now_Angle               // 原始反馈角度，约-12.5～12.5 rad
Previous_Angle          // 上一帧原始反馈角度
Now_Total_Angle         // STM32解包后的连续电机轴角度
Total_Angle_Initialized // 是否已经收到第一帧有效角度
```

增加只读接口，用于任务层读取连续电机轴角度和初始化状态。原有 `Get_Now_Angle()` 语义保持不变，避免影响其他DM使用者。

### 5.2 跨界解包

反馈周期为：

```text
Feedback_Period = 2 × Angle_Max = 25 rad
```

首帧反馈用于初始化：

```text
Previous_Angle = Now_Angle
Now_Total_Angle = Now_Angle
```

后续每帧执行：

```cpp
delta = Now_Angle - Previous_Angle;

if (delta > Angle_Max)
{
    delta -= 2.0f * Angle_Max;
}
else if (delta < -Angle_Max)
{
    delta += 2.0f * Angle_Max;
}

Now_Total_Angle += delta;
Previous_Angle = Now_Angle;
```

该算法依赖相邻有效反馈间的实际转角绝对值小于 `12.5 rad`。当前DM反馈周期和速度上限满足正常运行需求；长时间掉线后的多圈歧义作为诊断状态保留，本阶段不联动VESC。

### 5.3 控制目标限幅

当前 `TIM_Send_PeriodElapsedCallback()` 对所有模式统一执行 `Control_Angle` 的 `±Angle_Max` 限幅，会把位置速度模式的多圈目标错误截断。

修改原则：

- `Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA`：位置目标为32位浮点多圈角度，不执行 `±Angle_Max` 软件限幅。
- `Motor_DM_Control_Method_NORMAL_MIT`：继续执行 `±Angle_Max` 限幅。
- 其他控制模式保持现有行为，不扩大本次改动范围。
- 角速度、扭矩、电流、`K_P` 和 `K_D` 继续执行原有限幅。
- CAN ID、报文长度和字段布局保持不变。

## 6. 舵轮机构换算设计

### 6.1 常量

任务层集中定义：

```cpp
#define SWERVE_STEER_GEAR_RATIO (215.0f / 17.0f)
#define SWERVE_STEER_GEAR_DIRECTION (-1.0f)
```

`SWERVE_STEER_GEAR_DIRECTION` 的负号表示外啮合反向。四个轮组继续分别保留安装方向和零点偏移，用于处理装配差异。

### 6.2 电机反馈到机构角度

```text
机构实际角度
= (DM连续电机轴角度 - DM电机轴零点)
× 17/215
× 外啮合方向
× 轮组安装方向
```

机构实际角速度：

```text
机构实际角速度
= DM电机轴实际角速度
× 17/215
× 外啮合方向
× 轮组安装方向
```

### 6.3 机构目标到电机目标

```text
DM目标角度
= DM电机轴零点
+ 机构目标角度
× 215/17
× 外啮合方向
× 轮组安装方向
```

每个DM在舵轮朝正前方时保存零点，因此第一阶段四个软件零点偏移均设为 `0 rad`，但保留独立常量以便实车校准。

### 6.4 速度限制

当前DM电机轴目标速度限制为：

```text
37.699 rad/s = 360 rpm
```

换算为舵轮机构速度：

```text
37.699 × 17 / 215 ≈ 2.981 rad/s ≈ 170.8°/s
```

位置速度模式中的该参数继续表示DM电机轴速度限制，VOFA显示时换算为机构侧 `°/s`。

## 7. 舵轮运动学设计

`mod_chassis_swerve` 继续以机构角度为统一单位，保留现有：

- 四个轮组位置矢量计算；
- `atan2f()`目标角度计算；
- 静止时保持上一目标角度；
- 正向目标和反向目标的最短路径比较；
- 行进轮反转；
- 10°切换迟滞；
- 舵向角误差限速；
- 四轮速度同比例限幅。

初始化时不再传入DM电机轴的 `12 rad`软限位：

```cpp
Chassis_Swerve.Init(CHASSIS_SWERVE_WHEEL_RADIUS,
                    CHASSIS_SWERVE_RADIUS,
                    0.0f);
```

`0.0f` 表示机构角度不设软限位。解算器在连续角度空间中选择距离当前位置最近的等效目标；结合行进轮反转，正常单次舵向变化保持在最短路径附近。

## 8. 四轮诊断数据

`tsk_chassis` 将现有只包含左前轮两个数值的诊断结构扩展为四个轮组统一数据：

```text
机构目标角度             rad
机构实际连续角度         rad
机构角度误差             rad
机构实际角速度           rad/s
机构目标速度限制         rad/s
舵向误差限速比例         0～1
连续角度初始化状态       0/1
```

调试模块读取这些机构侧数据，不重复实现传动比公式。DM电机轴扭矩、温度和状态仍直接读取对应DM对象。

## 9. tsk_debug设计

### 9.1 文件和接口

新增：

```text
User_File/4_Task/tsk_debug/tsk_debug.h
User_File/4_Task/tsk_debug/tsk_debug.cpp
```

提供：

```cpp
void Debug_Init();
void Debug_UART9_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
void Debug_20ms_PeriodElapsedCallback();
void Debug_Set_Swerve_Wheel(Enum_Swerve_Wheel Wheel);
```

### 9.2 四轮对象映射

```text
LF：DM ID 1 + VESC ID 1
RF：DM ID 2 + VESC ID 2
LB：DM ID 3 + VESC ID 3
RB：DM ID 4 + VESC ID 4
```

默认选择：

```cpp
Debug_Set_Swerve_Wheel(Swerve_Wheel_LF);
```

ID 2～4的映射和选择接口同时完成，但当前不接入遥控或VOFA命令进行动态切换。

### 9.3 USART9 VOFA通道

现有 `Class_Vofa_UART` 最多支持24个通道，本设计使用18个：

| 通道 | 数据 | 单位 | 来源 |
| --- | --- | --- | --- |
| I0 | VESC目标机械转速 | rpm | VESC控制目标机械角速度换算 |
| I1 | VESC实际机械转速 | rpm | VESC STATUS反馈换算 |
| I2 | VESC速度误差 | rpm | I0 - I1 |
| I3 | VESC电机电流 | A | STATUS |
| I4 | VESC占空比 | % | STATUS |
| I5 | VESC输入电流 | A | STATUS_4 |
| I6 | VESC母线电压 | V | STATUS_5 |
| I7 | VESC FET温度 | ℃ | STATUS_4 |
| I8 | 舵轮机构目标角度 | ° | `tsk_chassis`诊断数据 |
| I9 | 舵轮机构实际连续角度 | ° | `tsk_chassis`诊断数据 |
| I10 | 舵轮机构角度误差 | ° | `tsk_chassis`诊断数据 |
| I11 | 舵轮机构实际角速度 | °/s | `tsk_chassis`诊断数据 |
| I12 | DM电机轴反馈扭矩 | N·m | DM反馈，不冒充机构输出扭矩 |
| I13 | 舵轮机构目标速度限制 | °/s | `tsk_chassis`诊断数据 |
| I14 | DM驱动器状态码 | 数值 | DM控制状态反馈 |
| I15 | DM MOS温度 | ℃ | DM反馈，内部开尔文值换算 |
| I16 | DM电机温度 | ℃ | DM反馈，内部开尔文值换算 |
| I17 | DM软件在线状态 | 0/1 | DM 100ms存活检测状态 |

### 9.4 发送时序

- `Debug_20ms_PeriodElapsedCallback()` 在裸机20ms周期中调用。
- VESC的 `STATUS`、`STATUS_4`、`STATUS_5` 均有效，并且收到新的 `STATUS_5` 后才发送一帧。
- 不以高于VESC反馈更新频率的速度重复发送旧数据。
- DM数据与该次VESC数据在同一时刻生成快照并组成同一个JustFloat帧。
- 不在10ms底盘控制函数中调用阻塞式串口输出。

### 9.5 tsk_config_and_callback边界

`tsk_config_and_callback.cpp` 删除以下具体实现：

- `Class_Vofa_UART Vofa_UART9`对象；
- 现有8个VESC VOFA全局变量；
- `Vofa_VESC_Status_5_Previous_Update_Count`；
- `UART9_Vofa_Output()`具体拼帧逻辑；
- `Task_Init()`中的18通道注册细节。

该文件只保留：

- UART9硬件回调到 `Debug_UART9_RxCpltCallback()` 的转发；
- `Task_Init()`中调用 `Debug_Init()`；
- 20ms任务中调用 `Debug_20ms_PeriodElapsedCallback()`。

## 10. 故障与状态处理

- DM在线状态和连续角度初始化状态仅用于诊断，不联动修改VESC目标。
- VESC数据未形成完整有效状态组时不发送VOFA帧，避免把旧数据显示为实时数据。
- 选择轮组编号越界时保持原选择，不访问非法对象。
- USART9未初始化或VOFA未配置数据时沿用 `Class_Vofa_UART` 的静默返回行为。
- 遥控离线、整车电机失能和原有CAN超时行为不因本设计改变。

## 11. 测试设计

### 11.1 DM主机测试

- 首帧角度初始化连续角度。
- `+12.4 rad → -12.4 rad`时连续角度继续正向增加。
- `-12.4 rad → +12.4 rad`时连续角度继续反向减少。
- 普通小角度变化不误判跨界。
- 位置速度模式的 `79.464 rad`目标不被截成 `12.5 rad`。
- MIT模式仍保留 `±12.5 rad`保护。

### 11.2 舵轮解算测试

- 纯前后、纯左右、原地旋转和组合运动结果正确。
- 静止时保持上一舵向目标。
- 正反向目标选择保持最短路径。
- 90°附近的10°迟滞不发生频繁翻转。
- 当前连续机构角度超过一圈时，目标仍连续。

### 11.3 任务层传动比测试

- DM电机轴转动 `215/17` 圈时，机构角度变化一圈。
- 机构目标一圈对应DM目标 `215/17` 圈。
- 外啮合方向反转正确。
- 每轮安装方向和零点只影响对应轮组。
- 机构角度误差限速使用机构侧角度，不使用电机轴角度。

### 11.4 调试模块测试

- 默认选择 `DM ID 1 + VESC ID 1`。
- ID 1～4轮组映射正确。
- 18个通道的顺序、单位和数值来源固定。
- DM角度和速度使用机构侧数据。
- DM温度正确从开尔文转换为摄氏度。
- 没有新 `STATUS_5` 时不重复发送。
- 新完整状态组到达时发送一帧76字节JustFloat数据。

### 11.5 完整验证

- 运行与改动直接相关的主机测试。
- 运行全部现有主机测试。
- 完整编译并链接Debug固件目标 `Robocon-Elian.elf`。
- 实物验证时先仅连接左前 `DM ID 1 + VESC ID 1`，低速确认方向、零点、连续角度和VOFA通道，再逐轮启用ID 2～4。

## 12. 预计修改文件

- `User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.h`
- `User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp`
- `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.h`
- `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp`
- `User_File/4_Task/Config/motor_init.h`
- `User_File/4_Task/Config/motor_init.cpp`
- `User_File/4_Task/tsk_chassis/tsk_chassis.h`
- `User_File/4_Task/tsk_chassis/tsk_chassis.cpp`
- `User_File/4_Task/tsk_debug/tsk_debug.h`（新增）
- `User_File/4_Task/tsk_debug/tsk_debug.cpp`（新增）
- `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp`
- `CMakeLists.txt`（加入 `tsk_debug.cpp`）
- `tests/host` 下与DM、舵轮任务和VOFA调试直接相关的测试与测试桩

不修改CubeMX生成的 `Core`、`Drivers` 和 `cmake/stm32cubemx` 内容。

## 13. 验收标准

- 舵轮机构转向角与DM电机轴角严格按17:215换算。
- 外啮合反向明确且四轮可独立校准。
- DM位置速度模式可以发送多圈目标。
- STM32能够把周期25rad的DM反馈还原为本次上电期间的连续角度。
- 舵轮运动学始终使用机构侧连续角度。
- USART9一次输出固定18个VOFA通道。
- 调试模块默认观察左前轮，并已封装四轮切换能力。
- `tsk_config_and_callback`不再承担具体VOFA数据组织职责。
- 不引入RTOS、队列、SUBS或新的上层框架。
- 主机测试和Debug固件构建通过；实物方向和零点验证明确留给烧录后的低速测试。
