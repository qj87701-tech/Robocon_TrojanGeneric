# DM-J8006-2EC 按键速度控制设计

## 1. 背景

目标工程 `Robocon-Elian` 已经逐字节复刻并接入中科大工程的 `Motor_DM`、`drv_can`、`alg_basic`、`alg_pid`、`sys_timestamp` 与 CMSIS-DSP 静态库。当前这些模块仅参与编译和链接，应用层尚未创建电机对象、启动 CAN 驱动或发送控制帧。

本阶段在 STM32H723ZGT6 目标板上驱动一台 DM-J8006-2EC V1.1 减速电机。电机已经通过达妙调试助手设置为速度模式，使用 FDCAN1；KEY1 和 KEY2 用于离散调整输出轴目标速度。

## 2. 已确认的硬件和电机参数

### 2.1 电机与 CAN

- 电机：DM-J8006-2EC V1.1，减速比 6:1。
- CAN 总线：FDCAN1。
- CAN 波特率：1 Mbps，由 CubeMX 和电机参数共同保证。
- 电机 CAN ID：`0x03`。
- Master ID（反馈帧 ID）：`0x33`。
- 控制模式：速度模式，调试助手编码为 3。
- PMAX：`12.5 rad`。
- VMAX：`45 rad/s`。
- TMAX：`20 Nm`。
- 阻尼因子：4，已经在电机参数中配置。
- CAN Timeout：保持 0，不在本阶段修改。

速度模式的控制帧使用标准 CAN ID `0x200 + 0x03 = 0x203`。数据区为一个低字节在前的 32 位浮点数，单位为减速后输出轴的 `rad/s`。例如 `1.0f` 的四个字节为 `00 00 80 3F`。

### 2.2 按键

- KEY1：PF2。
- KEY2：PF4。
- 两个按键均由外部 10 kΩ 电阻上拉到 3.3 V，按下时连接 GND，因此低电平表示按下。
- 每个按键均有 100 nF 硬件滤波电容。
- CubeMX 配置为 `GPIO_Input + GPIO_NOPULL`。
- 不使用 EXTI；按键由前台任务轮询。

## 3. 目标行为

- 系统上电后完成初始化，目标速度保持 `0 rad/s`，然后自动使能电机。
- KEY1 每次有效按下，将目标速度增加 `1 rad/s`。
- KEY2 每次有效按下，将目标速度减少 `1 rad/s`。
- 按住按键不会连续增加或减少；必须释放后再次按下才触发下一次调整。
- KEY1 与 KEY2 同时按下时，目标速度立即变为 `0 rad/s`，并抑制本次组合按键产生的单键加减，直到两键释放。
- 软件目标速度限制在 `-5 rad/s` 到 `+5 rad/s`。
- 不新增公开的目标速度全局变量、读取函数、串口输出或调试接口。

## 4. 架构

```text
Core/Src/main.c
  └─ Task_Init() / Task_Loop()
       ├─ User_File/2_Device/BSP/Key
       │    └─ 按键采样、消抖和单次触发状态
       ├─ User_File/2_Device/Motor/Motor_DM
       │    └─ 使能、速度控制帧、反馈解析、状态检测
       ├─ User_File/1_Middleware/Driver/CAN
       │    └─ FDCAN1 初始化、收发和 HAL 接收回调
       └─ User_File/1_Middleware/System/Timestamp
            └─ TIM5 微秒时间戳
```

原样复刻的中科大模块保持不变。目标板特有的按键绑定、电机实例、任务调度和回调分发放在新增按键 BSP 与现有任务接入层中。

## 5. 按键 BSP

在用户已经创建的目录中添加：

```text
User_File/2_Device/BSP/Key/
├─ bsp_key.h
└─ bsp_key.cpp
```

按键 BSP 参考中科大 `Class_Key` 的状态机思路，但改造成可在初始化时绑定 `GPIO_TypeDef*` 和引脚号的通用类，从而支持 KEY1 和 KEY2 两个独立实例。

按键状态至少区分：

- 释放；
- 释放到按下的单次触发；
- 按下；
- 按下到释放的单次触发。

任务层每 10 ms 调用一次按键处理。输入连续两个采样周期保持一致后才更新稳定状态，从而提供约 20 ms 的软件消抖。只有“释放到按下”的状态触发一次速度调整。

## 6. 应用接入层

扩展：

```text
User_File/4_Task/tsk_config_and_callback.h
User_File/4_Task/tsk_config_and_callback.cpp
```

任务层拥有：

- 一个 `Class_Motor_DM_Normal` 电机对象；
- KEY1 与 KEY2 两个按键对象；
- 前台调度时间戳；
- 双键归零锁存状态。

不额外保存或导出目标速度。按键逻辑通过 `Get_Control_Omega()` 读取电机对象当前命令，通过 `Set_Control_Omega()` 写入限幅后的新命令。

### 6.1 `Task_Init()` 顺序

1. 将 `init_finished` 保持为 `false`。
2. 使用 TIM5 初始化 `SYS_Timestamp`。
3. 初始化 DM-J8006 对象：FDCAN1、Master ID `0x33`、CAN ID `0x03`、普通速度模式、PMAX `12.5`、VMAX `45`、TMAX `20`。
4. 初始化 KEY1（PF2）和 KEY2（PF4）对象。
5. 将目标速度设置为 `0 rad/s`。
6. 调用 `CAN_Init(&hfdcan1, CAN1_Callback)`，配置过滤器、通知并启动 FDCAN1。
7. 启动 TIM5 基本定时器中断。
8. 将 `init_finished` 设置为 `true`，允许 `drv_can` 分发接收回调。
9. 发送电机使能命令。

初始化期间即使 CAN FIFO 收到数据，`drv_can` 也会在 `init_finished == false` 时只清空 FIFO，不调用尚未准备好的上层对象。

### 6.2 `Task_Loop()` 调度

不新增 CubeMX 定时器，使用 `HAL_GetTick()` 进行无阻塞前台调度：

- 每 10 ms：处理两个按键，执行步进/归零逻辑，然后调用电机发送周期回调。
- 每 100 ms：调用电机在线检测回调。

计时比较使用无符号时间差，允许 HAL tick 自然回绕。任务循环不使用阻塞延时。

### 6.3 CAN 回调

`CAN1_Callback` 按 `Header.Identifier` 分发：

- `0x33`：调用 DM-J8006 对象的 `CAN_RxCpltCallback()`；
- 其他 ID：忽略，为未来 FDCAN1 设备保留扩展空间。

### 6.4 TIM5 回调

任务接入层实现 `HAL_TIM_PeriodElapsedCallback()`。仅当回调来源为 TIM5 时，调用
`SYS_Timestamp.TIM_3600s_PeriodElapsedCallback()`；其他定时器实例不在本阶段处理，
为后续模块保留扩展空间。

## 7. 中科大原版行为的保留范围

- 不修改 `Motor_DM`、`drv_can`、`alg_basic`、`alg_pid` 或 `sys_timestamp`。
- 每 10 ms 调用原版 `TIM_Send_PeriodElapsedCallback()`：
  - 电机处于使能状态时发送速度控制帧；
  - 电机处于失能状态时自动发送使能；
  - 电机报告错误时发送清错并重新使能。
- 每 100 ms 调用原版 `TIM_100ms_Alive_PeriodElapsedCallback()`；没有新反馈时沿用原版自动发送使能的策略。
- TIM5 一小时溢出时调用 `SYS_Timestamp.TIM_3600s_PeriodElapsedCallback()`。

不增加“反馈丢失后失能锁定”策略。电机内部 CAN Timeout 保持关闭，因此主控断电、程序完全卡死或 CAN 物理断线时，软件不能保证电机自动归零；这是用户明确接受的当前限制。

## 8. `main.c` 与 CMake 接入

本阶段新增的手写接入代码只放在 `main.c` 的 CubeMX `USER CODE` 区域；保留 CubeMX
为 KEY1、KEY2 等配置自动生成的其他变化。手写接入包括：

- 包含 `tsk_config_and_callback.h`；
- 在所有 CubeMX 外设初始化完成后调用 `Task_Init()`；
- 在 `while (1)` 中调用 `Task_Loop()`。

任务头文件为 C 调用方提供具有 C 链接的 `Task_Init()` 和 `Task_Loop()`，实现仍由 C++ 编译。

顶层 `CMakeLists.txt` 显式加入 `User_File/2_Device/BSP/Key/bsp_key.cpp`。继续只修改顶层 CMake，不修改 CubeMX 生成的 `cmake/stm32cubemx/CMakeLists.txt`。

## 9. 非目标

- 不修改电机内部参数或向寄存器写入控制模式、VMAX、TMAX、Master ID、CAN ID、阻尼因子或 CAN Timeout。
- 不增加新的 CubeMX 定时器或 EXTI。
- 不增加串口、USB、VOFA、日志或调试变量。
- 不加入 IMU、ADC、遥控器或其他设备。
- 不修改原样复刻的中科大源码来适配当前应用。

## 10. 验证

### 10.1 静态和构建验证

- 检查 CubeMX 生成配置仍为 KEY1=PF2、KEY2=PF4、GPIO 输入、无内部上下拉。
- 检查应用参数为 FDCAN1、`0x33`、`0x03`、普通速度模式、`12.5/45/20`。
- 检查软件限幅为 `-5～+5 rad/s`，步进为 `1 rad/s`。
- 检查本阶段写入 `main.c` 的手写接入只位于 `USER CODE` 区域，并保留现有 CubeMX
  生成改动；CubeMX 子 CMake 保持不变。
- 对之前复刻的十个源码文件和 DSP 静态库重新执行 SHA-256 一致性验证。
- 使用 CLion CMake/Ninja 对 Debug 配置执行 clean-first 构建和完整链接。
- 检查 ELF 中不存在 `init_finished`、`SYS_Timestamp`、`hfdcan1` 等意外未定义符号。

### 10.2 上板验证

上板测试不由离线构建代替。测试时先架空输出轴并确保可立即断电：

1. 上电后确认电机使能但保持静止。
2. 单击 KEY1，确认输出轴约以 `1 rad/s`（约 9.55 rpm）正向转动。
3. 再次单击 KEY1，确认目标变为 `2 rad/s`。
4. 单击 KEY2，确认目标回到 `1 rad/s`。
5. 同时按下两个按键，确认目标归零。
6. 分别重复按键，确认目标不会超过 `+5 rad/s` 或低于 `-5 rad/s`。
7. 长按任一按键，确认只触发一次；释放后再次按下才再次触发。

## 11. 完成定义

当按键 BSP、任务接入、main USER CODE 调用与顶层 CMake 接入完成，Debug clean-first 构建通过，原版模块哈希保持一致，并由用户完成上述上板步骤后，本阶段 DM-J8006 按键速度控制完成。
