# Motor_DM 最小依赖复刻设计

## 1. 背景

目标工程为 `Robocon-Elian`，MCU 是 STM32H723ZGT6，使用 STM32CubeMX 生成 CMake 工程并由 CLion 开发。参考工程为中科大 `dm02_test`，MCU 是 STM32H723VGT6，使用 STM32CubeIDE 工程。

本阶段只把参考工程中已经封装好的达妙电机模块及其最小依赖复刻到目标工程。复刻完成后代码必须可以编译和链接，但不配置具体电机型号、CAN ID 或控制参数，也不产生任何电机控制行为。

## 2. 目标

- 原样复制 `Motor_DM` 及其最小源码依赖。
- 保持参考源码的接口、目录关系和行为不变。
- 在目标工程顶层 CMake 中接入 C++ 源码和 CMSIS-DSP 静态库。
- 建立可继续承载 IMU、通信、ADC 等模块的统一应用接入层。
- 完成 Debug 配置的编译和链接验证。

## 3. 非目标

- 不实例化 DM 电机对象。
- 不选择普通模式、一拖四模式或具体控制模式。
- 不设置电机型号、CAN ID、目标位置、速度、力矩或 PID 参数。
- 不启动 `drv_can` 封装层，不注册电机 CAN 回调，不发送 CAN 报文。
- 不修改目标工程的芯片、引脚、时钟、启动文件或链接脚本。
- 不复制参考工程中的 DJI 电机、BMI088、USB、ADC、SPI、OSPI、VOFA、滤波器或板级测试代码。
- 不进行上板电机功能测试。

## 4. 已满足的 CubeMX 前置条件

- FDCAN1、FDCAN2、FDCAN3 的 Tx FIFO/Queue elements 均为 8。
- TIM5 输入时钟为 240 MHz，Prescaler 为 `240-1`，计数频率为 1 MHz。
- TIM5 Counter Period 为 `3600000000-1`，对应一小时溢出一次。
- TIM5 全局中断和 IRQ Handler 已生成。
- X-CUBE-ALGOBUILD DSP Library 已启用，`arm_math.h` 已生成并加入 CubeMX CMake include path。

## 5. 文件范围

### 5.1 原样复制的参考源码

以下文件从 `dm02_test` 复制到 `Robocon-Elian` 后必须与参考文件逐字节一致：

```text
User_File/
├─ 1_Middleware/
│  ├─ Algorithm/
│  │  ├─ Basic/
│  │  │  ├─ alg_basic.h
│  │  │  └─ alg_basic.cpp
│  │  └─ PID/
│  │     ├─ alg_pid.h
│  │     └─ alg_pid.cpp
│  ├─ Driver/
│  │  └─ CAN/
│  │     ├─ drv_can.h
│  │     └─ drv_can.cpp
│  └─ System/
│     └─ Timestamp/
│        ├─ sys_timestamp.h
│        └─ sys_timestamp.cpp
└─ 2_Device/
   └─ Motor/
      └─ Motor_DM/
         ├─ dvc_motor_dm.h
         └─ dvc_motor_dm.cpp
```

CMSIS-DSP 静态库也从参考工程复制并保持二进制一致：

```text
Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a
```

目标工程已经由 CubeMX 生成以下头文件，不重复覆盖：

```text
Middlewares/ST/ARM/DSP/Inc/arm_math.h
```

### 5.2 目标工程本地接入层

建立以下目标工程专用文件：

```text
User_File/4_Task/
├─ tsk_config_and_callback.h
└─ tsk_config_and_callback.cpp
```

这些文件学习参考工程的集中式应用接入结构，但不要求与参考工程内容一致。本阶段只提供 `init_finished` 全局初始化状态，值固定为 `false`。头文件只暴露该状态；源文件负责唯一一次定义。

未来加入 IMU、通信、ADC 等模块时，该接入层可逐步承载：

- 驱动初始化顺序；
- 设备对象初始化；
- CAN、SPI、UART、ADC、GPIO 回调分发；
- 周期任务入口；
- `Task_Init()` 和 `Task_Loop()`；
- 所有模块初始化完成后将 `init_finished` 设置为 `true`。

具体业务任务增长后可以拆分到其他 `tsk_*.cpp` 文件，但 HAL 全局回调入口应保持唯一，避免重复定义。

## 6. 架构与依赖方向

```text
应用接入层
tsk_config_and_callback
        │ 使用
        ▼
设备封装层
dvc_motor_dm
        │ 使用
        ▼
基础依赖层
alg_basic、alg_pid、sys_timestamp、drv_can
        │ 使用
        ▼
CubeMX 自动生成层
Core/、Drivers/、cmake/stm32cubemx/
```

常规依赖从应用层指向设备层、基础依赖层和 CubeMX/HAL。参考 `drv_can` 还声明了由应用接入层提供的外部状态 `init_finished`，这是保留原版源码产生的唯一反向依赖；第 7 节明确其定义位置和安全职责。除此之外，参考模块不依赖目标工程的业务对象或未来任务实现。

## 7. `init_finished` 的职责

参考 `drv_can.h` 声明了外部变量 `init_finished`，参考工程在 `tsk_config_and_callback.cpp` 中定义它。FDCAN 接收中断使用该变量保护回调分发：

- `false`：只读取并清空接收 FIFO，不调用上层设备回调。
- `true`：正常调用已经注册的 CAN 回调。

本阶段不复制参考工程完整任务文件，否则会引入大量无关设备依赖。因此目标工程本地接入层提供同名全局变量，并保持为 `false`。不通过修改 `drv_can` 来消除这项依赖。

## 8. CMake 接入

只修改目标工程顶层 `CMakeLists.txt`。不修改 CubeMX 自动生成的 `cmake/stm32cubemx/CMakeLists.txt`。

顶层 CMake 需要完成以下配置：

- 启用 C、C++ 和 ASM 语言。
- 将 C++ 标准设置为 C++17。
- 通过 `target_sources` 显式列出本阶段复制的 `.cpp` 文件和本地接入文件。
- 将 `User_File` 加入目标 include 路径。
- 定义 `ARM_MATH_CM7`、`ARM_MATH_MATRIX_CHECK` 和 `ARM_MATH_ROUNDING`。
- 链接 `Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a`。
- 继续链接 CubeMX 的 `stm32cubemx` target。
- 不使用 `GLOB_RECURSE` 自动加入当前空目录或未来尚未评审的源码。

目标工程继续使用现有 ZGT6 链接脚本以及 `-mcpu=cortex-m7`、硬浮点等工具链参数。VGT6 参考工程的启动文件、链接脚本和生成 CMake 不参与复制。

## 9. 当前运行时行为

本阶段完成后，各模块会参与编译和链接，但应用层不会调用它们：

- `main.c` 不调用 `Task_Init()` 或任何 DM 电机接口。
- 不创建 `Class_Motor_DM_Normal` 或 `Class_Motor_DM_1_To_4` 对象。
- 不调用 `CAN_Init()`，不激活 `drv_can` 的通知和总线启动流程。
- 不调用 `TIM_1ms_CAN_PeriodElapsedCallback()`。
- 不初始化 `SYS_Timestamp`，也不调用其读取或延时函数。
- `init_finished` 始终为 `false`。

因此不会使能电机、清除电机错误、保存零点或发送任何控制帧。

## 10. 后续驱动阶段的数据流

后续在明确电机型号、控制模式、ID 和使用总线后，再设计以下流程：

```text
Task_Init()
  ├─ SYS_Timestamp.Init(&htim5)
  ├─ CAN_Init(&hfdcanX, CANx_Callback)
  ├─ 初始化 Motor_DM 对象及所需 PID
  ├─ 启动所需定时器
  └─ init_finished = true

FDCAN 接收中断
  → drv_can 读取报文
  → CANx_Callback 按 Identifier 分发
  → Motor_DM.CAN_RxCpltCallback()
  → 更新角度、速度、力矩、温度和在线状态
```

普通模式和一拖四模式的发送机制不同，发送周期和共享发送缓冲区安排不在本阶段决定。

## 11. 错误处理与安全约束

- 保留 `drv_can` 原有 Bus-Off 停止并重启 FDCAN 的行为。
- 保留 `Motor_DM` 原有在线检测、状态枚举、数据解析和限幅行为。
- 不新增自动重试、日志系统或错误码封装。
- 若 DSP 静态库与当前工具链不兼容，优先调整 CMake 或 FPU 链接设置，不修改 `alg_basic` 绕过 DSP。
- 当前阶段不调用任何需要已初始化时间戳对象的函数。
- 当前阶段不接入原版周期 CAN 发送函数。
- 后续发现的一拖四发送调度等运行问题单独设计，不在本次复刻中修改参考代码。

## 12. 验收标准

### 12.1 一致性验证

对所有原样复制源码和 DSP 静态库进行 SHA-256 或逐字节比较。参考文件与目标文件必须一致。

### 12.2 CMake 验证

确认：

- C、C++、ASM 均已启用；
- 使用 C++17；
- 源文件为显式列表；
- `User_File` include 路径有效；
- DSP 宏存在；
- DSP 静态库已经链接；
- CubeMX 子 CMake 没有被修改。

### 12.3 编译和链接验证

使用 Debug CMake 配置完成一次干净构建，要求：

- 所有本次新增 C++ 文件编译成功；
- DSP 静态库链接成功；
- 不存在 `init_finished`、`SYS_Timestamp`、`arm_sin_f32` 等未定义符号；
- 成功生成 ELF 和 map 文件；
- 继续使用 ZGT6 链接脚本和 Cortex-M7 硬浮点参数。

如终端环境未暴露 `cmake`，应定位并使用 CLion 配置的 CMake，或明确记录无法从当前环境执行构建的限制。

### 12.4 零运行行为验证

静态检查确认：

- `main.c` 没有新增任务或电机调用；
- 没有 DM 电机对象；
- 没有电机 ID 或控制参数；
- 没有 `CAN_Init()` 调用；
- 没有 DM 使能或发送函数调用；
- `init_finished` 没有被设置为 `true`。

## 13. 完成定义

当参考源码与目标副本一致、顶层 CMake 接入完成、Debug 构建和链接通过、且静态检查确认没有电机运行行为时，本次 Motor_DM 最小依赖复刻完成。
