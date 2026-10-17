# User_File Module Boundary Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 审查并收紧 `User_File` 全部手写模块的公开边界，在不改变固件行为的前提下恢复中科大 USTC RoboWalker 风格的 `.h/.cpp` 职责划分。

**Architecture:** 按来源和层级实施：第三方源码只读审查，中科大移植模块仅修复明确的链接边界，当前工程新增模块完整检查公开接口、对象归属和内部链接。每个独立层级完成一次定向主机测试，最后统一执行完整主机测试和一次Debug固件构建。

**Tech Stack:** C/C++17、STM32H723 HAL、裸机前台调度、MinGW g++主机断言测试、STM32Cube CMake/Ninja构建。

**Spec:** `docs/superpowers/specs/2026-10-11-user-file-module-boundary-audit-design.md`

## Global Constraints

- 保留中科大文件头、分区注释、命名、对象组织、回调布局和直接可读流程。
- 不修改电机控制、底盘解算、协议数据、调度周期、参数或安全逻辑。
- 不重写HiPNUC、NRF寄存器定义等第三方协议源码。
- 不引入RTOS、队列、SUBS、匿名命名空间、管理器或配置框架。
- 不修改CubeMX生成文件。
- 生产头文件不得仅为了主机测试而公开内部函数。
- 使用Native执行；按模块运行定向测试，全部完成后只进行一次完整固件编译。
- 不创建Git分支、提交、推送或Pull Request。

## Review Focus

- HAL的FDCAN/UART全局回调必须继续保持外部链接并被HAL正确调用。
- 任务层静态回调传给CAN/UART注册函数后，接收转发必须与整理前一致。
- `SBUS_Receive`移动定义位置后只能有一个实例，所有任务必须继续读取同一份状态。
- 测试改用公开周期入口后，舵轮控制、初始化和失能行为必须保持原有断言。
- HiPNUC打包结构、NRF寄存器宏、CAN报文及DM/VESC协议数据不得发生变化。

---

### Task 1: 将测试切换到正式公开入口

**Files:**
- Modify: `tests/host/test_motor_init.cpp`
- Modify: `tests/host/test_chassis_task.cpp`
- Modify: `tests/host/test_task_control.cpp`

**Interfaces:**
- Consumes: `void Motor_Init()`
- Consumes: `void Chassis_Swerve_10ms_PeriodElapsedCallback()`
- Preserves: 现有电机初始化、舵轮目标、失能和斜坡行为断言。

- [ ] **Step 1: 将电机初始化测试改为调用 `Motor_Init()`**

保留默认舵轮配置和四个VESC对象配置断言；删除生产测试对 `Motor_Swerve_Init()`、`Motor_VESC_Init()` 的直接依赖。若仍需验证未启用的全向轮初始化，将其作为测试内独立对象行为验证，不要求生产头文件暴露内部配置函数。

- [ ] **Step 2: 将舵轮任务测试改为调用正式10ms入口**

把 `Chassis_Swerve_Control()` 的直接调用替换为 `Chassis_Swerve_10ms_PeriodElapsedCallback()`，保持每个测试原有调用次数和断言时序。

- [ ] **Step 3: 编译并运行受影响测试，建立GREEN基线**

Run:

```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_motor_init.cpp User_File/4_Task/tsk_motor_init/motor_init.cpp -o build/host/test_motor_init.exe
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_chassis_task.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp User_File/4_Task/tsk_motor_init/motor_init.cpp User_File/4_Task/tsk_chassis/tsk_chassis.cpp -o build/host/test_chassis_task.exe
& build/host/test_motor_init.exe
& build/host/test_chassis_task.exe
```

Expected: 两个测试均以0退出，且无编译警告。`test_task_control`在Task 4与任务层源码一起重新编译。

### Task 2: 收紧Middleware与Device内部链接和对象归属

**Files:**
- Modify: `User_File/1_Middleware/Driver/CAN/drv_can.cpp`
- Modify: `User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp`
- Modify: `User_File/4_Task/tsk_remote/tsk_remote.cpp`
- Modify: `User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp`
- Verify unchanged: `User_File/1_Middleware/Hipnuc/*`
- Verify unchanged: `User_File/2_Device/NRF24L01/nrf24l01_define.h`
- Test: `tests/host/test_drv_can.cpp`
- Test: `tests/host/test_pro_sbus.cpp`
- Test: `tests/host/test_motor_dm_enable.cpp`
- Test: `tests/host/test_remote_receive_task.cpp`

**Interfaces:**
- Preserves: `CAN_Init`、`CAN_Extended_Init`和HAL FDCAN回调外部接口。
- Preserves: `extern Class_SBUS_Receive SBUS_Receive`。
- Produces: `SBUS_Receive`由声明它的协议模块定义，且全工程仅有一个定义。
- Preserves: DM固定控制报文内容及发送函数行为。

- [ ] **Step 1: 收紧CAN过滤器辅助函数**

将 `can_filter_mask_config(FDCAN_HandleTypeDef *)` 与 `can_extended_filter_mask_config(FDCAN_HandleTypeDef *)` 的声明和定义改为 `static`；HAL回调和公开驱动函数保持全局。

- [ ] **Step 2: 修正SBUS对象归属**

在 `pro_sbus.cpp` 的 `Exported variables` 分区定义 `Class_SBUS_Receive SBUS_Receive;`，并从 `tsk_remote.cpp` 删除该定义。保留 `pro_sbus.h` 中现有 `extern` 声明，保证配置、遥控和调试任务继续访问同一对象。

- [ ] **Step 3: 收紧DM固定控制报文**

将 `DM_Motor_CAN_Message_Clear_Error`、`DM_Motor_CAN_Message_Enter`、`DM_Motor_CAN_Message_Exit`、`DM_Motor_CAN_Message_Save_Zero`设为文件私有；保持数组可写类型以兼容现有 `CAN_Transmit_Data(uint8_t *)` 签名，不改变8字节内容。

- [ ] **Step 4: 运行Middleware与Device定向测试**

Run:

```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_drv_can.cpp User_File/1_Middleware/Driver/CAN/drv_can.cpp -o build/host/test_drv_can.exe
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_pro_sbus.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp -o build/host/test_pro_sbus.exe
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/motor_stubs -Itests/host/stubs -ICore/Inc -IUser_File -I. tests/host/test_motor_dm_enable.cpp User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp -o build/host/test_motor_dm_enable.exe
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_remote_receive_task.cpp User_File/4_Task/tsk_remote/tsk_remote.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp -o build/host/test_remote_receive_task.exe
& build/host/test_drv_can.exe
& build/host/test_pro_sbus.exe
& build/host/test_motor_dm_enable.exe
& build/host/test_remote_receive_task.exe
```

Expected: 四个测试均以0退出且无警告；链接阶段没有 `SBUS_Receive`重复定义或未定义错误。

### Task 3: 整理Chassis模块私有常量

**Files:**
- Modify: `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.h`
- Modify: `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp`
- Verify unchanged: `User_File/3_Module/Chassis/Omni/mod_chassis_omni.h`
- Verify unchanged: `User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp`
- Test: `tests/host/test_chassis_swerve.cpp`
- Test: `tests/host/test_chassis_omni.cpp`

**Interfaces:**
- Preserves: `Enum_Swerve_Wheel`与 `Class_Chassis_Swerve`全部公开接口。
- Internalizes: `SWERVE_MOTION_ENTER_SPEED`与 `SWERVE_MOTION_EXIT_SPEED`。

- [ ] **Step 1: 移动舵轮运动迟滞阈值**

从 `mod_chassis_swerve.h` 删除两个仅实现使用的宏，在 `mod_chassis_swerve.cpp` 的 `Private macros` 分区按原值定义；不修改比较条件和数值。

- [ ] **Step 2: 运行两种底盘运动学测试**

Run:

```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -IUser_File tests/host/test_chassis_swerve.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp -o build/host/test_chassis_swerve.exe
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -IUser_File tests/host/test_chassis_omni.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp -o build/host/test_chassis_omni.exe
& build/host/test_chassis_swerve.exe
& build/host/test_chassis_omni.exe
```

Expected: 两个测试均以0退出，舵轮迟滞和全向轮解算断言不变。

### Task 4: 收紧Task层公开接口与文件内状态

**Files:**
- Modify: `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp`
- Modify: `User_File/4_Task/tsk_remote/tsk_remote.cpp`
- Modify: `User_File/4_Task/tsk_motor_init/motor_init.h`
- Modify: `User_File/4_Task/tsk_motor_init/motor_init.cpp`
- Modify: `User_File/4_Task/tsk_chassis/tsk_chassis.h`
- Modify: `User_File/4_Task/tsk_chassis/tsk_chassis.cpp`
- Verify unchanged or minimally include-cleaned: `User_File/4_Task/tsk_debug/tsk_debug.h`
- Verify unchanged or minimally include-cleaned: `User_File/4_Task/tsk_debug/tsk_debug.cpp`
- Test: `tests/host/test_task_control.cpp`
- Test: `tests/host/test_task_debug.cpp`
- Test: files modified in Task 1

**Interfaces:**
- Preserves: `Task_Init()`、`Task_Loop()`和`init_finished`。
- Preserves: `Remote_Init()`、`Remote_10ms_PeriodElapsedCallback()`、`NRF24L01`与`Remote_Chassis_Command`。
- Preserves: `Motor_Init()`以及跨任务使用的DM/VESC对象和ID宏。
- Preserves: Chassis初始化、周期入口和`Chassis_Swerve_Diagnostic[]`。
- Removes from public headers: `Motor_Omni_Init()`、`Motor_Swerve_Init()`、`Motor_VESC_Init()`、`Chassis_Swerve_Control()`。

- [ ] **Step 1: 收紧配置任务内部状态和回调**

为 `Key1`、`Key2`、五个调度Tick、双按键锁存和遥控按键历史状态增加内部链接。为CAN1、CAN3、UART4、UART9转发回调及五个周期内部回调的声明和定义增加 `static`。保持HAL注册、任务调用顺序和周期不变。

- [ ] **Step 2: 收紧遥控任务辅助函数**

为 `Remote_Channel_To_Speed(uint16_t)`、`Remote_Channel_To_Angular_Speed(uint16_t)`、`Remote_Update_Chassis_Command()`的声明和定义增加 `static`。`SBUS_Receive`保持跨模块公开，因为配置与调试任务直接读取其原始数据。

- [ ] **Step 3: 收紧电机初始化分项函数**

从 `motor_init.h`删除三个分项初始化声明，在 `motor_init.cpp`的私有函数分区增加对应 `static` 声明，并将定义设为 `static`。`Motor_Init()`中的选择逻辑和初始化参数不变。

- [ ] **Step 4: 收紧底盘任务对象和辅助函数**

将 `Chassis_Omni`、`Chassis_Swerve`改为文件私有。为全向轮控制、电机失能、电机发送、舵轮失能、舵向控制、行进控制和 `Chassis_Swerve_Control()`增加内部链接；从 `tsk_chassis.h`删除 `Chassis_Swerve_Control()`声明。保留所有正式周期入口。

- [ ] **Step 5: 重新编译并运行任务层测试**

使用Task 1命令重新构建并运行 `test_motor_init.exe`与`test_chassis_task.exe`。随后按当前完整源集合重新构建：

```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_task_debug.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp User_File/2_Device/Plotter/Vofa/dvc_vofa.cpp User_File/4_Task/tsk_motor_init/motor_init.cpp User_File/4_Task/tsk_debug/tsk_debug.cpp -o build/host/test_task_debug.exe
```

按以下完整源集合重新构建 `test_task_control.exe`：

```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_task_control.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp User_File/2_Device/BSP/Key/bsp_key.cpp User_File/2_Device/Plotter/Vofa/dvc_vofa.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp User_File/4_Task/tsk_motor_init/motor_init.cpp User_File/4_Task/tsk_remote/tsk_remote.cpp User_File/4_Task/tsk_chassis/tsk_chassis.cpp User_File/4_Task/tsk_debug/tsk_debug.cpp User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp -o build/host/test_task_control.exe
& build/host/test_motor_init.exe
& build/host/test_chassis_task.exe
& build/host/test_task_debug.exe
& build/host/test_task_control.exe
```

Expected: 所有测试以0退出；不存在对已移除内部接口的测试或生产引用；注册得到的CAN/UART回调仍可被测试触发。

### Task 5: 完成全量审查并验证整个工程

**Files:**
- Verify: `User_File`全部 `.h/.cpp/.c`
- Verify: `tests/host`全部现有测试
- Verify: `build/Debug/Robocon-Elian.elf`

**Interfaces:**
- Consumes: Tasks 1至4的边界整理。
- Produces: 全部模块的“修改或保留”结论及最新验证证据。

- [ ] **Step 1: 对未修改模块完成只读边界复核**

逐项确认：头文件公开声明具有真实调用方；类成员依赖完整；第三方打包布局未变；HAL回调未被设为静态；不存在仅在单个源文件使用却意外暴露的新符号。只在发现有直接引用证据的明确问题时进行最小修正，并立即运行所属模块现有测试。

- [ ] **Step 2: 运行全部主机测试**

Tasks 1至4已经重新构建所有受源码调整影响的测试。执行全部现有主机测试：

```powershell
$tests = @(
    'test_bsp_key.exe',
    'test_chassis_omni.exe',
    'test_chassis_swerve.exe',
    'test_chassis_task.exe',
    'test_drv_can.exe',
    'test_drv_uart.exe',
    'test_imu_hi06.exe',
    'test_motor_dm_enable.exe',
    'test_motor_init.exe',
    'test_motor_vesc.exe',
    'test_pro_sbus.exe',
    'test_remote_receive_task.exe',
    'test_task_control.exe',
    'test_task_debug.exe',
    'test_vofa.exe'
)
foreach ($test in $tests)
{
    & (Join-Path 'build/host' $test)
    if ($LASTEXITCODE -ne 0)
    {
        throw "$test failed with exit code $LASTEXITCODE"
    }
}
```

Expected: 15个测试全部以0退出。不要把旧二进制的成功当成已修改源码的编译证据；若Step 1的最小修正触及新的生产源码，先按该测试现有源码集合重新编译对应测试。

- [ ] **Step 3: 执行唯一一次完整固件构建**

Run:

```powershell
& 'C:\Users\jiang\AppData\Local\stm32cube\bundles\cmake\4.0.1+st.3\bin\cmake.exe' --build --preset Debug --target Robocon-Elian
```

Expected: `Robocon-Elian.elf`编译和链接成功，0 errors；记录warnings数量并调查任何新增warning。

- [ ] **Step 4: 检查最终修改边界**

确认没有修改CubeMX生成文件、第三方协议布局、电机参数、遥控映射、底盘解算、斜坡、PID或调度周期。确认所有改动仅涉及声明位置、内部链接、对象归属、必要测试和注释。

- [ ] **Step 5: 输出完成报告**

列出修改文件、每个模块的修改或保留理由、定向测试、全部主机测试、固件构建结果，并明确说明尚未经过物理板回归。不要提交或推送Git。
