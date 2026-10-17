# 四舵轮完整控制与 DM3519 转向修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 DM3519 位置速度模式因速度限制为零而不转的问题，把当前单左前轮控制扩展为四个 DM3519 + 四个 VESC 的完整舵轮控制，并保持遥控四挡速度有效。

**Architecture:** `tsk_remote` 只把遥控端已经按挡位缩放的 SBUS 通道映射一次；`mod_chassis_swerve` 保持只负责运动学；`tsk_chassis` 向四个 DM3519 下发舵角与 `2.0 rad/s` 速度限制，并向四个 VESC 下发行进轮速度。电机在线状态只用于诊断，不作为控制前置条件；遥控在线、MANUAL 模式和总使能继续作为统一安全门控。

**Tech Stack:** STM32H723、C++17、HAL FDCAN、裸机周期回调、NRF24L01/SBUS、DM3519 位置速度模式、VESC CAN RPM、主机测试、STM32 CMake 构建。

**Spec:** `docs/superpowers/specs/2026-10-06-vesc-swerve-drive-design.md`

## Global Constraints

- 保持中科大 RoboWalker 风格，不引入 RTOS、SUBS、动态管理器或无关抽象。
- 四个 DM3519 使用 CAN1：Motor ID `0x01..0x04`，Master ID `0x11/0x22/0x33/0x44`。
- 四个 VESC 使用 CAN3，VESC ID 按 LF/RF/LB/RB 固定为 `1/2/3/4`。
- DM3519 位置速度模式的初始舵向速度限制统一为 `2.0 rad/s`，不能继续保持为零。
- 当前只连接一个 DM3519 和一个 VESC 时，其余离线设备不得阻止已连接轮组执行命令。
- 遥控离线、非 MANUAL 或 `motor_enable=0` 时，四个舵向电机与四个行进电机全部安全失能。
- 四挡速度由遥控端缩放 SBUS 通道幅值，车端只做一次 `200..1800 -> 底盘速度` 映射，不再叠乘挡位系数。
- 修改行为必须先补主机测试，再改生产代码；完成时运行全部现有主机测试和 Debug 固件完整构建。
- 用户未要求 Git 操作，不创建分支、不提交、不推送。

## Review Focus

- DM 目标角变化但 `Control_Omega=0`，导致位置速度模式无法运动。
- 当前任务层只实际驱动 ID 1，ID 2..4 被主动失能。
- LF/RF/LB/RB 轮序必须同时匹配运动学输出、DM ID 和 VESC ID。
- 遥控四挡不得在遥控端和车端重复缩放。
- 遥控断联或失能后不得保留旧舵角速度、旧 ERPM 或软件使能状态。

## File Map

### Modify

- `User_File/4_Task/Config/motor_init.cpp`：集中定义 DM 舵向速度限制，并初始化四个 DM3519 的 `Control_Omega`。
- `User_File/4_Task/tsk_chassis/tsk_chassis.cpp`：按 LF/RF/LB/RB 完整控制四个 DM3519 和四个 VESC。
- `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp`：在现有 USART10 单行诊断中补充 DM 目标速度限制。
- `tests/host/test_motor_init.cpp`：验证四个 DM3519 初始化后的模式与速度限制。
- `tests/host/test_remote_receive_task.cpp`：验证挡位缩放后的 SBUS 值只映射一次。
- `tests/host/test_chassis_task.cpp`：验证完整四轮命令、离线不阻塞和统一安全失能。
- `tests/host/test_task_control.cpp`：验证诊断输出包含 DM 目标速度限制。

## Task 1: 先锁定 DM3519 速度限制与遥控挡位语义

**Files:**
- Modify: `tests/host/test_motor_init.cpp`
- Modify: `tests/host/test_remote_receive_task.cpp`
- Modify: `User_File/4_Task/Config/motor_init.cpp`

- [ ] **Step 1: 为四个 DM3519 写初始化失败测试**

在 `test_motor_init.cpp` 中断言调用 `Motor_Swerve_Init()` 后：

- LF/RF/LB/RB 均保持 `NORMAL_ANGLE_OMEGA`；
- 四个对象的 `Control_Omega` 均为 `2.0f`；
- Motor ID 与 Master ID 保持既定的 `1..4` 和 `0x11/0x22/0x33/0x44`。

- [ ] **Step 2: 为遥控挡位写单次映射测试**

在 `test_remote_receive_task.cpp` 增加 `Test_Remote_Gear_Range_Is_Mapped_Only_Once()`，分别输入中心值、挡位缩放后的中间值和最大值，验证输出为零、按幅值同比例变化和最大 `3.0f`，不出现第二次挡位乘法。

- [ ] **Step 3: 运行测试并确认失败原因**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_motor_init.cpp User_File/4_Task/Config/motor_init.cpp -o build/host/test_motor_init.exe
& build/host/test_motor_init.exe
```

Expected: DM 初始化测试因当前 `Control_Omega=0.0f` 失败；遥控单次映射测试若直接通过，则证明生产代码无需增加挡位系数。

- [ ] **Step 4: 最小修改 DM 初始化**

在 `motor_init.cpp` 中集中定义带单位的常量，例如 `DM_CHASSIS_SWERVE_OMEGA_LIMIT = 2.0f`，并在 `Motor_Swerve_Init()` 内给四个 DM3519 设置相同速度限制。保持完整语句单行，不增加配置管理层。

- [ ] **Step 5: 重新运行两项测试**

Expected: 两项测试均通过。

## Task 2: 把任务层从单左前轮扩展为完整四轮

**Files:**
- Modify: `tests/host/test_chassis_task.cpp`
- Modify: `User_File/4_Task/tsk_chassis/tsk_chassis.cpp`

- [ ] **Step 1: 写完整四舵向电机测试**

增加 `Test_Swerve_All_Four_Steer_Motors_Enter_And_Receive_Angle_Omega_Targets()`，输入同时产生四轮不同目标的底盘速度，断言：

- 四个 DM3519 均进入控制；
- LF/RF/LB/RB 的目标角与运动学对应轮序一致；
- 四个 DM3519 的目标速度限制均为 `2.0f`。

- [ ] **Step 2: 写完整四行进电机测试**

增加 `Test_Swerve_All_Four_Drive_Motors_Enable_And_Receive_Matching_Wheel_Speeds()`，断言四个 VESC 均软件使能，并收到与对应轮组 `Drive_Omega` 一致的目标。

- [ ] **Step 3: 写部分硬件和安全测试**

增加：

- `Test_Swerve_Missing_Feedback_Does_Not_Block_Other_Wheels()`：将部分或全部反馈状态设为离线，合法遥控命令仍应向四个对象更新目标；
- `Test_Swerve_Remote_Safety_Disables_All_Eight_Motors()`：遥控离线、非 MANUAL 或总使能关闭时，四个 DM3519 退出控制，四个 VESC 目标归零并软件失能。

- [ ] **Step 4: 运行测试并确认当前单轮实现失败**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_chassis_task.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp User_File/4_Task/Config/motor_init.cpp User_File/4_Task/tsk_chassis/tsk_chassis.cpp -o build/host/test_chassis_task.exe
& build/host/test_chassis_task.exe
```

Expected: RF/LB/RB 相关断言失败，因为现有代码主动失能这三组电机。

- [ ] **Step 5: 以显式轮序实现完整四轮控制**

在 `tsk_chassis.cpp` 中保持中科大直接可读风格，明确写出 LF/RF/LB/RB 映射：

- 四个 DM3519 分别接收对应 `Steer_Angle`，并保持 `2.0 rad/s` 速度限制；
- 四个 VESC 分别接收对应 `Drive_Omega`；
- 控制条件不读取任何单电机 `online` 状态；
- 安全条件不满足时统一清零并失能全部八个对象。

- [ ] **Step 6: 运行底盘、舵轮与 VESC 相关测试**

Expected: 新增测试和已有相关测试全部通过。

## Task 3: 补充可直接定位 DM 速度限制的诊断字段

**Files:**
- Modify: `tests/host/test_task_control.cpp`
- Modify: `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp`

- [ ] **Step 1: 写诊断输出失败测试**

让测试期望 USART10 单行输出包含：

```text
DM enable=1 online=1 control=1 target=0.50 target_omega=2.00 now=-0.25
```

- [ ] **Step 2: 运行测试并确认字段缺失**

Expected: 当前输出没有 `target_omega`，测试失败。

- [ ] **Step 3: 最小扩展现有单行诊断**

使用现有浮点转字符串方式读取 `Get_Control_Omega()`，只增加 `target_omega` 字段；继续使用静态 DMA 双缓冲，不改变打印周期，不恢复 IMU、电机全量或 VOFA 打印。

- [ ] **Step 4: 重新运行测试**

Expected: 诊断格式测试通过。

## Task 4: 全量验证与实物测试交接

- [ ] **Step 1: 以严格警告重新编译直接相关测试**

使用 `-Wall -Wextra -Werror` 重新编译并运行 `test_motor_init`、`test_remote_receive_task`、`test_chassis_task` 和 `test_task_control`。

- [ ] **Step 2: 运行全部现有主机测试**

运行以下正式测试，不把任何临时 `*_red.exe` 纳入结果：

```text
test_bsp_key
test_chassis_omni
test_chassis_swerve
test_chassis_task
test_drv_can
test_drv_uart
test_imu_hi06
test_motor_dm_enable
test_motor_init
test_motor_vesc
test_pro_sbus
test_remote_receive_task
test_task_control
test_vofa
```

- [ ] **Step 3: 完整清理构建 Debug 固件**

Run:

```powershell
& 'C:\Users\jiang\AppData\Local\stm32cube\bundles\cmake\4.0.1+st.3\bin\cmake.exe' --build --preset Debug --clean-first --target Robocon-Elian
```

Expected: 编译并链接生成 `Robocon-Elian.elf`。已有、与本次无关的警告需单独说明，不能声称为本次新增。

- [ ] **Step 4: 做范围审计**

确认：

- 没有新增车端挡位系数；
- 没有用在线状态阻塞控制；
- 没有重新启用 Omni；
- 没有更改 CAN ID、控制模式或 CubeMX 生成区；
- 没有残留临时诊断文件或测试可执行文件进入固件目标。

- [ ] **Step 5: 给出实物验证步骤**

先架空当前 LF 轮组：

1. 上电默认失能，遥控使能后观察 `target_omega=2.00`；
2. 缓慢拨动平移摇杆，DM `now` 应跟随 `target`，VESC `now_erpm` 应跟随 `target_erpm`；
3. 逐个检查四挡下相同满行程摇杆对应的底盘速度与 ERPM 上限；
4. 关闭使能、切出 MANUAL 或关闭遥控，确认 DM 退出控制且 VESC 归零失能；
5. 后续接入 ID 2..4 时逐轮核对 LF/RF/LB/RB 方向和零偏。

最终报告必须区分“主机测试/固件构建通过”和“尚待物理板验证”。
