# 舵轮传动比与 VOFA 调试模块 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将DM3519的周期反馈还原为连续电机轴角度，按17:215外啮合换算舵轮机构角度，并把四套DM+VESC诊断统一封装到USART9 `tsk_debug`。

**Architecture:** `dvc_motor_dm`只负责DM协议与连续电机轴角度；`tsk_chassis`负责电机轴与舵轮机构之间的传动换算并发布四轮诊断；`mod_chassis_swerve`继续只处理机构侧连续角度；`tsk_debug`负责选择轮组、采样诊断并输出18通道JustFloat帧。

**Tech Stack:** C++17、STM32H723 HAL、FDCAN1/FDCAN3、USART9、现有裸机周期调度、主机端g++断言测试、CMake/Ninja固件构建。

**Spec:** `docs/superpowers/specs/2026-10-11-swerve-gear-ratio-vofa-debug-design.md`

## Global Constraints

- `bldc`及CubeMX生成的`Core`、`Drivers`、`cmake/stm32cubemx`不修改。
- 不引入RTOS、队列、SUBS、动态内存或新的上层框架。
- 保留现有遥控离线和整车失能安全行为，不新增DM掉线或连续角度未初始化联动关闭VESC。
- DM ID1～4分别配对VESC ID1～4，默认调试左前轮ID1。
- 舵向机构为17T主动齿轮外啮合215T从动齿轮，机构可无限连续旋转。
- 不改DM增益、VESC FOC/速度PID/Observer、电流限制和现有VESC速度斜坡参数。
- 保留中科大RoboWalker文件头、分区、命名和中文Doxygen风格。
- 当前工作区含用户未提交改动；原地实施，不创建分支、不提交、不覆盖无关改动。

## Review Focus

- DM反馈跨越`+12.5/-12.5rad`时，连续角度必须按正确方向小步增长而非跳变约25rad；Task 1覆盖正反跨界。
- 位置速度模式必须保留多圈float角度，而MIT模式仍须限制在`±Angle_Max`；Task 1覆盖两种模式。
- 17:215外啮合换算必须同时正确处理角度、角速度、方向和四轮独立偏移；Task 2覆盖目标与反馈两条链路。
- 连续机构角度超过一圈时，舵轮解算必须选择最近等效目标，不能回到单圈角；Task 2复用并扩展舵轮主机测试。
- VOFA不得以20ms周期重复旧VESC状态，也不得错位18个通道；Task 3验证STATUS_5门控、76字节帧和通道顺序。

---

### Task 1: DM3519连续反馈与位置速度多圈目标

**Files:**
- Modify: `tests/host/test_motor_dm_enable.cpp`
- Modify: `tests/host/motor_stubs/1_Middleware/Algorithm/Basic/alg_basic.h`
- Modify: `User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.h`
- Modify: `User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp`

**Interfaces:**
- Produces: `float Class_Motor_DM_Normal::Get_Now_Total_Angle() const`
- Produces: `bool Class_Motor_DM_Normal::Get_Total_Angle_Initialized() const`
- Preserves: `Get_Now_Angle()` remains the raw wrapped angle.

- [ ] **Step 1: Add failing host tests**

Add tests that inject real normal-protocol CAN feedback and assert first-frame initialization, `+12.4→-12.4rad` forward unwrap, `-12.4→+12.4rad` reverse unwrap, and ordinary deltas. Add transmit tests asserting `NORMAL_ANGLE_OMEGA` sends `79.464f` unchanged while `NORMAL_MIT` still saturates its encoded target at `+Angle_Max`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:
```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/motor_stubs -Itests/host/stubs -ICore/Inc -IUser_File -I. tests/host/test_motor_dm_enable.cpp User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp -o build/host/test_motor_dm_enable.exe
```
Expected: compilation or assertions fail because the continuous-angle API/behavior and mode-specific clamp do not exist.

- [ ] **Step 3: Implement minimal DM behavior**

Add `Previous_Angle`, `Now_Total_Angle`, and `Total_Angle_Initialized`; reset them in `Init()`; update them after every valid `Data_Process()` using the `2*Angle_Max` period. Restrict angle clamping in `TIM_Send_PeriodElapsedCallback()` to modes whose wire format uses the bounded position field; specifically do not clamp `NORMAL_ANGLE_OMEGA`.

- [ ] **Step 4: Run focused test and verify GREEN**

Run the command from Step 2, then `build/host/test_motor_dm_enable.exe`.
Expected: exit code 0 with no warnings.

### Task 2: 舵轮17:215传动换算与四轮机构侧诊断

**Files:**
- Modify: `tests/host/stubs/2_Device/Motor/Motor_DM/dvc_motor_dm.h`
- Modify: `tests/host/test_chassis_task.cpp`
- Modify: `tests/host/test_chassis_swerve.cpp`
- Modify: `User_File/4_Task/tsk_chassis/tsk_chassis.h`
- Modify: `User_File/4_Task/tsk_chassis/tsk_chassis.cpp`
- Modify: `User_File/4_Task/tsk_motor_init/motor_init.h`
- Modify: direct includes in `tests/host/test_chassis_task.cpp` and production tasks from obsolete `4_Task/Config/motor_init.h` to actual `4_Task/tsk_motor_init/motor_init.h`

**Interfaces:**
- Consumes: `Get_Now_Total_Angle()` and `Get_Total_Angle_Initialized()` from Task 1.
- Produces: `Struct_Chassis_Swerve_Diagnostic Chassis_Swerve_Diagnostic[Swerve_Wheel_Number]` with mechanism-side target angle, actual continuous angle, error, actual angular speed, target speed limit, speed ratio, and initialization state.
- Preserves: existing `Class_Chassis_Swerve` kinematics and safety ratio behavior.

- [ ] **Step 1: Add failing gear-ratio and continuity tests**

Extend the DM fake with continuous-angle state. In `test_chassis_task`, assert one mechanism revolution maps to `-(215/17)*2π` motor target for the default external-mesh direction, and the reciprocal feedback conversion is used by steering/error diagnostics. Assert all four diagnostics update independently. In `test_chassis_swerve`, assert an actual mechanism angle beyond one revolution yields a nearby continuous target.

- [ ] **Step 2: Run focused tests and verify RED**

Run:
```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_chassis_task.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp User_File/4_Task/tsk_motor_init/motor_init.cpp User_File/4_Task/tsk_chassis/tsk_chassis.cpp -o build/host/test_chassis_task.exe
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -IUser_File tests/host/test_chassis_swerve.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp -o build/host/test_chassis_swerve.exe
```
Expected: compilation or assertions fail because the gear conversion and four-wheel diagnostic structure are absent.

- [ ] **Step 3: Implement the task-layer conversion**

Define centralized `215/17` ratio and external-mesh direction constants, plus per-wheel zero/direction calibration values. Initialize `Class_Chassis_Swerve` with angle limit `0.0f`. Convert DM continuous motor angle/omega to mechanism feedback before `Set_Now_Angle()`, and mechanism targets back to motor targets before `Set_Control_Angle()`. Publish all four diagnostics in mechanism units. Remove the obsolete motor-axis swerve angle limit macro while retaining DM protocol `Angle_Max`.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run both commands from Step 2 and both executables.
Expected: both exit code 0 with no warnings.

### Task 3: USART9 `tsk_debug`四轮VOFA封装

**Files:**
- Create: `tests/host/test_task_debug.cpp`
- Modify: `tests/host/stubs/2_Device/Motor/Motor_DM/dvc_motor_dm.h`
- Modify: `tests/host/stubs/2_Device/Motor/Motor_VESC/dvc_motor_vesc.h`
- Create: `User_File/4_Task/tsk_debug/tsk_debug.h`
- Create: `User_File/4_Task/tsk_debug/tsk_debug.cpp`
- Modify: `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: four DM/VESC global objects and `Chassis_Swerve_Diagnostic[]`.
- Produces: `Debug_Init()`, `Debug_UART9_RxCpltCallback(uint8_t *, uint16_t)`, `Debug_20ms_PeriodElapsedCallback()`, `Debug_Set_Swerve_Wheel(Enum_Swerve_Wheel)`.
- Produces: fixed 18-float JustFloat frame ordered exactly as spec channels I0～I17.

- [ ] **Step 1: Add failing debug-module host test**

Create a real `Class_Vofa_UART` integration test with UART transmit capture. Assert default LF mapping; switching to RF/LB/RB selects matching objects; invalid enum preserves selection; incomplete status sends nothing; unchanged STATUS_5 count sends nothing; a new complete STATUS group emits exactly 76 bytes and all 18 hand-derived channel values in the specified order, including Kelvin-to-Celsius DM temperatures and mechanism-side diagnostic angles/speeds in degrees.

- [ ] **Step 2: Run focused test and verify RED**

Run:
```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_task_debug.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp User_File/2_Device/Plotter/Vofa/dvc_vofa.cpp User_File/4_Task/tsk_motor_init/motor_init.cpp User_File/4_Task/tsk_debug/tsk_debug.cpp -o build/host/test_task_debug.exe
```
Expected: compilation fails because `tsk_debug` and its API do not exist.

- [ ] **Step 3: Implement `tsk_debug` and callback forwarding**

Use fixed pointer tables for four DM/VESC pairs and direct access to the matching chassis diagnostic entry. Keep all 18 float storage and STATUS_5 update gating private to `tsk_debug.cpp`. In `tsk_config_and_callback`, retain UART9 hardware registration but forward RX, initialization and 20ms work to the new module. Add `tsk_debug.cpp` and correct the stale `Config/motor_init.cpp` source path to `tsk_motor_init/motor_init.cpp` in root `CMakeLists.txt`.

- [ ] **Step 4: Run focused test and verify GREEN**

Run the command from Step 2, then `build/host/test_task_debug.exe`.
Expected: exit code 0 with no warnings.

### Task 4: 回归、固件构建与最终审查

**Files:**
- Verify all modified files against the spec; no additional production scope.

**Interfaces:**
- Consumes: Tasks 1～3 completed behavior.
- Produces: verified host-test and STM32 Debug build evidence.

- [ ] **Step 1: Run all existing host tests**

Compile every `tests/host/test_*.cpp` using its established source set and `-Wall -Wextra -Werror`, including the new `test_task_debug.exe`, then execute every produced test.
Expected: all executables exit 0 and all compilations are warning-free.

- [ ] **Step 2: Build the full firmware**

Run:
```powershell
cmake --build --preset Debug --target Robocon-Elian
```
If the configured environment requires the STM32 wrapper, use `cube-cmake --build --preset Debug --target Robocon-Elian` and ledger the ruling.
Expected: `Robocon-Elian.elf` compiles and links with 0 errors.

- [ ] **Step 3: Review scope and safety**

Inspect the diff against the spec and Review Focus. Confirm no CubeMX-generated files, VESC control parameters, remote safety logic, or unrelated ramp values changed. Run a fresh-context final review and fix only Critical/Important findings with RED→GREEN evidence.

- [ ] **Step 4: Record physical-test boundary**

Report software verification separately from hardware validation. The first burn-in test is LF `DM1+VESC1` at low speed, checking external-mesh direction, saved mechanical zero, continuous mechanism angle, and all 18 VOFA channels before enabling ID2～4.
