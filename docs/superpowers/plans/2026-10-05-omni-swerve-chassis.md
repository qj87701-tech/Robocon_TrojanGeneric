# Omni and Swerve Chassis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add independent Omni and Swerve chassis kinematics modules and move the existing J8006 object initialization into `motor_init` without changing current runtime behavior.

**Architecture:** `Class_Chassis_Omni` and `Class_Chassis_Swerve` are independent stateful calculation classes under `3_Module/Chassis` and do not depend on motor drivers. `motor_init` owns motor objects and initialization parameters; `tsk_config_and_callback` keeps callbacks, scheduling, and runtime control.

**Tech Stack:** C++17, STM32H723 HAL/CMake firmware build, host-side `g++` assertion tests, standard `math.h` functions only.

**Spec:** `docs/superpowers/specs/2026-10-05-omni-swerve-chassis-design.md`

## Global Constraints

- Follow `AGENTS.md` and the existing USTC RoboWalker-style headers, section separators, naming, object organization, and direct control flow.
- Keep the project bare-metal; do not add RTOS, queues, SUBS, inheritance hierarchies, templates, managers, dynamic allocation, exceptions, or complex STL.
- Coordinates are `X` right, `Y` forward, and `W` counterclockwise when viewed from above.
- Wheel order is always `RF`, `LF`, `RB`, `LB`.
- Linear speed uses `m/s`, angular speed uses `rad/s`, angle uses `rad`, and length uses `m`.
- Omni uses the USTC `sqrt(2)/2` coefficients, not the R1 `/ cos(45 degrees)` expression.
- Swerve speed dead zone is exactly `0.001f m/s`, applied independently to each wheel-group local speed.
- Chassis modules output wheel-side mechanical angular speed; motor reduction, protocol, mounting direction, and zero offset stay outside these modules.
- Do not create placeholder DM or ESC motor objects whose models, IDs, and parameters are unknown.
- Do not invoke Omni or Swerve from `Task_Loop()` in this change.
- Preserve J8006 Master ID `0x33`, Motor ID `0x03`, speed mode, angle range `12.5 rad`, speed range `45 rad/s`, and torque range `20 N*m`.
- Do not automatically enable the J8006 during initialization.
- The user has paused Git usage for this project; do not create branches or commits. End each task with a test-backed review checkpoint instead.

## Review Focus

- Re-initializing either chassis object after nonzero calculations must clear stale commands and restore all outputs to zero; covered in Tasks 1 and 2.
- `Wheel_Radius == 0` and `Wheel_Radius < 0` must both produce zero wheel speeds without division; covered in Tasks 1 and 2.
- A Swerve wheel speed exactly equal to `0.001f m/s` is outside the `< 0.001f` hold condition and must be calculated; covered in Task 2.
- An invalid `Enum_Swerve_Wheel` value must not index outside the four-wheel array; setters ignore it and getters return `0.0f`; covered in Task 2.
- Moving initialization must preserve its order relative to timestamp setup and CAN registration, and must not enable the motor; covered in Tasks 3 and 4.

---

### Task 1: Omni chassis kinematics

**Files:**
- Create: `User_File/3_Module/Chassis/Omni/mod_chassis_omni.h`
- Create: `User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp`
- Create: `tests/host/test_chassis_omni.cpp`

**Interfaces:**
- Consumes: no project service or motor driver; only `float` inputs and `<cmath>`.
- Produces: `Class_Chassis_Omni::Init(float, float)`, `Set_Speed_X(float)`, `Set_Speed_Y(float)`, `Set_Speed_W(float)`, `Calculate()`, and four `const` wheel-omega getters for `RF/LF/RB/LB`.

- [ ] **Step 1: Write the failing Omni tests**

Create named tests with `Assert_Near(actual, expected, 1.0e-5f)` for:

```cpp
Test_Init_And_Zero_Input_Clear_All_Wheels();
Test_Pure_X_Uses_USTC_Sqrt2_Over_2_Coefficients();
Test_Pure_Y_Uses_USTC_Sqrt2_Over_2_Coefficients();
Test_Pure_W_Gives_Equal_Positive_Wheel_Speeds();
Test_Combined_Velocity_Matches_All_Four_Formulas();
Test_Zero_And_Negative_Wheel_Radius_Clear_All_Wheels();
Test_Reinit_Clears_Previous_Input_And_Output();
```

Use `Wheel_Radius = 0.1f`, `Chassis_Radius = 0.5f`, and `k = sqrt(2)/2`. For pure `Vx = 1`, assert `RF/LF = -k/0.1` and `RB/LB = k/0.1`. For pure `Wz = 2`, assert all four outputs equal `10 rad/s`.

- [ ] **Step 2: Run the Omni test and verify it fails because the module is absent**

Run:

```powershell
New-Item -ItemType Directory -Force build\host | Out-Null
g++ -std=c++17 -IUser_File tests/host/test_chassis_omni.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp -o build/host/test_chassis_omni.exe
```

Expected: compilation fails because `mod_chassis_omni.h/.cpp` do not exist.

- [ ] **Step 3: Create the Omni header with the approved data model and exact interface**

Define `Class_Chassis_Omni` with private parameter, velocity, and four-wheel output structs. Initialize every member to zero. Use individual wheel getters rather than exposing the internal array.

- [ ] **Step 4: Implement the USTC Omni formula**

In `Calculate()`, clear all wheel outputs and return when `Wheel_Radius <= 0.0f`; otherwise apply the four formulas from the spec with `0.70710678118f` as `sqrt(2)/2`. Do not add reduction ratios or motor direction signs.

- [ ] **Step 5: Build and run the Omni test**

Run:

```powershell
g++ -std=c++17 -IUser_File tests/host/test_chassis_omni.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp -o build/host/test_chassis_omni.exe
.\build\host\test_chassis_omni.exe
```

Expected: exit code `0` with no assertion output.

- [ ] **Step 6: Review checkpoint**

Confirm the source contains no motor includes, no `/ cos(45)` expression, no gear ratios, and no global chassis object.

---

### Task 2: Swerve chassis kinematics and nearest steering

**Files:**
- Create: `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.h`
- Create: `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp`
- Create: `tests/host/test_chassis_swerve.cpp`

**Interfaces:**
- Consumes: current steering angles supplied by the caller through `Set_Now_Angle`.
- Produces: `Enum_Swerve_Wheel : uint8_t` with `Swerve_Wheel_RF/LF/RB/LB/Number`; `Class_Chassis_Swerve::Init(float, float)`, three speed setters, `Set_Now_Angle(Enum_Swerve_Wheel, float)`, `Calculate()`, `Get_Target_Angle(Enum_Swerve_Wheel) const`, and `Get_Wheel_Omega(Enum_Swerve_Wheel) const`.

- [ ] **Step 1: Write the failing Swerve geometry tests**

Add tests using `Wheel_Radius = 0.1f` and `Chassis_Radius = 0.5f`:

```cpp
Test_Forward_Translation_Points_All_Wheels_Forward();
Test_Right_Translation_Points_All_Wheels_Right();
Test_Pure_Rotation_Calculates_Tangent_Wheel_Vectors();
Test_Combined_Motion_Can_Stop_Only_One_Wheel();
```

For `Vy = 1`, assert all target angles are `0` and all wheel omegas are `10`. For `Vx = 1`, assert all target angles are `pi/2` and all wheel omegas are `10`. For pure `Wz = 2` with all current angles zero, assert:

```text
RF: target -pi/4, omega +10
LF: target +pi/4, omega -10
RB: target +pi/4, omega +10
LB: target -pi/4, omega -10
```

For the one-wheel cancellation case, use `Wz = 1`, `a = Chassis_Radius / sqrt(2)`, `Vx = a`, and `Vy = -a`; assert only `RF` enters low-speed hold while the other three wheels retain nonzero speed.

- [ ] **Step 2: Write the failing low-speed and invalid-parameter tests**

Add:

```cpp
Test_Speed_Below_Dead_Zone_Holds_Last_Target();
Test_Speed_Exactly_At_Dead_Zone_Is_Calculated();
Test_Zero_And_Negative_Wheel_Radius_Clear_Speeds_And_Hold_Angles();
Test_Reinit_Clears_All_Wheel_State();
Test_Invalid_Wheel_Enum_Is_Ignored_And_Reads_Zero();
```

First calculate a nonzero target, then command `wheel_speed = 0.0005f` and assert the target is unchanged and omega is zero. At exactly `0.001f`, assert omega is `0.01 rad/s` for a `0.1 m` wheel and the target is recalculated.

- [ ] **Step 3: Write the failing nearest-steering tests**

Add:

```cpp
Test_Error_Greater_Than_Positive_Half_Pi_Reverses_Wheel();
Test_Error_Less_Than_Negative_Half_Pi_Reverses_Wheel();
Test_Error_Exactly_Positive_Or_Negative_Half_Pi_Does_Not_Reverse();
Test_Target_Remains_Continuous_Across_Positive_Pi_Boundary();
Test_Repeated_Calculate_Does_Not_Accumulate_Target_Error();
```

Assert the corrected target is `angle_now + normalized_error - pi` or `+ pi` only outside the closed `[-pi/2, pi/2]` interval, and that the wheel omega sign changes only in those two branches.

- [ ] **Step 4: Run the Swerve test and verify it fails because the module is absent**

Run:

```powershell
g++ -std=c++17 -IUser_File tests/host/test_chassis_swerve.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp -o build/host/test_chassis_swerve.exe
```

Expected: compilation fails because `mod_chassis_swerve.h/.cpp` do not exist.

- [ ] **Step 5: Create the Swerve header and initialize all state**

Define the approved wheel enum, parameter/velocity/wheel-state structs, and exact public interface. `Init()` resets velocities, current angles, last targets, target angles, and wheel omegas to zero.

- [ ] **Step 6: Implement local wheel-vector calculation and low-speed hold**

Use the four approved `(rx, ry)` positions and:

```text
wheel_vx = Vx - Wz * ry
wheel_vy = Vy + Wz * rx
wheel_speed = sqrt(wheel_vx^2 + wheel_vy^2)
```

When `wheel_speed < 0.001f`, set only that wheel's omega to zero, copy its last target to the current target, and skip angle calculation.

- [ ] **Step 7: Implement angle normalization and nearest steering**

Calculate the nominal target with `atan2f(wheel_vx, wheel_vy)`. Normalize `target - current` to `(-pi, pi]`; apply the two strict `> pi/2` and `< -pi/2` reverse branches; output the continuous corrected target and signed wheel omega.

- [ ] **Step 8: Implement bounds checks for the public wheel enum**

For values not in `RF/LF/RB/LB`, `Set_Now_Angle` performs no write and both getters return `0.0f`.

- [ ] **Step 9: Build and run the Swerve test**

Run:

```powershell
g++ -std=c++17 -IUser_File tests/host/test_chassis_swerve.cpp User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp -o build/host/test_chassis_swerve.exe
.\build\host\test_chassis_swerve.exe
```

Expected: exit code `0` with no assertion output.

- [ ] **Step 10: Review checkpoint**

Confirm pure rotation is not suppressed, each wheel owns its hold state, angle outputs remain continuous, and the class has no DM or ESC dependency.

---

### Task 3: Central motor initialization module

**Files:**
- Create: `User_File/4_Task/Config/motor_init.h`
- Create: `User_File/4_Task/Config/motor_init.cpp`
- Create: `tests/host/test_motor_init.cpp`

**Interfaces:**
- Consumes: `hfdcan1` and `Class_Motor_DM_Normal`.
- Produces: `#define DM_J8006_MASTER_ID 0x33`, `extern Class_Motor_DM_Normal DM_J8006`, and `void Motor_Init()`.

- [ ] **Step 1: Write the failing motor initialization contract test**

Using the existing fake DM header, define `hfdcan1`, `fake_sequence`, and `fake_motor_state`, then assert after `Motor_Init()`:

```cpp
assert(DM_J8006_MASTER_ID == 0x33U);
assert(fake_motor_state.can_handler == &hfdcan1);
assert(fake_motor_state.init_rx_id == 0x33U);
assert(fake_motor_state.init_tx_id == 0x03U);
assert(fake_motor_state.method == Motor_DM_Control_Method_NORMAL_OMEGA);
assert(fake_motor_state.angle_max == 12.5f);
assert(fake_motor_state.omega_max == 45.0f);
assert(fake_motor_state.torque_max == 20.0f);
assert(fake_motor_state.control_omega == 0.0f);
assert(!fake_motor_state.motor_enable);
assert(fake_motor_state.enter_count == 0);
```

- [ ] **Step 2: Run the motor initialization test and verify it fails because the module is absent**

Run:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_motor_init.cpp User_File/4_Task/Config/motor_init.cpp -o build/host/test_motor_init.exe
```

Expected: compilation fails because `motor_init.h/.cpp` do not exist.

- [ ] **Step 3: Create `motor_init.h`**

Include the DM driver, export only the shared Master ID, the J8006 object, and `Motor_Init()`. Keep Motor ID and range constants private to the `.cpp` file.

- [ ] **Step 4: Create `motor_init.cpp`**

Define `DM_J8006`, initialize it on `hfdcan1` with the exact approved values, and set its control omega to zero. Do not call `CAN_Send_Enter()` and do not add placeholder Swerve motors.

- [ ] **Step 5: Build and run the motor initialization test**

Run:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_motor_init.cpp User_File/4_Task/Config/motor_init.cpp -o build/host/test_motor_init.exe
.\build\host\test_motor_init.exe
```

Expected: exit code `0` with no assertion output.

- [ ] **Step 6: Review checkpoint**

Confirm the module contains only motor objects, motor constants, and initialization, while callbacks and periodic control remain outside it.

---

### Task 4: Task integration, CMake, and complete regression

**Files:**
- Modify: `User_File/4_Task/tsk_config_and_callback.cpp:12-50,345-375`
- Modify: `CMakeLists.txt:47-59`
- Test: `tests/host/test_task_control.cpp`
- Test: `tests/host/test_remote_receive_task.cpp`

**Interfaces:**
- Consumes: all interfaces produced by Tasks 1-3.
- Produces: firmware target containing both chassis modules and centralized motor initialization while preserving the current task behavior.

- [ ] **Step 1: Run the existing task-control characterization test before refactoring**

Run the current test without `motor_init.cpp`:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_task_control.cpp User_File/4_Task/tsk_config_and_callback.cpp User_File/2_Device/BSP/Key/bsp_key.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp -o build/host/test_task_control_before_motor_init.exe
.\build\host\test_task_control_before_motor_init.exe
```

Expected: exit code `0`, including `Test_Initialization_Contract()`. This records the behavior that must remain unchanged.

- [ ] **Step 2: Replace direct J8006 ownership and initialization in the task file**

Include `4_Task/Config/motor_init.h`; remove the local J8006 object and initialization-only constants; change the CAN switch case to `DM_J8006_MASTER_ID`; replace the direct `Init()` and zero-target calls with `Motor_Init()`. Do not move callbacks, scheduler functions, remote logic, or printing.

- [ ] **Step 3: Add production sources to CMake**

Add exactly these sources to `target_sources`:

```cmake
User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp
User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp
User_File/4_Task/Config/motor_init.cpp
```

Do not add `tests/host` sources to the firmware target.

- [ ] **Step 4: Rebuild and run the task-control test with the new motor module linked**

Run:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_task_control.cpp User_File/4_Task/tsk_config_and_callback.cpp User_File/4_Task/Config/motor_init.cpp User_File/2_Device/BSP/Key/bsp_key.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp -o build/host/test_task_control.exe
.\build\host\test_task_control.exe
```

Expected: exit code `0`; the existing initialization order, CAN routing, control, timeout, scheduler, and UART assertions remain unchanged.

- [ ] **Step 5: Rebuild and run the remote-receive task test with the new motor module linked**

Run:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_remote_receive_task.cpp User_File/4_Task/tsk_config_and_callback.cpp User_File/4_Task/Config/motor_init.cpp User_File/2_Device/BSP/Key/bsp_key.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp -o build/host/test_remote_receive_task.exe
.\build\host\test_remote_receive_task.exe
```

Expected: exit code `0`; polling and one-second remote timeout behavior remain unchanged.

- [ ] **Step 6: Run all host regression tests**

Compile the remaining existing tests with:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_bsp_key.cpp User_File/2_Device/BSP/Key/bsp_key.cpp -o build/host/test_bsp_key.exe
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_pro_sbus.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp -o build/host/test_pro_sbus.exe
g++ -std=c++17 -Itests/host/motor_stubs -Itests/host/stubs -ICore/Inc -IUser_File -I. tests/host/test_motor_dm_enable.cpp User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp -o build/host/test_motor_dm_enable.exe
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_drv_uart.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp -o build/host/test_drv_uart.exe
```

Then execute all nine tests:

```powershell
.\build\host\test_chassis_omni.exe
.\build\host\test_chassis_swerve.exe
.\build\host\test_motor_init.exe
.\build\host\test_bsp_key.exe
.\build\host\test_pro_sbus.exe
.\build\host\test_motor_dm_enable.exe
.\build\host\test_drv_uart.exe
.\build\host\test_remote_receive_task.exe
.\build\host\test_task_control.exe
```

Expected: all nine executables exit with code `0` and no assertion output.

- [ ] **Step 7: Perform a clean Debug firmware build**

Run:

```powershell
cube-cmake --build --preset Debug --clean-first --target Robocon-Elian
```

Expected: `Robocon-Elian.elf` links successfully. Record any warnings and distinguish pre-existing warnings from new ones.

- [ ] **Step 8: Final scope and style review**

Verify there is no task-loop chassis invocation, no placeholder motor object, no RTOS/SUBS addition, no motor-driver include in either chassis module, no test source in the firmware target, and no unrelated workspace edits. Report software verification separately from future physical-bottom-chassis validation.

