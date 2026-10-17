# NRF24L01 Omni Chassis Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Use the existing NRF24L01 SBUS receiver to control four FDCAN3 DM3519 motors through the USTC Omni kinematics while preserving the current J8006 test and keeping Swerve inactive.

**Architecture:** `tsk_remote` owns NRF24L01/SBUS reception and exports one simple chassis-command structure. `tsk_chassis` consumes that structure, runs the existing USTC Omni solver, converts wheel-side speed through the two reduction ratios, and owns bottom-chassis enable/disable behavior; `tsk_config_and_callback` only performs top-level initialization, callback routing, and bare-metal scheduling.

**Tech Stack:** C++17, STM32H723 HAL, STM32CubeMX CMake/Ninja firmware build, host-side `g++` assertion tests, existing DM motor/NRF24L01/SBUS modules.

**Spec:** `docs/superpowers/specs/2026-10-05-omni-swerve-chassis-design.md`

## Global Constraints

- Follow `AGENTS.md` and the USTC RoboWalker-style headers, separators, naming, object organization, direct control flow, and clear Chinese comments.
- Keep bare-metal 10 ms/100 ms/1 s scheduling; do not add RTOS, queues, SUBS, dynamic allocation, exceptions, managers, or generic frameworks.
- Omni coordinates remain `X` right, `Y` forward, and `W` counterclockwise viewed from above; wheel order remains `RF`, `LF`, `RB`, `LB`.
- Omni must use the USTC `sqrt(2)/2` formula already implemented in `mod_chassis_omni`, not R1's `/cos(45 degrees)` translation terms.
- RC mapping is `Channel[0] -> Speed_Y`, `Channel[1] -> Speed_X`, `Channel[3] -> Speed_W`, with no extra `-1` coefficient.
- Map the clamped SBUS interval `200..1800` linearly to `-3.0f..3.0f`; clear mapped absolute values below `0.01f`.
- Chassis motion is permitted only for `Online != 0`, `Mode == 2`, and `Motor_Enable != 0`.
- Omni parameters are `Wheel_Radius=0.076f`, `Chassis_Radius=0.44717f`, `Chassis_Deceleration_Ratio=268.0f/17.0f`, and `DM3519_Deceleration_Ratio=19.2f`.
- FDCAN3 motor mapping is LF `0x01/0x11`, RF `0x02/0x22`, LB `0x03/0x33`, RB `0x04/0x44`; all direction coefficients are `+1.0f`.
- DM3519 Omni mode uses `NORMAL_OMEGA`, `PMAX=12.5f`, `VMAX=200.0f`, and `TMAX=10.0f`; startup is disabled with a zero target.
- Preserve the existing FDCAN1 J8006 behavior and its DEBUG/physical-key speed test.
- Keep the existing USTC Swerve module compiled but do not initialize, schedule, or command it in this change.
- The user is not using Git for this small project; replace commit steps with test-backed review checkpoints.

## Review Focus

- A raw SBUS channel below `200` or above `1800` must clamp before mapping, while `992..1008`-like center inputs inside the mapped `0.01f` dead zone must become exactly zero; covered by Task 1.
- A transition from valid MANUAL control to offline, non-MANUAL, or `Motor_Enable=0` must clear all four targets and issue disable without waiting for another packet; covered by Task 3.
- RF/LF/RB/LB solver order differs from LF/RF/LB/RB CAN-ID order, so each calculated wheel must reach the correct motor object; covered by Tasks 2 and 3.
- Repeated 10 ms callbacks must not repeatedly send enter/exit commands once motor state already matches the requested state; covered by Task 3.
- FDCAN1 and FDCAN3 callbacks must coexist, route only their own IDs, and remain registered before `init_finished` becomes true; covered by Task 4.

---

### Task 1: Extract the remote task and map SBUS channels

**Files:**
- Create: `User_File/4_Task/tsk_remote/tsk_remote.h`
- Create: `User_File/4_Task/tsk_remote/tsk_remote.cpp`
- Modify: `tests/host/test_remote_receive_task.cpp`

**Interfaces:**
- Consumes: `Class_NRF24L01::NRF24L01_Init()`, `NRF24L01_Receive()`, `Class_SBUS_Receive::Init()`, `Data_Process()`, and `TIM_10ms_PeriodElapsedCallback()`.
- Produces: `Struct_Remote_Chassis_Command`, global `Remote_Chassis_Command`, global `NRF24L01`, `Remote_Init()`, and `Remote_10ms_PeriodElapsedCallback()`; the existing `SBUS_Receive` global remains defined once and accessible through `pro_sbus.h`.

- [ ] **Step 1: Rewrite the remote task test around the new interface**

Add a test packet helper that packs 16 11-bit SBUS channels and preserves the existing custom bytes. Add named tests:

```cpp
Test_Remote_Init_Loads_Safe_Command();
Test_Channel_Endpoints_Map_To_Minus_Three_Zero_And_Plus_Three();
Test_Out_Of_Range_Channels_Clamp_And_Center_Dead_Zone_Clears();
Test_Channel_Zero_One_And_Three_Map_To_Y_X_And_W();
Test_Non_Manual_And_Offline_Clear_All_Chassis_Speeds();
Test_NRF_Polling_And_One_Second_Timeout_Are_Preserved();
```

Assert exact endpoint values within `1.0e-5f`, `Mode==2` is required for nonzero commands, and `Online/Mode/Motor_Enable` copy the parsed SBUS state.

- [ ] **Step 2: Run the focused test and confirm it fails on missing `tsk_remote`**

Run:

```powershell
New-Item -ItemType Directory -Force build\host | Out-Null
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_remote_receive_task.cpp User_File/4_Task/tsk_remote/tsk_remote.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp -o build/host/test_remote_receive_task.exe
```

Expected: compile failure because the new task interface is not implemented yet.

- [ ] **Step 3: Create the remote task header**

Declare exactly:

```cpp
struct Struct_Remote_Chassis_Command
{
    float Speed_X;
    float Speed_Y;
    float Speed_W;
    uint8_t Online;
    uint8_t Mode;
    uint8_t Motor_Enable;
};

extern Class_NRF24L01 NRF24L01;
extern Struct_Remote_Chassis_Command Remote_Chassis_Command;

void Remote_Init();
void Remote_10ms_PeriodElapsedCallback();
```

Use direct project-style declarations; do not introduce a remote manager class or queue.

- [ ] **Step 4: Implement receive, timeout, clamp, mapping, and safety clearing**

`Remote_Init()` initializes `SBUS_Receive`, `NRF24L01`, and safe zero command values. The 10 ms callback advances the SBUS timeout, polls NRF24L01, passes a received 31-byte packet to `Data_Process`, copies status fields, maps channels with `(channel-1000.0f)*3.0f/800.0f`, and clears all speeds unless online and MANUAL. Use one small private mapping function; keep complete formulas and calls on one line where readable.

- [ ] **Step 5: Build and run the remote task test**

Run the Step 2 compile command, then:

```powershell
.\build\host\test_remote_receive_task.exe
```

Expected: exit code `0` with no assertion output.

- [ ] **Step 6: Review checkpoint**

Confirm there is one NRF object, one SBUS object, no motor dependency, no RTOS code, and no hidden gear-mode multiplier.

---

### Task 2: Add four FDCAN3 DM3519 objects and initialization

**Files:**
- Modify: `User_File/4_Task/Config/motor_init.h`
- Modify: `User_File/4_Task/Config/motor_init.cpp`
- Modify: `tests/host/stubs/2_Device/Motor/Motor_DM/dvc_motor_dm.h`
- Modify: `tests/host/test_motor_init.cpp`
- Modify: existing host tests that directly read the old single `fake_motor_state`

**Interfaces:**
- Consumes: `hfdcan1`, `hfdcan3`, and `Class_Motor_DM_Normal`.
- Produces: `DM_J8006`, `DM_Chassis_LF/RF/LB/RB`, their four exported Master-ID macros, `Motor_Init()`, `Motor_Omni_Init()`, and inactive `Motor_Swerve_Init()`.

- [ ] **Step 1: Generalize the host DM stub for independent motor instances**

Make each fake `Class_Motor_DM_Normal` own its own `Fake_Motor_State` and expose a test-only `Get_Fake_State()` accessor. Preserve all existing fake methods and counters, then update existing tests to read the intended object's state rather than one shared global state.

- [ ] **Step 2: Extend the motor initialization contract test**

Keep all J8006 assertions and add four independent DM3519 assertions:

```text
LF: hfdcan3, rx/master 0x11, tx/motor 0x01
RF: hfdcan3, rx/master 0x22, tx/motor 0x02
LB: hfdcan3, rx/master 0x33, tx/motor 0x03
RB: hfdcan3, rx/master 0x44, tx/motor 0x04
```

For each chassis motor assert `NORMAL_OMEGA`, `12.5f`, `200.0f`, `10.0f`, zero target, and disabled state. Assert `Motor_Init()` still configures J8006 exactly as before and calls the active Omni initializer, not the Swerve initializer.

- [ ] **Step 3: Run the motor-init test and verify the four objects are missing**

Run:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_motor_init.cpp User_File/4_Task/Config/motor_init.cpp -o build/host/test_motor_init.exe
```

Expected: compile or assertion failure for missing chassis objects and FDCAN3 initialization.

- [ ] **Step 4: Add public motor objects, Master IDs, and initialization functions**

Export the four objects and Master IDs from `motor_init.h`. In `motor_init.cpp`, keep all IDs and ranges together, implement `Motor_Omni_Init()` with the exact table and limits, and have `Motor_Init()` perform current J8006 initialization followed by active Omni initialization.

Implement `Motor_Swerve_Init()` only as the approved alternative reconfiguration entry for the same four DM objects in `NORMAL_ANGLE_OMEGA`; do not call it, add geometry, add ESC objects, or invent an operating steering speed.

- [ ] **Step 5: Build and run the motor-init test**

Run the Step 3 compile command, then:

```powershell
.\build\host\test_motor_init.exe
```

Expected: exit code `0` with all five motors independently verified.

- [ ] **Step 6: Review checkpoint**

Confirm object names match physical positions, CAN IDs are not confused with wheel-solver order, all targets start at zero, and initialization sends no enter-control command.

---

### Task 3: Implement the active Omni chassis task and safety behavior

**Files:**
- Create: `User_File/4_Task/tsk_chassis/tsk_chassis.h`
- Create: `User_File/4_Task/tsk_chassis/tsk_chassis.cpp`
- Create: `tests/host/test_chassis_task.cpp`

**Interfaces:**
- Consumes: global `Remote_Chassis_Command`; `Class_Chassis_Omni`; `DM_Chassis_LF/RF/LB/RB`; each motor's enable, target-speed, send, and alive methods.
- Produces: `Chassis_Init()`, `Chassis_10ms_PeriodElapsedCallback()`, and `Chassis_100ms_PeriodElapsedCallback()`.

- [ ] **Step 1: Write the focused chassis-task tests**

Add:

```cpp
Test_Init_Uses_R1_Mechanics_But_USTC_Omni_Formula();
Test_Pure_X_Maps_RF_LF_RB_LB_To_The_Correct_Motors();
Test_Pure_Y_And_Pure_W_Preserve_All_Positive_Direction_Coefficients();
Test_Reduction_Ratios_Convert_Wheel_Omega_To_DM_Omega();
Test_Valid_Manual_Enable_Enters_Once_And_Sends_Every_10ms();
Test_Offline_NonManual_And_Disable_Clear_Targets_And_Exit_Once();
Test_100ms_Callback_Checks_All_Four_Motors();
```

For pure `Vx=1`, calculate expected wheel speed with `0.70710678118f/0.076f`, then multiply by `(268.0f/17.0f)/19.2f`. Assert RF and LF are negative, RB and LB positive, and each value reaches the position-matched object.

- [ ] **Step 2: Run the focused test and confirm it fails on the missing task**

Run:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_chassis_task.cpp User_File/4_Task/tsk_chassis/tsk_chassis.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp User_File/4_Task/Config/motor_init.cpp -o build/host/test_chassis_task.exe
```

Expected: compile failure because `tsk_chassis` is not implemented.

- [ ] **Step 3: Create the chassis task interface and initialize only Omni**

`Chassis_Init()` initializes `Chassis_Omni` with `0.076f` and `0.44717f`. Keep the `Class_Chassis_Swerve` object and future code out of the active callback; do not initialize it with guessed parameters.

- [ ] **Step 4: Implement the 10 ms valid-control path**

When online, MANUAL, and enabled, send enter commands only to motors whose fake/real driver reports disabled, copy the three remote speeds into `Chassis_Omni`, call `Calculate()`, apply `(268.0f/17.0f)/19.2f`, assign RF/LF/RB/LB outputs to the correct physical objects, and invoke all four send callbacks.

- [ ] **Step 5: Implement the immediate safe path and the 100 ms callback**

For any invalid control condition, set Omni inputs and all four motor targets to zero, send exit only to motors currently enabled, and still invoke the four periodic send callbacks. The 100 ms callback invokes all four alive callbacks exactly once.

- [ ] **Step 6: Build and run the chassis task test**

Run the Step 2 compile command, then:

```powershell
.\build\host\test_chassis_task.exe
```

Expected: exit code `0` with no assertion output.

- [ ] **Step 7: Review checkpoint**

Confirm the task contains no R1 Omni formula, no global `-1`, no Swerve scheduling, no ESC placeholder, and no duplicate DM initialization.

---

### Task 4: Integrate callbacks, scheduler, task paths, and startup safety

**Files:**
- Modify: `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp`
- Modify: `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.h` only if include guards or comments need path-neutral cleanup
- Modify: `tests/host/test_task_control.cpp`
- Modify: `CMakeLists.txt`
- Modify: `Core/Src/main.c`

**Interfaces:**
- Consumes: Tasks 1-3 interfaces and existing `CAN_Init`, key, UART, timestamp, and J8006 functions.
- Produces: fully scheduled firmware with independent FDCAN1 and FDCAN3 callbacks and current J8006 behavior preserved.

- [ ] **Step 1: Update the task integration characterization tests first**

Adapt the existing test to two registered CAN callbacks keyed by `hfdcan1` and `hfdcan3`. Preserve every existing J8006 key, DEBUG remote button, timeout, UART, timestamp, and scheduler assertion. Add:

```cpp
Test_CAN3_Routes_0x11_0x22_0x33_0x44_To_The_Correct_Motor();
Test_CAN1_Still_Routes_Only_J8006_0x33();
Test_Init_Registers_Both_CAN_Buses_Before_Init_Finished();
Test_10ms_Order_Updates_Remote_Before_Chassis_Control();
Test_Startup_Exits_J8006_And_All_Four_Chassis_Motors();
```

- [ ] **Step 2: Run the integration test and verify it fails against the old layout**

Run:

```powershell
g++ -std=c++17 -Itests/host/stubs -ICore/Inc -IUser_File tests/host/test_task_control.cpp User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp User_File/4_Task/tsk_remote/tsk_remote.cpp User_File/4_Task/tsk_chassis/tsk_chassis.cpp User_File/4_Task/Config/motor_init.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp User_File/2_Device/BSP/Key/bsp_key.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp -o build/host/test_task_control.exe
```

Expected: compile or assertion failure because CAN3 and the new task boundaries are not integrated yet.

- [ ] **Step 3: Move remote ownership and call the two task callbacks**

Remove local NRF24L01/SBUS object definitions and `Remote_Receive()` from `tsk_config_and_callback.cpp`. Call `Remote_Init()` in `Task_Init()`. In the 10 ms callback call remote first, keep current key/J8006 processing second, and call chassis third. Preserve direct use of `SBUS_Receive.SBUS_Data` for existing DEBUG button behavior and USART10 printing.

- [ ] **Step 4: Add FDCAN3 feedback routing and startup safety**

Register `CAN1_Callback` on `hfdcan1` and new `CAN3_Callback` on `hfdcan3`. Route only the four approved Master IDs to their matching objects. After all initialization succeeds and before normal looping, send exit to J8006 and all four chassis motors; keep `init_finished` false during CAN registration and true only after startup is complete.

- [ ] **Step 5: Add 100 ms chassis online checks without changing the 1 s print policy**

Call `Chassis_100ms_PeriodElapsedCallback()` alongside the existing J8006 alive callback. Keep USART10 remote printing at one second and leave motor printing as the existing separately callable/commented function.

- [ ] **Step 6: Fix build and include paths**

In `CMakeLists.txt`, replace the old task source path with:

```cmake
User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp
User_File/4_Task/tsk_remote/tsk_remote.cpp
User_File/4_Task/tsk_chassis/tsk_chassis.cpp
```

Keep existing Omni, Swerve, and `motor_init` sources. In `Core/Src/main.c`, include `4_Task/tsk_config_and_callback/tsk_config_and_callback.h`. Do not edit generated peripheral initialization outside CubeMX user regions.

- [ ] **Step 7: Build and run the integration test**

Run the Step 2 compile command, then:

```powershell
.\build\host\test_task_control.exe
```

Expected: exit code `0`; old J8006 behavior and all new CAN3/scheduling assertions pass.

- [ ] **Step 8: Review checkpoint**

Confirm `tsk_config_and_callback` is orchestration-only except for the intentionally retained J8006 test and USART10 functions, and confirm no duplicate object definitions remain.

---

### Task 5: Full regression, firmware build, and style/scope audit

**Files:**
- Verify: all files changed in Tasks 1-4
- Verify: `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.h`
- Verify: `User_File/3_Module/Chassis/Swerve/mod_chassis_swerve.cpp`
- Verify: `AGENTS.md`

**Interfaces:**
- Consumes: all previous tasks.
- Produces: one software-verified firmware integration ready for low-speed, wheels-off-ground hardware testing.

- [ ] **Step 1: Compile every host test with its production sources**

Rebuild the existing tests plus the new `test_chassis_task.exe`, using the same include ordering as their focused tasks. Required executables are:

```text
test_bsp_key.exe
test_chassis_omni.exe
test_chassis_swerve.exe
test_drv_uart.exe
test_motor_dm_enable.exe
test_motor_init.exe
test_pro_sbus.exe
test_remote_receive_task.exe
test_chassis_task.exe
test_task_control.exe
```

- [ ] **Step 2: Run all host tests**

Execute all ten binaries. Expected: every process exits with code `0` and produces no assertion failure.

- [ ] **Step 3: Configure and build the complete Debug firmware**

Run:

```powershell
cmake --preset Debug
cmake --build --preset Debug --target Robocon-Elian
```

Expected: `Robocon-Elian.elf` compiles and links successfully. If the configured STM32Cube CMake executable is not on `PATH`, use the same executable selected by the VS Code STM32Cube extension without changing project files.

- [ ] **Step 4: Inspect compiler diagnostics and generated target contents**

Confirm no new warning is introduced, the CMSIS-DSP archive path still resolves, no host stub/test enters the firmware target, and `hfdcan3` symbols resolve from CubeMX sources.

- [ ] **Step 5: Run the final code-style and scope audit**

Check:

- readable single-line formulas and short calls where practical;
- USTC file headers, section separators, names, and Chinese comments;
- no RTOS, queue, SUBS, manager, dynamic memory, or extra framework;
- no R1 Omni formula or global negative direction coefficient;
- no active Swerve call and no ESC placeholder;
- startup/timeout/mode/enable safety clears targets and disables motors;
- no unrelated files or temporary folders were created.

- [ ] **Step 6: Report verification boundaries**

Report host-test and firmware-build results as software verification only. The next physical step is low gear with wheels suspended, checking forward, lateral, and rotation directions before loading the chassis.
