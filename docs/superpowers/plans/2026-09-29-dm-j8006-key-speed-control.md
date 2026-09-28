# DM-J8006 Key Speed Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive one DM-J8006-2EC on FDCAN1 in velocity mode and adjust its output-shaft target speed by one radian per second for each debounced KEY1/KEY2 press.

**Architecture:** Add a reusable, GPIO-bound key state machine modeled on the USTC key BSP, then keep the board-specific motor object, key instances, scheduling, and HAL callback routing inside the existing task integration layer. `main.c` only calls the C-linkage task entry points from CubeMX user sections; the byte-identical USTC motor and middleware replicas remain untouched.

**Tech Stack:** STM32H723ZGT6, STM32CubeMX CMake output, STM32 HAL FDCAN/TIM/GPIO, C++17, USTC `Motor_DM`, MinGW host tests with HAL/device stubs, GNU Arm Embedded GCC/G++, CMake/Ninja.

**Spec:** `docs/superpowers/specs/2026-09-29-dm-j8006-key-speed-control-design.md`

## Global Constraints

- Use FDCAN1 at the existing CubeMX-configured 1 Mbps nominal bit rate.
- Initialize `Class_Motor_DM_Normal` with Master ID `0x33`, motor CAN ID `0x03`, `Motor_DM_Control_Method_NORMAL_OMEGA`, PMAX `12.5f`, VMAX `45.0f`, and TMAX `20.0f`.
- Start at `0.0f rad/s`; KEY1 adds `1.0f`, KEY2 subtracts `1.0f`, and clamp the command to `[-5.0f, +5.0f]`.
- Treat PF2/KEY1 and PF4/KEY2 as active-low inputs with external pull-ups; retain the generated `GPIO_NOPULL` configuration.
- A held key triggers once. Both stable pressed keys command zero and suppress single-key actions until both are released.
- Poll keys and call `TIM_Send_PeriodElapsedCallback()` every 10 ms; call `TIM_100ms_Alive_PeriodElapsedCallback()` every 100 ms without blocking delays.
- Keep motor CAN Timeout at 0 and preserve the original USTC error-clear/re-enable and feedback-loss/re-enable behavior.
- Do not export a target-speed variable/getter, add telemetry, add EXTI/a new timer, or configure motor registers.
- Do not modify the ten byte-identical USTC source replicas, the DSP archive, or `cmake/stm32cubemx/CMakeLists.txt`.
- Preserve the user's current CubeMX-generated KEY1/KEY2 changes in `Robocon-Elian.ioc`, `Core/Inc/main.h`, and `Core/Src/gpio.c`.

## Review Focus

- Key held during boot: initialization must adopt the current level without emitting a false press; Task 1 host test pins this behavior.
- Contact bounce and long holds: two equal 10 ms samples are required for a transition, and the trigger state lasts exactly one processing call; Task 1 host test covers both.
- Simultaneous or partly released dual-key input: zero must win, and neither key may step speed until both are released; Task 2 host test covers the latch sequence.
- Repeated presses at both limits: the command must stop exactly at `+5.0f` and `-5.0f`; Task 2 host test drives past both limits.
- HAL tick wrap or a long foreground stall: unsigned elapsed-time checks must continue after wrap and must not burst-send missed periods; Task 2 host test covers wrap and a 1-second jump.

---

### Task 1: Add the reusable debounced key BSP

**Files:**
- Create: `User_File/2_Device/BSP/Key/bsp_key.h`
- Create: `User_File/2_Device/BSP/Key/bsp_key.cpp`
- Create: `tests/host/stubs/stm32h7xx_hal.h`
- Create: `tests/host/stubs/gpio.h`
- Create: `tests/host/test_bsp_key.cpp`

**Interfaces:**
- Consumes: `HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t)` and the active-low PF2/PF4 electrical convention.
- Produces: `Enum_BSP_Key_Status`, `Class_Key::Init(GPIO_TypeDef *, uint16_t, GPIO_PinState)`, `Class_Key::Get_Key_Status() const`, and `Class_Key::TIM_10ms_PeriodElapsedCallback()`.

- [ ] **Step 1: Write the failing host test for the key state machine**

  In `tests/host/test_bsp_key.cpp`, provide a fake `HAL_GPIO_ReadPin` and assert these sequences against one `Class_Key`:

  ```cpp
  key.Init(fake_port, GPIO_PIN_2, GPIO_PIN_SET);
  assert(key.Get_Key_Status() == BSP_Key_Status_FREE);

  set_pin(GPIO_PIN_RESET);                 // first low sample
  key.TIM_10ms_PeriodElapsedCallback();
  assert(key.Get_Key_Status() == BSP_Key_Status_FREE);
  key.TIM_10ms_PeriodElapsedCallback();    // second low sample
  assert(key.Get_Key_Status() == BSP_Key_Status_TRIG_FREE_PRESSED);
  key.TIM_10ms_PeriodElapsedCallback();    // held low
  assert(key.Get_Key_Status() == BSP_Key_Status_PRESSED);

  set_pin(GPIO_PIN_SET);                   // first high sample
  key.TIM_10ms_PeriodElapsedCallback();
  assert(key.Get_Key_Status() == BSP_Key_Status_PRESSED);
  key.TIM_10ms_PeriodElapsedCallback();    // second high sample
  assert(key.Get_Key_Status() == BSP_Key_Status_TRIG_PRESSED_FREE);
  key.TIM_10ms_PeriodElapsedCallback();
  assert(key.Get_Key_Status() == BSP_Key_Status_FREE);
  ```

  Add assertions that alternating low/high bounce never creates a trigger and that `Init()` while the pin is low starts at `BSP_Key_Status_PRESSED`, not a press trigger.

- [ ] **Step 2: Compile the test and verify that it fails before the BSP exists**

  Run:

  ```powershell
  New-Item -ItemType Directory -Force -Path 'build/host' | Out-Null
  & 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror `
      -I tests/host/stubs -I User_File `
      tests/host/test_bsp_key.cpp User_File/2_Device/BSP/Key/bsp_key.cpp `
      -o build/host/test_bsp_key.exe
  ```

  Expected: FAIL because `bsp_key.h/.cpp` or the declared generic interface does not exist.

- [ ] **Step 3: Implement the minimal generic key class**

  Use the USTC names and four-state model:

  ```cpp
  enum Enum_BSP_Key_Status {
      BSP_Key_Status_FREE = 0,
      BSP_Key_Status_TRIG_FREE_PRESSED,
      BSP_Key_Status_TRIG_PRESSED_FREE,
      BSP_Key_Status_PRESSED,
  };

  class Class_Key {
  public:
      void Init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin,
                GPIO_PinState key_free_state = GPIO_PIN_SET);
      Enum_BSP_Key_Status Get_Key_Status() const;
      void TIM_10ms_PeriodElapsedCallback();
  };
  ```

  Store the bound port/pin, stable level, candidate level, candidate count, free level, and status. Require two consecutive candidate samples before changing the stable level. A transition status is visible for exactly one callback; during a first/bouncing candidate sample, report the steady status corresponding to the previous stable level.

- [ ] **Step 4: Build and run the key test**

  Re-run the Step 2 compile command, then:

  ```powershell
  & 'build/host/test_bsp_key.exe'
  ```

  Expected: compiler exits 0 and the test executable exits 0 with all initialization, debounce, edge, bounce, and hold assertions passing.

- [ ] **Step 5: Commit the key BSP and its host test**

  ```powershell
  git add User_File/2_Device/BSP/Key tests/host/stubs/stm32h7xx_hal.h tests/host/stubs/gpio.h tests/host/test_bsp_key.cpp
  git commit -m "feat: add debounced key BSP"
  ```

### Task 2: Implement the DM motor task, scheduling, and callback routing

**Files:**
- Modify: `User_File/4_Task/tsk_config_and_callback.h`
- Modify: `User_File/4_Task/tsk_config_and_callback.cpp`
- Create: `tests/host/stubs/fdcan.h`
- Create: `tests/host/stubs/tim.h`
- Create: `tests/host/stubs/1_Middleware/Driver/CAN/drv_can.h`
- Create: `tests/host/stubs/1_Middleware/System/Timestamp/sys_timestamp.h`
- Create: `tests/host/stubs/2_Device/Motor/Motor_DM/dvc_motor_dm.h`
- Create: `tests/host/test_task_control.cpp`

**Interfaces:**
- Consumes: Task 1 `Class_Key`; generated `hfdcan1`, `htim5`, `Key1_GPIO_Port/Key1_Pin`, `Key2_GPIO_Port/Key2_Pin`; USTC `CAN_Init`, `SYS_Timestamp`, and `Class_Motor_DM_Normal` APIs.
- Produces: C-linkage `void Task_Init(void)`, `void Task_Loop(void)`, one C++ `bool init_finished`, FDCAN1 feedback routing for ID `0x33`, and the TIM5 overflow callback.

- [ ] **Step 1: Write the failing task-level host test and device stubs**

  Make the motor stub expose one recorder named `Fake_Motor_State fake_motor_state`; make the other stubs expose the recorded call order, send/alive counts, callback pointer, timer-start count, and GPIO levels. Compile the real production `bsp_key.cpp` and `tsk_config_and_callback.cpp`; do not duplicate the production policy in the test.

  Assert the initialization contract:

  ```cpp
  fake_tick = 0;
  set_key_levels(GPIO_PIN_SET, GPIO_PIN_SET);
  Task_Init();
  assert(init_finished);
  assert(fake_motor_state.init_rx_id == 0x33);
  assert(fake_motor_state.init_tx_id == 0x03);
  assert(fake_motor_state.method == Motor_DM_Control_Method_NORMAL_OMEGA);
  assert(fake_motor_state.angle_max == 12.5f);
  assert(fake_motor_state.omega_max == 45.0f);
  assert(fake_motor_state.torque_max == 20.0f);
  assert(fake_motor_state.control_omega == 0.0f);
  assert(fake_can_init_saw_init_finished == false);
  assert(fake_tim5_start_count == 1);
  assert(fake_motor_state.enter_count == 1);
  ```

  Add separate test cases that assert:

  - two 10 ms low samples on KEY1 produce `+1.0f`, holding does not repeat, releasing and pressing again produces `+2.0f`;
  - KEY2 produces the symmetric decrement;
  - more than ten presses clamp exactly at `+5.0f` and `-5.0f`;
  - both keys becoming stably pressed set `0.0f`; releasing only one keeps the latch; after both release a new press works;
  - every due 10 ms iteration calls motor send after key processing, and every due 100 ms iteration calls alive processing;
  - a tick sequence spanning `UINT32_MAX` still schedules correctly, while a 1000 ms jump causes only one send and one alive call;
  - invoking the registered CAN callback with ID `0x33` increments the motor receive count, while another ID does not;
  - `HAL_TIM_PeriodElapsedCallback(&htim5)` increments the timestamp overflow count, while a different timer does not.

- [ ] **Step 2: Compile the task test and verify that it fails**

  Run:

  ```powershell
  & 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror `
      -I tests/host/stubs -I User_File `
      tests/host/test_task_control.cpp `
      User_File/2_Device/BSP/Key/bsp_key.cpp `
      User_File/4_Task/tsk_config_and_callback.cpp `
      -o build/host/test_task_control.exe
  ```

  Expected: FAIL because `Task_Init()` and `Task_Loop()` are not declared or defined.

- [ ] **Step 3: Make the task header safe for both C and C++**

  Keep `init_finished` visible only to C++ translation units, and expose the two application entry points with C linkage so `main.c` can include the header without needing the C `bool` type:

  ```cpp
  #ifdef __cplusplus
  extern bool init_finished;
  extern "C" {
  #endif

  void Task_Init(void);
  void Task_Loop(void);

  #ifdef __cplusplus
  }
  #endif
  ```

  Do not expose the motor object, key objects, current target speed, CAN callback, or scheduler state.

- [ ] **Step 4: Implement initialization and private application state**

  In `tsk_config_and_callback.cpp`, include the real key, motor, CAN, timestamp, GPIO, FDCAN, and TIM headers. Keep one private `Class_Motor_DM_Normal`, two private `Class_Key` objects, two `uint32_t` scheduler timestamps, and one dual-key latch.

  Implement `Task_Init()` in this exact order: set `init_finished=false`; initialize `SYS_Timestamp` with `&htim5`; initialize the motor with the fixed `0x33/0x03/NORMAL_OMEGA/12.5f/45.0f/20.0f` arguments; initialize PF2 and PF4 as active-low keys; set omega to zero; call `CAN_Init(&hfdcan1, CAN1_Callback)`; start TIM5 interrupts and route a non-`HAL_OK` result to `Error_Handler()`; capture the current tick for both schedules; set `init_finished=true`; call `CAN_Send_Enter()`.

- [ ] **Step 5: Implement key policy and non-blocking scheduling**

  In each due 10 ms cycle, process both keys before inspecting their statuses. Treat `TRIG_FREE_PRESSED` and `PRESSED` as down. Apply policy in this priority order: both down sets zero and latches; while latched, suppress single-key work until both are up; otherwise a KEY1 press edge adds `1.0f` or a KEY2 press edge subtracts `1.0f`; constrain before `Set_Control_Omega()`; then call `TIM_Send_PeriodElapsedCallback()`.

  In each due 100 ms cycle call `TIM_100ms_Alive_PeriodElapsedCallback()`. Compare elapsed time as `(uint32_t)(now - previous) >= period` and set `previous = now` when due so a long stall does not create catch-up bursts.

- [ ] **Step 6: Implement CAN and TIM callback routing**

  Keep `CAN1_Callback(FDCAN_RxHeaderTypeDef &, uint8_t *)` private to the `.cpp`; route only standard identifier `0x33` to the motor object's `CAN_RxCpltCallback()` and ignore all others. Define `extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *)`; route only TIM5 to `SYS_Timestamp.TIM_3600s_PeriodElapsedCallback()`.

- [ ] **Step 7: Build and run both host test executables**

  Re-run Task 1 Step 4 and Task 2 Step 2, then:

  ```powershell
  & 'build/host/test_bsp_key.exe'
  & 'build/host/test_task_control.exe'
  ```

  Expected: both executables exit 0; no production-only target-speed accessor or debug global is required.

- [ ] **Step 8: Commit the task runtime and tests**

  ```powershell
  git add User_File/4_Task tests/host
  git commit -m "feat: control DM motor speed from keys"
  ```

### Task 3: Activate the task from CubeMX user code and prove the firmware build

**Files:**
- Modify: `CMakeLists.txt`
- Modify only USER CODE sections: `Core/Src/main.c`
- Preserve and include the existing CubeMX changes: `Core/Inc/main.h`
- Preserve and include the existing CubeMX changes: `Core/Src/gpio.c`
- Preserve and include the existing CubeMX changes: `Robocon-Elian.ioc`
- Verify unchanged: `cmake/stm32cubemx/CMakeLists.txt`
- Verify unchanged: the ten USTC replica source files and `Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a`

**Interfaces:**
- Consumes: Task 2 `Task_Init()`/`Task_Loop()` and the already-generated PF2/PF4 GPIO definitions.
- Produces: A linked firmware that starts the task after all CubeMX peripheral initialization and services it continuously from the foreground loop.

- [ ] **Step 1: Run a failing activation contract check**

  Run:

  ```powershell
  $cmakeText = Get-Content -Raw -LiteralPath 'CMakeLists.txt'
  $mainText = Get-Content -Raw -LiteralPath 'Core/Src/main.c'
  $missing = @()
  if (!$cmakeText.Contains('User_File/2_Device/BSP/Key/bsp_key.cpp')) { $missing += 'bsp_key.cpp in CMake' }
  if (!$mainText.Contains('#include "4_Task/tsk_config_and_callback.h"')) { $missing += 'task header in main.c' }
  if (!$mainText.Contains('Task_Init();')) { $missing += 'Task_Init call' }
  if (!$mainText.Contains('Task_Loop();')) { $missing += 'Task_Loop call' }
  if (!$missing) { throw 'Expected activation contract to fail before integration' }
  $missing
  ```

  Expected: reports all four missing activation items.

- [ ] **Step 2: Add the key BSP to the top-level firmware target**

  Add exactly `User_File/2_Device/BSP/Key/bsp_key.cpp` to the existing top-level `target_sources` list. Do not add host tests/stubs to the firmware target and do not edit the generated sub-CMake.

- [ ] **Step 3: Call the task from CubeMX user sections**

  In `Core/Src/main.c`, add `#include "4_Task/tsk_config_and_callback.h"` inside `USER CODE BEGIN Includes`, call `Task_Init();` inside `USER CODE BEGIN 2` after all `MX_*_Init()` calls, and call `Task_Loop();` inside `USER CODE BEGIN 3` on each `while (1)` iteration. Preserve all existing CubeMX-generated text outside those user sections, including the user's current GPIO changes.

- [ ] **Step 4: Re-run the activation and generated-configuration checks**

  Re-run the Step 1 token checks without the deliberate failure branch; require no missing token. Then run:

  ```powershell
  rg -n "#define Key1_Pin GPIO_PIN_2|#define Key1_GPIO_Port GPIOF|#define Key2_Pin GPIO_PIN_4|#define Key2_GPIO_Port GPIOF" Core/Inc/main.h
  rg -n "Key1_Pin\|Key2_Pin|GPIO_MODE_INPUT|GPIO_NOPULL" Core/Src/gpio.c
  rg -n "PF2.Signal=GPIO_Input|PF4.Signal=GPIO_Input|TIM5.Period=3600000000-1|TIM5.Prescaler=240-1" Robocon-Elian.ioc
  git diff --exit-code HEAD -- cmake/stm32cubemx/CMakeLists.txt
  ```

  Expected: all key/TIM5 tokens appear and the generated sub-CMake has no diff.

- [ ] **Step 5: Re-run all host tests after activation edits**

  Rebuild and execute `test_bsp_key.exe` and `test_task_control.exe` using the commands from Tasks 1 and 2.

  Expected: both compile and exit 0.

- [ ] **Step 6: Configure and clean-build the ARM Debug firmware**

  Run:

  ```powershell
  & 'D:\CLion\CLion 2026.2.1\bin\cmake\win\x64\bin\cmake.exe' --preset Debug '-DCMAKE_MAKE_PROGRAM=D:/CLion/CLion 2026.2.1/bin/ninja/win/x64/ninja.exe'
  & 'D:\CLion\CLion 2026.2.1\bin\cmake\win\x64\bin\cmake.exe' --build --preset Debug --clean-first
  ```

  Expected: configure, C/C++ compilation, and link all exit 0 and regenerate `build/Debug/Robocon-Elian.elf` plus the map file.

- [ ] **Step 7: Re-prove the USTC replica and DSP archive hashes**

  Compare SHA-256 for the ten files under `Algorithm/Basic`, `Algorithm/PID`, `Driver/CAN`, `System/Timestamp`, and `Motor/Motor_DM`, plus `libarm_cortexM7lfsp_math.a`, against the matching paths in `D:\27Robocon\damiao_mc02_bsp-main\damiao_mc02_bsp-main\dm02_test`.

  Expected: `PASS: 11 artifacts match`; no replica was changed to implement this application.

- [ ] **Step 8: Inspect the linked ELF and the final scoped diff**

  Run:

  ```powershell
  $undefined = & 'C:\clionToolchains\gcc-arm-none-eabi-10.3-2021.10\bin\arm-none-eabi-nm.exe' -u 'build/Debug/Robocon-Elian.elf' |
      Select-String -Pattern 'Task_Init|Task_Loop|init_finished|SYS_Timestamp|HAL_TIM_PeriodElapsedCallback|hfdcan1|htim5'
  if ($undefined) { $undefined; exit 1 }
  & 'C:\clionToolchains\gcc-arm-none-eabi-10.3-2021.10\bin\arm-none-eabi-nm.exe' -C 'build/Debug/Robocon-Elian.elf' |
      Select-String -Pattern 'Task_Init|Task_Loop|HAL_TIM_PeriodElapsedCallback|Class_Key'
  git diff --stat
  ```

  Expected: no relevant undefined symbols; the four task/key symbols are present; the diff contains only the planned production/test files plus the user's preserved CubeMX changes.

- [ ] **Step 9: Commit activation and CubeMX configuration together**

  ```powershell
  git add CMakeLists.txt Core/Inc/main.h Core/Src/gpio.c Core/Src/main.c Robocon-Elian.ioc
  git commit -m "build: activate DM-J8006 key control"
  ```

- [ ] **Step 10: Perform the user-owned bench test**

  With the output shaft unloaded and an immediate power cutoff available, flash the firmware and verify: startup holds zero; KEY1 produces `1`, then `2 rad/s`; KEY2 returns to `1 rad/s`; both keys stop the motor; long holds trigger once; repeated presses clamp at `+5/-5 rad/s`. Treat `1 rad/s` as approximately `9.55 rpm` at the geared output shaft.

  Expected: all physical behaviors match the spec. If hardware behavior differs, stop the motor and diagnose before changing control parameters or safety behavior.
