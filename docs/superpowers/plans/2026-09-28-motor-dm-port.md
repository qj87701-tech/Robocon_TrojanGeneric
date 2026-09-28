# Motor_DM Minimal Dependency Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Copy the USTC Motor_DM module and its minimal dependency closure into `Robocon-Elian`, integrate it with the CMake firmware target, and prove that it builds without enabling any motor behavior.

**Architecture:** Keep all copied USTC sources byte-identical and isolate target-specific decisions in the top-level `CMakeLists.txt` and `User_File/4_Task/tsk_config_and_callback.*`. The task layer supplies the legacy `init_finished` application state while leaving it `false`; no motor object, CAN registration, or transmit schedule is introduced.

**Tech Stack:** STM32H723ZGT6, STM32CubeMX CMake output, GNU Arm Embedded GCC/G++, C++17, STM32 HAL FDCAN/TIM, CMSIS-DSP static library, PowerShell verification.

**Spec:** `docs/superpowers/specs/2026-09-28-motor-dm-port-design.md`

## Global Constraints

- Copy `Motor_DM`, `drv_can`, `alg_pid`, `alg_basic`, and `sys_timestamp` from `dm02_test` without changing any byte.
- Copy `libarm_cortexM7lfsp_math.a` from the reference project without changing the binary.
- Keep the replica paths marked `-text` in `.gitattributes` so Git never normalizes their bytes.
- Do not overwrite CubeMX-generated `Middlewares/ST/ARM/DSP/Inc/arm_math.h`.
- Modify only the target top-level `CMakeLists.txt`; do not edit `cmake/stm32cubemx/CMakeLists.txt`.
- Keep the target on C++17, Cortex-M7 hard-float flags, and `STM32H723xG_flash.ld`.
- Do not create a DM motor object, set an ID or control parameter, call `CAN_Init()`, or send a CAN frame.
- Keep `init_finished` defined exactly once and fixed at `false` in this phase.
- Do not copy unrelated reference modules or board test behavior.

## Review Focus

- Source drift from line-ending or encoding conversion: Task 1 must prove byte-for-byte equality for every copied source file.
- Missing or incompatible DSP archive: Tasks 1 and 2 must prove archive equality and a successful hard-float Debug link with no unresolved `arm_sin_f32`.
- CubeMX regeneration safety: Task 2 must prove that all user integration lives in the top-level CMake and the generated sub-CMake remains unchanged.
- Unintended runtime activation: Task 2 must prove that `main.c` has no new task/motor call and that no motor object, `CAN_Init()`, or DM send/enable call exists in target-owned code.
- Legacy cross-layer symbols: Task 2 must complete a full link with exactly one `init_finished` definition and no unresolved `SYS_Timestamp` or FDCAN handle references.

---

### Task 1: Reproduce the USTC source closure and DSP archive

**Files:**
- Create: `User_File/1_Middleware/Algorithm/Basic/alg_basic.h`
- Create: `User_File/1_Middleware/Algorithm/Basic/alg_basic.cpp`
- Create: `User_File/1_Middleware/Algorithm/PID/alg_pid.h`
- Create: `User_File/1_Middleware/Algorithm/PID/alg_pid.cpp`
- Create: `User_File/1_Middleware/Driver/CAN/drv_can.h`
- Create: `User_File/1_Middleware/Driver/CAN/drv_can.cpp`
- Create: `User_File/1_Middleware/System/Timestamp/sys_timestamp.h`
- Create: `User_File/1_Middleware/System/Timestamp/sys_timestamp.cpp`
- Create: `User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.h`
- Create: `User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp`
- Create: `Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a`

**Interfaces:**
- Consumes: Byte content from the matching paths under `D:\27Robocon\damiao_mc02_bsp-main\damiao_mc02_bsp-main\dm02_test`.
- Produces: The USTC `Class_Motor_DM_Normal`, `Class_Motor_DM_1_To_4`, `CAN_Init`, `CAN_Transmit_Data`, `Class_PID`, `Basic_Math_*`, `SYS_Timestamp`, and CMSIS-DSP link archive at the same include-relative paths used by the reference project.

- [ ] **Step 1: Run the replica equality check before copying**

  Run:

  ```powershell
  $replicaSourceRoot = 'D:\27Robocon\damiao_mc02_bsp-main\damiao_mc02_bsp-main\dm02_test'
  $replicaTargetRoot = 'D:\27Robocon\Robocon-Elian'
  $replicaPaths = @(
      'User_File\1_Middleware\Algorithm\Basic\alg_basic.h',
      'User_File\1_Middleware\Algorithm\Basic\alg_basic.cpp',
      'User_File\1_Middleware\Algorithm\PID\alg_pid.h',
      'User_File\1_Middleware\Algorithm\PID\alg_pid.cpp',
      'User_File\1_Middleware\Driver\CAN\drv_can.h',
      'User_File\1_Middleware\Driver\CAN\drv_can.cpp',
      'User_File\1_Middleware\System\Timestamp\sys_timestamp.h',
      'User_File\1_Middleware\System\Timestamp\sys_timestamp.cpp',
      'User_File\2_Device\Motor\Motor_DM\dvc_motor_dm.h',
      'User_File\2_Device\Motor\Motor_DM\dvc_motor_dm.cpp',
      'Middlewares\ST\ARM\DSP\Lib\libarm_cortexM7lfsp_math.a'
  )
  $replicaFailures = foreach ($replicaPath in $replicaPaths) {
      $replicaSource = Join-Path $replicaSourceRoot $replicaPath
      $replicaTarget = Join-Path $replicaTargetRoot $replicaPath
      if (!(Test-Path -LiteralPath $replicaTarget)) {
          "MISSING $replicaPath"
      } elseif ((Get-FileHash -LiteralPath $replicaSource -Algorithm SHA256).Hash -ne
                (Get-FileHash -LiteralPath $replicaTarget -Algorithm SHA256).Hash) {
          "MISMATCH $replicaPath"
      }
  }
  if ($replicaFailures) { $replicaFailures; exit 1 }
  "PASS: $($replicaPaths.Count) artifacts match"
  ```

  Expected: FAIL because the target source files and DSP archive do not yet exist.

- [ ] **Step 2: Copy the ten source files and one DSP archive without text conversion**

  Use binary-safe filesystem copies from the reference tree to the exact target paths. Do not open and resave the reference sources, and do not copy `arm_math.h`, CubeMX files, startup files, or link scripts.

- [ ] **Step 3: Run the replica equality check after copying**

  Run the same PowerShell SHA-256 comparison from Step 1.

  Expected: PASS with eleven matching hashes and no missing path.

- [ ] **Step 4: Inspect the copied include closure**

  Run:

  ```powershell
  rg -n "^#include" User_File/1_Middleware/Algorithm/Basic User_File/1_Middleware/Algorithm/PID User_File/1_Middleware/Driver/CAN User_File/1_Middleware/System/Timestamp User_File/2_Device/Motor/Motor_DM
  ```

  Expected: Includes resolve only through `User_File`, CubeMX `Core/Inc`/HAL headers, generated `arm_math.h`, and compiler standard headers; no unrelated USTC device module appears.

- [ ] **Step 5: Commit the exact replica**

  ```powershell
  git add User_File/1_Middleware/Algorithm/Basic User_File/1_Middleware/Algorithm/PID User_File/1_Middleware/Driver/CAN User_File/1_Middleware/System/Timestamp User_File/2_Device/Motor/Motor_DM Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a
  git commit -m "feat: add Motor_DM source dependencies"
  ```

### Task 2: Add the application state boundary and CMake integration

**Files:**
- Create: `User_File/4_Task/tsk_config_and_callback.h`
- Create: `User_File/4_Task/tsk_config_and_callback.cpp`
- Modify: `CMakeLists.txt:10-68`
- Verify unchanged: `cmake/stm32cubemx/CMakeLists.txt`
- Verify unchanged: `Core/Src/main.c`

**Interfaces:**
- Consumes: The copied APIs and DSP archive from Task 1, CubeMX `hfdcan1`/`hfdcan2`/`hfdcan3`, generated `htim5`, and the current `stm32cubemx` CMake target.
- Produces: One C++ definition `bool init_finished = false`, one matching `extern bool init_finished` declaration, and a firmware target that compiles and links all copied `.cpp` files under C++17.

- [ ] **Step 1: Run the integration contract check before editing**

  Run:

  ```powershell
  $integrationCMake = Get-Content -Raw -LiteralPath 'CMakeLists.txt'
  $integrationRequired = @(
      'CMAKE_CXX_STANDARD 17', 'enable_language(C CXX ASM)',
      'User_File/1_Middleware/Algorithm/Basic/alg_basic.cpp',
      'User_File/1_Middleware/Algorithm/PID/alg_pid.cpp',
      'User_File/1_Middleware/Driver/CAN/drv_can.cpp',
      'User_File/1_Middleware/System/Timestamp/sys_timestamp.cpp',
      'User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp',
      'User_File/4_Task/tsk_config_and_callback.cpp',
      'User_File', 'ARM_MATH_CM7', 'ARM_MATH_MATRIX_CHECK',
      'ARM_MATH_ROUNDING', 'libarm_cortexM7lfsp_math.a'
  )
  $integrationMissing = $integrationRequired | Where-Object { !$integrationCMake.Contains($_) }
  $integrationDefinitionCount = if (Test-Path -LiteralPath 'User_File\4_Task\tsk_config_and_callback.cpp') {
      @(Select-String -Path 'User_File\4_Task\tsk_config_and_callback.cpp' -SimpleMatch 'bool init_finished = false').Count
  } else { 0 }
  if ($integrationMissing -or $integrationDefinitionCount -ne 1) {
      $integrationMissing | ForEach-Object { "MISSING $_" }
      "init_finished definitions: $integrationDefinitionCount"
      exit 1
  }
  'PASS: integration contract present'
  ```

  Expected: FAIL because none of the target integration is present.

- [ ] **Step 2: Create the target-owned task boundary**

  Create `tsk_config_and_callback.h` with an include guard and this C++ interface:

  ```cpp
  extern bool init_finished;
  ```

  Create `tsk_config_and_callback.cpp`, include that header, and define:

  ```cpp
  bool init_finished = false;
  ```

  Do not define `Task_Init()`, `Task_Loop()`, HAL callbacks, or any device object in this phase.

- [ ] **Step 3: Enable C++17 in the top-level CMake**

  In `CMakeLists.txt`, set `CMAKE_CXX_STANDARD` to `17`, `CMAKE_CXX_STANDARD_REQUIRED` to `ON`, and `CMAKE_CXX_EXTENSIONS` to `ON`; change `enable_language(C ASM)` to `enable_language(C CXX ASM)`.

- [ ] **Step 4: Add the six C++ sources explicitly**

  Add these exact target sources to the existing `target_sources(${CMAKE_PROJECT_NAME} PRIVATE ...)` block:

  ```text
  User_File/1_Middleware/Algorithm/Basic/alg_basic.cpp
  User_File/1_Middleware/Algorithm/PID/alg_pid.cpp
  User_File/1_Middleware/Driver/CAN/drv_can.cpp
  User_File/1_Middleware/System/Timestamp/sys_timestamp.cpp
  User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.cpp
  User_File/4_Task/tsk_config_and_callback.cpp
  ```

- [ ] **Step 5: Add include path and DSP definitions**

  Add `${CMAKE_CURRENT_SOURCE_DIR}/User_File` to `target_include_directories`. Add `ARM_MATH_CM7`, `ARM_MATH_MATRIX_CHECK`, and `ARM_MATH_ROUNDING` to the existing target compile definitions.

- [ ] **Step 6: Link the DSP archive using the existing plain CMake signature**

  Add this archive to the existing non-keyword `target_link_libraries(${CMAKE_PROJECT_NAME} ...)` block, keeping the call style compatible with the generated sub-CMake's plain signature:

  ```cmake
  "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a"
  ```

- [ ] **Step 7: Re-run the integration contract check**

  Expected: PASS for all CMake tokens, six explicit source paths, both task-layer files, and exactly one `bool init_finished = false` definition.

- [ ] **Step 8: Configure and build Debug with the CLion toolchain**

  Use CLion's CMake at `D:\CLion\CLion 2026.2.1\bin\cmake\win\x64\bin\cmake.exe`, its Windows Ninja at `D:\CLion\CLion 2026.2.1\bin\ninja\win\x64\ninja.exe`, and the configured Arm GNU toolchain under `D:\DevEnv\DevEnv\GNU-tools-for-STM32\bin`. Configure the existing `Debug` preset, then build it.

  Run:

  ```powershell
  & 'D:\CLion\CLion 2026.2.1\bin\cmake\win\x64\bin\cmake.exe' --preset Debug '-DCMAKE_MAKE_PROGRAM=D:/CLion/CLion 2026.2.1/bin/ninja/win/x64/ninja.exe'
  & 'D:\CLion\CLion 2026.2.1\bin\cmake\win\x64\bin\cmake.exe' --build --preset Debug
  $undefinedMatches = arm-none-eabi-nm -u 'build\Debug\Robocon-Elian.elf' |
      Select-String -Pattern 'init_finished|SYS_Timestamp|arm_sin_f32|hfdcan[123]'
  if ($undefinedMatches) { $undefinedMatches; exit 1 }
  'PASS: no unexpected undefined symbols'
  ```

  Expected: configuration and compilation exit 0; the link produces `build/Debug/Robocon-Elian.elf` and `build/Debug/Robocon-Elian.map`; the `nm` check prints no matching undefined symbol.

- [ ] **Step 9: Prove generated files were not used as integration points**

  Run:

  ```powershell
  git diff --exit-code HEAD -- cmake/stm32cubemx/CMakeLists.txt Core/Src/main.c
  ```

  Expected: no diff in either file.

- [ ] **Step 10: Prove there is no motor runtime activation**

  Run:

  ```powershell
  $runtimeMatches = rg -n 'Class_Motor_DM_|CAN_Init\(|CAN_Send_|TIM_Send_PeriodElapsedCallback|TIM_Calculate_PeriodElapsedCallback|init_finished\s*=\s*true' User_File/4_Task Core/Src/main.c
  if ($runtimeMatches) { $runtimeMatches; exit 1 }
  'PASS: no motor runtime activation'
  ```

  Expected: class names occur only in the byte-identical copied module; target-owned task and generated main contain no object, initialization, send call, ID, control value, or `init_finished = true`.

- [ ] **Step 11: Re-run source and archive equality verification**

  Expected: all eleven copied artifacts still match the reference SHA-256 hashes after CMake integration.

- [ ] **Step 12: Commit the integration**

  ```powershell
  git add CMakeLists.txt User_File/4_Task/tsk_config_and_callback.h User_File/4_Task/tsk_config_and_callback.cpp
  git commit -m "build: integrate Motor_DM module"
  ```

