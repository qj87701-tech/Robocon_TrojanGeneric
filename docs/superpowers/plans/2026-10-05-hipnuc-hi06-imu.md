# HiPNUC HI06 IMU Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Robocon-Elian 裸机工程中通过 UART4 接收并解析 HI06 的 HI91 二进制数据，封装为独立设备模块，并由 USART10 每 500 ms 打印完整 IMU 状态。

**Architecture:** UART4 的 Receive-to-Idle DMA 回调只把字节复制到 HI06 的 1024 字节固定环形缓冲区，10 ms 前台任务调用官方 HiPNUC 解码器并维护最新 SI 数据、在线状态和错误统计。调试打印位于任务层，不进入设备类，也不接入底盘、遥控或电机控制。

**Tech Stack:** STM32H723 HAL、C11 官方 HiPNUC 解码器、C++17 设备封装、现有 `drv_uart`、裸机前台调度、MinGW 主机测试、STM32 Cube CMake。

**Spec:** `docs/superpowers/specs/2026-10-05-hipnuc-hi06-imu-design.md`

## Global Constraints

- UART4 固定使用 `115200、8N1`，设备输出固定为 HI91 二进制。
- 官方 `hipnuc_dec.*`、`hipnuc_sample.*` 保持原结构和行为，不混入 HAL 或业务代码。
- 设备层保存 SI 单位：姿态 rad、角速度 rad/s、加速度 m/s²、温度 °C；只有 USART10 显示时把姿态转为度。
- 环形缓冲区固定 1024 字节，离线超时固定 500 ms，调试打印周期固定 500 ms。
- 不引入 RTOS、队列、SUBS、动态内存、NMEA、HI81/HI83 业务数据或新的 DSP/数学库。
- 不接入底盘、导航、遥控或电机控制；保留但暂停遥控和电机的周期打印。
- 遵循 USTC RoboWalker 文件头、分区、命名、直接控制流和中文注释风格。
- 用户当前不使用 Git；计划执行期间不创建分支、不提交、不推送，各任务以测试结果作为检查点。

## Review Focus

- UART 一次回调恰好填到环形缓冲区边界或超过容量时，不覆盖未处理数据；溢出只按一次受影响事件计数，并能在下一帧重新同步。由 Task 2 的边界和恢复测试固定。
- `HAL_GetTick()` 在 `uint32_t` 回绕附近跨过 500 ms 时，在线超时仍使用无符号差值正确判断。由 Task 2 的回绕测试固定。
- 收到 CRC 正确的 HI81/HI83 时，官方解码器可以识别，但 HI06 设备数据和在线时间戳不得更新。由 Task 2 的非 HI91 测试固定。
- HI91 被任意拆分到多次 UART 空闲回调或多帧粘在一次回调中时，每帧只能计数一次，对外保留最后一帧。由 Task 1 的分段和连续帧测试固定。
- USART10 DMA 忙或发送失败时只跳过本次调试行，不阻塞任务，也不改变 IMU 状态。由 Task 3 的发送失败测试固定。

---

### Task 1: HI06 数据模型与正常 HI91 解码路径

**Files:**
- Create: `User_File/2_Device/IMU/HI06/HI06_Define.h`
- Create: `User_File/2_Device/IMU/HI06/dvc_imu_hi06.h`
- Create: `User_File/2_Device/IMU/HI06/dvc_imu_hi06.cpp`
- Create: `tests/host/test_imu_hi06.cpp`
- Reference unchanged: `User_File/1_Middleware/Hipnuc/hipnuc_dec.c`
- Reference unchanged: `User_File/1_Middleware/Hipnuc/hipnuc_dec.h`
- Reference unchanged: `User_File/1_Middleware/Hipnuc/hipnuc_sample.c`
- Reference unchanged: `User_File/1_Middleware/Hipnuc/hipnuc_sample.h`

**Interfaces:**
- Consumes: `hipnuc_input(hipnuc_raw_t *, uint8_t)`, `hipnuc_sample_from_raw(const hipnuc_raw_t *, hipnuc_sample_t *)`, `HAL_GetTick()`.
- Produces: `Class_HI06 HI06`; `Init()`; `UART_RxCpltCallback(uint8_t *, uint16_t)`; `TIM_10ms_PeriodElapsedCallback()`; `Get_Status()`; `Get_Data()`.
- Produces types: `Enum_HI06_Status { HI06_Status_DISABLE, HI06_Status_OFFLINE, HI06_Status_ONLINE }` and `Struct_HI06_Data` with the exact fields from spec section 7.

- [ ] **Step 1: Write the successful-frame tests**

In `tests/host/test_imu_hi06.cpp`, add a `Build_HI91_Frame(const hi91_t &Packet)` helper that writes `0x5A 0xA5`, little-endian payload length, the packed 76-byte `hi91_t`, and the CRC returned by `hipnuc_crc16()`.

Add tests with exact assertions:

- `Test_HI91_Split_Across_Callbacks()` feeds one known frame in two callback calls, runs the 10 ms processor after each call, and asserts no frame before completion then one online frame afterward.
- Use a known packet with temperature `27`, acceleration `{1.0, -0.5, 0.25}` G, angular velocity `{180, -90, 45}` deg/s, and attitude `{10, -20, 30}` deg; assert the exposed data is `{9.8, -4.9, 2.45}` m/s², `{pi, -pi/2, pi/4}` rad/s, and `{10, -20, 30}` converted to rad within float tolerance.
- `Test_Multiple_HI91_Frames_Keep_The_Latest()` feeds two concatenated frames in one callback and asserts `Frame_Count == 2` and the second frame's temperature and yaw are exposed.
- `Test_Fields_Follow_Official_Valid_Mask()` asserts status, acceleration, gyro, roll/pitch, yaw and temperature validity bits are present for the known HI91 sample.

- [ ] **Step 2: Compile and confirm the test fails before the device files exist**

Run:

```powershell
New-Item -ItemType Directory -Force build/host | Out-Null
& 'C:\clionToolchains\mingw64\bin\gcc.exe' -std=c11 -Wall -Wextra -Werror -IUser_File/1_Middleware/Hipnuc -c User_File/1_Middleware/Hipnuc/hipnuc_dec.c -o build/host/hipnuc_dec.o
& 'C:\clionToolchains\mingw64\bin\gcc.exe' -std=c11 -Wall -Wextra -Werror -IUser_File/1_Middleware/Hipnuc -c User_File/1_Middleware/Hipnuc/hipnuc_sample.c -o build/host/hipnuc_sample.o
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -IUser_File tests/host/test_imu_hi06.cpp User_File/2_Device/IMU/HI06/dvc_imu_hi06.cpp build/host/hipnuc_dec.o build/host/hipnuc_sample.o -o build/host/test_imu_hi06.exe
```

Expected: FAIL because `HI06_Define.h` and `dvc_imu_hi06.*` have not been implemented.

- [ ] **Step 3: Define the public data and class interface**

Create `HI06_Define.h` with:

- `HI06_RX_BUFFER_SIZE = 1024U`.
- `HI06_OFFLINE_TIMEOUT = 500U` ms.
- the three-state `Enum_HI06_Status`.
- `Struct_HI06_Data` fields: `Valid`, `Main_Status`, `Roll`, `Pitch`, `Yaw`, `Accel_X/Y/Z`, `Gyro_X/Y/Z`, `Temperature`, `Last_Rx_Tick`, `Rx_Byte_Count`, `Frame_Count`, `CRC_Error_Count`, `Invalid_Frame_Count`, `Buffer_Overflow_Count`.

Create `dvc_imu_hi06.h` with the public signatures in the Interfaces block and private storage for one `hipnuc_raw_t`, one `hipnuc_sample_t`, the 1024-byte ring, volatile producer/consumer indices and overflow state.

- [ ] **Step 4: Implement the minimum normal receive/decode path**

In `dvc_imu_hi06.cpp`:

- zero all state in `Init()` and enter `HI06_Status_OFFLINE`.
- copy callback bytes into the single-producer ring without decoding.
- drain all currently available bytes in `TIM_10ms_PeriodElapsedCallback()`.
- after `hipnuc_input()` returns success, call `hipnuc_sample_from_raw()` and accept only `HIPNUC_SOURCE_HI91`.
- copy only fields indicated by the sample validity mask, update `Last_Rx_Tick`, increment the HI91 `Frame_Count`, and enter `HI06_Status_ONLINE`.
- publish the global `Class_HI06 HI06` object.

- [ ] **Step 5: Run the normal-path test**

Repeat the compile commands from Step 2, then run:

```powershell
& .\build\host\test_imu_hi06.exe
```

Expected: PASS with exit code 0.

- [ ] **Step 6: Checkpoint review without Git**

Review only Task 1 files for official-unit conversion, no HAL work inside official C files, no dynamic allocation, and no protocol parsing inside the UART callback.

### Task 2: Overflow recovery, invalid input and online timeout

**Files:**
- Modify: `tests/host/test_imu_hi06.cpp`
- Modify: `tests/host/stubs/stm32h7xx_hal.h`
- Modify: `User_File/2_Device/IMU/HI06/dvc_imu_hi06.cpp`
- Modify: `User_File/2_Device/IMU/HI06/dvc_imu_hi06.h`

**Interfaces:**
- Consumes: Task 1's `Class_HI06` interface and `Struct_HI06_Data`.
- Produces: stable overflow recovery, error counters, 500 ms offline timeout and tick-wrap-safe behavior through the same public interface.

- [ ] **Step 1: Add failing error and boundary tests**

Extend `test_imu_hi06.cpp` with:

- `Test_Bad_CRC_Does_Not_Update_Data()` corrupts one payload byte after CRC generation; assert status stays offline, `Frame_Count == 0`, CRC count increases, and measurement fields remain unchanged.
- `Test_HI81_Is_Ignored_By_HI06()` builds a valid HI81 envelope; assert no HI06 frame, timestamp or online update.
- `Test_Ring_Overflow_Drops_Affected_Span_And_Recovers()` supplies more than one ring capacity without foreground processing, asserts one overflow event, then supplies a valid HI91 frame after recovery and asserts it becomes online with correct data.
- `Test_Exactly_One_Ring_Boundary_Does_Not_Write_Past_Storage()` exercises the last usable ring slot and checks the canary values surrounding the class object remain unchanged.
- `Test_Offline_At_500ms_And_Tick_Wrap()` asserts online at elapsed 499 ms, offline at 500 ms, and repeats across `UINT32_MAX` wrap using unsigned subtraction.

Add host stubs for the short critical section primitives used only during overflow recovery: `__get_PRIMASK`, `__disable_irq`, `__set_PRIMASK`, and `__DMB`.

- [ ] **Step 2: Run the tests and confirm the new cases fail**

Rebuild and run `build/host/test_imu_hi06.exe` using Task 1 commands.

Expected: FAIL in CRC/error counts, overflow recovery or timeout behavior not yet implemented.

- [ ] **Step 3: Implement statistics, recovery and timeout**

- Mirror `hipnuc_raw_t.crc_error_count` and `invalid_count` into the public data after parsing.
- Count one buffer overflow for each callback that cannot enqueue its complete input, set a recovery flag, and do not overwrite unread bytes.
- At the next 10 ms foreground call, use a short interrupt-masked snapshot to discard the affected unread span, clear only the partial decoder state while preserving cumulative decoder counters, then accept new bytes normally.
- Calculate offline state with `(uint32_t)(HAL_GetTick() - Last_Rx_Tick) >= HI06_OFFLINE_TIMEOUT` so tick wrap is safe.
- Preserve last valid measurements when moving offline.

- [ ] **Step 4: Run all HI06 unit tests**

Rebuild and run:

```powershell
& .\build\host\test_imu_hi06.exe
```

Expected: PASS with exit code 0.

- [ ] **Step 5: Checkpoint review without Git**

Confirm the ISR-side path performs only bounded byte copies/index publication, overflow cannot overwrite unread data, and a failed frame never refreshes online time.

### Task 3: UART4, 10 ms processing and USART10 500 ms printing

**Files:**
- Create: `tests/host/stubs/2_Device/IMU/HI06/dvc_imu_hi06.h`
- Modify: `tests/host/stubs/stm32h7xx_hal.h`
- Modify: `tests/host/stubs/usart.h`
- Modify: `tests/host/test_drv_uart.cpp`
- Modify: `tests/host/test_task_control.cpp`
- Modify: `User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp`

**Interfaces:**
- Consumes: `Class_HI06 HI06`, `HI06.Init()`, `HI06.UART_RxCpltCallback()`, `HI06.TIM_10ms_PeriodElapsedCallback()`, `HI06.Get_Status()`, `HI06.Get_Data()`.
- Produces: `UART4_Callback(uint8_t *, uint16_t)`, `UART10_Print_IMU_Data()`, 10 ms parsing schedule and independent 500 ms print schedule.

- [ ] **Step 1: Add UART4 driver and task integration tests**

Update `test_drv_uart.cpp` to assert `UART_Init(&UART4_Handler, callback)` starts Receive-to-Idle DMA on `UART4_Manage_Object`, swaps its double buffer on RX event, timestamps it and forwards the exact pointer/length.

Create the HI06 task stub using the real `HI06_Define.h`, with counters for `Init`, UART callback and 10 ms processing, plus configurable status/data returned by getters.

Extend the HAL host status enum with `HAL_BUSY` so the USART10 non-blocking failure path can be exercised without changing production HAL definitions.

Update `test_task_control.cpp` with exact behavior tests:

- `Test_Initialization_Contract()` additionally asserts HI06 initialized once, UART4 DMA started once, and UART4 callback registered before `init_finished` becomes true.
- `Test_UART4_Callback_Forwards_To_HI06()` invokes the registered callback and asserts exact buffer pointer and length.
- `Test_HI06_Processes_Every_10ms()` advances the scheduler and asserts one HI06 processing call per 10 ms period, including when motors are disabled.
- `Test_IMU_Data_Prints_Every_500ms()` asserts no output at 499 ms, one IMU line at 500 ms, correct units/values/counters, and no `RC` or `DM` text.
- `Test_IMU_Print_Converts_Only_Attitude_To_Degrees()` supplies known radian values and asserts acceleration and gyro remain SI while RPY is shown in degrees.
- `Test_IMU_Print_Skips_When_USART10_DMA_Is_Busy()` returns `HAL_BUSY` from the transmit stub and asserts the scheduler continues and HI06 processing counts remain correct.

- [ ] **Step 2: Compile and confirm integration tests fail**

Run the UART driver test build:

```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -IUser_File tests/host/test_drv_uart.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp -o build/host/test_drv_uart.exe
& .\build\host\test_drv_uart.exe
```

Run the task-control build:

```powershell
& 'C:\clionToolchains\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Werror -Itests/host/stubs -IUser_File tests/host/test_task_control.cpp User_File/1_Middleware/Driver/UART/drv_uart.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp User_File/2_Device/BSP/Key/bsp_key.cpp User_File/3_Module/Chassis/Omni/mod_chassis_omni.cpp User_File/4_Task/Config/motor_init.cpp User_File/4_Task/tsk_chassis/tsk_chassis.cpp User_File/4_Task/tsk_config_and_callback/tsk_config_and_callback.cpp User_File/4_Task/tsk_remote/tsk_remote.cpp -o build/host/test_task_control.exe
```

Expected: FAIL because UART4/HI06 initialization, callback routing, 500 ms schedule and IMU print do not yet exist.

- [ ] **Step 3: Connect HI06 to the existing bare-metal task flow**

In `tsk_config_and_callback.cpp`:

- include `2_Device/IMU/HI06/dvc_imu_hi06.h`.
- add `TASK_500MS_PERIOD = 500`, `Task_500ms_Previous_Tick`, `UART4_Callback()`, `Task500ms_Callback()` and `UART10_Print_IMU_Data()`.
- call `HI06.TIM_10ms_PeriodElapsedCallback()` at the beginning of `Task10ms_Callback()` so motor early returns cannot skip IMU parsing.
- in `Task_Init()`, call `HI06.Init()` then `UART_Init(&huart4, UART4_Callback)` while `init_finished` is false.
- initialize the 500 ms scheduler timestamp alongside existing timestamps.
- run IMU printing from the independent 500 ms branch in `Task_Loop()`.
- keep `UART10_Print_Remote_Data()` and `UART10_Print_Motor_Data()`, but comment out their calls from periodic callbacks.
- use the existing manual two-decimal float conversion helper; multiply only RPY by `57.2957795f` before formatting.
- call `UART_Transmit_Data()` once per line and ignore a non-`HAL_OK` return without retrying or blocking.

- [ ] **Step 4: Run UART and task integration tests**

Rebuild both tests with Step 2 commands, then run:

```powershell
& .\build\host\test_drv_uart.exe
& .\build\host\test_task_control.exe
```

Expected: both PASS with exit code 0.

- [ ] **Step 5: Checkpoint review without Git**

Confirm UART4 is owned by the existing UART driver, only IMU printing is active, no device parsing occurs in interrupt context, and USART10 failures do not affect control flow.

### Task 4: CMake integration and complete verification

**Files:**
- Modify: `CMakeLists.txt`
- Verify unchanged: `cmake/stm32cubemx/CMakeLists.txt`
- Verify: all canonical executables in `build/host`

**Interfaces:**
- Consumes: official C sources and Task 1's C++ device module.
- Produces: `Robocon-Elian.elf` containing the HI06 module and a complete verification record.

- [ ] **Step 1: Add the three required source files to the root firmware target**

Add only these entries to `target_sources(${CMAKE_PROJECT_NAME} PRIVATE ...)`:

```text
User_File/1_Middleware/Hipnuc/hipnuc_dec.c
User_File/1_Middleware/Hipnuc/hipnuc_sample.c
User_File/2_Device/IMU/HI06/dvc_imu_hi06.cpp
```

The two headers are found through the existing `${CMAKE_CURRENT_SOURCE_DIR}/User_File` include path and are not listed as compiled sources. Do not add or change linked libraries.

- [ ] **Step 2: Build and link the Debug firmware**

Run:

```powershell
& 'C:\Users\jiang\AppData\Local\stm32cube\bundles\cmake\4.0.1+st.3\bin\cmake.exe' --build --preset Debug --target Robocon-Elian
```

Expected: exit code 0 and updated `build/Debug/Robocon-Elian.elf`; no undefined HiPNUC, UART4 or math symbols.

- [ ] **Step 3: Run all canonical host tests**

Run the newly rebuilt HI06/UART/task tests first, then all other current canonical host executables:

```powershell
$tests = @(
    'test_imu_hi06.exe',
    'test_drv_uart.exe',
    'test_task_control.exe',
    'test_bsp_key.exe',
    'test_chassis_omni.exe',
    'test_chassis_swerve.exe',
    'test_chassis_task.exe',
    'test_motor_dm_enable.exe',
    'test_motor_init.exe',
    'test_pro_sbus.exe',
    'test_remote_receive_task.exe'
)
foreach ($test in $tests) {
    & (Join-Path 'build/host' $test)
    if ($LASTEXITCODE -ne 0) { throw "$test failed with exit code $LASTEXITCODE" }
}
```

Expected: every executable exits with code 0.

- [ ] **Step 4: Inspect the final change boundary**

Run:

```powershell
git diff -- CMakeLists.txt User_File/2_Device/IMU/HI06 User_File/4_Task/tsk_config_and_callback tests/host
```

Confirm no generated CubeMX source was hand-edited, official HiPNUC C files remain unchanged, no RTOS/SUBS code was introduced, and remote/motor logic was not altered beyond disabling their periodic debug-print calls.

- [ ] **Step 5: Prepare the physical-board handoff**

Report software verification separately from hardware verification. Ask the user to connect HI06 TX to PA1/UART4_RX with common ground, flash the ELF, check one USART10 line every 500 ms, rotate each axis, then disconnect/reconnect the IMU to verify 500 ms offline detection and automatic recovery.

- [ ] **Step 6: Final checkpoint without Git**

Do not commit or push. Summarize exact files changed, tests/build results and the remaining physical validation steps.
