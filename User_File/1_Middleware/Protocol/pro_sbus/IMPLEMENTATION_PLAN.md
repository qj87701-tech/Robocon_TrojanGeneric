# NRF24L01 Remote Receiver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在裸机前台循环中接收31字节NRF24L01遥控数据，解析16路SBUS和串口屏自定义数据，并在100 ms失联后恢复安全值。

**Architecture:** `Class_NRF24L01`继续只负责无线收包；新增的`Class_SBUS_Receive`负责完整帧校验、协议解包和在线状态。任务层只连接两个模块，不把遥控数据接入达妙电机。

**Tech Stack:** STM32H723 HAL、C++17、CMake、NRF24L01软件SPI、裸机前台调度。

**Spec:** `User_File/1_Middleware/Protocol/pro_sbus/README.md`

## Global Constraints

- 代码风格沿用当前工程和中科大RoboWalker的直接写法、分区注释与命名习惯。
- 复用R1工程的16路SBUS位解包公式，只修改实际帧标志和接收数据结构。
- 不使用RTOS、队列、SUBS或动态内存。
- 不直接修改或驱动现有达妙电机控制量。
- 不创建新的测试目录；协议测试放入现有`tests/host`。
- 本次不执行Git分支、提交或远程仓库操作。

## Review Focus

- 跨字节的16路11 bit通道必须全部解包正确，由Task 1的全通道测试覆盖。
- 任一帧标志错误必须保持上一帧数据且不能刷新在线状态，由Task 1的坏帧测试覆盖。
- `data[25..27]`中的模式、电机使能和16个按钮位序必须正确，由Task 1的自定义数据测试覆盖。
- 100 ms之前不能提前离线，第10次10 ms回调必须复位，由Task 1的边界测试覆盖。
- `data[29..30]`变化不能导致有效帧被拒绝，由Task 1的保留字节测试覆盖。

---

### Task 1: SBUS接收协议模块

**Files:**
- Create: `User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.h`
- Create: `User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp`
- Create: `tests/host/test_pro_sbus.cpp`

**Interfaces:**
- Consumes: 至少31字节的`const uint8_t *data`和每10 ms一次的存活回调。
- Produces: `Struct_SBUS_Data SBUS_Data`、`void Init()`、`uint8_t Data_Process(const uint8_t *data)`、`void TIM_10ms_PeriodElapsedCallback()`，以及供后续模块使用的`extern Class_SBUS_Receive SBUS_Receive`声明。

- [x] **Step 1: 编写协议失败测试**

在`test_pro_sbus.cpp`中覆盖：初始化安全值；一帧包含16个不同通道值的有效数据；模式3、电机使能1和按钮1/8/9/16；四个校验位置分别错误；保留字节非零；第9和第10次10 ms失联回调。

关键断言：

```cpp
assert(receiver.SBUS_Data.Channel[0] == expected_channels[0]);
assert(receiver.SBUS_Data.Channel[15] == expected_channels[15]);
assert(receiver.SBUS_Data.Motor_Enable == 1U);
assert(receiver.SBUS_Data.Function[0] == 1U);
assert(receiver.SBUS_Data.Function[15] == 1U);
assert(receiver.SBUS_Data.Online == 1U);
```

- [x] **Step 2: 运行测试并确认尚未实现**

Run:

```powershell
New-Item -ItemType Directory -Force build/host
g++ -std=c++17 -ICore/Inc -Itests/host/stubs -IUser_File tests/host/test_pro_sbus.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp -o build/host/test_pro_sbus.exe
```

Expected: FAIL，提示`pro_sbus.h`或对应接口尚不存在。

- [x] **Step 3: 实现协议类和安全复位**

在`pro_sbus.h/.cpp`实现计划中的四个接口。校验固定值为`0x0F`、`0xFF`、`0xAF`、`0x88`；有效帧解析全部16路通道；第10次10 ms回调执行失联复位；所有通道安全值为`1000`。

- [x] **Step 4: 运行协议测试**

Run:

```powershell
g++ -std=c++17 -ICore/Inc -Itests/host/stubs -IUser_File tests/host/test_pro_sbus.cpp User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp -o build/host/test_pro_sbus.exe
build/host/test_pro_sbus.exe
```

Expected: 编译成功，程序返回码为0且无断言失败。

### Task 2: 裸机任务和CMake接入

**Files:**
- Modify: `User_File/4_Task/tsk_config_and_callback.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1的`Class_SBUS_Receive`接口和现有`Class_NRF24L01::NRF24L01_Receive()`。
- Produces: 初始化完成的全局NRF/SBUS对象、IRQ低电平收包路径和10 ms失联调度。

- [x] **Step 1: 接入全局对象和初始化**

包含`pro_sbus.h`与`dvc_nrf24l01.h`，创建内部使用的`Class_NRF24L01 NRF24L01`，并定义`pro_sbus.h`已声明的`Class_SBUS_Receive SBUS_Receive`；在`Task_Init()`中先初始化协议安全值，再初始化NRF24L01。

- [x] **Step 2: 接入接收和存活回调**

在`Task_Loop()`的周期任务之前检查`NRF_IRQ`低电平；仅当`NRF24L01_Receive()`返回1时调用`SBUS_Receive.Data_Process()`。在`Task10ms_Callback()`中调用`SBUS_Receive.TIM_10ms_PeriodElapsedCallback()`。

- [x] **Step 3: 加入固件构建源文件**

在`CMakeLists.txt`的`target_sources`中加入`User_File/1_Middleware/Protocol/pro_sbus/pro_sbus.cpp`，不改动链接库和CubeMX生成文件。

- [x] **Step 4: 构建完整固件**

Run:

```powershell
cube-cmake --build --preset Debug --target Robocon-Elian
```

Expected: `Robocon-Elian.elf`成功生成，无编译或链接错误。

- [x] **Step 5: 最终检查**

重新运行`build/host/test_pro_sbus.exe`，并检查差异只包含规格、计划、协议模块、协议测试、任务接入和CMake源文件列表；不烧录硬件，不宣称完成实机通信验证。

