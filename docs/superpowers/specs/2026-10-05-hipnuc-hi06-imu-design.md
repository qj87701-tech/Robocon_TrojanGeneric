# HiPNUC HI06 IMU 接收模块设计

## 1. 目标

在 `Robocon-Elian` 裸机工程中接入 HiPNUC HI06 IMU，使用 UART4 接收设备输出的 `115200、8N1、HI91 二进制`数据，使用官方解码器完成协议解析，并通过 USART10 每 500 ms 打印一次最新测量数据。

本阶段仅完成独立的 IMU 接收、解析、状态维护和调试输出，不接入底盘、导航、姿态控制或其他业务模块。

## 2. 已有条件

- 主控为 STM32H723ZGT6，工程使用 CubeMX CMake、VS Code 和裸机前台调度。
- CubeMX 已配置 UART4：115200、8N1、收发模式、RX/TX DMA 和 UART4 全局中断。
- UART4 引脚为 PA0/TX、PA1/RX；HI06 的 TX 应连接 PA1，双方共地。
- USART10 已用于调试输出，波特率为 115200。
- 工程已有中科大风格的 `drv_uart`，支持 UART4 的 Receive-to-Idle DMA 双缓冲接收和回调。
- 官方 `hipnuc_dec.c/.h` 与 `hipnuc_sample.c/.h` 已放入 `User_File/1_Middleware/Hipnuc`。
- HI06 当前输出 HI91 二进制报文。

## 3. 设计边界

### 3.1 本阶段包含

- UART4 数据接收与 HI06 设备对象初始化。
- 固定容量环形缓冲区。
- HI91 协议解码、CRC/格式错误统计。
- 最新一帧 IMU 数据、有效字段、在线状态和接收统计维护。
- USART10 每 500 ms 打印 IMU 数据。
- 主机测试、全部现有测试和固件 Debug 构建验证。

### 3.2 本阶段不包含

- 不修改 HI06 的输出配置，不在启动时发送配置命令。
- 不解析 NMEA、HI81、HI83、CAN 或 CAN FD 数据。
- 不进行安装方向补偿、坐标系旋转、零偏标定或姿态二次融合。
- 不把 IMU 数据接入 Omni、Swerve、电机或遥控模块。
- 不引入 RTOS、队列、SUBS、动态内存或新的数学/DSP 静态库。

## 4. 文件组织

```text
User_File/
├── 1_Middleware/
│   └── Hipnuc/
│       ├── hipnuc_dec.c
│       ├── hipnuc_dec.h
│       ├── hipnuc_sample.c
│       └── hipnuc_sample.h
└── 2_Device/
    └── IMU/
        └── HI06/
            ├── HI06_Define.h
            ├── dvc_imu_hi06.cpp
            └── dvc_imu_hi06.h
```

各文件职责：

- `hipnuc_dec.*`、`hipnuc_sample.*`：保留官方实现，只负责二进制协议解析、CRC 校验和 SI 单位转换。
- `HI06_Define.h`：定义 HI06 状态、缓冲区长度、在线超时和对外数据结构。
- `dvc_imu_hi06.*`：负责字节缓存、前台解码、最新数据、在线状态和统计信息。
- `tsk_config_and_callback.cpp`：负责 UART4 回调转发、周期调用和 USART10 调试打印。

根目录 `CMakeLists.txt` 显式加入四个官方 C 源文件和 HI06 C++ 设备源文件；不修改 CubeMX 可重新生成的 `cmake/stm32cubemx/CMakeLists.txt`。

## 5. 数据流

```text
HI06 TX
  -> UART4 RX DMA + IDLE
  -> drv_uart 双缓冲
  -> UART4_Callback(Buffer, Length)
  -> HI06.UART_RxCpltCallback()
  -> 1024 字节固定环形缓冲区
  -> HI06.TIM_10ms_PeriodElapsedCallback()
  -> hipnuc_input() / hipnuc_sample_from_raw()
  -> Struct_HI06_Data
  -> UART10_Print_IMU_Data()，每 500 ms 调用一次
```

UART4 回调只复制字节，不执行 CRC、浮点转换和字符串格式化。协议解析在裸机前台 10 ms 周期中完成，避免扩大 UART 中断执行时间。

## 6. HI06 设备接口

设备对象采用与当前工程一致的直接类接口：

```cpp
class Class_HI06
{
public:
    void Init();
    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void TIM_10ms_PeriodElapsedCallback();

    Enum_HI06_Status Get_Status();
    const Struct_HI06_Data &Get_Data();
};

extern Class_HI06 HI06;
```

- `Init()` 清空协议状态、环形缓冲区、数据和统计信息。
- `UART_RxCpltCallback()` 作为单生产者，把 UART4 回调数据复制到固定环形缓冲区。
- `TIM_10ms_PeriodElapsedCallback()` 作为单消费者，取出当前全部待处理字节并执行协议解码。
- `Get_Status()` 返回初始化、离线或在线状态。
- `Get_Data()` 返回只读的最新数据和统计信息，供以后其他模块读取；调试打印不进入设备类。

## 7. 对外数据与单位

`Struct_HI06_Data` 保存：

- `Valid`：官方 `HIPNUC_VALID_*` 有效字段位图。
- `Main_Status`：HI91 原始主状态位。
- `Roll`、`Pitch`、`Yaw`：rad。
- `Accel_X/Y/Z`：m/s²，保留重力分量。
- `Gyro_X/Y/Z`：rad/s。
- `Temperature`：°C。
- `Last_Rx_Tick`：最后一帧有效 HI91 的 HAL 毫秒时间戳。
- `Rx_Byte_Count`：进入 HI06 环形缓冲区的字节总数。
- `Frame_Count`：成功解析的 HI91 帧数。
- `CRC_Error_Count`：官方解码器报告的 CRC 错误数。
- `Invalid_Frame_Count`：官方解码器报告的格式错误数。
- `Buffer_Overflow_Count`：环形缓冲区溢出次数。

设备层始终保留官方 SI 单位。USART10 打印时只将 Roll、Pitch、Yaw 乘以 `180 / pi` 转为度，避免以后控制代码混用角度制和弧度制。

每个成功解析的 HI91 样本独立更新数据，不把无效字段与上一帧拼接。读取字段时以 `Valid` 位图为准。

## 8. 缓冲区与并发规则

- 环形缓冲区固定为 1024 字节，不使用堆内存。
- UART4 回调是唯一生产者，10 ms 前台处理是唯一消费者。
- 生产者只更新写索引，消费者只更新读索引；跨上下文发布索引时使用必要的内存屏障。
- 如果写入时发现缓冲区已满，设置溢出标志并增加 `Buffer_Overflow_Count`，不覆盖尚未处理的数据。
- 前台发现溢出标志后，在短临界区内丢弃该段未处理数据，随后清除未完成协议帧，但保留累计统计值，让下一帧帧头重新同步。
- 每次 10 ms 回调处理当时已有的全部字节；一次收到多帧时逐帧计数，对外保留最新一帧 HI91 数据。

在 115200、8N1 下，1024 字节约对应 88 ms 线速数据，明显大于 10 ms 解析周期，但溢出统计仍用于发现中断阻塞或前台任务延迟。

## 9. 在线状态

- 初始化完成但尚未收到有效 HI91 时为离线。
- CRC 和格式均正确、且来源为 `HIPNUC_SOURCE_HI91` 的帧才更新时间戳并标记在线。
- 连续 500 ms 未收到有效 HI91 时标记离线。
- 离线后保留最后一帧数值和累计统计，便于查看断线前状态；业务模块必须先检查在线状态和有效位后再使用数据。
- CRC 错误、格式错误和缓冲区溢出不会更新在线时间戳。

## 10. 裸机任务接入

在 `tsk_config_and_callback.cpp` 中增加：

- `UART4_Callback(uint8_t *Buffer, uint16_t Length)`：转发给 `HI06.UART_RxCpltCallback()`。
- `Task500ms_Callback()`：只负责调用 `UART10_Print_IMU_Data()`。
- `UART10_Print_IMU_Data()`：格式化并发送最新数据。
- `Task_500ms_Previous_Tick` 和 `TASK_500MS_PERIOD`。

`Task_Init()` 的相关顺序：

1. 初始化时间戳和公共状态。
2. 调用 `HI06.Init()`。
3. 调用 `UART_Init(&huart4, UART4_Callback)` 启动 UART4 Receive-to-Idle DMA。
4. 保留 UART7、UART9、USART10 的现有初始化。
5. 继续初始化遥控器、电机、底盘、按键和 CAN。
6. 全部初始化完成后设置 `init_finished = true`。

`Task10ms_Callback()` 调用 `HI06.TIM_10ms_PeriodElapsedCallback()`。`Task_Loop()` 增加独立 500 ms 调度，不借用现有 1 s 周期。

现有 `UART10_Print_Remote_Data()` 和 `UART10_Print_Motor_Data()` 函数保留；本阶段注释它们的周期调用，只启用 IMU 打印。

## 11. USART10 输出

每 500 ms 输出一行 ASCII 文本，内容包括：

```text
IMU online=1 valid=... rpy_deg=...,...,... acc=...,...,... gyr=...,...,... temp=... frame=... crc=... invalid=... overflow=...
```

- `rpy_deg` 单位为度。
- `acc` 单位为 m/s²。
- `gyr` 单位为 rad/s。
- `temp` 单位为 °C。
- 浮点数继续使用工程现有的手动字符串转换方式，不依赖浮点 `printf` 配置。
- USART10 使用现有 DMA 发送函数；500 ms 周期保证上一行在正常 115200 波特率下已经发送完成。

## 12. 错误处理

- UART 硬件错误继续由 `drv_uart` 的 `HAL_UART_ErrorCallback()` 重新启动接收。
- 不完整帧保留在官方解码器状态中，等待后续 UART 数据补齐。
- CRC 或格式错误只增加统计并继续搜索后续帧，不覆盖最新有效数据。
- 环形缓冲区溢出时丢弃受影响的数据段并重置未完成帧，避免把断裂字节组合成伪帧。
- USART10 正忙或 DMA 发送失败时跳过本次调试输出，不阻塞控制循环，也不影响 IMU 接收与解析。

## 13. 测试与验收

### 13.1 主机测试

- 一帧 HI91 分多次进入 UART 回调，最终只能产生一帧有效数据。
- 多帧连续进入，帧计数正确且对外保留最后一帧。
- CRC 错误帧不更新数据和在线时间戳，错误计数增加。
- 非 HI91 的受支持报文不更新 HI06 数据。
- 缓冲区溢出后能清除残帧，并能在下一帧重新同步。
- 收到有效帧后为在线，超过 500 ms 无有效帧后变为离线。
- 姿态、加速度、角速度、温度和有效位保持官方转换结果。

测试桩只位于 `tests/host`，不加入 STM32 固件目标。

### 13.2 工程验证

- 执行 HI06 相关主机测试。
- 执行全部现有主机测试。
- 完整编译、链接 Debug 固件并生成 `Robocon-Elian.elf`。

### 13.3 物理板验收

- 确认 HI06 TX 接 UART4 RX（PA1），并与主控共地；电平必须为 3.3 V TTL。
- USART10 每 500 ms 输出一行 IMU 数据。
- 静止时加速度模长应接近重力加速度，角速度接近零。
- 分别绕三个轴转动，姿态角与对应角速度轴应有合理变化。
- 断开 HI06 后 500 ms 内在线状态变为 0；恢复连接后可自动重新同步。

固件构建和主机测试通过不等同于物理板验证通过，最终结果必须以实物接线和传感器输出为准。
