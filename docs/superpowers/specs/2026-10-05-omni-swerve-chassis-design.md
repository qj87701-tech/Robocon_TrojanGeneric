# NRF24L01 遥控 Omni 与 Swerve 底盘设计

## 1. 目标

在 `Robocon-Elian` 裸机工程中，将现有 NRF24L01、SBUS、Omni/Swerve 解算和 DM 电机驱动整理为清楚的遥控底盘控制流程：

- 遥控器摇杆通过 NRF24L01 发送现有“SBUS 主体 + 串口屏自定义数据”数据包；
- `tsk_remote` 负责接收、解析并生成统一底盘指令；
- `tsk_chassis` 负责底盘解算、安全判断和电机控制；
- 当前只启用四轮 Omni 底盘；
- Swerve 保留中科大四舵轮解算和后续接入接口，本次不实际调用；
- 现有 FDCAN1 上的 J8006 测试功能保持不变。

代码遵循中科大 RoboWalker 的直接、易读风格，不引入 RTOS、队列、SUBS、管理器或额外框架。

## 2. 本次范围

本次实现：

1. 新增 `tsk_remote`，迁入当前 NRF24L01 和 SBUS 接收职责。
2. 将遥控通道转换为统一的 `Speed_X`、`Speed_Y`、`Speed_W` 底盘指令。
3. 新增 `tsk_chassis`，当前调用中科大 Omni 公式控制四个 DM3519。
4. 在 `motor_init` 中增加四个底盘 DM3519 对象、Omni 初始化和未启用的 Swerve 初始化入口。
5. 增加 FDCAN3 初始化、四个反馈 Master ID 的分发、周期发送和在线检测。
6. 保留现有 Swerve 解算模块和航模驱动电机输出接口，但不编造未知硬件参数。
7. 更新 CMake、任务文件路径和 `main.c` 头文件路径。
8. 添加主机侧回归测试，并完成 Debug 固件构建。

本次不实现：

- 不采用 R1 的 Omni 数学公式；
- 不启用 Swerve 控制；
- 不创建尚未完成的航模电机驱动对象；
- 不猜测 Swerve 的轮径、底盘半径、机械零位和舵向速度；
- 不改变当前 J8006 的按键和 DEBUG 遥控按键控制逻辑；
- 不声称已经完成物理底盘验证。

## 3. 模块边界

### 3.1 `tsk_remote`

文件：

```text
User_File/4_Task/tsk_remote/tsk_remote.h
User_File/4_Task/tsk_remote/tsk_remote.cpp
```

该任务拥有并初始化现有对象：

```cpp
Class_NRF24L01 NRF24L01;
Class_SBUS_Receive SBUS_Receive;
```

对外提供：

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

extern Struct_Remote_Chassis_Command Remote_Chassis_Command;

void Remote_Init();
void Remote_10ms_PeriodElapsedCallback();
```

`tsk_remote` 只负责接收、解析和指令换算，不包含任何电机对象或底盘公式。

### 3.2 `tsk_chassis`

文件：

```text
User_File/4_Task/tsk_chassis/tsk_chassis.h
User_File/4_Task/tsk_chassis/tsk_chassis.cpp
```

该任务拥有底盘解算对象：

```cpp
Class_Chassis_Omni Chassis_Omni;
Class_Chassis_Swerve Chassis_Swerve;
```

对外提供：

```cpp
void Chassis_Init();
void Chassis_10ms_PeriodElapsedCallback();
void Chassis_100ms_PeriodElapsedCallback();
```

当前 `Chassis_Init()` 只初始化 Omni 参数。Swerve 对象参与编译但不初始化、不调度、不发送电机指令。

### 3.3 `tsk_config_and_callback`

该文件只保留以下顶层职责：

- 初始化顺序；
- FDCAN1/FDCAN3 接收回调；
- 10 ms、100 ms、1 s 裸机周期调度；
- 当前 J8006 按键测试；
- USART10 调试打印；
- `init_finished` 定义。

NRF24L01/SBUS 接收细节和底盘控制细节分别移到对应任务中。

## 4. 遥控数据换算

### 4.1 通道定义

根据当前 RC 发送端：

- `Channel[0]`：右摇杆 Y，映射为 `Speed_Y`；
- `Channel[1]`：右摇杆 X，映射为 `Speed_X`；
- `Channel[2]`：左摇杆 Y，本次底盘控制不使用；
- `Channel[3]`：左摇杆 X，映射为 `Speed_W`。

不对任何通道额外乘以 `-1`。

### 4.2 量程与挡位

接收端始终按完整 SBUS 量程 `200～1800` 映射到 `-3.0～3.0`：

```cpp
speed = (channel - 1000.0f) * 3.0f / 800.0f;
```

输入先限制在 `200～1800`。映射后绝对值小于 `0.01` 时置零。

四挡速度已经由 RC 发送端通过不同摇杆输出范围实现，接收端不再重复判断挡位：

- 低速挡：约 `975～1025`；
- 中速挡：约 `680～1320`；
- 高速挡：约 `360～1640`；
- 快速挡：`200～1800`。

### 4.3 模式和失联

只有同时满足以下条件时，底盘指令有效：

```text
Online != 0
Mode == 2（MANUAL）
Motor_Enable != 0
```

`tsk_remote` 在离线或非 MANUAL 模式下将三轴目标速度清零；`tsk_chassis` 仍再次检查全部三个状态，避免单一模块错误使电机误动作。

## 5. Omni 解算

### 5.1 坐标和轮序

- `X` 向右为正；
- `Y` 向前为正；
- `Wz` 从底盘上方向下观察时逆时针为正；
- 轮序固定为右前 `RF`、左前 `LF`、右后 `RB`、左后 `LB`；
- 线速度单位为 `m/s`；
- 角速度单位为 `rad/s`。

### 5.2 中科大公式

令 `k = √2 / 2`、`R` 为轮组投影点到底盘中心的距离、`S` 为轮半径：

```text
RF = (-k×Vx + k×Vy + Wz×R) / S
LF = (-k×Vx - k×Vy + Wz×R) / S
RB = ( k×Vx + k×Vy + Wz×R) / S
LB = ( k×Vx - k×Vy + Wz×R) / S
```

必须继续使用上述中科大课件公式。R1 中平移项采用 `/cos(45°)` 的写法，与本工程公式相差两倍，本次不采用。

### 5.3 机械参数

Omni 当前使用 R1 的机械参数：

```cpp
Wheel_Radius = 0.076f;                 // m
Chassis_Radius = 0.44717f;             // m
Chassis_Deceleration_Ratio = 268.0f / 17.0f;
DM3519_Deceleration_Ratio = 19.2f;
```

Omni 类输出轮端机械角速度，转换为 DM3519 目标角速度时使用：

```cpp
Motor_Omega = Wheel_Omega * Chassis_Deceleration_Ratio / DM3519_Deceleration_Ratio;
```

四个电机方向系数均为 `+1.0f`。

## 6. DM3519 配置和轮组映射

四个底盘 DM3519 均使用 `FDCAN3`：

| 位置 | 对象 | Motor ID | Master ID |
|---|---|---:|---:|
| 左前 LF | `DM_Chassis_LF` | `0x01` | `0x11` |
| 右前 RF | `DM_Chassis_RF` | `0x02` | `0x22` |
| 左后 LB | `DM_Chassis_LB` | `0x03` | `0x33` |
| 右后 RB | `DM_Chassis_RB` | `0x04` | `0x44` |

Omni 初始化采用：

- 控制模式：`Motor_DM_Control_Method_NORMAL_OMEGA`；
- `PMAX = 12.5 rad`；
- `VMAX = 200 rad/s`；
- `TMAX = 10 N·m`；
- 初始化目标速度为 `0 rad/s`；
- 初始化结束后发送失能，不自动运行。

`motor_init` 增加：

```cpp
void Motor_Omni_Init();
void Motor_Swerve_Init();
```

`Motor_Init()` 初始化并保持现有 J8006 行为，同时调用当前启用的 `Motor_Omni_Init()`。`Motor_Swerve_Init()` 复用同一组底盘 DM 对象，将其配置为角度速度模式，但本次不调用；待 Swerve 真实参数确认后再启用。

## 7. FDCAN3 接收与周期控制

`tsk_config_and_callback.cpp` 新增 `CAN3_Callback()`，按 `0x11、0x22、0x33、0x44` 将反馈数据分发给四个 DM3519 对象。初始化时调用：

```cpp
CAN_Init(&hfdcan3, CAN3_Callback);
```

每 10 ms 执行顺序：

1. `Remote_10ms_PeriodElapsedCallback()` 接收并更新遥控指令；
2. 保留当前按键和 J8006 测试处理；
3. `Chassis_10ms_PeriodElapsedCallback()` 解算并发送四个底盘电机报文。

每 100 ms：

- 保留 J8006 在线检测；
- 调用四个底盘 DM3519 的在线检测回调。

每 1 s：

- 保留 USART10 遥控数据打印；
- 不强制增加大量底盘调试输出。

## 8. 电机使能和安全行为

### 8.1 允许运行

当遥控在线、处于 MANUAL 且 `Motor_Enable=1` 时：

1. 如电机尚未使能，分别发送进入控制命令；
2. 使用最新三轴速度执行中科大 Omni 解算；
3. 完成减速比换算；
4. 写入四个 DM3519 的目标速度；
5. 调用四个电机的周期发送函数。

### 8.2 禁止运行

以下任一条件发生时立即禁止底盘运行：

- 遥控离线；
- 模式不是 MANUAL；
- `Motor_Enable=0`。

禁止运行时：

1. 三轴目标速度清零；
2. 四个电机目标速度清零；
3. 对仍处于使能状态的电机发送失能命令；
4. 继续保留必要的周期调用，使驱动内部状态保持一致。

启动完成后四个底盘电机默认失能。软件保护不得依赖串口打印或物理按键。

## 9. Swerve 保留方式

Swerve 继续采用现有中科大正方形四舵轮公式和逐轮就近转位，不参考 R1 的三舵轮结构。

本次只保留：

- `Class_Chassis_Swerve` 数学模块；
- 当前舵向角输入接口；
- 四个舵向目标角输出；
- 四个驱动轮机械角速度输出，供未来航模电机模块读取；
- 未调用的 `Motor_Swerve_Init()` 入口。

本次不填写未知的 Swerve 几何参数，不创建航模电机占位对象，也不在调度器中调用 Swerve。未来启用时，同一组 DM3519 只能在 Omni 速度模式和 Swerve 角度速度模式中选择一种初始化方式。

## 10. 工程文件调整

根目录 `CMakeLists.txt`：

- 修正用户已移动的 `tsk_config_and_callback` 源文件路径和包含路径；
- 加入 `tsk_remote` 和 `tsk_chassis` 源文件；
- 保留现有 Omni、Swerve 和 `motor_init`；
- 主机测试桩不加入固件目标。

`Core/Src/main.c`：

- 将任务头文件包含路径更新为 `4_Task/tsk_config_and_callback/tsk_config_and_callback.h`；
- 不修改 CubeMX 外设初始化代码结构。

## 11. 测试与验证

### 11.1 遥控换算测试

- `200、1000、1800` 分别映射为 `-3、0、3`；
- 超出范围的输入被限制；
- 中心死区正确清零；
- `Channel[0]、[1]、[3]` 分别进入 `Y、X、W`；
- 四挡输入范围自然得到对应速度，不做二次挡位缩放；
- 离线和非 MANUAL 时目标速度清零。

### 11.2 Omni 与电机映射测试

- 零输入、纯 `Vx`、纯 `Vy`、纯 `Wz` 和组合运动符合中科大公式；
- 验证使用 `√2/2`，不误用 R1 的 `/cos(45°)`；
- 验证 `RF、LF、RB、LB` 解算结果映射到正确电机对象；
- 验证四个方向系数为 `+1`；
- 验证底盘减速比和 DM3519 内部减速比换算；
- 验证离线、非 MANUAL 和关闭使能时目标清零并失能。

### 11.3 Swerve 回归测试

保留现有 Swerve 数学测试，确认本次任务整理未破坏：

- 原地旋转；
- 逐轮低速保持；
- 就近转位；
- 跨越 `±π` 的连续目标角。

### 11.4 工程验证

实现完成后执行：

1. 新增相关主机测试；
2. 全部现有主机测试；
3. Debug 固件完整编译和链接。

软件验证通过后，首次物理测试使用低速挡并将车轮悬空，分别检查前进、横移和原地旋转方向。真实电机安装方向和机械行为只有物理测试后才能确认。

## 12. 完成标准

- `tsk_remote`、`tsk_chassis` 和配置任务职责清晰；
- 当前只运行 Omni，Swerve 明确保留但未调用；
- Omni 严格使用中科大课件公式；
- 遥控量程、MANUAL 模式和 Motor_Enable 语义正确；
- 四个 DM3519 的 FDCAN3 ID、轮序和参数与确认内容一致；
- J8006 原有控制不受影响；
- 遥控离线或控制条件无效时底盘立即清零并失能；
- 未引入 RTOS、队列、SUBS 或不必要抽象；
- 代码排版、命名、注释和回调组织符合 `AGENTS.md`；
- 所有相关主机测试和 Debug 固件构建通过；
- 明确区分软件验证和物理板验证。
