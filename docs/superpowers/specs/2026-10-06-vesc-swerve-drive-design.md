# VESC 航模电机与四舵轮底盘控制设计

## 1. 目标

在 `Robocon-Elian` 裸机工程中增加 Makerbase MKS ESC MINI V6.7 Pro 的 VESC CAN 驱动，并完成四舵轮底盘从遥控指令、运动学解算到舵向电机和行进电机输出的完整接口。

实现必须满足：

- 保持中科大 RoboWalker 的目录分层、类结构、命名、回调和在线检测习惯；
- 不改写现有达妙电机协议和标准 CAN 调用方式；
- CAN1 专门连接 DM3519 和 J8006，CAN3 专门连接 VESC；
- 当前运行入口使用 Swerve 底盘，四个舵向电机和四个行进电机均使用正式控制流程；
- 当前实物只有左前一个 DM3519 和一个 U8 Lite KV85/VESC，但软件完整控制 ID 1 至 4，未接入节点不得阻止已接入轮组运行；
- 不引入 RTOS、队列、SUBS、完整 Vedder 固件栈或额外管理框架；
- 遥控离线、非 MANUAL、关闭电机使能或数据无效时，全部舵向和行进电机清零并失能；电机反馈在线状态只用于诊断，不作为其他轮组控制的前置条件。

本文补充并更新 `2026-10-05-omni-swerve-chassis-design.md` 中与 CAN 分配、DM ID、VESC 驱动和 Swerve 启用条件有关的内容。未被本文修改的遥控解析和 Omni 公式继续沿用原设计。

## 2. 硬件与协议参数

### 2.1 电机和电调

- 行进电机：T-Motor U8 Lite KV85；
- 电调：Makerbase MKS ESC MINI V6.7 Pro；
- VESC 固件：`v6.02`；
- VESC 硬件版本：`60`；
- U8 Lite KV85：`36N42P`，磁极数 42，极对数 21；
- U8 转子与行进轮直连，传动比 `1.0`；
- 当前没有 AS5047，第一阶段使用 Sensorless；
- 后续 AS5047 通过 VESC 的 SENSE 接口使用 SPI 接入，由 VESC 完成采样和 FOC 处理。

### 2.2 CAN 总线分配

| 总线 | 波特率 | 帧格式 | 设备 |
|---|---:|---|---|
| FDCAN1 | 1 Mbps | 11 位标准帧 | 四个 DM3519 和一个 J8006 |
| FDCAN2 | 1 Mbps | 当前不参与业务 | 保留 |
| FDCAN3 | 500 Kbps | 29 位扩展帧、Classic CAN | 四个 VESC |

FDCAN3 的 CubeMX 配置已经包含一个扩展过滤器、16 帧 RX FIFO0 和 8 帧 TX FIFO。FDCAN1 保持标准过滤器配置。CAN3 只接受扩展数据帧，不接受远程帧。

硬件连接必须经过 CAN 收发器，STM32 FDCAN 引脚不得直接连接 CANH/CANL。总线两端使用 120 Ω 终端电阻，STM32、收发器和 VESC 共地。

### 2.3 电机 ID

达妙电机全部迁移到 CAN1：

| 位置/用途 | 对象 | Motor ID | Master ID |
|---|---|---:|---:|
| 左前舵向/Omni | `DM_Chassis_LF` | `0x01` | `0x11` |
| 右前舵向/Omni | `DM_Chassis_RF` | `0x02` | `0x22` |
| 左后舵向/Omni | `DM_Chassis_LB` | `0x03` | `0x33` |
| 右后舵向/Omni | `DM_Chassis_RB` | `0x04` | `0x44` |
| 单独测试电机 | `DM_J8006` | `0x05` | `0x55` |

VESC 全部连接 CAN3：

| 位置 | 对象 | VESC ID |
|---|---|---:|
| 左前 | `VESC_Chassis_LF` | `1` |
| 右前 | `VESC_Chassis_RF` | `2` |
| 左后 | `VESC_Chassis_LB` | `3` |
| 右后 | `VESC_Chassis_RB` | `4` |

当前实物 VESC 配置为 ID 1，后续三个 VESC 依次配置为 ID 2、3、4，与同位置 DM3519 的 Motor ID 保持一致。

## 3. 模块架构

```text
drv_can
  -> 只负责标准帧和扩展帧收发
dvc_motor_vesc
  -> 负责 VESC 协议、单位转换、状态解析和安全输出
motor_init
  -> 负责电机对象、总线、ID和电机参数
tsk_chassis
  -> 负责遥控安全条件、底盘解算和电机目标分配
mod_chassis_swerve
  -> 只负责四舵轮运动学
```

模块间只传递有明确物理单位的数据。Swerve 和任务层使用 `rad/s`，VESC 类内部才使用 ERPM。CAN 公共驱动不包含 VESC 命令，Swerve 数学模块不包含 CAN 或电机对象。

## 4. CAN 驱动扩展

现有接口和行为保持不变：

```cpp
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function);
uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data, uint16_t Length);
```

仅追加扩展帧接口：

```cpp
void CAN_Extended_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function);

uint8_t CAN_Transmit_Extended_Data(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ID,
                                   uint8_t *Data,
                                   uint16_t Length);
```

`CAN_Extended_Init()` 继续复用 `Struct_CAN_Manage_Object` 和现有 HAL FIFO0 回调框架，但配置 `FDCAN_EXTENDED_ID` 掩码过滤器。CAN3 是 VESC 专用总线，因此硬件过滤器允许扩展数据帧进入 FIFO0，再由软件检查 VESC ID、报文类型和长度。原 `CAN_Init()`、标准帧过滤器和 DM 调用点不改变。

初始化方式：

```cpp
CAN_Init(&hfdcan1, CAN1_Callback);
CAN_Extended_Init(&hfdcan3, CAN3_Callback);
```

## 5. VESC CAN 协议子集

只实现当前底盘需要的 VESC CAN 子集，不移植完整 Vedder 通信栈。

扩展 ID 格式：

```text
Identifier = (Packet_ID << 8) | VESC_ID
VESC_ID    = Identifier & 0xFF
Packet_ID  = (Identifier >> 8) & 0xFF
```

使用的命令：

| 报文 | Packet ID | 方向 | 用途 |
|---|---:|---|---|
| `CAN_PACKET_SET_CURRENT` | 1 | STM32 -> VESC | 失能和故障时发送 0 A，使电机释放 |
| `CAN_PACKET_SET_RPM` | 3 | STM32 -> VESC | 发送目标 ERPM |
| `CAN_PACKET_STATUS` | 9 | VESC -> STM32 | ERPM、电机电流、Duty |
| `CAN_PACKET_STATUS_4` | 16 | VESC -> STM32 | MOS温度、电机温度、输入电流、PID位置 |
| `CAN_PACKET_STATUS_5` | 27 | VESC -> STM32 | Tachometer、输入电压 |

所有数值按照 VESC 官方协议使用大端整数。接收回调只有同时满足以下条件时才更新对象：

- `Header.IdType == FDCAN_EXTENDED_ID`；
- VESC ID 为 1 至 4；
- Packet ID 为 9、16 或 27；
- `STATUS_1` 和 `STATUS_4` 的长度为 8 字节；
- `STATUS_5` 的长度为 6 字节。

未知 ID、未知报文、远程帧和错误长度直接忽略，不增加接收计数，也不改变目标速度。

数据布局固定为：

```text
SET_CURRENT
  B0..B3: int32，大端，单位 A * 1000

SET_RPM
  B0..B3: int32，大端，单位 ERPM

STATUS_1
  B0..B3: int32，大端，ERPM
  B4..B5: int16，大端，电机电流 A * 10
  B6..B7: int16，大端，Duty * 1000

STATUS_4
  B0..B1: int16，大端，MOS 温度 °C * 10
  B2..B3: int16，大端，电机温度 °C * 10
  B4..B5: int16，大端，输入电流 A * 10
  B6..B7: int16，大端，PID 位置 degree * 50

STATUS_5
  B0..B3: int32，大端，VESC Tachometer 原始计数
  B4..B5: int16，大端，输入电压 V * 10
```

大端整数打包和解析函数只作为 `dvc_motor_vesc.cpp` 的内部辅助函数，不增加通用序列化框架。

## 6. VESC 设备类

文件：

```text
User_File/2_Device/Motor/Motor_VESC/dvc_motor_vesc.h
User_File/2_Device/Motor/Motor_VESC/dvc_motor_vesc.cpp
```

类结构沿用中科大 DM 电机类：

```cpp
enum Enum_Motor_VESC_Status
{
    Motor_VESC_Status_DISABLE = 0,
    Motor_VESC_Status_ENABLE,
};

struct Struct_Motor_VESC_Rx_Data
{
    int32_t Now_ERPM;
    float Now_Omega;
    float Motor_Current;
    float Duty;
    float MOS_Temperature;
    float Motor_Temperature;
    float Input_Current;
    float PID_Position;
    int32_t Tachometer;
    float Input_Voltage;
};
```

对外接口：

```cpp
class Class_Motor_VESC
{
public:
    void Init(FDCAN_HandleTypeDef *hcan,
              uint8_t VESC_ID,
              uint8_t Pole_Pairs,
              float Gearbox_Rate,
              int32_t ERPM_Max);

    inline Enum_Motor_VESC_Status Get_Status() const;
    inline bool Get_Motor_Enable() const;
    inline int32_t Get_Now_ERPM() const;
    inline float Get_Now_Omega() const;
    inline float Get_Motor_Current() const;
    inline float Get_Duty() const;
    inline float Get_MOS_Temperature() const;
    inline float Get_Motor_Temperature() const;
    inline float Get_Input_Current() const;
    inline float Get_PID_Position() const;
    inline int32_t Get_Tachometer() const;
    inline float Get_Input_Voltage() const;
    inline int32_t Get_Control_ERPM() const;
    inline float Get_Control_Omega() const;
    inline void Set_Motor_Enable(bool Motor_Enable);
    inline void Set_Control_Omega(float Control_Omega);

    void CAN_RxCpltCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_Send_PeriodElapsedCallback();

protected:
    void Data_Process();
    void Output();
};
```

类内部保存中科大风格的 `CAN_Manage_Object`、ID、参数、`Flag`、`Pre_Flag`、`Offline_Count`、接收结构体和控制量。`bool` 只用于含义明确的 `Motor_Enable`，不在任务中堆积派生状态变量。

### 6.1 单位转换

U8 极对数为 21，传动比为 1.0：

```text
Mechanical_RPM = Wheel_Omega * 60 / (2 * PI)
Target_ERPM    = Mechanical_RPM * 21 * 1.0
Target_ERPM    = Wheel_Omega * 200.535...

Wheel_Omega    = Feedback_ERPM / 21 / 1.0 * 2 * PI / 60
```

四挡最高机械转速为 `1440 rpm`，因此 `Target_ERPM` 限制为 `-30240` 至 `+30240`。Swerve 和任务层不直接处理 ERPM。

### 6.2 使能和在线状态

- 初始化后 `Motor_Enable=false`、`Control_Omega=0`；
- `Motor_Enable=true` 时周期发送 `CAN_PACKET_SET_RPM`，不等待状态反馈；
- `Motor_Enable=false` 时清零速度目标并周期发送 `CAN_PACKET_SET_CURRENT(0 A)`；
- 每 100 ms执行一次在线检测；
- 连续两个 100 ms 周期没有收到合法状态帧时判定离线；
- 掉线和恢复只更新 `Motor_VESC_Status` 供诊断，不改写遥控使能和当前目标；
- VESC 自身通信超时设置为 200 ms、超时制动电流为 0 A。

VESC 没有 DM 的 Enter/Exit 帧，因此不伪造类似协议。软件使能打开时允许发送 RPM 目标；软件使能关闭时使用 0 A 电流命令释放电机，而不是使用可能主动维持零速的 `SET_RPM(0)`。

## 7. 电机初始化

`motor_init.h/.cpp` 增加 `Class_Motor_VESC` 头文件、四个 VESC 对象和独立初始化函数。VESC 参数集中配置：

```text
Pole_Pairs  = 21
Gearbox_Rate = 1.0
ERPM_Max = 30240
```

`Motor_Omni_Init()` 将四个 DM3519 绑定到 CAN1，保持原 Motor ID、Master ID和速度模式。`DM_J8006` 绑定 CAN1，使用 Motor ID `0x05`、Master ID `0x55`。

`Motor_Swerve_Init()` 复用同一组 DM3519 对象并选择角度速度模式。Omni 和 Swerve 不能同时初始化或同时控制这四个 DM 对象。当前 `Motor_Init()` 选择 Swerve 初始化和四个默认失能的 VESC 对象，`Motor_Omni_Init()` 只作为后续切换底盘类型时的备选入口保留。

位置速度模式的第二个 `float` 是转向速度限制。四个 DM3519 初始化和运行时均使用：

```cpp
#define SWERVE_STEER_OMEGA_LIMIT 37.69911184f
```

不得把该值设为 `0.0f`。目标角度变化时若速度限制为零，DM3519 虽然在线且已使能，也不会执行转向。

## 8. Swerve 机械参数和校准

初始机械参数：

```cpp
#define CHASSIS_SWERVE_WHEEL_RADIUS 0.070f
#define CHASSIS_SWERVE_RADIUS       0.450f
```

- `Wheel_Radius` 是行进轮半径；
- `Chassis_Radius` 是底盘旋转中心到舵轮转向轴中心的距离；
- 两者后续根据实测只修改参数，不修改公式。

坐标约定：

- `+X` 向右；
- `+Y` 向前；
- `+W` 从底盘上方向下观察为逆时针；
- 舵向角 `0 rad` 表示轮子朝车头。

每轮预留独立校准参数，初始偏移为 0、方向为 `+1`：

```cpp
#define SWERVE_STEER_OFFSET_LF 0.0f
#define SWERVE_STEER_OFFSET_RF 0.0f
#define SWERVE_STEER_OFFSET_LB 0.0f
#define SWERVE_STEER_OFFSET_RB 0.0f

#define SWERVE_STEER_DIRECTION_LF 1.0f
#define SWERVE_STEER_DIRECTION_RF 1.0f
#define SWERVE_STEER_DIRECTION_LB 1.0f
#define SWERVE_STEER_DIRECTION_RB 1.0f

#define SWERVE_DRIVE_DIRECTION_LF 1.0f
#define SWERVE_DRIVE_DIRECTION_RF 1.0f
#define SWERVE_DRIVE_DIRECTION_LB 1.0f
#define SWERVE_DRIVE_DIRECTION_RB 1.0f
```

角度转换：

```text
Chassis_Now_Angle = Direction * (DM_Now_Angle - Offset)
DM_Target_Angle   = Chassis_Target_Angle * Direction + Offset
```

实车调试时以轮子朝车头为零点，只修改每轮偏移和方向，不在解算公式或任务流程中散落负号。

## 9. Swerve 控制流程

轮序映射固定并显式书写：

```text
Swerve_Wheel_RF <-> DM_Chassis_RF <-> VESC_Chassis_RF
Swerve_Wheel_LF <-> DM_Chassis_LF <-> VESC_Chassis_LF
Swerve_Wheel_RB <-> DM_Chassis_RB <-> VESC_Chassis_RB
Swerve_Wheel_LB <-> DM_Chassis_LB <-> VESC_Chassis_LB
```

每 10 ms 的正式 Swerve 控制流程：

1. 检查遥控 `Online`、`Mode == MANUAL`、`Motor_Enable`；
2. 将四个 DM 反馈角度按偏移和方向转换后传给 `Set_Now_Angle()`；
3. 将 `Speed_X`、`Speed_Y`、`Speed_W` 写入 `Class_Chassis_Swerve`；
4. 调用 `Calculate()`；
5. 将四个目标角度转换回 DM 坐标并写入对应 DM3519，同时设置 `37.69911184 rad/s`（360 rpm）转向速度限制；
6. 遥控使能时对四个 DM3519 发送 Enter，并按 10 ms 周期发送位置速度报文；
7. 将四个机械轮速乘以各自驱动方向后写入同位置 VESC，并设置软件使能；
8. 四个 VESC 按独立 20 ms 路径发送 RPM 报文。

Swerve 数学模块继续保留：

- 单个轮组速度接近零时保持上一次舵向目标；
- 90°反向优化使用85°退出、95°进入的滞环，防止临界角附近反复切换轮速方向；
- `Vx=Vy=0、Wz!=0` 原地旋转时各轮速度非零，不进入静止保持分支；
- 目标角跨越 `+PI/-PI` 时保持连续的就近角度。

遥控离线、非 MANUAL、总使能关闭或指令无效时，四个 VESC 目标立即清零并置为软件失能，随后发送 0 A 电流命令释放电机；四个 DM 保持当前角度目标、速度限制清零并发送 Exit。电机自身的反馈在线状态不阻止其他轮组控制，因而当前只连接左前一组时仍可独立调试。

当前正式入口调用 Swerve 控制。四轮机械零位、方向和实车尺寸后续只通过集中常量校准，不改动运动学公式和电机协议。

### 9.1 遥控四挡速度

遥控器保留独立的 XY 和 W 四挡速度选择，两个挡位上电后都默认为中速挡。挡位在遥控器端通过改变 CH0、CH1、CH3 的 SBUS 输出范围生效，不额外占用 NRF24L01 自定义字段。低速、中速、高速、超速挡的通道范围依次为 `800～1200`、`600～1400`、`400～1600`、`200～1800`。车端仍将 CH0、CH1 的 `200～1800` 线性映射到 `-10.55575～10.55575 m/s`；CH3 则根据450 mm底盘旋转半径映射到 `-23.45722～23.45722 rad/s`。这样无论纯平移还是原地旋转，摇杆推满时四挡对应的行进电机机械转速均为360、720、1080、1440 rpm。

### 9.2 平滑驱动和舵向保护

- VESC目标机械转速按10 ms周期进行斜坡规划，加速为720 rpm/s，减速为1440 rpm/s；
- 正反转切换必须先减速到0，下一周期才向相反方向加速；
- 遥控离线、非MANUAL或电机失能属于安全事件，不经过斜坡，立即清零并失能；
- 第一次真实运动命令之前，零速舵向目标持续跟随最新DM反馈，避免反馈晚到时回到软件0点；
- 舵向误差小于等于5°时允许全速，大于等于45°时行进轮目标为0，中间线性过渡；
- 四轮解算结果按 `150.79644737 rad/s`（1440 rpm）统一同比例归一化，保持组合运动时各轮速度比例。

车端不得再次乘挡位比例，否则会产生二次缩放。当前诊断中摇杆最大值约为 `1.20`，已经证明遥控器挡位缩放到达车端。

## 10. 裸机任务调度

现有前台调度增加 20 ms 周期：

```text
Task10ms_Callback
  -> Remote_10ms_PeriodElapsedCallback()
  -> Chassis_Swerve_10ms_PeriodElapsedCallback()

Task20ms_Callback
  -> Chassis_20ms_PeriodElapsedCallback()
  -> 四个 VESC 的 TIM_Send_PeriodElapsedCallback()

Task100ms_Callback
  -> DM 在线检测
  -> VESC TIM_100ms_Alive_PeriodElapsedCallback()
```

`tsk_config_and_callback` 只增加周期入口和 CAN1/CAN3 顶层分发，不包含 ERPM换算、底盘公式或电机控制策略。

`tsk_chassis` 内部按职责拆分：

```cpp
void Chassis_Omni_Control();
void Chassis_Swerve_Control();
void Chassis_Swerve_Motor_Disable();
void Chassis_Swerve_Steer_Motor_Control();
void Chassis_Swerve_Drive_Motor_Control();
void Chassis_Motor_Send();
```

## 11. VESC 反馈和 AS5047

VESC Tool 状态报文配置：

- Rate 1 为 50 Hz，启用 `Status 1` 和 `Status 4`；
- Rate 2 为 10 Hz，启用 `Status 5`；
- 关闭 `Status 2`、`Status 3` 和 `Status 6`。

`Status 4` 中的 PID Position 是 VESC 处理后的内部位置，单位为度，不是 AS5047 原始 14 位数值。当前驱动将其用于调试和状态观察，不将其作为舵向角反馈；舵向角仍来自 DM3519。

后续安装 AS5047 时：

1. 按 MKS VESC SENSE 接口定义核对电源和 `CS/MISO/SCK/MOSI`；
2. 在 VESC Tool 中选择 AS5047 Encoder；
3. 执行编码器检测，确认 Offset、Ratio、Inverted；
4. 将 `APP to Use` 改为 `No App`；
5. 将 `Enable Permanent UART` 改为 `False`；
6. 保存配置并重新上电；
7. 继续通过 CAN3 控制和接收状态，STM32 VESC 类接口不改变。

升级 MKS 克隆电调固件不属于本次范围。当前 v6.02 已满足 CAN RPM 和 AS5047 规划需要。

## 12. 电流、超时和首次测试限制

当前检测配置接近电机完整能力。第一次工程联调前，将 VESC 临时设置为：

```text
Motor Current Max          = 5.00 A
Motor Current Max Brake    = -5.00 A
Battery Current Max        = 3.00 A
Battery Current Max Regen  = 0.00 A
Absolute Maximum Current   = 29.38 A
Timeout                    = 200 ms
Timeout Brake Current      = 0.00 A
```

修改后写入电机和 App 配置。`Absolute Maximum Current` 保持硬件保护值，不由STM32驱动修改。

STM32每 20 ms发送一次目标，VESC 自身超时负责在STM32卡死或CAN断线时释放电机。STM32接收侧使用相同的200 ms离线判定。

## 13. USART9 VOFA 调试

USART9 继续作为 VOFA 专用串口，第一阶段不再输出IMU曲线，改为左前 VESC ID 1：

| 通道 | 数据 | 单位 |
|---:|---|---|
| 1 | Motor_Enable | 0/1 |
| 2 | Control_ERPM | ERPM |
| 3 | Now_ERPM | ERPM |
| 4 | Control_Omega | rad/s |
| 5 | Now_Omega | rad/s |
| 6 | Motor_Current | A |
| 7 | Duty | 比例 |
| 8 | MOS_Temperature | °C |
| 9 | Input_Voltage | V |

VOFA以50 Hz更新。USART10普通文本打印保持现有行为，不因VESC调试改成二进制输出。

## 14. 测试与验证

### 14.1 主机测试

新增可观察行为测试：

- `0 rad/s` 打包为 `0 ERPM`；
- `1 rad/s` 四舍五入为约 `201 ERPM`；
- 正负方向转换正确；
- 超出范围时限制为 `+30240/-30240 ERPM`；
- 已使能时即使尚未收到在线反馈也发送 RPM 命令；
- 失能时清零目标并发送 `SET_CURRENT(0 A)`，不发送 `SET_RPM(0)`；
- `STATUS_1/4/5` 大端数据解析和单位缩放正确；
- ERPM反馈正确换算为机械 `rad/s`；
- 未知ID、未知报文和错误长度不更新状态，包括拒绝非8字节的 `STATUS_1/4` 和非6字节的 `STATUS_5`；
- 一个100 ms周期无反馈仍保留在线，连续两个周期无反馈后离线；
- VESC 离线和恢复不清除遥控仍有效的目标；
- DM3519 位置速度模式始终发送非零 `37.69911184 rad/s` 转向速度限制；
- 四个 DM3519 与四个 VESC 均按 RF、LF、RB、LB 同名映射得到目标，缺少任意节点不阻止其他节点；
- 遥控端四挡缩放后的 SBUS 输入只进行一次车端线性映射；
- RF、LF、RB、LB映射、偏移和方向换算正确；
- Swerve零速同步、原地旋转、90°滞环优化、舵向到位限速、轮速归一化和跨 `+PI/-PI` 回归测试通过；
- Omni现有行为和DM标准帧路径不受影响。

测试桩位于 `tests/host`，不加入STM32固件目标。

### 14.2 工程验证

实现完成后执行：

1. 与 VESC 和 Swerve 改动直接相关的主机测试；
2. 全部现有主机测试；
3. Debug固件完整编译和链接，产物为 `Robocon-Elian.elf`。

### 14.3 物理验证

1. 将行进轮架空并准备独立断电手段；
2. 写入5 A测试电流限制和200 ms超时；
3. 只连接 DM3519 ID 1 和 VESC ID 1，保持 `Motor_Enable=0`，确认两条 CAN 总线在线和状态反馈；
4. 确认失能状态持续发送 0 A 电流命令，电机不主动保持零速；
5. 单独启用ID 1，先使用低挡 `+7560 ERPM`以内检查正向；
6. 清零并完全停止后，先使用低挡 `-7560 ERPM`以内检查反向；
7. 通过VOFA核对目标、反馈、电流、Duty、温度和电压；
8. 关闭 `Motor_Enable`，确认立即回零；
9. 低速运行时断开CAN，确认不超过200 ms停止输出；
10. 逐步接入 ID 2、3、4，核对每组舵向和行进轮映射；完整四轮控制不需要清理单电机临时分支，因为生产代码不包含单轮特判。

物理验证前只能声明软件测试和固件构建结果，不能声称真实电机行为已经验证。

## 15. 完成标准

- CAN1上的五个DM对象使用确认后的ID并保持中科大标准帧路径；
- CAN3以500 Kbps正确收发VESC 29位扩展帧；
- 四个VESC对象、ID、状态结构和RPM接口完整；
- `rad/s <-> ERPM`转换使用21极对、1:1传动比并执行30240 ERPM限幅；
- 默认失能和遥控失联均切换为 0 A 电流命令；VESC 在线状态只用于诊断，不阻止有效遥控目标发送；
- Swerve使用70 mm轮半径、450 mm底盘半径和每轮独立校准参数；
- Swerve轮序、DM舵向目标和VESC行进目标映射明确；
- 当前运行入口为完整四轮 Swerve，Omni 只保留独立备选初始化和控制函数；
- 四个 DM3519 使用位置速度模式和 `37.69911184 rad/s`（360 rpm）转向速度限制；
- 四个 VESC 使用与同位置 DM3519 对应的 ID 1 至 4；
- 遥控器 XY/W 四挡速度通过 SBUS 输出范围自然生效，车端不重复缩放；
- USART9 VOFA能够观察单个VESC关键状态，USART10保持文本调试；
- 未引入RTOS、队列、SUBS、动态内存或完整VESC固件依赖；
- 代码结构、命名、注释、周期回调和存活检测接近中科大RoboWalker习惯；
- 相关主机测试、全部主机测试和Debug固件构建具有最新结果；
- 尚未完成的实车参数只通过集中常量调整，不修改协议和公式。
