/**
 * @file tsk_chassis.h
 * @brief 底盘控制任务
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

#ifndef TSK_CHASSIS_H
#define TSK_CHASSIS_H

/* Includes ------------------------------------------------------------------*/

#include "3_Module/Chassis/Omni/mod_chassis_omni.h"
#include "3_Module/Chassis/Swerve/mod_chassis_swerve.h"
#include "4_Task/tsk_motor_init/motor_init.h"

/* Exported macros -----------------------------------------------------------*/

#define CHASSIS_SWERVE_ACTIVE_WHEEL_MASK (1U << Swerve_Wheel_LF)
#define CHASSIS_SWERVE_CONTROL_PERIOD_MS 10U

// U8线性速度斜坡: 21极对, 加减速度均为21000ERPM/s; 不修改VESC内部Ramp
#define VESC_ACCEL_RPM_PER_S 1000.0f  // 机械转速幅值增大时的加速度, 机械rpm/s
#define VESC_DECEL_RPM_PER_S 1000.0f  // 机械转速幅值减小时的减速度, 机械rpm/s

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 舵轮回零判定参数, 计数单位为10ms控制周期
 */
struct Struct_Chassis_Swerve_Homing_Parameter
{
    float Position_Limit = 0.05f;                      // 电机轴位置误差上限, rad
    float Omega_Limit = 0.5f;                          // 电机轴速度上限, rad/s
    uint8_t Stable_Cycles = 10U;                       // 连续稳定周期数
    uint16_t Timeout_Cycles = 300U;                    // 回零超时周期数
};

/**
 * @brief 串口MIT调参的允许范围, 不是驱动器电流或转矩保护配置
 */
struct Struct_Chassis_Swerve_MIT_Limit
{
    float K_P_Min = 0.0f;
    float K_P_Max = 50.0f;
    float K_D_Min = 0.0f;
    float K_D_Max = 5.0f;
    float Torque_Min = -2.0f;                          // 前馈转矩下限, N·m
    float Torque_Max = 2.0f;                           // 前馈转矩上限, N·m
};

/**
 * @brief 舵轮任务固定配置, 默认值集中在此处, 运行中不修改
 * @note CAN ID、协议量程及MIT初值仍由motor_init模块负责
 */
struct Struct_Chassis_Swerve_Config
{
    float Wheel_Radius = 0.070f;                       // 行进轮半径, m
    float Chassis_Radius = 0.450f;                     // 轮组到底盘中心距离, m
    float Wheel_Omega_Max = 150.79644737f;             // 最大轮速, rad/s
    float Steer_Angle_Limit = 245.0f;                  // 舵向目标软限位, rad
    float Steer_Full_Speed_Angle = 0.08726646f;        // 全速允许舵向误差, rad(5°)
    float Steer_Stop_Angle = 0.78539816f;              // 停止驱动舵向误差, rad(45°)
    float Accel_RPM_Per_S = VESC_ACCEL_RPM_PER_S;      // 加速度, 机械rpm/s
    float Decel_RPM_Per_S = VESC_DECEL_RPM_PER_S;      // 减速度, 机械rpm/s
    uint32_t Control_Period_MS = CHASSIS_SWERVE_CONTROL_PERIOD_MS;
    uint8_t Active_Wheel_Mask = CHASSIS_SWERVE_ACTIVE_WHEEL_MASK;
    Struct_Chassis_Swerve_Homing_Parameter Homing;
    Struct_Chassis_Swerve_MIT_Limit MIT_Limit;
};

/**
 * @brief 保留的全向轮任务配置, 不参与当前舵轮控制
 */
struct Struct_Chassis_Omni_Config
{
    float Wheel_Radius = 0.076f;                       // 轮半径, m
    float Chassis_Radius = 0.44717f;                   // 底盘旋转半径, m
    float Deceleration_Ratio = 268.0f / 17.0f;
    float Motor_Deceleration_Ratio = 19.2f;
    float Motor_Direction[Swerve_Wheel_Number] = {1.0f, 1.0f, 1.0f, 1.0f};
};

/**
 * @brief 单轮电机绑定与安装配置, 仅引用motor_init中的既有对象
 */
struct Struct_Chassis_Swerve_Wheel_Config
{
    Class_Motor_DM_Normal *Steer_Motor;
    Class_Motor_VESC *Drive_Motor;
    float Steer_Direction;                             // 舵向安装方向, ±1
    float Steer_Offset;                                // 舵向机构角度偏移, rad
    float Drive_Direction;                             // 行进安装方向, ±1
};

/**
 * @brief 单轮MIT可调参数, 保存在RAM, 通过既有校验函数修改
 */
struct Struct_Chassis_Swerve_MIT_Parameter
{
    float K_P;
    float K_D;
    float Feedforward_Torque;                          // 前馈转矩, N·m
};

/**
 * @brief 单个舵轮遥控使能后的回零状态
 */
enum Enum_Chassis_Swerve_Homing_Status : uint8_t
{
    Chassis_Swerve_Homing_Status_DISABLED = 0,
    Chassis_Swerve_Homing_Status_WAIT_FEEDBACK,
    Chassis_Swerve_Homing_Status_HOMING,
    Chassis_Swerve_Homing_Status_READY,
    Chassis_Swerve_Homing_Status_TIMEOUT,
};

/**
 * @brief 单轮回零运行状态, 失能后重新开始回零
 */
struct Struct_Chassis_Swerve_Homing_Control
{
    Enum_Chassis_Swerve_Homing_Status Status;
    uint32_t Feedback_Update_Count;
    uint16_t Elapsed_Cycles;
    uint8_t Stable_Cycles;
};

/**
 * @brief 单轮任务运行状态, 与固定配置及MIT可调参数分开保存
 */
struct Struct_Chassis_Swerve_Wheel_Control
{
    float Drive_Control_Omega;                         // 当前下发机械角速度, rad/s
    Struct_Chassis_Swerve_Homing_Control Homing;
};

/**
 * @brief 单个舵轮控制诊断数据
 * @note 由10ms底盘任务更新，供低频串口诊断读取
 */
struct Struct_Chassis_Swerve_Diagnostic
{
    float Steer_Target_Angle;
    float Steer_Now_Angle;
    float Steer_Angle_Error;
    float Steer_Now_Omega;
    float Steer_Target_Omega_Limit;
    float Steer_Speed_Ratio;
    bool Total_Angle_Initialized;
    Enum_Chassis_Swerve_Homing_Status Steer_Homing_Status;
    uint32_t Steer_Homing_Elapsed_Time;
    bool Steer_Target_Enable;
};

/**
 * @brief 底盘任务控制类, 按中科大模块习惯集中声明配置、状态和方法
 * @note 内部解算类仅负责运动学, 本类负责回零、斜坡、安全和电机控制
 * @note 绑定既有电机对象, 生产调度只使用cpp中的一个实例; 不拥有电机驱动
 */
class Class_Chassis_Control
{
public:
    /**
     * @brief 初始化底盘解算及运行状态, 需先执行Motor_Init, 不在此处使能电机
     */
    void Init();

    /**
     * @brief 保留的全向轮10ms控制入口, 当前生产调度不使用
     */
    void TIM_10ms_Omni_PeriodElapsedCallback();

    /**
     * @brief 舵轮10ms控制入口: 安全检查、回零、解算、控制并发送DM
     */
    void TIM_10ms_Swerve_PeriodElapsedCallback();

    /**
     * @brief 舵轮控制更新后每10ms发送VESC报文, 发送频率100Hz
     */
    void TIM_10ms_VESC_PeriodElapsedCallback();

    /**
     * @brief 每100ms检查当前活动轮组的电机在线状态
     */
    void TIM_100ms_PeriodElapsedCallback();

    /**
     * @brief 校验并保存单轮MIT位置增益, 仅修改RAM
     */
    bool Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P);

    /**
     * @brief 校验并保存单轮MIT速度阻尼, 仅修改RAM
     */
    bool Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D);

    /**
     * @brief 校验并保存单轮MIT前馈转矩, 单位N·m, 仅修改RAM
     */
    bool Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque);

private:
    // 固定配置: 默认值见上方结构体, 不通过串口在运行中修改
    const Struct_Chassis_Omni_Config Chassis_Omni_Config = {};
    const Struct_Chassis_Swerve_Config Chassis_Swerve_Config = {};
    static constexpr float RPM_TO_RAD_S = 0.10471975511965977f;

    static_assert(VESC_ACCEL_RPM_PER_S > 0.0f && VESC_DECEL_RPM_PER_S > 0.0f, "VESC ramp limits must be positive");
    static_assert(Swerve_Wheel_RF == 0 && Swerve_Wheel_LF == 1 && Swerve_Wheel_RB == 2 && Swerve_Wheel_LB == 3, "Wheel configuration order must match Enum_Swerve_Wheel");

    // 初始化相关常量: RF、LF、RB、LB, 顺序与轮组枚举一致
    // 每项依次为舵向电机、行进电机、舵向方向、角度偏移(rad)、行进方向
    const Struct_Chassis_Swerve_Wheel_Config Swerve_Wheel_Config[Swerve_Wheel_Number] = {
        {&DM_Chassis_RF, &VESC_Chassis_RF, 1.0f, 0.0f, 1.0f}, // RF
        {&DM_Chassis_LF, &VESC_Chassis_LF, 1.0f, 0.0f, 1.0f}, // LF
        {&DM_Chassis_RB, &VESC_Chassis_RB, 1.0f, 0.0f, 1.0f}, // RB
        {&DM_Chassis_LB, &VESC_Chassis_LB, 1.0f, 0.0f, 1.0f}, // LB
    };

    // 解算对象: 不负责CAN通信或电机驱动
    Class_Chassis_Omni Chassis_Omni;
    Class_Chassis_Swerve Chassis_Swerve;

    // 内部变量: 运行状态与MIT可调参数分开保存, 禁止外部直接修改
    Struct_Chassis_Swerve_Wheel_Control Swerve_Wheel_Control[Swerve_Wheel_Number] = {};
    Struct_Chassis_Swerve_MIT_Parameter Swerve_MIT_Parameter[Swerve_Wheel_Number] = {};

    // 内部函数: 电机控制与发送
    void Omni_Control();

    void Motor_Disable();

    void Motor_Send();

    void Swerve_Motor_Send();

    void Swerve_Steer_Motor_Control();

    void Swerve_Drive_Motor_Control();

    void Swerve_Motor_Disable();

    void Swerve_Control();

    // 内部函数: 斜坡、安装角度换算、回零及诊断
    Class_Motor_DM_Normal *Swerve_Get_Steer_Motor(Enum_Swerve_Wheel Wheel);

    Class_Motor_VESC *Swerve_Get_Drive_Motor(Enum_Swerve_Wheel Wheel);

    bool Swerve_Is_Wheel_Active(Enum_Swerve_Wheel Wheel);

    float Swerve_Motor_To_Mechanism_Angle(Enum_Swerve_Wheel Wheel, float Motor_Angle);

    float Swerve_Mechanism_To_Motor_Angle(Enum_Swerve_Wheel Wheel, float Mechanism_Angle);

    float Swerve_Motor_To_Mechanism_Omega(Enum_Swerve_Wheel Wheel, float Motor_Omega);

    void Swerve_Hold_Homing_Target(Enum_Swerve_Wheel Wheel);

    bool Swerve_Update_Homing(Enum_Swerve_Wheel Wheel);

    void Swerve_Update_Diagnostic(Enum_Swerve_Wheel Wheel);

    float Swerve_Update_Drive_Omega(Enum_Swerve_Wheel Wheel, float Target_Omega);

    float Swerve_Get_Steer_Speed_Ratio(Enum_Swerve_Wheel Wheel);

    void Swerve_Reset_Drive_Omega();
};

/* Exported variables --------------------------------------------------------*/

// 保留原诊断快照接口供VOFA和UART10读取, 不参与控制状态计算
extern Struct_Chassis_Swerve_Diagnostic Chassis_Swerve_Diagnostic[Swerve_Wheel_Number];

/* Exported function declarations --------------------------------------------*/

// 保留现有任务与调参入口, 内部统一转调同一个Class_Chassis_Control对象

void Chassis_Init();

void Chassis_10ms_PeriodElapsedCallback();

void Chassis_Swerve_10ms_PeriodElapsedCallback();

void Chassis_VESC_10ms_PeriodElapsedCallback();

void Chassis_100ms_PeriodElapsedCallback();

bool Chassis_Swerve_Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P);

bool Chassis_Swerve_Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D);

bool Chassis_Swerve_Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque);

#endif
