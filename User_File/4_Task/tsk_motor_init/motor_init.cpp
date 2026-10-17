/**
 * @file motor_init.cpp
 * @brief 电机对象和初始化配置
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "motor_init.h"

#include "fdcan.h"

/* Private variables ---------------------------------------------------------*/

static const Struct_Motor_J8006_Init_Parameter J8006_Parameter{};
static const Struct_Motor_DM_Init_Parameter DM_Parameter{};
static const Struct_Motor_VESC_Init_Parameter VESC_Parameter{};

/* Exported variables --------------------------------------------------------*/

Class_Motor_DM_Normal DM_J8006;
Class_Motor_DM_Normal DM_Chassis_LF;
Class_Motor_DM_Normal DM_Chassis_RF;
Class_Motor_DM_Normal DM_Chassis_LB;
Class_Motor_DM_Normal DM_Chassis_RB;
Class_Motor_VESC VESC_Chassis_LF;
Class_Motor_VESC VESC_Chassis_RF;
Class_Motor_VESC VESC_Chassis_LB;
Class_Motor_VESC VESC_Chassis_RB;

/* Private function declarations ---------------------------------------------*/

// 当前生产入口选择舵轮底盘，保留全向轮初始化供后续切换。
[[maybe_unused]] static void Motor_Omni_Init();

static void Motor_Swerve_Init();

static void Motor_VESC_Init();

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化全部电机
 */
void Motor_Init()
{
    // 达妙J8006电机, 速度控制模式
    DM_J8006.Init(&hfdcan1, J8006_Parameter.Master_ID, J8006_Parameter.Motor_ID, Motor_DM_Control_Method_NORMAL_OMEGA, J8006_Parameter.Angle_Max, J8006_Parameter.Omega_Max, J8006_Parameter.Torque_Max);
    DM_J8006.Set_Control_Omega(0.0f);

    // Motor_Omni_Init();
    Motor_Swerve_Init();
    Motor_VESC_Init();
}

/**
 * @brief 初始化全向轮底盘电机
 */
static void Motor_Omni_Init()
{
    DM_Chassis_LF.Init(&hfdcan1, DM_CHASSIS_LF_MASTER_ID, DM_CHASSIS_LF_MOTOR_ID, Motor_DM_Control_Method_NORMAL_OMEGA, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);
    DM_Chassis_RF.Init(&hfdcan1, DM_CHASSIS_RF_MASTER_ID, DM_CHASSIS_RF_MOTOR_ID, Motor_DM_Control_Method_NORMAL_OMEGA, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);
    DM_Chassis_LB.Init(&hfdcan1, DM_CHASSIS_LB_MASTER_ID, DM_CHASSIS_LB_MOTOR_ID, Motor_DM_Control_Method_NORMAL_OMEGA, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);
    DM_Chassis_RB.Init(&hfdcan1, DM_CHASSIS_RB_MASTER_ID, DM_CHASSIS_RB_MOTOR_ID, Motor_DM_Control_Method_NORMAL_OMEGA, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);

    DM_Chassis_LF.Set_Control_Omega(0.0f);
    DM_Chassis_RF.Set_Control_Omega(0.0f);
    DM_Chassis_LB.Set_Control_Omega(0.0f);
    DM_Chassis_RB.Set_Control_Omega(0.0f);
}

/**
 * @brief 初始化舵轮底盘舵向电机
 * @note 当前默认使用舵轮底盘
 */
static void Motor_Swerve_Init()
{
    DM_Chassis_LF.Init(&hfdcan1, DM_CHASSIS_LF_MASTER_ID, DM_CHASSIS_LF_MOTOR_ID, Motor_DM_Control_Method_NORMAL_MIT, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);
    DM_Chassis_RF.Init(&hfdcan1, DM_CHASSIS_RF_MASTER_ID, DM_CHASSIS_RF_MOTOR_ID, Motor_DM_Control_Method_NORMAL_MIT, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);
    DM_Chassis_LB.Init(&hfdcan1, DM_CHASSIS_LB_MASTER_ID, DM_CHASSIS_LB_MOTOR_ID, Motor_DM_Control_Method_NORMAL_MIT, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);
    DM_Chassis_RB.Init(&hfdcan1, DM_CHASSIS_RB_MASTER_ID, DM_CHASSIS_RB_MOTOR_ID, Motor_DM_Control_Method_NORMAL_MIT, DM_Parameter.Angle_Max, DM_Parameter.Omega_Max, DM_Parameter.Torque_Max);

    DM_Chassis_LF.Set_Control_Angle(0.0f);
    DM_Chassis_RF.Set_Control_Angle(0.0f);
    DM_Chassis_LB.Set_Control_Angle(0.0f);
    DM_Chassis_RB.Set_Control_Angle(0.0f);
    DM_Chassis_LF.Set_Control_Omega(0.0f);
    DM_Chassis_RF.Set_Control_Omega(0.0f);
    DM_Chassis_LB.Set_Control_Omega(0.0f);
    DM_Chassis_RB.Set_Control_Omega(0.0f);
    DM_Chassis_LF.Set_Control_Torque(DM_Parameter.Torque_Feedforward);
    DM_Chassis_RF.Set_Control_Torque(DM_Parameter.Torque_Feedforward);
    DM_Chassis_LB.Set_Control_Torque(DM_Parameter.Torque_Feedforward);
    DM_Chassis_RB.Set_Control_Torque(DM_Parameter.Torque_Feedforward);
    DM_Chassis_LF.Set_K_P(DM_Parameter.K_P);
    DM_Chassis_RF.Set_K_P(DM_Parameter.K_P);
    DM_Chassis_LB.Set_K_P(DM_Parameter.K_P);
    DM_Chassis_RB.Set_K_P(DM_Parameter.K_P);
    DM_Chassis_LF.Set_K_D(DM_Parameter.K_D);
    DM_Chassis_RF.Set_K_D(DM_Parameter.K_D);
    DM_Chassis_LB.Set_K_D(DM_Parameter.K_D);
    DM_Chassis_RB.Set_K_D(DM_Parameter.K_D);
}

/**
 * @brief 初始化舵轮底盘行进电机VESC
 */
static void Motor_VESC_Init()
{
    VESC_Chassis_LF.Init(&hfdcan3, VESC_CHASSIS_LF_ID, VESC_Parameter.Pole_Pairs, VESC_Parameter.Gearbox_Rate, VESC_Parameter.ERPM_Max, VESC_Parameter.Current_Max, VESC_Parameter.Current_Brake_Max);
    VESC_Chassis_RF.Init(&hfdcan3, VESC_CHASSIS_RF_ID, VESC_Parameter.Pole_Pairs, VESC_Parameter.Gearbox_Rate, VESC_Parameter.ERPM_Max, VESC_Parameter.Current_Max, VESC_Parameter.Current_Brake_Max);
    VESC_Chassis_LB.Init(&hfdcan3, VESC_CHASSIS_LB_ID, VESC_Parameter.Pole_Pairs, VESC_Parameter.Gearbox_Rate, VESC_Parameter.ERPM_Max, VESC_Parameter.Current_Max, VESC_Parameter.Current_Brake_Max);
    VESC_Chassis_RB.Init(&hfdcan3, VESC_CHASSIS_RB_ID, VESC_Parameter.Pole_Pairs, VESC_Parameter.Gearbox_Rate, VESC_Parameter.ERPM_Max, VESC_Parameter.Current_Max, VESC_Parameter.Current_Brake_Max);

    VESC_Chassis_LF.Set_Motor_Enable(false);
    VESC_Chassis_RF.Set_Motor_Enable(false);
    VESC_Chassis_LB.Set_Motor_Enable(false);
    VESC_Chassis_RB.Set_Motor_Enable(false);
}
