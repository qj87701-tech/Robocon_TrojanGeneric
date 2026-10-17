/**
 * @file motor_init.h
 * @brief 电机对象和初始化配置
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

#ifndef MOTOR_INIT_H
#define MOTOR_INIT_H

/* Includes ------------------------------------------------------------------*/

#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Motor/Motor_VESC/dvc_motor_vesc.h"

/* Exported macros -----------------------------------------------------------*/

// 达妙J8006反馈报文ID
#define DM_J8006_MASTER_ID 0x55

// 底盘达妙DM6220反馈报文ID
#define DM_CHASSIS_LF_MASTER_ID 0x11
#define DM_CHASSIS_RF_MASTER_ID 0x22
#define DM_CHASSIS_LB_MASTER_ID 0x33
#define DM_CHASSIS_RB_MASTER_ID 0x44

// 底盘达妙DM6220电机ID
#define DM_CHASSIS_LF_MOTOR_ID 0x01
#define DM_CHASSIS_RF_MOTOR_ID 0x02
#define DM_CHASSIS_LB_MOTOR_ID 0x03
#define DM_CHASSIS_RB_MOTOR_ID 0x04

// DM6220 MIT协议量程
#define DM_CHASSIS_ANGLE_MAX 251.3f
#define DM_CHASSIS_OMEGA_MAX 45.0f
#define DM_CHASSIS_TORQUE_MAX 10.0f

// DM6220 MIT舵向初始参数，后续可通过USART9在RAM中调整
#define DM_CHASSIS_MIT_K_P_DEFAULT 7.0f
#define DM_CHASSIS_MIT_K_D_DEFAULT 0.20f
#define DM_CHASSIS_MIT_TORQUE_DEFAULT 0.0f

// 底盘VESC节点ID
#define VESC_CHASSIS_LF_ID 1U
#define VESC_CHASSIS_RF_ID 2U
#define VESC_CHASSIS_LB_ID 3U
#define VESC_CHASSIS_RB_ID 4U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief J8006初始化配置, CAN收发ID及协议量程
 */
struct Struct_Motor_J8006_Init_Parameter
{
    uint8_t Master_ID = DM_J8006_MASTER_ID;  // 反馈ID
    uint8_t Motor_ID = 0x05U;                // 控制ID
    float Angle_Max = 12.5f;                 // 位置协议范围, ±rad
    float Omega_Max = 45.0f;                 // 速度协议范围, ±rad/s
    float Torque_Max = 20.0f;                // 转矩协议范围, ±N·m
};

/**
 * @brief 四个DM6220共用的MIT协议量程和初值, 轮组ID沿用上方定义
 */
struct Struct_Motor_DM_Init_Parameter
{
    float Angle_Max = DM_CHASSIS_ANGLE_MAX;                    // 位置协议范围, ±rad
    float Omega_Max = DM_CHASSIS_OMEGA_MAX;                    // 速度协议范围, ±rad/s
    float Torque_Max = DM_CHASSIS_TORQUE_MAX;                  // 转矩协议范围, ±N·m
    float K_P = DM_CHASSIS_MIT_K_P_DEFAULT;                    // 初始位置增益
    float K_D = DM_CHASSIS_MIT_K_D_DEFAULT;                    // 初始速度阻尼
    float Torque_Feedforward = DM_CHASSIS_MIT_TORQUE_DEFAULT;  // 初始前馈转矩, N·m
};

/**
 * @brief U8 KV100的STM32侧VESC驱动初始化配置, 不改写VESC内部Flash参数
 * @note 21极对, 极速档1440机械rpm对应30240ERPM
 */
struct Struct_Motor_VESC_Init_Parameter
{
    uint8_t Pole_Pairs = 21U;        // 电机极对数
    float Gearbox_Rate = 1.0f;       // 驱动内转速换算比
    int32_t ERPM_Max = 30240;        // STM32侧目标电转速限幅, ERPM
    float Current_Max = 5.0f;        // STM32侧电流命令限幅, A
    float Current_Brake_Max = 5.0f;  // STM32侧制动电流命令限幅, A
};

/* Exported variables --------------------------------------------------------*/

// 达妙J8006电机
extern Class_Motor_DM_Normal DM_J8006;

// 底盘达妙DM6220舵向电机
extern Class_Motor_DM_Normal DM_Chassis_LF;
extern Class_Motor_DM_Normal DM_Chassis_RF;
extern Class_Motor_DM_Normal DM_Chassis_LB;
extern Class_Motor_DM_Normal DM_Chassis_RB;

// 舵轮底盘行进电机VESC
extern Class_Motor_VESC VESC_Chassis_LF;
extern Class_Motor_VESC VESC_Chassis_RF;
extern Class_Motor_VESC VESC_Chassis_LB;
extern Class_Motor_VESC VESC_Chassis_RB;

/* Exported function declarations --------------------------------------------*/

void Motor_Init();

#endif
