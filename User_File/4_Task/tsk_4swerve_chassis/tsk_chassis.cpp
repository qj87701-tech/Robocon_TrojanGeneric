/**
 * @file tsk_chassis.cpp
 * @brief 底盘控制任务
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_chassis.h"

#include "4_Task/tsk_remote/tsk_remote.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 生产调度只使用此控制对象, 不在头文件中定义全局实例
static Class_Chassis_Control Chassis_Control;

/* Exported variables --------------------------------------------------------*/

// 对外兼容的诊断快照, 不替代类内部的控制状态
Struct_Chassis_Swerve_Diagnostic Chassis_Swerve_Diagnostic[Swerve_Wheel_Number] = {};

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


/**
 * @brief 初始化底盘控制
 */
void Class_Chassis_Control::Init()
{
    Chassis_Omni.Init(Chassis_Omni_Config.Wheel_Radius, Chassis_Omni_Config.Chassis_Radius);
    Chassis_Swerve.Init(Chassis_Swerve_Config.Wheel_Radius, Chassis_Swerve_Config.Chassis_Radius, Chassis_Swerve_Config.Steer_Angle_Limit);
    Chassis_Swerve.Set_Wheel_Omega_Max(Chassis_Swerve_Config.Wheel_Omega_Max);
    Swerve_Reset_Drive_Omega();
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Chassis_Swerve_Diagnostic[i] = {};
        Swerve_Wheel_Control[i] = {};
        Swerve_MIT_Parameter[i] = {DM_CHASSIS_MIT_K_P_DEFAULT, DM_CHASSIS_MIT_K_D_DEFAULT, DM_CHASSIS_MIT_TORQUE_DEFAULT};
    }
}

/**
 * @brief 获取轮组对应的DM6220对象
 *
 * @param Wheel 舵轮轮组编号
 * @return Class_Motor_DM_Normal* 对应舵向电机
 */
Class_Motor_DM_Normal *Class_Chassis_Control::Swerve_Get_Steer_Motor(Enum_Swerve_Wheel Wheel)
{
    return (Wheel < Swerve_Wheel_Number) ? Swerve_Wheel_Config[Wheel].Steer_Motor : nullptr;
}

/**
 * @brief 获取轮组对应的VESC对象
 */
Class_Motor_VESC *Class_Chassis_Control::Swerve_Get_Drive_Motor(Enum_Swerve_Wheel Wheel)
{
    return (Wheel < Swerve_Wheel_Number) ? Swerve_Wheel_Config[Wheel].Drive_Motor : nullptr;
}

/**
 * @brief 判断轮组是否已接入本轮硬件测试
 */
bool Class_Chassis_Control::Swerve_Is_Wheel_Active(Enum_Swerve_Wheel Wheel)
{
    return ((Chassis_Swerve_Config.Active_Wheel_Mask & (1U << static_cast<uint8_t>(Wheel))) != 0U);
}

/**
 * @brief 将DM连续电机轴角度换算为舵轮机构连续角度
 */
float Class_Chassis_Control::Swerve_Motor_To_Mechanism_Angle(Enum_Swerve_Wheel Wheel, float Motor_Angle)
{
    return (Motor_Angle * Swerve_Wheel_Config[Wheel].Steer_Direction - Swerve_Wheel_Config[Wheel].Steer_Offset);
}

/**
 * @brief 将舵轮机构目标角度换算为DM多圈电机轴目标角度
 */
float Class_Chassis_Control::Swerve_Mechanism_To_Motor_Angle(Enum_Swerve_Wheel Wheel, float Mechanism_Angle)
{
    return ((Mechanism_Angle + Swerve_Wheel_Config[Wheel].Steer_Offset) * Swerve_Wheel_Config[Wheel].Steer_Direction);
}

/**
 * @brief 将DM电机轴角速度换算为舵轮机构角速度
 */
float Class_Chassis_Control::Swerve_Motor_To_Mechanism_Omega(Enum_Swerve_Wheel Wheel, float Motor_Omega)
{
    return (Motor_Omega * Swerve_Wheel_Config[Wheel].Steer_Direction);
}

/**
 * @brief 设置单个DM6220的MIT位置增益
 *
 * @param Wheel 舵轮轮组编号
 * @param K_P MIT位置增益, 0.0~50.0
 * @return true 参数已保存到RAM
 * @return false 轮组或参数越界
 */
bool Class_Chassis_Control::Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P)
{
    if (Wheel >= Swerve_Wheel_Number || K_P < Chassis_Swerve_Config.MIT_Limit.K_P_Min || K_P > Chassis_Swerve_Config.MIT_Limit.K_P_Max)
    {
        return (false);
    }
    Swerve_MIT_Parameter[Wheel].K_P = K_P;
    return (true);
}

/**
 * @brief 设置单个DM6220的MIT速度阻尼增益
 *
 * @param Wheel 舵轮轮组编号
 * @param K_D MIT速度阻尼增益, 0.0~5.0
 * @return true 参数已保存到RAM
 * @return false 轮组或参数越界
 */
bool Class_Chassis_Control::Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D)
{
    if (Wheel >= Swerve_Wheel_Number || K_D < Chassis_Swerve_Config.MIT_Limit.K_D_Min || K_D > Chassis_Swerve_Config.MIT_Limit.K_D_Max)
    {
        return (false);
    }
    Swerve_MIT_Parameter[Wheel].K_D = K_D;
    return (true);
}

/**
 * @brief 设置单个DM6220的MIT前馈转矩
 *
 * @param Wheel 舵轮轮组编号
 * @param Torque 前馈转矩, -2.0~2.0N·m
 * @return true 参数已保存到RAM
 * @return false 轮组或参数越界
 */
bool Class_Chassis_Control::Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque)
{
    if (Wheel >= Swerve_Wheel_Number || Torque < Chassis_Swerve_Config.MIT_Limit.Torque_Min || Torque > Chassis_Swerve_Config.MIT_Limit.Torque_Max)
    {
        return (false);
    }
    Swerve_MIT_Parameter[Wheel].Feedforward_Torque = Torque;
    return (true);
}

/**
 * @brief 回零期间持续发送保存零点后的0rad目标
 */
void Class_Chassis_Control::Swerve_Hold_Homing_Target(Enum_Swerve_Wheel Wheel)
{
    Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(Wheel);
    motor->Set_Control_Angle(Swerve_Mechanism_To_Motor_Angle(Wheel, 0.0f));
    motor->Set_Control_Omega(0.0f);
    motor->Set_Control_Torque(0.0f);
    motor->Set_K_P(Swerve_MIT_Parameter[Wheel].K_P);
    motor->Set_K_D(Swerve_MIT_Parameter[Wheel].K_D);
}

/**
 * @brief 更新单个活动舵轮的10ms回零状态机
 *
 * @return true 回零完成, 可以进入正常底盘控制
 */
bool Class_Chassis_Control::Swerve_Update_Homing(Enum_Swerve_Wheel Wheel)
{
    Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(Wheel);
    Struct_Chassis_Swerve_Homing_Control &homing = Swerve_Wheel_Control[Wheel].Homing;

    if (homing.Status == Chassis_Swerve_Homing_Status_DISABLED)
    {
        homing.Feedback_Update_Count = motor->Get_Rx_Update_Count();
        homing.Elapsed_Cycles = 1U;
        homing.Stable_Cycles = 0U;
        homing.Status = Chassis_Swerve_Homing_Status_WAIT_FEEDBACK;
        if (!motor->Get_Motor_Enable())
        {
            motor->CAN_Send_Enter();
        }
        Swerve_Hold_Homing_Target(Wheel);
        return (false);
    }

    if (homing.Status == Chassis_Swerve_Homing_Status_READY)
    {
        return (true);
    }

    if (homing.Elapsed_Cycles < Chassis_Swerve_Config.Homing.Timeout_Cycles)
    {
        homing.Elapsed_Cycles++;
    }

    if (homing.Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK && motor->Get_Rx_Update_Count() != homing.Feedback_Update_Count)
    {
        homing.Status = Chassis_Swerve_Homing_Status_HOMING;
    }

    if (homing.Status == Chassis_Swerve_Homing_Status_HOMING)
    {
        if (fabsf(motor->Get_Now_Total_Angle()) <= Chassis_Swerve_Config.Homing.Position_Limit && fabsf(motor->Get_Now_Omega()) <= Chassis_Swerve_Config.Homing.Omega_Limit)
        {
            homing.Stable_Cycles++;
        }
        else
        {
            homing.Stable_Cycles = 0U;
        }

        if (homing.Stable_Cycles >= Chassis_Swerve_Config.Homing.Stable_Cycles)
        {
            // DM上位机已保存的机械零点始终是0rad, 不得用回零完成时的实际角度重定义零点
            Chassis_Swerve.Reset_Wheel(Wheel, 0.0f);
            homing.Status = Chassis_Swerve_Homing_Status_READY;
            return (true);
        }
    }

    if (homing.Elapsed_Cycles >= Chassis_Swerve_Config.Homing.Timeout_Cycles)
    {
        homing.Status = Chassis_Swerve_Homing_Status_TIMEOUT;
        homing.Stable_Cycles = 0U;
    }

    Swerve_Hold_Homing_Target(Wheel);
    return (false);
}

/**
 * @brief 更新单个舵轮的低频诊断快照
 */
void Class_Chassis_Control::Swerve_Update_Diagnostic(Enum_Swerve_Wheel Wheel)
{
    Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(Wheel);
    Struct_Chassis_Swerve_Diagnostic &diagnostic = Chassis_Swerve_Diagnostic[Wheel];
    diagnostic.Steer_Target_Angle = Chassis_Swerve.Get_Target_Angle(Wheel);
    diagnostic.Steer_Now_Angle = Swerve_Motor_To_Mechanism_Angle(Wheel, motor->Get_Now_Total_Angle());
    diagnostic.Steer_Angle_Error = Chassis_Swerve.Get_Steer_Angle_Error(Wheel);
    diagnostic.Steer_Now_Omega = Swerve_Motor_To_Mechanism_Omega(Wheel, motor->Get_Now_Omega());
    diagnostic.Steer_Target_Omega_Limit = 0.0f;
    diagnostic.Steer_Speed_Ratio = (Swerve_Wheel_Control[Wheel].Homing.Status == Chassis_Swerve_Homing_Status_READY) ? Swerve_Get_Steer_Speed_Ratio(Wheel) : 0.0f;
    diagnostic.Total_Angle_Initialized = motor->Get_Total_Angle_Initialized();
    diagnostic.Steer_Homing_Status = Swerve_Wheel_Control[Wheel].Homing.Status;
    diagnostic.Steer_Homing_Elapsed_Time = static_cast<uint32_t>(Swerve_Wheel_Control[Wheel].Homing.Elapsed_Cycles) * Chassis_Swerve_Config.Control_Period_MS;
    diagnostic.Steer_Target_Enable = motor->Get_Motor_Enable();
}

/**
 * @brief 按10ms周期限制行进轮机械速度的变化量
 *
 * @param Wheel 舵轮轮组编号
 * @param Target_Omega 解算得到的目标机械角速度, rad/s
 * @return float 本周期允许下发的机械角速度, rad/s
 * @note 每次变速直接使用加减速斜率, 无启动计时和加速度累积延迟
 * @note 反转先减至零再反向; 失能直接清零, 不受正常斜坡延迟
 */
float Class_Chassis_Control::Swerve_Update_Drive_Omega(Enum_Swerve_Wheel Wheel, float Target_Omega)
{
    float &control_omega = Swerve_Wheel_Control[Wheel].Drive_Control_Omega;

    if (control_omega * Target_Omega < 0.0f)
    {
        Target_Omega = 0.0f;
    }

    float ramp = (fabsf(Target_Omega) > fabsf(control_omega)) ? Chassis_Swerve_Config.Accel_RPM_Per_S : Chassis_Swerve_Config.Decel_RPM_Per_S;
    float step = ramp * RPM_TO_RAD_S * (Chassis_Swerve_Config.Control_Period_MS / 1000.0f);
    float difference = Target_Omega - control_omega;
    if (fabsf(difference) <= step)
    {
        control_omega = Target_Omega;
    }
    else
    {
        control_omega += (difference > 0.0f) ? step : -step;
    }
    return (control_omega);
}

/**
 * @brief 根据舵向误差限制行进轮速度
 *
 * @param Wheel 舵轮轮组编号
 * @return float 允许的速度比例, 0.0~1.0
 * @note 误差小于5°允许全速, 大于45°停止驱动, 中间线性过渡
 */
float Class_Chassis_Control::Swerve_Get_Steer_Speed_Ratio(Enum_Swerve_Wheel Wheel)
{
    float angle_error = fabsf(Chassis_Swerve.Get_Steer_Angle_Error(Wheel));
    if (angle_error <= Chassis_Swerve_Config.Steer_Full_Speed_Angle)
    {
        return (1.0f);
    }
    if (angle_error >= Chassis_Swerve_Config.Steer_Stop_Angle)
    {
        return (0.0f);
    }
    return ((Chassis_Swerve_Config.Steer_Stop_Angle - angle_error) / (Chassis_Swerve_Config.Steer_Stop_Angle - Chassis_Swerve_Config.Steer_Full_Speed_Angle));
}

/**
 * @brief 清零四个行进轮速度斜坡状态
 */
void Class_Chassis_Control::Swerve_Reset_Drive_Omega()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Swerve_Wheel_Control[i].Drive_Control_Omega = 0.0f;
    }
}

/**
 * @brief 全向轮底盘速度解算和电机控制
 */
void Class_Chassis_Control::Omni_Control()
{
    Chassis_Omni.Set_Speed_X(Remote_Chassis_Command.Speed_X);
    Chassis_Omni.Set_Speed_Y(Remote_Chassis_Command.Speed_Y);
    Chassis_Omni.Set_Speed_W(Remote_Chassis_Command.Speed_W);
    Chassis_Omni.Calculate();

    float motor_ratio = Chassis_Omni_Config.Deceleration_Ratio / Chassis_Omni_Config.Motor_Deceleration_Ratio;
    DM_Chassis_RF.Set_Control_Omega(Chassis_Omni.Get_Wheel_Omega_RF() * motor_ratio * Chassis_Omni_Config.Motor_Direction[Swerve_Wheel_RF]);
    DM_Chassis_LF.Set_Control_Omega(Chassis_Omni.Get_Wheel_Omega_LF() * motor_ratio * Chassis_Omni_Config.Motor_Direction[Swerve_Wheel_LF]);
    DM_Chassis_RB.Set_Control_Omega(Chassis_Omni.Get_Wheel_Omega_RB() * motor_ratio * Chassis_Omni_Config.Motor_Direction[Swerve_Wheel_RB]);
    DM_Chassis_LB.Set_Control_Omega(Chassis_Omni.Get_Wheel_Omega_LB() * motor_ratio * Chassis_Omni_Config.Motor_Direction[Swerve_Wheel_LB]);
}

/**
 * @brief 清零并失能全部底盘电机
 */
void Class_Chassis_Control::Motor_Disable()
{
    Chassis_Omni.Set_Speed_X(0.0f);
    Chassis_Omni.Set_Speed_Y(0.0f);
    Chassis_Omni.Set_Speed_W(0.0f);
    Chassis_Omni.Calculate();

    DM_Chassis_LF.Set_Control_Omega(0.0f);
    DM_Chassis_RF.Set_Control_Omega(0.0f);
    DM_Chassis_LB.Set_Control_Omega(0.0f);
    DM_Chassis_RB.Set_Control_Omega(0.0f);

    if (DM_Chassis_LF.Get_Motor_Enable())
    {
        DM_Chassis_LF.CAN_Send_Exit();
    }
    if (DM_Chassis_RF.Get_Motor_Enable())
    {
        DM_Chassis_RF.CAN_Send_Exit();
    }
    if (DM_Chassis_LB.Get_Motor_Enable())
    {
        DM_Chassis_LB.CAN_Send_Exit();
    }
    if (DM_Chassis_RB.Get_Motor_Enable())
    {
        DM_Chassis_RB.CAN_Send_Exit();
    }
}

/**
 * @brief 发送全部底盘电机控制报文
 */
void Class_Chassis_Control::Motor_Send()
{
    DM_Chassis_LF.TIM_Send_PeriodElapsedCallback();
    DM_Chassis_RF.TIM_Send_PeriodElapsedCallback();
    DM_Chassis_LB.TIM_Send_PeriodElapsedCallback();
    DM_Chassis_RB.TIM_Send_PeriodElapsedCallback();
}

/**
 * @brief 仅向当前已接入的舵向电机发送MIT控制报文
 */
void Class_Chassis_Control::Swerve_Motor_Send()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (Swerve_Is_Wheel_Active(wheel))
        {
            Swerve_Get_Steer_Motor(wheel)->TIM_Send_PeriodElapsedCallback();
        }
    }
}

/**
 * @brief 控制已经完成回零的DM6220舵向电机
 */
void Class_Chassis_Control::Swerve_Steer_Motor_Control()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (!Swerve_Is_Wheel_Active(wheel))
        {
            continue;
        }

        Class_Motor_DM_Normal *motor = Swerve_Get_Steer_Motor(wheel);
        motor->Set_Control_Angle(Swerve_Mechanism_To_Motor_Angle(wheel, Chassis_Swerve.Get_Target_Angle(wheel)));
        motor->Set_Control_Omega(0.0f);
        motor->Set_Control_Torque(Swerve_MIT_Parameter[wheel].Feedforward_Torque);
        motor->Set_K_P(Swerve_MIT_Parameter[wheel].K_P);
        motor->Set_K_D(Swerve_MIT_Parameter[wheel].K_D);
    }
}

/**
 * @brief 控制当前已接入的VESC行进电机
 * @note 在线状态仅用于诊断, 回零完成后才允许非零速度
 * @note 舵向误差达到停止阈值时, 行进轮目标立即按减速斜坡下降
 */
void Class_Chassis_Control::Swerve_Drive_Motor_Control()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (!Swerve_Is_Wheel_Active(wheel))
        {
            continue;
        }

        float drive_direction = Swerve_Wheel_Config[wheel].Drive_Direction;
        float steer_speed_ratio = Swerve_Get_Steer_Speed_Ratio(wheel);
        float target_omega = Chassis_Swerve.Get_Wheel_Omega(wheel) * drive_direction * steer_speed_ratio;
        Class_Motor_VESC *motor = Swerve_Get_Drive_Motor(wheel);
        motor->Set_Control_Omega(Swerve_Update_Drive_Omega(wheel, target_omega));
        motor->Set_Motor_Enable(true);
        Swerve_Update_Diagnostic(wheel);
    }
}

/**
 * @brief 清零并失能当前已接入的舵轮底盘电机
 */
void Class_Chassis_Control::Swerve_Motor_Disable()
{
    Chassis_Swerve.Set_Speed_X(0.0f);
    Chassis_Swerve.Set_Speed_Y(0.0f);
    Chassis_Swerve.Set_Speed_W(0.0f);
    Chassis_Swerve.Calculate();
    Swerve_Reset_Drive_Omega();
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (!Swerve_Is_Wheel_Active(wheel))
        {
            continue;
        }

        Class_Motor_DM_Normal *steer_motor = Swerve_Get_Steer_Motor(wheel);
        Class_Motor_VESC *drive_motor = Swerve_Get_Drive_Motor(wheel);
        steer_motor->Set_Control_Angle(0.0f);
        steer_motor->Set_Control_Omega(0.0f);
        steer_motor->Set_Control_Torque(0.0f);
        drive_motor->Set_Control_Omega(0.0f);
        drive_motor->Set_Motor_Enable(false);
        if (steer_motor->Get_Motor_Enable())
        {
            steer_motor->CAN_Send_Exit();
        }
        Swerve_Wheel_Control[i].Homing = {};
        Chassis_Swerve_Diagnostic[i] = {};
    }
}

/**
 * @brief 舵轮底盘回零、速度解算和电机控制
 * @note 当前10ms生产入口使用本函数
 */
void Class_Chassis_Control::Swerve_Control()
{
    if (Remote_Chassis_Command.Online == 0 || Remote_Chassis_Command.Mode != REMOTE_MODE_MANUAL || Remote_Chassis_Command.Motor_Enable == 0)
    {
        Swerve_Motor_Disable();
        return;
    }

    bool all_active_wheels_ready = true;
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (!Swerve_Is_Wheel_Active(wheel))
        {
            continue;
        }

        Class_Motor_DM_Normal *steer_motor = Swerve_Get_Steer_Motor(wheel);
        Class_Motor_VESC *drive_motor = Swerve_Get_Drive_Motor(wheel);
        Chassis_Swerve.Set_Now_Angle(wheel, Swerve_Motor_To_Mechanism_Angle(wheel, steer_motor->Get_Now_Total_Angle()));
        drive_motor->Set_Motor_Enable(true);
        if (!Swerve_Update_Homing(wheel))
        {
            all_active_wheels_ready = false;
            drive_motor->Set_Control_Omega(0.0f);
        }
        Swerve_Update_Diagnostic(wheel);
    }

    if (!all_active_wheels_ready)
    {
        Swerve_Reset_Drive_Omega();
        return;
    }

    Chassis_Swerve.Set_Speed_X(Remote_Chassis_Command.Speed_X);
    Chassis_Swerve.Set_Speed_Y(Remote_Chassis_Command.Speed_Y);
    Chassis_Swerve.Set_Speed_W(Remote_Chassis_Command.Speed_W);
    Chassis_Swerve.Calculate();

    Swerve_Steer_Motor_Control();
    Swerve_Drive_Motor_Control();
}

/**
 * @brief 每10ms处理一次底盘控制
 */
void Class_Chassis_Control::TIM_10ms_Omni_PeriodElapsedCallback()
{
    if (Remote_Chassis_Command.Online == 0 || Remote_Chassis_Command.Mode != REMOTE_MODE_MANUAL || Remote_Chassis_Command.Motor_Enable == 0)
    {
        Motor_Disable();
        Motor_Send();
        return;
    }

    if (!DM_Chassis_LF.Get_Motor_Enable())
    {
        DM_Chassis_LF.CAN_Send_Enter();
    }
    if (!DM_Chassis_RF.Get_Motor_Enable())
    {
        DM_Chassis_RF.CAN_Send_Enter();
    }
    if (!DM_Chassis_LB.Get_Motor_Enable())
    {
        DM_Chassis_LB.CAN_Send_Enter();
    }
    if (!DM_Chassis_RB.Get_Motor_Enable())
    {
        DM_Chassis_RB.CAN_Send_Enter();
    }

    Omni_Control();
    Motor_Send();
}

/**
 * @brief 每10ms处理一次舵轮底盘控制
 * @note 当前生产入口使用本函数
 */
void Class_Chassis_Control::TIM_10ms_Swerve_PeriodElapsedCallback()
{
    Swerve_Control();
    Swerve_Motor_Send();
}

/**
 * @brief 每10ms发送一次当前已接入VESC的控制报文, 发送频率100Hz
 * @note 在同周期底盘解算后调用, 使用更新后的速度目标
 */
void Class_Chassis_Control::TIM_10ms_VESC_PeriodElapsedCallback()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (Swerve_Is_Wheel_Active(wheel))
        {
            Swerve_Get_Drive_Motor(wheel)->TIM_Send_PeriodElapsedCallback();
        }
    }
}

/**
 * @brief 每100ms检查一次底盘电机在线状态
 */
void Class_Chassis_Control::TIM_100ms_PeriodElapsedCallback()
{
    for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
    {
        Enum_Swerve_Wheel wheel = static_cast<Enum_Swerve_Wheel>(i);
        if (Swerve_Is_Wheel_Active(wheel))
        {
            Swerve_Get_Steer_Motor(wheel)->TIM_100ms_Alive_PeriodElapsedCallback();
            Swerve_Get_Drive_Motor(wheel)->TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
}

/* Task entry wrappers ------------------------------------------------------*/

/**
 * @brief 初始化底盘解算及运行状态, 需先执行Motor_Init, 不在此处使能电机
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_Init()
{
    Chassis_Control.Init();
}

/**
 * @brief 保留的全向轮10ms控制入口, 当前生产调度不使用
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_10ms_PeriodElapsedCallback()
{
    Chassis_Control.TIM_10ms_Omni_PeriodElapsedCallback();
}

/**
 * @brief 舵轮10ms控制入口: 安全检查、回零、解算、控制并发送DM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_Swerve_10ms_PeriodElapsedCallback()
{
    Chassis_Control.TIM_10ms_Swerve_PeriodElapsedCallback();
}

/**
 * @brief 舵轮控制更新后每10ms发送VESC报文, 发送频率100Hz
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_VESC_10ms_PeriodElapsedCallback()
{
    Chassis_Control.TIM_10ms_VESC_PeriodElapsedCallback();
}

/**
 * @brief 每100ms检查当前活动轮组的电机在线状态
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
void Chassis_100ms_PeriodElapsedCallback()
{
    Chassis_Control.TIM_100ms_PeriodElapsedCallback();
}

/**
 * @brief 校验并保存单轮MIT位置增益, 仅修改RAM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
bool Chassis_Swerve_Set_MIT_K_P(Enum_Swerve_Wheel Wheel, float K_P)
{
    return Chassis_Control.Set_MIT_K_P(Wheel, K_P);
}

/**
 * @brief 校验并保存单轮MIT速度阻尼, 仅修改RAM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
bool Chassis_Swerve_Set_MIT_K_D(Enum_Swerve_Wheel Wheel, float K_D)
{
    return Chassis_Control.Set_MIT_K_D(Wheel, K_D);
}

/**
 * @brief 校验并保存单轮MIT前馈转矩, 单位N·m, 仅修改RAM
 * @note 兼容现有裸机任务和调试调用, 实际逻辑由Chassis_Control执行
 */
bool Chassis_Swerve_Set_MIT_Feedforward_Torque(Enum_Swerve_Wheel Wheel, float Torque)
{
    return Chassis_Control.Set_MIT_Feedforward_Torque(Wheel, Torque);
}
