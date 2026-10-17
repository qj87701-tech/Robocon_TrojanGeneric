/**
 * @file mod_chassis_swerve.h
 * @brief 四舵轮底盘速度解算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

#ifndef MOD_CHASSIS_SWERVE_H
#define MOD_CHASSIS_SWERVE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 舵轮轮组编号
 */
enum Enum_Swerve_Wheel : uint8_t
{
    Swerve_Wheel_RF = 0,
    Swerve_Wheel_LF,
    Swerve_Wheel_RB,
    Swerve_Wheel_LB,
    Swerve_Wheel_Number,
};

/**
 * @brief 四舵轮底盘解算类
 */
class Class_Chassis_Swerve
{
public:
    void Init(float Wheel_Radius, float Chassis_Radius, float Steer_Angle_Limit = 12.0f);

    inline void Set_Speed_X(float Speed_X);
    inline void Set_Speed_Y(float Speed_Y);
    inline void Set_Speed_W(float Speed_W);
    inline void Set_Now_Angle(Enum_Swerve_Wheel Wheel, float Angle);
    inline void Set_Wheel_Omega_Max(float Wheel_Omega_Max);

    void Reset_Wheel(Enum_Swerve_Wheel Wheel, float Hold_Angle);

    inline float Get_Target_Angle(Enum_Swerve_Wheel Wheel) const;
    inline float Get_Wheel_Omega(Enum_Swerve_Wheel Wheel) const;
    float Get_Steer_Angle_Error(Enum_Swerve_Wheel Wheel) const;

    void Calculate();

private:
    /**
     * @brief 底盘机械参数
     */
    struct Struct_Chassis_Swerve_Parameter
    {
        float Wheel_Radius = 0.0f;   // 行进轮半径, m
        float Chassis_Radius = 0.0f; // 轮组投影点到底盘中心的距离, m
        float Wheel_Omega_Max = 0.0f; // 行进轮最大机械角速度, rad/s, 0表示不限制
        float Steer_Angle_Limit = 0.0f; // 舵向目标软限位, rad, 0表示不限制
    } Parameter;

    /**
     * @brief 底盘目标速度
     */
    struct Struct_Chassis_Swerve_Velocity
    {
        float Speed_X = 0.0f; // 向右为正, m/s
        float Speed_Y = 0.0f; // 向前为正, m/s
        float Speed_W = 0.0f; // 逆时针为正, rad/s
    } Velocity;

    /**
     * @brief 单个舵轮轮组状态
     */
    struct Struct_Chassis_Swerve_Wheel
    {
        float Now_Angle = 0.0f;
        float Last_Target_Angle = 0.0f;
        float Target_Angle = 0.0f;
        float Wheel_Omega = 0.0f;
        bool Now_Angle_Initialized = false;
        bool Motion_Target_Initialized = false;
        bool Motion_Command_Active = false;
        bool Drive_Reversed = false;
    } Wheel[Swerve_Wheel_Number];
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设置底盘X轴目标速度
 *
 * @param Speed_X X轴目标速度, 向右为正, m/s
 */
inline void Class_Chassis_Swerve::Set_Speed_X(float Speed_X)
{
    Velocity.Speed_X = Speed_X;
}

/**
 * @brief 设置底盘Y轴目标速度
 *
 * @param Speed_Y Y轴目标速度, 向前为正, m/s
 */
inline void Class_Chassis_Swerve::Set_Speed_Y(float Speed_Y)
{
    Velocity.Speed_Y = Speed_Y;
}

/**
 * @brief 设置底盘旋转目标速度
 *
 * @param Speed_W 旋转目标角速度, 逆时针为正, rad/s
 */
inline void Class_Chassis_Swerve::Set_Speed_W(float Speed_W)
{
    Velocity.Speed_W = Speed_W;
}

/**
 * @brief 设置舵向电机当前角度
 *
 * @param Wheel_ID 舵轮轮组编号
 * @param Angle 舵向电机当前角度, rad
 */
inline void Class_Chassis_Swerve::Set_Now_Angle(Enum_Swerve_Wheel Wheel_ID, float Angle)
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return;
    }

    Wheel[Wheel_ID].Now_Angle = Angle;
    if (!Wheel[Wheel_ID].Now_Angle_Initialized)
    {
        Wheel[Wheel_ID].Last_Target_Angle = Angle;
        Wheel[Wheel_ID].Target_Angle = Angle;
        Wheel[Wheel_ID].Now_Angle_Initialized = true;
    }
}

/**
 * @brief 设置行进轮最大机械角速度
 *
 * @param Wheel_Omega_Max 最大机械角速度, rad/s, 小于等于0表示不限制
 */
inline void Class_Chassis_Swerve::Set_Wheel_Omega_Max(float Wheel_Omega_Max)
{
    Parameter.Wheel_Omega_Max = Wheel_Omega_Max;
}

/**
 * @brief 获取舵向电机目标角度
 *
 * @param Wheel_ID 舵轮轮组编号
 * @return float 舵向电机目标角度, rad
 */
inline float Class_Chassis_Swerve::Get_Target_Angle(Enum_Swerve_Wheel Wheel_ID) const
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return 0.0f;
    }

    return Wheel[Wheel_ID].Target_Angle;
}

/**
 * @brief 获取行进轮机械角速度
 *
 * @param Wheel_ID 舵轮轮组编号
 * @return float 行进轮机械角速度, rad/s
 */
inline float Class_Chassis_Swerve::Get_Wheel_Omega(Enum_Swerve_Wheel Wheel_ID) const
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return 0.0f;
    }

    return Wheel[Wheel_ID].Wheel_Omega;
}

#endif
