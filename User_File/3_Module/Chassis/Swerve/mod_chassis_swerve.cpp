/**
 * @file mod_chassis_swerve.cpp
 * @brief 四舵轮底盘速度解算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "mod_chassis_swerve.h"

#include <math.h>

/* Private macros ------------------------------------------------------------*/

#define SWERVE_PI 3.14159265358979323846f
#define SWERVE_2_PI (2.0f * SWERVE_PI)
#define SWERVE_SQRT_2_DIV_2 0.70710678118f
#define SWERVE_REVERSE_HYSTERESIS_ANGLE (10.0f * SWERVE_PI / 180.0f)

// 静止轮组超过此速度才开始更新舵向，避免摇杆刚离开死区时目标角抖动, m/s
#define SWERVE_MOTION_ENTER_SPEED 0.10f
// 运动轮组低于此速度后停止更新舵向，和进入阈值组成迟滞, m/s
#define SWERVE_MOTION_EXIT_SPEED 0.05f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

static bool Chassis_Swerve_Get_Nearest_Target(float Base_Angle, float Now_Angle, float Angle_Limit, float *Target_Angle, float *Travel_Angle);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化舵轮底盘参数和轮组状态
 *
 * @param Wheel_Radius 行进轮半径, m
 * @param Chassis_Radius 轮组投影点到底盘中心的距离, m
 * @param Steer_Angle_Limit 舵向目标软限位, rad, 0表示不限制
 */
void Class_Chassis_Swerve::Init(float Wheel_Radius, float Chassis_Radius, float Steer_Angle_Limit)
{
    Parameter.Wheel_Radius = Wheel_Radius;
    Parameter.Chassis_Radius = Chassis_Radius;
    Parameter.Wheel_Omega_Max = 0.0f;
    Parameter.Steer_Angle_Limit = fabsf(Steer_Angle_Limit);

    Velocity.Speed_X = 0.0f;
    Velocity.Speed_Y = 0.0f;
    Velocity.Speed_W = 0.0f;

    for (uint8_t i = 0; i < Swerve_Wheel_Number; i++)
    {
        Wheel[i].Now_Angle = 0.0f;
        Wheel[i].Last_Target_Angle = 0.0f;
        Wheel[i].Target_Angle = 0.0f;
        Wheel[i].Wheel_Omega = 0.0f;
        Wheel[i].Now_Angle_Initialized = false;
        Wheel[i].Motion_Target_Initialized = false;
        Wheel[i].Motion_Command_Active = false;
        Wheel[i].Drive_Reversed = false;
    }
}

/**
 * @brief 重置单个轮组并锁定指定舵向目标
 *
 * @param Wheel 舵轮轮组编号
 * @param Hold_Angle 需要保持的舵轮机构连续角度, rad
 * @note 当前反馈角度应先通过Set_Now_Angle()更新, 本函数不用目标角覆盖真实反馈
 */
void Class_Chassis_Swerve::Reset_Wheel(Enum_Swerve_Wheel Wheel_ID, float Hold_Angle)
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return;
    }

    Wheel[Wheel_ID].Last_Target_Angle = Hold_Angle;
    Wheel[Wheel_ID].Target_Angle = Hold_Angle;
    Wheel[Wheel_ID].Wheel_Omega = 0.0f;
    Wheel[Wheel_ID].Motion_Target_Initialized = true;
    Wheel[Wheel_ID].Motion_Command_Active = false;
    Wheel[Wheel_ID].Drive_Reversed = false;
}

/**
 * @brief 在舵向软限位内寻找距离当前位置最近的等效角度
 *
 * @param Base_Angle 单圈目标角度, rad
 * @param Now_Angle 当前连续角度, rad
 * @param Angle_Limit 舵向目标软限位, rad, 0表示不限制
 * @param Target_Angle 输出的连续目标角度, rad
 * @param Travel_Angle 输出需要转过的角度绝对值, rad
 * @return bool 是否找到可用目标
 */
static bool Chassis_Swerve_Get_Nearest_Target(float Base_Angle, float Now_Angle, float Angle_Limit, float *Target_Angle, float *Travel_Angle)
{
    if (Angle_Limit <= 0.0f)
    {
        float angle_error = fmodf(Base_Angle - Now_Angle, SWERVE_2_PI);
        if (angle_error <= -SWERVE_PI)
        {
            angle_error += SWERVE_2_PI;
        }
        else if (angle_error > SWERVE_PI)
        {
            angle_error -= SWERVE_2_PI;
        }
        *Target_Angle = Now_Angle + angle_error;
        *Travel_Angle = fabsf(angle_error);
        return (true);
    }

    int32_t turn_min = (int32_t)ceilf((-Angle_Limit - Base_Angle) / SWERVE_2_PI);
    int32_t turn_max = (int32_t)floorf((Angle_Limit - Base_Angle) / SWERVE_2_PI);
    if (turn_min > turn_max)
    {
        return (false);
    }

    int32_t turn = (int32_t)roundf((Now_Angle - Base_Angle) / SWERVE_2_PI);
    if (turn < turn_min)
    {
        turn = turn_min;
    }
    else if (turn > turn_max)
    {
        turn = turn_max;
    }

    *Target_Angle = Base_Angle + (float)turn * SWERVE_2_PI;
    *Travel_Angle = fabsf(*Target_Angle - Now_Angle);
    return (true);
}

/**
 * @brief 将底盘目标速度解算为四个轮组的速度和舵向角度
 */
void Class_Chassis_Swerve::Calculate()
{
    if (Parameter.Wheel_Radius <= 0.0f)
    {
        for (uint8_t i = 0; i < Swerve_Wheel_Number; i++)
        {
            Wheel[i].Target_Angle = Wheel[i].Last_Target_Angle;
            Wheel[i].Wheel_Omega = 0.0f;
        }
        return;
    }

    float position = Parameter.Chassis_Radius * SWERVE_SQRT_2_DIV_2;
    float position_x[Swerve_Wheel_Number] = {position, -position, position, -position};
    float position_y[Swerve_Wheel_Number] = {position, position, -position, -position};

    for (uint8_t i = 0; i < Swerve_Wheel_Number; i++)
    {
        float wheel_speed_x = Velocity.Speed_X - Velocity.Speed_W * position_y[i];
        float wheel_speed_y = Velocity.Speed_Y + Velocity.Speed_W * position_x[i];
        float wheel_speed = sqrtf(wheel_speed_x * wheel_speed_x + wheel_speed_y * wheel_speed_y);

        // 轮组速度使用启停迟滞，避免摇杆刚离开死区时atan2目标角反复跳变
        if (Wheel[i].Motion_Command_Active)
        {
            if (wheel_speed < SWERVE_MOTION_EXIT_SPEED)
            {
                Wheel[i].Motion_Command_Active = false;
            }
        }
        else if (wheel_speed > SWERVE_MOTION_ENTER_SPEED)
        {
            Wheel[i].Motion_Command_Active = true;
        }

        if (!Wheel[i].Motion_Command_Active)
        {
            if (!Wheel[i].Motion_Target_Initialized && Wheel[i].Now_Angle_Initialized)
            {
                Wheel[i].Last_Target_Angle = Wheel[i].Now_Angle;
            }
            Wheel[i].Target_Angle = Wheel[i].Last_Target_Angle;
            Wheel[i].Wheel_Omega = 0.0f;
            continue;
        }

        float target_angle = atan2f(wheel_speed_x, wheel_speed_y);
        Wheel[i].Wheel_Omega = wheel_speed / Parameter.Wheel_Radius;

        float normal_target = 0.0f;
        float normal_travel = 0.0f;
        float reversed_target = 0.0f;
        float reversed_travel = 0.0f;
        bool normal_valid = Chassis_Swerve_Get_Nearest_Target(target_angle, Wheel[i].Now_Angle, Parameter.Steer_Angle_Limit, &normal_target, &normal_travel);
        bool reversed_valid = Chassis_Swerve_Get_Nearest_Target(target_angle + SWERVE_PI, Wheel[i].Now_Angle, Parameter.Steer_Angle_Limit, &reversed_target, &reversed_travel);

        if (!normal_valid && !reversed_valid)
        {
            Wheel[i].Target_Angle = Wheel[i].Last_Target_Angle;
            Wheel[i].Wheel_Omega = 0.0f;
            continue;
        }

        // 正反两组角度都表示同一轮组速度方向，保留10°迟滞避免90°附近反复切换
        if (Wheel[i].Drive_Reversed)
        {
            if (!reversed_valid || (normal_valid && normal_travel + SWERVE_REVERSE_HYSTERESIS_ANGLE < reversed_travel))
            {
                Wheel[i].Drive_Reversed = false;
            }
        }
        else if (!normal_valid || (reversed_valid && reversed_travel + SWERVE_REVERSE_HYSTERESIS_ANGLE < normal_travel))
        {
            Wheel[i].Drive_Reversed = true;
        }

        if (Wheel[i].Drive_Reversed)
        {
            Wheel[i].Target_Angle = reversed_target;
            Wheel[i].Wheel_Omega = -Wheel[i].Wheel_Omega;
        }
        else
        {
            Wheel[i].Target_Angle = normal_target;
        }

        Wheel[i].Last_Target_Angle = Wheel[i].Target_Angle;
        Wheel[i].Motion_Target_Initialized = true;
    }

    if (Parameter.Wheel_Omega_Max > 0.0f)
    {
        float wheel_omega_abs_max = 0.0f;
        for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
        {
            float wheel_omega_abs = fabsf(Wheel[i].Wheel_Omega);
            if (wheel_omega_abs > wheel_omega_abs_max)
            {
                wheel_omega_abs_max = wheel_omega_abs;
            }
        }

        if (wheel_omega_abs_max > Parameter.Wheel_Omega_Max)
        {
            float scale = Parameter.Wheel_Omega_Max / wheel_omega_abs_max;
            for (uint8_t i = 0U; i < Swerve_Wheel_Number; i++)
            {
                Wheel[i].Wheel_Omega *= scale;
            }
        }
    }
}

/**
 * @brief 获取舵向目标与当前角度之间的最短角度误差
 *
 * @param Wheel_ID 舵轮轮组编号
 * @return float 舵向角度误差, rad, 范围(-PI, PI]
 */
float Class_Chassis_Swerve::Get_Steer_Angle_Error(Enum_Swerve_Wheel Wheel_ID) const
{
    if (Wheel_ID >= Swerve_Wheel_Number)
    {
        return (0.0f);
    }

    float angle_error = fmodf(Wheel[Wheel_ID].Target_Angle - Wheel[Wheel_ID].Now_Angle, SWERVE_2_PI);
    if (angle_error <= -SWERVE_PI)
    {
        angle_error += SWERVE_2_PI;
    }
    else if (angle_error > SWERVE_PI)
    {
        angle_error -= SWERVE_2_PI;
    }
    return (angle_error);
}
