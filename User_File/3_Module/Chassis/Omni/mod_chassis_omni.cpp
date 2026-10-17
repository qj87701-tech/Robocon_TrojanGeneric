/**
 * @file mod_chassis_omni.cpp
 * @brief 四轮全向轮底盘速度解算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "mod_chassis_omni.h"

/* Private macros ------------------------------------------------------------*/

#define OMNI_SQRT_2_DIV_2 0.70710678118f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化全向轮底盘参数
 *
 * @param Wheel_Radius 轮子半径, m
 * @param Chassis_Radius 轮组投影点到底盘中心的距离, m
 */
void Class_Chassis_Omni::Init(float Wheel_Radius, float Chassis_Radius)
{
    Parameter.Wheel_Radius = Wheel_Radius;
    Parameter.Chassis_Radius = Chassis_Radius;

    Velocity.Speed_X = 0.0f;
    Velocity.Speed_Y = 0.0f;
    Velocity.Speed_W = 0.0f;

    Wheel.Omega_RF = 0.0f;
    Wheel.Omega_LF = 0.0f;
    Wheel.Omega_RB = 0.0f;
    Wheel.Omega_LB = 0.0f;
}

/**
 * @brief 将底盘目标速度解算为四个轮子的机械角速度
 */
void Class_Chassis_Omni::Calculate()
{
    if (Parameter.Wheel_Radius <= 0.0f)
    {
        Wheel.Omega_RF = 0.0f;
        Wheel.Omega_LF = 0.0f;
        Wheel.Omega_RB = 0.0f;
        Wheel.Omega_LB = 0.0f;
        return;
    }

    float speed_x = Velocity.Speed_X;
    float speed_y = Velocity.Speed_Y;
    float speed_w = Velocity.Speed_W;
    float rotate_speed = speed_w * Parameter.Chassis_Radius;

    Wheel.Omega_RF = (-OMNI_SQRT_2_DIV_2 * speed_x + OMNI_SQRT_2_DIV_2 * speed_y + rotate_speed) / Parameter.Wheel_Radius;
    Wheel.Omega_LF = (-OMNI_SQRT_2_DIV_2 * speed_x - OMNI_SQRT_2_DIV_2 * speed_y + rotate_speed) / Parameter.Wheel_Radius;
    Wheel.Omega_RB = (OMNI_SQRT_2_DIV_2 * speed_x + OMNI_SQRT_2_DIV_2 * speed_y + rotate_speed) / Parameter.Wheel_Radius;
    Wheel.Omega_LB = (OMNI_SQRT_2_DIV_2 * speed_x - OMNI_SQRT_2_DIV_2 * speed_y + rotate_speed) / Parameter.Wheel_Radius;
}

