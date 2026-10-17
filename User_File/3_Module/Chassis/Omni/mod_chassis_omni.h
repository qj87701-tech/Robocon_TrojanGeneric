/**
 * @file mod_chassis_omni.h
 * @brief 四轮全向轮底盘速度解算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

#ifndef MOD_CHASSIS_OMNI_H
#define MOD_CHASSIS_OMNI_H

/* Includes ------------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 四轮全向轮底盘解算类
 */
class Class_Chassis_Omni
{
public:
    void Init(float Wheel_Radius, float Chassis_Radius);

    inline void Set_Speed_X(float Speed_X);
    inline void Set_Speed_Y(float Speed_Y);
    inline void Set_Speed_W(float Speed_W);

    inline float Get_Wheel_Omega_RF() const;
    inline float Get_Wheel_Omega_LF() const;
    inline float Get_Wheel_Omega_RB() const;
    inline float Get_Wheel_Omega_LB() const;

    void Calculate();

private:
    /**
     * @brief 底盘机械参数
     */
    struct Struct_Chassis_Omni_Parameter
    {
        float Wheel_Radius = 0.0f;   // 轮子半径, m
        float Chassis_Radius = 0.0f; // 轮组投影点到底盘中心的距离, m
    } Parameter;

    /**
     * @brief 底盘目标速度
     */
    struct Struct_Chassis_Omni_Velocity
    {
        float Speed_X = 0.0f; // 向右为正, m/s
        float Speed_Y = 0.0f; // 向前为正, m/s
        float Speed_W = 0.0f; // 逆时针为正, rad/s
    } Velocity;

    /**
     * @brief 四个轮子的机械角速度
     */
    struct Struct_Chassis_Omni_Wheel
    {
        float Omega_RF = 0.0f;
        float Omega_LF = 0.0f;
        float Omega_RB = 0.0f;
        float Omega_LB = 0.0f;
    } Wheel;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设置底盘X轴目标速度
 *
 * @param Speed_X X轴目标速度, 向右为正, m/s
 */
inline void Class_Chassis_Omni::Set_Speed_X(float Speed_X)
{
    Velocity.Speed_X = Speed_X;
}

/**
 * @brief 设置底盘Y轴目标速度
 *
 * @param Speed_Y Y轴目标速度, 向前为正, m/s
 */
inline void Class_Chassis_Omni::Set_Speed_Y(float Speed_Y)
{
    Velocity.Speed_Y = Speed_Y;
}

/**
 * @brief 设置底盘旋转目标速度
 *
 * @param Speed_W 旋转目标角速度, 逆时针为正, rad/s
 */
inline void Class_Chassis_Omni::Set_Speed_W(float Speed_W)
{
    Velocity.Speed_W = Speed_W;
}

/**
 * @brief 获取右前轮机械角速度
 *
 * @return float 右前轮机械角速度, rad/s
 */
inline float Class_Chassis_Omni::Get_Wheel_Omega_RF() const
{
    return Wheel.Omega_RF;
}

/**
 * @brief 获取左前轮机械角速度
 *
 * @return float 左前轮机械角速度, rad/s
 */
inline float Class_Chassis_Omni::Get_Wheel_Omega_LF() const
{
    return Wheel.Omega_LF;
}

/**
 * @brief 获取右后轮机械角速度
 *
 * @return float 右后轮机械角速度, rad/s
 */
inline float Class_Chassis_Omni::Get_Wheel_Omega_RB() const
{
    return Wheel.Omega_RB;
}

/**
 * @brief 获取左后轮机械角速度
 *
 * @return float 左后轮机械角速度, rad/s
 */
inline float Class_Chassis_Omni::Get_Wheel_Omega_LB() const
{
    return Wheel.Omega_LB;
}

#endif

