/**
 * @file dvc_motor_vesc.h
 * @brief VESC航模电机CAN驱动
 * @version 0.3
 * @date 2026-10-06 0.1 新建文档
 * @date 2026-10-07 0.2 增加六种VESC CAN控制方式
 * @date 2026-10-08 0.3 增加状态报文独立有效性、更新时间和更新计数
 */

#ifndef DVC_MOTOR_VESC_H
#define DVC_MOTOR_VESC_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/CAN/drv_can.h"
#include <math.h>

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief VESC在线状态
 */
enum Enum_Motor_VESC_Status
{
    Motor_VESC_Status_DISABLE = 0,
    Motor_VESC_Status_ENABLE,
};

/**
 * @brief VESC控制方式
 */
enum Enum_Motor_VESC_Control_Method
{
    // 电气转速, ERPM
    Motor_VESC_Control_Method_RPM = 0,
    // 电机电流, A
    Motor_VESC_Control_Method_CURRENT,
    // 正制动电流, A
    Motor_VESC_Control_Method_CURRENT_BRAKE,
    // 占空比, -1.0~1.0
    Motor_VESC_Control_Method_DUTY,
    // 位置角度, 0~360°
    Motor_VESC_Control_Method_POSITION,
    // 相对电流, -1.0~1.0
    Motor_VESC_Control_Method_CURRENT_REL,
};

/**
 * @brief VESC反馈数据
 */
struct Struct_Motor_VESC_Rx_Data
{
    int32_t Now_ERPM = 0;
    float Now_Omega = 0.0f;
    float Motor_Current = 0.0f;
    float Duty = 0.0f;
    float MOS_Temperature = 0.0f;
    float Motor_Temperature = 0.0f;
    float Input_Current = 0.0f;
    float PID_Position = 0.0f;
    int32_t Tachometer = 0;
    float Input_Voltage = 0.0f;
};

/**
 * @brief 单类VESC状态报文的新鲜度信息
 */
struct Struct_Motor_VESC_Rx_Frame_State
{
    uint32_t Last_Update_Time = 0U;
    uint32_t Update_Count = 0U;
    bool Valid = false;
};

/**
 * @brief VESC航模电机类
 */
class Class_Motor_VESC
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, uint8_t VESC_ID, uint8_t Pole_Pairs, float Gearbox_Rate, int32_t ERPM_Max, float Current_Max = 5.0f, float Current_Brake_Max = 5.0f);

    inline Enum_Motor_VESC_Status Get_Status() const;
    inline Enum_Motor_VESC_Control_Method Get_Control_Method() const;
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
    bool Get_Status_1_Valid() const;
    bool Get_Status_4_Valid() const;
    bool Get_Status_5_Valid() const;
    inline uint32_t Get_Status_1_Last_Update_Time() const;
    inline uint32_t Get_Status_4_Last_Update_Time() const;
    inline uint32_t Get_Status_5_Last_Update_Time() const;
    inline uint32_t Get_Status_1_Update_Count() const;
    inline uint32_t Get_Status_4_Update_Count() const;
    inline uint32_t Get_Status_5_Update_Count() const;
    inline int32_t Get_Control_ERPM() const;
    inline float Get_Control_Omega() const;
    inline float Get_Control_Current() const;
    inline float Get_Control_Current_Brake() const;
    inline float Get_Control_Duty() const;
    inline float Get_Control_Position() const;
    inline float Get_Control_Current_Rel() const;
    inline void Set_Motor_Enable(bool Motor_Enable);
    inline void Set_Control_Method(Enum_Motor_VESC_Control_Method Control_Method);
    inline void Set_Control_Omega(float Control_Omega);
    inline void Set_Control_Current(float Control_Current);
    inline void Set_Control_Current_Brake(float Control_Current_Brake);
    inline void Set_Control_Duty(float Control_Duty);
    inline void Set_Control_Position(float Control_Position);
    inline void Set_Control_Current_Rel(float Control_Current_Rel);

    void CAN_RxCpltCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();
    void TIM_Send_PeriodElapsedCallback();

protected:
    void Data_Process();
    void Output();
    void Reset_Control_Targets();
    bool Get_Rx_Frame_Valid(const Struct_Motor_VESC_Rx_Frame_State &Frame_State) const;

    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;
    uint8_t VESC_ID = 0U;
    uint8_t Pole_Pairs = 0U;
    float Gearbox_Rate = 1.0f;
    int32_t ERPM_Max = 0;
    float Current_Max = 0.0f;
    float Current_Brake_Max = 0.0f;

    bool Motor_Enable = false;
    Enum_Motor_VESC_Status Motor_VESC_Status = Motor_VESC_Status_DISABLE;
    Enum_Motor_VESC_Control_Method Control_Method = Motor_VESC_Control_Method_RPM;

    Struct_Motor_VESC_Rx_Data Rx_Data;
    Struct_Motor_VESC_Rx_Frame_State Status_1_State;
    Struct_Motor_VESC_Rx_Frame_State Status_4_State;
    Struct_Motor_VESC_Rx_Frame_State Status_5_State;
    float Control_Omega = 0.0f;
    int32_t Control_ERPM = 0;
    float Control_Current = 0.0f;
    float Control_Current_Brake = 0.0f;
    float Control_Duty = 0.0f;
    float Control_Position = 0.0f;
    float Control_Current_Rel = 0.0f;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取VESC在线状态
 *
 * @return Enum_Motor_VESC_Status VESC在线状态
 */
inline Enum_Motor_VESC_Status Class_Motor_VESC::Get_Status() const
{
    return (Motor_VESC_Status);
}

/**
 * @brief 获取VESC控制方式
 *
 * @return Enum_Motor_VESC_Control_Method VESC控制方式
 */
inline Enum_Motor_VESC_Control_Method Class_Motor_VESC::Get_Control_Method() const
{
    return (Control_Method);
}

/**
 * @brief 获取电机使能状态
 *
 * @return bool 电机使能状态
 */
inline bool Class_Motor_VESC::Get_Motor_Enable() const
{
    return (Motor_Enable);
}

/**
 * @brief 获取当前电气转速
 *
 * @return int32_t 当前电气转速, ERPM
 */
inline int32_t Class_Motor_VESC::Get_Now_ERPM() const
{
    return (Rx_Data.Now_ERPM);
}

/**
 * @brief 获取当前轮组机械角速度
 *
 * @return float 当前轮组机械角速度, rad/s
 */
inline float Class_Motor_VESC::Get_Now_Omega() const
{
    return (Rx_Data.Now_Omega);
}

/**
 * @brief 获取当前电机电流
 *
 * @return float 当前电机电流, A
 */
inline float Class_Motor_VESC::Get_Motor_Current() const
{
    return (Rx_Data.Motor_Current);
}

/**
 * @brief 获取当前占空比
 *
 * @return float 当前占空比, -1.0~1.0
 */
inline float Class_Motor_VESC::Get_Duty() const
{
    return (Rx_Data.Duty);
}

/**
 * @brief 获取VESC功率管温度
 *
 * @return float 功率管温度, ℃
 */
inline float Class_Motor_VESC::Get_MOS_Temperature() const
{
    return (Rx_Data.MOS_Temperature);
}

/**
 * @brief 获取电机温度
 *
 * @return float 电机温度, ℃
 */
inline float Class_Motor_VESC::Get_Motor_Temperature() const
{
    return (Rx_Data.Motor_Temperature);
}

/**
 * @brief 获取VESC输入电流
 *
 * @return float 输入电流, A
 */
inline float Class_Motor_VESC::Get_Input_Current() const
{
    return (Rx_Data.Input_Current);
}

/**
 * @brief 获取VESC位置环反馈角度
 *
 * @return float 位置环反馈角度, °
 */
inline float Class_Motor_VESC::Get_PID_Position() const
{
    return (Rx_Data.PID_Position);
}

/**
 * @brief 获取VESC累计换相计数
 *
 * @return int32_t 累计换相计数
 */
inline int32_t Class_Motor_VESC::Get_Tachometer() const
{
    return (Rx_Data.Tachometer);
}

/**
 * @brief 获取VESC输入电压
 *
 * @return float 输入电压, V
 */
inline float Class_Motor_VESC::Get_Input_Voltage() const
{
    return (Rx_Data.Input_Voltage);
}

/**
 * @brief 获取STATUS报文最近更新时间
 *
 * @return uint32_t HAL毫秒时间戳
 */
inline uint32_t Class_Motor_VESC::Get_Status_1_Last_Update_Time() const
{
    return (Status_1_State.Last_Update_Time);
}

/**
 * @brief 获取STATUS_4报文最近更新时间
 *
 * @return uint32_t HAL毫秒时间戳
 */
inline uint32_t Class_Motor_VESC::Get_Status_4_Last_Update_Time() const
{
    return (Status_4_State.Last_Update_Time);
}

/**
 * @brief 获取STATUS_5报文最近更新时间
 *
 * @return uint32_t HAL毫秒时间戳
 */
inline uint32_t Class_Motor_VESC::Get_Status_5_Last_Update_Time() const
{
    return (Status_5_State.Last_Update_Time);
}

/**
 * @brief 获取STATUS报文累计更新次数
 */
inline uint32_t Class_Motor_VESC::Get_Status_1_Update_Count() const
{
    return (Status_1_State.Update_Count);
}

/**
 * @brief 获取STATUS_4报文累计更新次数
 */
inline uint32_t Class_Motor_VESC::Get_Status_4_Update_Count() const
{
    return (Status_4_State.Update_Count);
}

/**
 * @brief 获取STATUS_5报文累计更新次数
 */
inline uint32_t Class_Motor_VESC::Get_Status_5_Update_Count() const
{
    return (Status_5_State.Update_Count);
}

/**
 * @brief 获取目标电气转速
 *
 * @return int32_t 目标电气转速, ERPM
 */
inline int32_t Class_Motor_VESC::Get_Control_ERPM() const
{
    return (Control_ERPM);
}

/**
 * @brief 获取目标轮组机械角速度
 *
 * @return float 目标轮组机械角速度, rad/s
 */
inline float Class_Motor_VESC::Get_Control_Omega() const
{
    return (Control_Omega);
}

/**
 * @brief 获取目标电机电流
 *
 * @return float 目标电机电流, A
 */
inline float Class_Motor_VESC::Get_Control_Current() const
{
    return (Control_Current);
}

/**
 * @brief 获取目标制动电流
 *
 * @return float 目标制动电流, A
 */
inline float Class_Motor_VESC::Get_Control_Current_Brake() const
{
    return (Control_Current_Brake);
}

/**
 * @brief 获取目标占空比
 *
 * @return float 目标占空比, -1.0~1.0
 */
inline float Class_Motor_VESC::Get_Control_Duty() const
{
    return (Control_Duty);
}

/**
 * @brief 获取目标位置角度
 *
 * @return float 目标位置角度, °
 */
inline float Class_Motor_VESC::Get_Control_Position() const
{
    return (Control_Position);
}

/**
 * @brief 获取目标相对电流
 *
 * @return float 目标相对电流, -1.0~1.0
 */
inline float Class_Motor_VESC::Get_Control_Current_Rel() const
{
    return (Control_Current_Rel);
}

/**
 * @brief 设置电机使能状态, 失能时清空全部控制目标
 *
 * @param __Motor_Enable 电机使能状态
 */
inline void Class_Motor_VESC::Set_Motor_Enable(bool __Motor_Enable)
{
    Motor_Enable = __Motor_Enable;
    if (!Motor_Enable)
    {
        Reset_Control_Targets();
    }
}

/**
 * @brief 设置VESC控制方式, 切换方式时清空全部控制目标
 *
 * @param __Control_Method VESC控制方式
 */
inline void Class_Motor_VESC::Set_Control_Method(Enum_Motor_VESC_Control_Method __Control_Method)
{
    if (Control_Method != __Control_Method)
    {
        Control_Method = __Control_Method;
        Reset_Control_Targets();
    }
}

/**
 * @brief 设置目标轮组机械角速度并换算为ERPM
 *
 * @param __Control_Omega 目标轮组机械角速度, rad/s
 */
inline void Class_Motor_VESC::Set_Control_Omega(float __Control_Omega)
{
    Control_Omega = __Control_Omega;
    Control_ERPM = static_cast<int32_t>(roundf(Control_Omega * 60.0f / (2.0f * 3.14159265358979323846f) * static_cast<float>(Pole_Pairs) * Gearbox_Rate));
    if (Control_ERPM > ERPM_Max)
    {
        Control_ERPM = ERPM_Max;
    }
    else if (Control_ERPM < -ERPM_Max)
    {
        Control_ERPM = -ERPM_Max;
    }
}


/**
 * @brief 设置目标电机电流
 *
 * @param __Control_Current 目标电机电流, A
 */
inline void Class_Motor_VESC::Set_Control_Current(float __Control_Current)
{
    Control_Current = __Control_Current;
    if (Control_Current > Current_Max)
    {
        Control_Current = Current_Max;
    }
    else if (Control_Current < -Current_Max)
    {
        Control_Current = -Current_Max;
    }
}

/**
 * @brief 设置目标制动电流
 *
 * @param __Control_Current_Brake 目标制动电流, A
 */
inline void Class_Motor_VESC::Set_Control_Current_Brake(float __Control_Current_Brake)
{
    Control_Current_Brake = __Control_Current_Brake;
    if (Control_Current_Brake > Current_Brake_Max)
    {
        Control_Current_Brake = Current_Brake_Max;
    }
    else if (Control_Current_Brake < 0.0f)
    {
        Control_Current_Brake = 0.0f;
    }
}

/**
 * @brief 设置目标占空比
 *
 * @param __Control_Duty 目标占空比, -1.0~1.0
 */
inline void Class_Motor_VESC::Set_Control_Duty(float __Control_Duty)
{
    Control_Duty = __Control_Duty;
    if (Control_Duty > 1.0f)
    {
        Control_Duty = 1.0f;
    }
    else if (Control_Duty < -1.0f)
    {
        Control_Duty = -1.0f;
    }
}

/**
 * @brief 设置目标位置角度
 *
 * @param __Control_Position 目标位置角度, °
 */
inline void Class_Motor_VESC::Set_Control_Position(float __Control_Position)
{
    Control_Position = __Control_Position;
    if (Control_Position > 360.0f)
    {
        Control_Position = 360.0f;
    }
    else if (Control_Position < 0.0f)
    {
        Control_Position = 0.0f;
    }
}

/**
 * @brief 设置目标相对电流
 *
 * @param __Control_Current_Rel 目标相对电流, -1.0~1.0
 */
inline void Class_Motor_VESC::Set_Control_Current_Rel(float __Control_Current_Rel)
{
    Control_Current_Rel = __Control_Current_Rel;
    if (Control_Current_Rel > 1.0f)
    {
        Control_Current_Rel = 1.0f;
    }
    else if (Control_Current_Rel < -1.0f)
    {
        Control_Current_Rel = -1.0f;
    }
}

#endif
