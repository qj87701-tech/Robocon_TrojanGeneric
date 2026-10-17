#ifndef TESTS_HOST_STUBS_DVC_MOTOR_DM_H
#define TESTS_HOST_STUBS_DVC_MOTOR_DM_H

#include "fdcan.h"

enum Enum_Motor_DM_Control_Method
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
};

enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
};

enum Enum_Motor_DM_Control_Status_Normal
{
    Motor_DM_Control_Status_DISABLE = 0,
    Motor_DM_Control_Status_ENABLE,
};

struct Fake_Motor_State
{
    const FDCAN_HandleTypeDef *can_handler = nullptr;
    std::uint8_t init_rx_id = 0U;
    std::uint8_t init_tx_id = 0U;
    Enum_Motor_DM_Control_Method method = Motor_DM_Control_Method_NORMAL_MIT;
    float angle_max = 0.0f;
    float omega_max = 0.0f;
    float torque_max = 0.0f;
    float control_angle = 0.0f;
    float control_omega = 0.0f;
    float control_torque = 0.0f;
    float k_p = 0.0f;
    float k_d = 0.0f;
    float now_angle = 0.0f;
    float now_total_angle = 0.0f;
    bool total_angle_initialized = false;
    std::uint32_t rx_update_count = 0U;
    float now_omega = 0.0f;
    float now_torque = 0.0f;
    float mos_temperature = 0.0f;
    float rotor_temperature = 0.0f;
    bool motor_enable = false;
    Enum_Motor_DM_Status status = Motor_DM_Status_DISABLE;
    Enum_Motor_DM_Control_Status_Normal control_status = Motor_DM_Control_Status_DISABLE;
    int init_sequence = 0;
    int set_angle_sequence = 0;
    int set_omega_sequence = 0;
    int enter_sequence = 0;
    int enter_count = 0;
    int exit_count = 0;
    int receive_count = 0;
    int send_count = 0;
    int alive_count = 0;
};

extern int fake_sequence;

class Class_Motor_DM_Normal
{
public:
    void Init(const FDCAN_HandleTypeDef *hcan, const std::uint8_t &rx_id, const std::uint8_t &tx_id, const Enum_Motor_DM_Control_Method &method, const float &angle_max, const float &omega_max, const float &torque_max)
    {
        State.can_handler = hcan;
        State.init_rx_id = rx_id;
        State.init_tx_id = tx_id;
        State.method = method;
        State.angle_max = angle_max;
        State.omega_max = omega_max;
        State.torque_max = torque_max;
        State.init_sequence = ++fake_sequence;
    }

    Fake_Motor_State &Get_Fake_State()
    {
        return State;
    }

    const Fake_Motor_State &Get_Fake_State() const
    {
        return State;
    }

    float Get_Control_Omega() const
    {
        return State.control_omega;
    }

    float Get_Control_Angle() const
    {
        return State.control_angle;
    }

    bool Get_Motor_Enable() const
    {
        return State.motor_enable;
    }

    Enum_Motor_DM_Status Get_Status() const
    {
        return State.status;
    }

    Enum_Motor_DM_Control_Status_Normal Get_Control_Status() const
    {
        return State.control_status;
    }

    float Get_Now_Angle() const
    {
        return State.now_angle;
    }

    float Get_Now_Total_Angle() const
    {
        return State.now_total_angle;
    }

    bool Get_Total_Angle_Initialized() const
    {
        return State.total_angle_initialized;
    }

    std::uint32_t Get_Rx_Update_Count() const
    {
        return State.rx_update_count;
    }

    float Get_Now_Omega() const
    {
        return State.now_omega;
    }

    float Get_Now_Torque() const
    {
        return State.now_torque;
    }

    float Get_Control_Torque() const
    {
        return State.control_torque;
    }

    float Get_K_P() const
    {
        return State.k_p;
    }

    float Get_K_D() const
    {
        return State.k_d;
    }

    float Get_Now_MOS_Temperature() const
    {
        return State.mos_temperature;
    }

    float Get_Now_Rotor_Temperature() const
    {
        return State.rotor_temperature;
    }

    void Set_Control_Angle(const float &control_angle)
    {
        State.control_angle = control_angle;
        State.set_angle_sequence = ++fake_sequence;
    }

    void Set_Control_Omega(const float &control_omega)
    {
        State.control_omega = control_omega;
        State.set_omega_sequence = ++fake_sequence;
    }

    void Set_Control_Torque(const float &control_torque)
    {
        State.control_torque = control_torque;
    }

    void Set_K_P(const float &k_p)
    {
        State.k_p = k_p;
    }

    void Set_K_D(const float &k_d)
    {
        State.k_d = k_d;
    }

    void CAN_RxCpltCallback()
    {
        ++State.receive_count;
    }

    void CAN_Send_Enter() const
    {
        State.motor_enable = true;
        ++State.enter_count;
        State.enter_sequence = ++fake_sequence;
    }

    void CAN_Send_Exit() const
    {
        State.motor_enable = false;
        ++State.exit_count;
    }

    void TIM_100ms_Alive_PeriodElapsedCallback()
    {
        ++State.alive_count;
    }

    void TIM_Send_PeriodElapsedCallback()
    {
        ++State.send_count;
    }

private:
    mutable Fake_Motor_State State;
};

#endif
