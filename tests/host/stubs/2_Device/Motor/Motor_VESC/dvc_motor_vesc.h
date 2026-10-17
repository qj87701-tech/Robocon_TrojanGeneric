#ifndef TESTS_HOST_STUBS_DVC_MOTOR_VESC_H
#define TESTS_HOST_STUBS_DVC_MOTOR_VESC_H

#include "fdcan.h"

#include <cmath>
#include <cstdint>

enum Enum_Motor_VESC_Status
{
    Motor_VESC_Status_DISABLE = 0,
    Motor_VESC_Status_ENABLE,
};

enum Enum_Motor_VESC_Control_Method
{
    Motor_VESC_Control_Method_RPM = 0,
    Motor_VESC_Control_Method_CURRENT,
    Motor_VESC_Control_Method_CURRENT_BRAKE,
    Motor_VESC_Control_Method_DUTY,
    Motor_VESC_Control_Method_POSITION,
    Motor_VESC_Control_Method_CURRENT_REL,
};

struct Fake_VESC_State
{
    FDCAN_HandleTypeDef *can_handler = nullptr;
    std::uint8_t vesc_id = 0U;
    std::uint8_t pole_pairs = 0U;
    float gearbox_rate = 0.0f;
    std::int32_t erpm_max = 0;
    float current_max = 0.0f;
    float current_brake_max = 0.0f;
    Enum_Motor_VESC_Control_Method control_method = Motor_VESC_Control_Method_RPM;
    Enum_Motor_VESC_Status status = Motor_VESC_Status_DISABLE;
    bool motor_enable = false;
    std::int32_t now_erpm = 0;
    float now_omega = 0.0f;
    float motor_current = 0.0f;
    float duty = 0.0f;
    float mos_temperature = 0.0f;
    float motor_temperature = 0.0f;
    float input_current = 0.0f;
    float pid_position = 0.0f;
    std::int32_t tachometer = 0;
    float input_voltage = 0.0f;
    bool status_1_valid = false;
    bool status_4_valid = false;
    bool status_5_valid = false;
    std::uint32_t status_1_last_update_time = 0U;
    std::uint32_t status_4_last_update_time = 0U;
    std::uint32_t status_5_last_update_time = 0U;
    std::uint32_t status_1_update_count = 0U;
    std::uint32_t status_4_update_count = 0U;
    std::uint32_t status_5_update_count = 0U;
    std::int32_t control_erpm = 0;
    float control_omega = 0.0f;
    int init_sequence = 0;
    int receive_count = 0;
    int send_count = 0;
    int alive_count = 0;
};

extern int fake_sequence;

class Class_Motor_VESC
{
public:
    void Init(FDCAN_HandleTypeDef *hcan, std::uint8_t vesc_id, std::uint8_t pole_pairs, float gearbox_rate, std::int32_t erpm_max, float current_max, float current_brake_max)
    {
        State.can_handler = hcan;
        State.vesc_id = vesc_id;
        State.pole_pairs = pole_pairs;
        State.gearbox_rate = gearbox_rate;
        State.erpm_max = erpm_max;
        State.current_max = current_max;
        State.current_brake_max = current_brake_max;
        State.control_method = Motor_VESC_Control_Method_RPM;
        State.status = Motor_VESC_Status_DISABLE;
        State.motor_enable = false;
        State.control_erpm = 0;
        State.control_omega = 0.0f;
        State.init_sequence = ++fake_sequence;
    }

    Fake_VESC_State &Get_Fake_State()
    {
        return State;
    }

    const Fake_VESC_State &Get_Fake_State() const
    {
        return State;
    }

    Enum_Motor_VESC_Status Get_Status() const { return State.status; }
    bool Get_Motor_Enable() const { return State.motor_enable; }
    std::int32_t Get_Now_ERPM() const { return State.now_erpm; }
    float Get_Now_Omega() const { return State.now_omega; }
    float Get_Motor_Current() const { return State.motor_current; }
    float Get_Duty() const { return State.duty; }
    float Get_MOS_Temperature() const { return State.mos_temperature; }
    float Get_Motor_Temperature() const { return State.motor_temperature; }
    float Get_Input_Current() const { return State.input_current; }
    float Get_PID_Position() const { return State.pid_position; }
    std::int32_t Get_Tachometer() const { return State.tachometer; }
    float Get_Input_Voltage() const { return State.input_voltage; }
    bool Get_Status_1_Valid() const { return State.status_1_valid; }
    bool Get_Status_4_Valid() const { return State.status_4_valid; }
    bool Get_Status_5_Valid() const { return State.status_5_valid; }
    std::uint32_t Get_Status_1_Last_Update_Time() const { return State.status_1_last_update_time; }
    std::uint32_t Get_Status_4_Last_Update_Time() const { return State.status_4_last_update_time; }
    std::uint32_t Get_Status_5_Last_Update_Time() const { return State.status_5_last_update_time; }
    std::uint32_t Get_Status_1_Update_Count() const { return State.status_1_update_count; }
    std::uint32_t Get_Status_4_Update_Count() const { return State.status_4_update_count; }
    std::uint32_t Get_Status_5_Update_Count() const { return State.status_5_update_count; }
    std::int32_t Get_Control_ERPM() const { return State.control_erpm; }
    float Get_Control_Omega() const { return State.control_omega; }

    void Set_Motor_Enable(bool motor_enable)
    {
        State.motor_enable = motor_enable;
        if (!motor_enable)
        {
            State.control_erpm = 0;
            State.control_omega = 0.0f;
        }
    }

    void Set_Control_Omega(float control_omega)
    {
        State.control_omega = control_omega;
        State.control_erpm = static_cast<std::int32_t>(roundf(control_omega * 60.0f / (2.0f * 3.14159265358979323846f) * static_cast<float>(State.pole_pairs) * State.gearbox_rate));
    }

    void CAN_RxCpltCallback() { ++State.receive_count; }
    void TIM_100ms_Alive_PeriodElapsedCallback() { ++State.alive_count; }
    void TIM_Send_PeriodElapsedCallback() { ++State.send_count; }

private:
    Fake_VESC_State State;
};

#endif
