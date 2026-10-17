#include <cmath>
#include <cstdint>
#include <cstdio>

#include "test_assert.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "4_Task/tsk_4swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_remote/tsk_remote.h"
#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1{FDCAN1};
FDCAN_HandleTypeDef hfdcan3{FDCAN3};
int fake_sequence = 0;
Struct_Remote_Chassis_Command Remote_Chassis_Command{};

namespace
{
void Assert_Near(const float value, const float expected)
{
    if (std::fabs(value - expected) >= 1.0e-4f)
    {
        std::fprintf(stderr, "Expected %.6f, got %.6f\n", static_cast<double>(expected), static_cast<double>(value));
    }
    assert(std::fabs(value - expected) < 1.0e-4f);
}

float Get_LF_Drive_RPM()
{
    return VESC_Chassis_LF.Get_Fake_State().control_omega * 9.54929659f;
}

void Assert_LF_Drive_RPM(const float expected)
{
    const float actual = Get_LF_Drive_RPM();
    if (std::fabs(actual - expected) >= 0.001f)
    {
        std::fprintf(stderr, "Expected drive %.4f rpm, got %.4f rpm\n", static_cast<double>(expected), static_cast<double>(actual));
    }
    assert(std::fabs(actual - expected) < 0.001f);
}

void Reset_Chassis()
{
    fake_sequence = 0;
    DM_Chassis_LF.Get_Fake_State() = {};
    DM_Chassis_RF.Get_Fake_State() = {};
    DM_Chassis_LB.Get_Fake_State() = {};
    DM_Chassis_RB.Get_Fake_State() = {};
    VESC_Chassis_LF.Get_Fake_State() = {};
    VESC_Chassis_RF.Get_Fake_State() = {};
    VESC_Chassis_LB.Get_Fake_State() = {};
    VESC_Chassis_RB.Get_Fake_State() = {};
    Motor_Init();
    Chassis_Init();
    Remote_Chassis_Command = {};
}

void Set_Valid_Command(const float speed_x, const float speed_y, const float speed_w)
{
    Remote_Chassis_Command.Speed_X = speed_x;
    Remote_Chassis_Command.Speed_Y = speed_y;
    Remote_Chassis_Command.Speed_W = speed_w;
    Remote_Chassis_Command.Online = 1U;
    Remote_Chassis_Command.Mode = 2U;
    Remote_Chassis_Command.Motor_Enable = 1U;
}

void Set_LF_Target_RPM(const float rpm)
{
    Set_Valid_Command(0.0f, rpm * 0.00733038286f, 0.0f); // 轮半径0.07m, 将机械rpm转换为m/s
}

void Run_Swerve_Cycles(const int cycles)
{
    for (int i = 0; i < cycles; i++)
    {
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
}

void Check_LF_Ramp_Cycles(const int cycles, float &previous_rpm)
{
    for (int i = 0; i < cycles; i++)
    {
        Chassis_Swerve_10ms_PeriodElapsedCallback();
        const float rpm = Get_LF_Drive_RPM();
        const float acceleration = (rpm - previous_rpm) / 0.01f;
        assert(std::isfinite(rpm));
        assert(std::fabs(rpm) <= 1440.01f);
        assert(std::fabs(acceleration) <= 1000.1f);
        previous_rpm = rpm;
    }
}

void Feed_LF_DM(const float angle, const float omega)
{
    DM_Chassis_LF.Get_Fake_State().now_total_angle = angle;
    DM_Chassis_LF.Get_Fake_State().now_omega = omega;
    DM_Chassis_LF.Get_Fake_State().total_angle_initialized = true;
    ++DM_Chassis_LF.Get_Fake_State().rx_update_count;
}

void Reach_LF_Homing_Ready()
{
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    for (int i = 0; i < 10; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
}

void Assert_All_Targets_Zero()
{
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_RF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_RB.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_RF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_LB.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_RB.Get_Fake_State().control_omega, 0.0f);
}

void Assert_All_Enter_Count(const int expected)
{
    assert(DM_Chassis_LF.Get_Fake_State().enter_count == expected);
    assert(DM_Chassis_RF.Get_Fake_State().enter_count == expected);
    assert(DM_Chassis_LB.Get_Fake_State().enter_count == expected);
    assert(DM_Chassis_RB.Get_Fake_State().enter_count == expected);
}

void Assert_All_Exit_Count(const int expected)
{
    assert(DM_Chassis_LF.Get_Fake_State().exit_count == expected);
    assert(DM_Chassis_RF.Get_Fake_State().exit_count == expected);
    assert(DM_Chassis_LB.Get_Fake_State().exit_count == expected);
    assert(DM_Chassis_RB.Get_Fake_State().exit_count == expected);
}

void Assert_All_Send_Count(const int expected)
{
    assert(DM_Chassis_LF.Get_Fake_State().send_count == expected);
    assert(DM_Chassis_RF.Get_Fake_State().send_count == expected);
    assert(DM_Chassis_LB.Get_Fake_State().send_count == expected);
    assert(DM_Chassis_RB.Get_Fake_State().send_count == expected);
}

void Test_Init_Uses_R1_Mechanics_But_USTC_Omni_Formula()
{
    Reset_Chassis();
    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Chassis_10ms_PeriodElapsedCallback();

    Assert_Near(DM_Chassis_RF.Get_Fake_State().control_omega, -7.63934377f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, -7.63934377f);
    Assert_Near(DM_Chassis_RB.Get_Fake_State().control_omega, 7.63934377f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().control_omega, 7.63934377f);
}

void Test_Pure_Y_And_Pure_W_Preserve_All_Positive_Direction_Coefficients()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Chassis_10ms_PeriodElapsedCallback();
    Assert_Near(DM_Chassis_RF.Get_Fake_State().control_omega, 7.63934377f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, -7.63934377f);
    Assert_Near(DM_Chassis_RB.Get_Fake_State().control_omega, 7.63934377f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().control_omega, -7.63934377f);

    Set_Valid_Command(0.0f, 0.0f, 1.0f);
    Chassis_10ms_PeriodElapsedCallback();
    Assert_Near(DM_Chassis_RF.Get_Fake_State().control_omega, 4.83107424f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, 4.83107424f);
    Assert_Near(DM_Chassis_RB.Get_Fake_State().control_omega, 4.83107424f);
    Assert_Near(DM_Chassis_LB.Get_Fake_State().control_omega, 4.83107424f);
}

void Test_Valid_Manual_Enable_Enters_Once_And_Sends_Every_10ms()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Chassis_10ms_PeriodElapsedCallback();
    Chassis_10ms_PeriodElapsedCallback();

    Assert_All_Enter_Count(1);
    Assert_All_Exit_Count(0);
    Assert_All_Send_Count(2);
}

void Assert_Invalid_Command_Disables(const std::uint8_t online, const std::uint8_t mode, const std::uint8_t motor_enable)
{
    Reset_Chassis();
    Set_Valid_Command(1.0f, 1.0f, 1.0f);
    Chassis_10ms_PeriodElapsedCallback();

    Remote_Chassis_Command.Online = online;
    Remote_Chassis_Command.Mode = mode;
    Remote_Chassis_Command.Motor_Enable = motor_enable;
    Chassis_10ms_PeriodElapsedCallback();
    Chassis_10ms_PeriodElapsedCallback();

    Assert_All_Targets_Zero();
    Assert_All_Enter_Count(1);
    Assert_All_Exit_Count(1);
    Assert_All_Send_Count(3);
}

void Test_Offline_Nonmanual_And_Disable_Clear_Targets_And_Exit_Once()
{
    Assert_Invalid_Command_Disables(0U, 2U, 1U);
    Assert_Invalid_Command_Disables(1U, 1U, 1U);
    Assert_Invalid_Command_Disables(1U, 2U, 0U);
}

void Test_100ms_Callback_Checks_Only_Active_Swerve_Pair()
{
    Reset_Chassis();
    Chassis_100ms_PeriodElapsedCallback();

    assert(DM_Chassis_LF.Get_Fake_State().alive_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().alive_count == 0);
    assert(DM_Chassis_LB.Get_Fake_State().alive_count == 0);
    assert(DM_Chassis_RB.Get_Fake_State().alive_count == 0);
    assert(VESC_Chassis_LF.Get_Fake_State().alive_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().alive_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().alive_count == 0);
    assert(VESC_Chassis_RB.Get_Fake_State().alive_count == 0);
}

void Test_Omni_10ms_Does_Not_Enable_Or_Command_VESC()
{
    Reset_Chassis();
    Set_Valid_Command(1.0f, 0.5f, 0.2f);
    Chassis_10ms_PeriodElapsedCallback();

    assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_LB.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RB.Get_Fake_State().motor_enable);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_RF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_LB.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(VESC_Chassis_RB.Get_Fake_State().control_omega, 0.0f);
}

void Test_Swerve_Only_LF_DM_And_VESC_Are_Active()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();

    assert(DM_Chassis_LF.Get_Fake_State().enter_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().enter_count == 0);
    assert(DM_Chassis_LB.Get_Fake_State().enter_count == 0);
    assert(DM_Chassis_RB.Get_Fake_State().enter_count == 0);
    assert(VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_LB.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RB.Get_Fake_State().motor_enable);
    assert(DM_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(DM_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(DM_Chassis_RB.Get_Fake_State().send_count == 0);
}

void Test_Enable_Waits_For_A_New_DM_Feedback_Frame()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_angle, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);

    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);

    Feed_LF_DM(0.2f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_HOMING);
}

void Test_Homing_Holds_VESC_At_Zero()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Feed_LF_DM(1.0f, 2.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();

    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_angle, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
}

void Test_Homing_Requires_Ten_Consecutive_Stable_Cycles()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    for (int i = 0; i < 9; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_HOMING);

    Feed_LF_DM(0.04f, 0.4f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
}

void Test_Homing_Stability_Count_Resets_When_Error_Returns()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    for (int i = 0; i < 9; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
    Feed_LF_DM(0.06f, 0.4f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    for (int i = 0; i < 9; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_HOMING);

    Feed_LF_DM(0.04f, 0.4f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
}

void Test_Homing_Reaches_Ready_And_Uses_Direct_Drive_Angle()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Feed_LF_DM(1.57079633f, 1.0f);
    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Chassis_Swerve_10ms_PeriodElapsedCallback();

    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_angle, 1.57079633f);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Target_Angle, 1.57079633f);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Now_Angle, 1.57079633f);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Now_Omega, 1.0f);
}

void Test_Homing_Times_Out_After_Three_Seconds()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 300; i++)
    {
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_TIMEOUT);
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);
}

void Test_Disable_Then_Enable_Retries_After_Timeout()
{
    Reset_Chassis();
    Set_Valid_Command(0.0f, 1.0f, 0.0f);
    for (int i = 0; i < 300; i++)
    {
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
    Remote_Chassis_Command.Motor_Enable = 0U;
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_DISABLED);

    Remote_Chassis_Command.Motor_Enable = 1U;
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);
    assert(DM_Chassis_LF.Get_Fake_State().enter_count == 2);
}

void Test_MIT_RAM_Setters_Validate_Ranges_And_Apply_To_Active_Motor()
{
    Reset_Chassis();
    assert(Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_LF, 8.5f));
    assert(Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_LF, 0.35f));
    assert(Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_LF, 0.8f));
    assert(!Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_LF, 50.1f));
    assert(!Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_LF, 5.1f));
    assert(!Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_LF, -2.1f));
    assert(!Chassis_Swerve_Set_MIT_K_P(static_cast<Enum_Swerve_Wheel>(Swerve_Wheel_Number), 1.0f));

    Reach_LF_Homing_Ready();
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 8.5f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.35f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.8f);
}

void Test_Control_Class_Callbacks_Preserve_Homing_Ramp_And_Offline_Stop()
{
    Reset_Chassis();
    Class_Chassis_Control chassis;
    chassis.Init();
    Set_Valid_Command(0.0f, 0.0f, 0.0f);
    chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_WAIT_FEEDBACK);
    for (int i = 0; i < 10; i++)
    {
        Feed_LF_DM(0.04f, 0.4f);
        chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    }
    assert(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Homing_Status == Chassis_Swerve_Homing_Status_READY);
    assert(chassis.Set_MIT_K_P(Swerve_Wheel_LF, 8.5f));
    assert(chassis.Set_MIT_K_D(Swerve_Wheel_LF, 0.35f));
    assert(chassis.Set_MIT_Feedforward_Torque(Swerve_Wheel_LF, 0.8f));
    assert(!chassis.Set_MIT_K_P(Swerve_Wheel_LF, 50.1f));
    Set_LF_Target_RPM(200.0f);
    chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    Assert_LF_Drive_RPM(10.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 8.5f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.35f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.8f);
    chassis.TIM_10ms_VESC_PeriodElapsedCallback();
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    chassis.TIM_100ms_PeriodElapsedCallback();
    assert(DM_Chassis_LF.Get_Fake_State().alive_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().alive_count == 0);
    Remote_Chassis_Command.Online = 0U;
    chassis.TIM_10ms_Swerve_PeriodElapsedCallback();
    Assert_LF_Drive_RPM(0.0f);
    assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_LF.Get_Fake_State().motor_enable);
}

void Test_MIT_Parameters_Are_Isolated_Between_Wheels()
{
    Reset_Chassis();
    assert(Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_RF, 12.0f));
    assert(Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_RB, 0.8f));
    assert(Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_LB, -0.6f));
    Reach_LF_Homing_Ready();
    Run_Swerve_Cycles(1);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 7.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.2f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    assert(DM_Chassis_RF.Get_Fake_State().enter_count == 0);
    assert(DM_Chassis_RB.Get_Fake_State().enter_count == 0);
    assert(DM_Chassis_LB.Get_Fake_State().enter_count == 0);
}

void Test_Disable_Preserves_MIT_Tuning_But_Init_Restores_Defaults()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    assert(Chassis_Swerve_Set_MIT_K_P(Swerve_Wheel_LF, 8.5f));
    assert(Chassis_Swerve_Set_MIT_K_D(Swerve_Wheel_LF, 0.35f));
    assert(Chassis_Swerve_Set_MIT_Feedforward_Torque(Swerve_Wheel_LF, 0.8f));
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(20);
    Assert_LF_Drive_RPM(200.0f);
    Remote_Chassis_Command.Motor_Enable = 0U;
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(0.0f);
    Reach_LF_Homing_Ready();
    Run_Swerve_Cycles(1);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 8.5f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.35f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.8f);
    Chassis_Init();
    Reach_LF_Homing_Ready();
    Run_Swerve_Cycles(1);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_p, 7.0f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().k_d, 0.2f);
    Assert_Near(DM_Chassis_LF.Get_Fake_State().control_torque, 0.0f);
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(10.0f);
}

void Test_Swerve_Drive_Linear_Ramp_Reaches_Target_Without_Startup_Delay()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(10.0f); // 1000机械rpm/s, 10ms立即增加10rpm
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(20.0f);
    Run_Swerve_Cycles(98);
    Assert_LF_Drive_RPM(1000.0f);
    Run_Swerve_Cycles(20);
    Assert_LF_Drive_RPM(1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(1200.0f);
}

void Test_Swerve_Drive_Negative_Start_Uses_The_Same_Acceleration()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(-1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(-500.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(-1000.0f);
    Run_Swerve_Cycles(20);
    Assert_LF_Drive_RPM(-1200.0f);
}

void Test_Swerve_Drive_Stop_And_Restart_Use_Linear_Ramp()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(120);
    Assert_LF_Drive_RPM(200.0f);
    Set_LF_Target_RPM(0.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(190.0f);
    Run_Swerve_Cycles(19);
    Assert_LF_Drive_RPM(0.0f);
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(10.0f);
}

void Test_Swerve_Drive_Does_Not_Overshoot_A_Small_Target()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_Valid_Command(0.0f, 0.14f, 0.0f); // 高于运动进入阈值, 半径0.07m对应轮速2rad/s
    for (int i = 0; i < 80; i++)
    {
        Chassis_Swerve_10ms_PeriodElapsedCallback();
        assert(VESC_Chassis_LF.Get_Fake_State().control_omega <= 2.00001f);
    }
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 2.0f);
}

void Test_Swerve_Drive_Nonzero_Speed_Changes_Use_Linear_Ramp()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(200);
    Assert_LF_Drive_RPM(200.0f);
    Set_LF_Target_RPM(400.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(210.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(220.0f);
    Run_Swerve_Cycles(18);
    Assert_LF_Drive_RPM(400.0f);
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(390.0f);
    Run_Swerve_Cycles(19);
    Assert_LF_Drive_RPM(200.0f);
}

void Test_Swerve_Drive_Lower_Target_Decelerates_On_The_Next_Cycle()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(500.0f);
    Set_LF_Target_RPM(1400.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(510.0f);
    Set_LF_Target_RPM(400.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(500.0f); // 不继承增速加速度, 降低目标后立即减速
    Run_Swerve_Cycles(10);
    Assert_LF_Drive_RPM(400.0f);
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(10);
    Assert_LF_Drive_RPM(500.0f);
    Set_LF_Target_RPM(0.0f);
    Run_Swerve_Cycles(1);
    Assert_LF_Drive_RPM(490.0f); // 加速途中回中也必须立即降低下发目标
}

void Test_Swerve_Drive_Acceleration_Bounds_During_Retargets()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    float previous_rpm = 0.0f;
    Set_LF_Target_RPM(1200.0f);
    Check_LF_Ramp_Cycles(250, previous_rpm);
    Set_LF_Target_RPM(200.0f);
    Check_LF_Ramp_Cycles(230, previous_rpm);
    Assert_LF_Drive_RPM(200.0f);
    Set_LF_Target_RPM(1400.0f);
    Check_LF_Ramp_Cycles(50, previous_rpm);
    Set_LF_Target_RPM(210.0f);
    Check_LF_Ramp_Cycles(250, previous_rpm);
    Assert_LF_Drive_RPM(210.0f);
    const float targets[] = {300.0f, 250.0f, 450.0f, 350.0f, 600.0f, 300.0f};
    for (float target : targets)
    {
        Set_LF_Target_RPM(target);
        Check_LF_Ramp_Cycles(15, previous_rpm);
    }
    Check_LF_Ramp_Cycles(250, previous_rpm);
    Assert_LF_Drive_RPM(300.0f);
    Set_LF_Target_RPM(0.0f);
    Check_LF_Ramp_Cycles(150, previous_rpm);
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Swerve_Drive_Rapid_Retargets_Keep_Bounded_Continuous_Output()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    float previous_rpm = 0.0f;
    std::uint32_t seed = 0x12345678U;
    for (int i = 0; i < 300; i++)
    {
        seed = seed * 1664525U + 1013904223U;
        const float target = static_cast<float>(static_cast<int>(seed % 2801U) - 1400);
        Set_LF_Target_RPM(target);
        Check_LF_Ramp_Cycles(7 + static_cast<int>(seed % 23U), previous_rpm);
    }
    Set_LF_Target_RPM(0.0f);
    Check_LF_Ramp_Cycles(500, previous_rpm);
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Swerve_Drive_Steering_Stop_Immediately_Starts_Ramp_Down()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(1200.0f);
    Run_Swerve_Cycles(50);
    Assert_LF_Drive_RPM(500.0f);
    DM_Chassis_LF.Get_Fake_State().now_total_angle = 1.0f; // 舵向误差超过45°, 限速比例必须为0
    Run_Swerve_Cycles(1);
    Assert_Near(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Speed_Ratio, 0.0f);
    Assert_LF_Drive_RPM(490.0f); // 舵向保护触发后立即按减速斜坡下降
    Run_Swerve_Cycles(150);
    Assert_LF_Drive_RPM(0.0f);
}

void Test_Swerve_Drive_Invalid_Command_Resets_Ramp()
{
    for (int reason = 0; reason < 3; reason++)
    {
        Reset_Chassis();
        Reach_LF_Homing_Ready();
        Set_Valid_Command(0.0f, 10.0f, 0.0f);
        for (int i = 0; i < 60; i++)
        {
            Chassis_Swerve_10ms_PeriodElapsedCallback();
        }
        if (reason == 0) { Remote_Chassis_Command.Online = 0U; }
        if (reason == 1) { Remote_Chassis_Command.Motor_Enable = 0U; }
        if (reason == 2) { Remote_Chassis_Command.Mode = 1U; }
        Chassis_Swerve_10ms_PeriodElapsedCallback();
        Assert_LF_Drive_RPM(0.0f);
        assert(!VESC_Chassis_LF.Get_Fake_State().motor_enable);
        Reach_LF_Homing_Ready();
        Set_Valid_Command(0.0f, 10.0f, 0.0f);
        Chassis_Swerve_10ms_PeriodElapsedCallback();
        Assert_LF_Drive_RPM(10.0f);
    }
}

void Test_Swerve_Drive_Reverses_Only_After_Ramping_To_Zero()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_LF_Target_RPM(200.0f);
    Run_Swerve_Cycles(120);
    Set_LF_Target_RPM(-200.0f);
    float previous_rpm = Get_LF_Drive_RPM();
    bool crossed_zero = false;
    for (int i = 0; i < 250; i++)
    {
        Run_Swerve_Cycles(1);
        const float rpm = Get_LF_Drive_RPM();
        if (!crossed_zero && rpm < -0.001f)
        {
            Assert_Near(previous_rpm, 0.0f); // 必须至少有一次零速下发, 再向相反方向起步
            assert(std::fabs(rpm + 10.0f) < 0.001f);
            crossed_zero = true;
        }
        previous_rpm = rpm;
    }
    assert(crossed_zero);
    Assert_LF_Drive_RPM(-200.0f);
}

void Test_Swerve_Drive_Waits_Until_Steer_Is_Near_Target()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    Set_Valid_Command(1.0f, 0.0f, 0.0f);

    for (int i = 0; i < 60; i++)
    {
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }
    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 0.0f);

    DM_Chassis_LF.Get_Fake_State().now_total_angle = 1.57079633f;
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Assert_LF_Drive_RPM(10.0f); // 舵向等待结束后从零速按线性斜坡起步
}

void Test_Swerve_Diagonal_Command_Normalizes_All_Wheels_To_1440_RPM()
{
    Reset_Chassis();
    Reach_LF_Homing_Ready();
    DM_Chassis_LF.Get_Fake_State().now_total_angle = 0.78539816f;
    Set_Valid_Command(10.55575f, 10.55575f, 0.0f);

    for (int i = 0; i < 500; i++)
    {
        Chassis_Swerve_10ms_PeriodElapsedCallback();
    }

    Assert_Near(VESC_Chassis_LF.Get_Fake_State().control_omega, 150.79644737f);
}

void Test_Swerve_Rejects_Invalid_Remote_Commands()
{
    Reset_Chassis();
    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Remote_Chassis_Command.Online = 0U;
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Assert_All_Targets_Zero();

    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Remote_Chassis_Command.Mode = 1U;
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Assert_All_Targets_Zero();

    Set_Valid_Command(1.0f, 0.0f, 0.0f);
    Remote_Chassis_Command.Motor_Enable = 0U;
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Assert_All_Targets_Zero();
}

void Test_10ms_VESC_Callback_Sends_Only_Active_VESC_Object()
{
    Reset_Chassis();
    Chassis_VESC_10ms_PeriodElapsedCallback();

    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_RB.Get_Fake_State().send_count == 0);
}
}

int main()
{
    Test_Control_Class_Callbacks_Preserve_Homing_Ramp_And_Offline_Stop();
    Test_MIT_Parameters_Are_Isolated_Between_Wheels();
    Test_Disable_Preserves_MIT_Tuning_But_Init_Restores_Defaults();
    Test_Swerve_Drive_Linear_Ramp_Reaches_Target_Without_Startup_Delay();
    Test_Swerve_Drive_Steering_Stop_Immediately_Starts_Ramp_Down();
    Test_Swerve_Drive_Negative_Start_Uses_The_Same_Acceleration();
    Test_Swerve_Drive_Stop_And_Restart_Use_Linear_Ramp();
    Test_Swerve_Drive_Does_Not_Overshoot_A_Small_Target();
    Test_Swerve_Drive_Nonzero_Speed_Changes_Use_Linear_Ramp();
    Test_Swerve_Drive_Lower_Target_Decelerates_On_The_Next_Cycle();
    Test_Swerve_Drive_Acceleration_Bounds_During_Retargets();
    Test_Swerve_Drive_Rapid_Retargets_Keep_Bounded_Continuous_Output();
    Test_Swerve_Drive_Invalid_Command_Resets_Ramp();
    Test_Init_Uses_R1_Mechanics_But_USTC_Omni_Formula();
    Test_Pure_Y_And_Pure_W_Preserve_All_Positive_Direction_Coefficients();
    Test_Valid_Manual_Enable_Enters_Once_And_Sends_Every_10ms();
    Test_Offline_Nonmanual_And_Disable_Clear_Targets_And_Exit_Once();
    Test_100ms_Callback_Checks_Only_Active_Swerve_Pair();
    Test_Omni_10ms_Does_Not_Enable_Or_Command_VESC();
    Test_Swerve_Only_LF_DM_And_VESC_Are_Active();
    Test_Enable_Waits_For_A_New_DM_Feedback_Frame();
    Test_Homing_Holds_VESC_At_Zero();
    Test_Homing_Requires_Ten_Consecutive_Stable_Cycles();
    Test_Homing_Stability_Count_Resets_When_Error_Returns();
    Test_Homing_Reaches_Ready_And_Uses_Direct_Drive_Angle();
    Test_Homing_Times_Out_After_Three_Seconds();
    Test_Disable_Then_Enable_Retries_After_Timeout();
    Test_MIT_RAM_Setters_Validate_Ranges_And_Apply_To_Active_Motor();
    Test_Swerve_Drive_Reverses_Only_After_Ramping_To_Zero();
    Test_Swerve_Drive_Waits_Until_Steer_Is_Near_Target();
    Test_Swerve_Diagonal_Command_Normalizes_All_Wheels_To_1440_RPM();
    Test_Swerve_Rejects_Invalid_Remote_Commands();
    Test_10ms_VESC_Callback_Sends_Only_Active_VESC_Object();
    return 0;
}
