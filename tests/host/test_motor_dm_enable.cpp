#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "test_assert.h"
#include "User_File/2_Device/Motor/Motor_DM/dvc_motor_dm.h"

FDCAN_HandleTypeDef hfdcan1{FDCAN1};
FDCAN_HandleTypeDef hfdcan2{FDCAN2};
FDCAN_HandleTypeDef hfdcan3{FDCAN3};

Struct_CAN_Manage_Object CAN1_Manage_Object{&hfdcan1, nullptr, {}, {}, 0U};
Struct_CAN_Manage_Object CAN2_Manage_Object{&hfdcan2, nullptr, {}, {}, 0U};
Struct_CAN_Manage_Object CAN3_Manage_Object{&hfdcan3, nullptr, {}, {}, 0U};

std::uint8_t CAN1_0x3fe_Tx_Data[8]{};
std::uint8_t CAN1_0x4fe_Tx_Data[8]{};
std::uint8_t CAN2_0x3fe_Tx_Data[8]{};
std::uint8_t CAN2_0x4fe_Tx_Data[8]{};
std::uint8_t CAN3_0x3fe_Tx_Data[8]{};
std::uint8_t CAN3_0x4fe_Tx_Data[8]{};

namespace
{
struct Struct_CAN_Frame
{
    std::uint16_t ID;
    std::array<std::uint8_t, 8> Data;
};

std::vector<Struct_CAN_Frame> CAN_Frames;

std::uint16_t Encode_Signed(const float value, const float maximum)
{
    const float normalized = (value + maximum) / (2.0f * maximum);
    return static_cast<std::uint16_t>(std::lround(normalized * 65535.0f));
}

void Feed_Normal_Angle(Class_Motor_DM_Normal &motor, const float angle)
{
    const std::uint16_t encoded_angle = Encode_Signed(angle, 12.5f);
    std::memset(CAN1_Manage_Object.Rx_Buffer, 0, sizeof(CAN1_Manage_Object.Rx_Buffer));
    CAN1_Manage_Object.Rx_Buffer[0] = 0x11U;
    CAN1_Manage_Object.Rx_Buffer[1] = static_cast<std::uint8_t>(encoded_angle >> 8U);
    CAN1_Manage_Object.Rx_Buffer[2] = static_cast<std::uint8_t>(encoded_angle & 0xffU);
    motor.CAN_RxCpltCallback();
}

void Assert_Near(const float value, const float expected, const float tolerance = 0.01f)
{
    assert(std::fabs(value - expected) <= tolerance);
}

void Test_Exit_Prevents_Automatic_Reenable()
{
    Class_Motor_DM_Normal motor;
    motor.Init(&hfdcan1, 0x33U, 0x03U, Motor_DM_Control_Method_NORMAL_OMEGA,
               12.5f, 45.0f, 20.0f);

    motor.CAN_Send_Exit();
    CAN_Frames.clear();

    motor.TIM_Send_PeriodElapsedCallback();
    motor.TIM_100ms_Alive_PeriodElapsedCallback();

    assert(CAN_Frames.empty());
}

void Test_Enter_Allows_Automatic_Reenable()
{
    Class_Motor_DM_Normal motor;
    motor.Init(&hfdcan1, 0x33U, 0x03U, Motor_DM_Control_Method_NORMAL_OMEGA,
               12.5f, 45.0f, 20.0f);

    motor.CAN_Send_Enter();
    CAN_Frames.clear();
    motor.TIM_Send_PeriodElapsedCallback();

    assert(CAN_Frames.size() == 1U);
    assert(CAN_Frames[0].ID == 0x203U);
    assert(CAN_Frames[0].Data[7] == 0xFCU);
}

void Test_Normal_Feedback_Unwraps_Continuous_Angle_In_Both_Directions()
{
    Class_Motor_DM_Normal motor;
    motor.Init(&hfdcan1, 0x11U, 0x01U, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
               12.5f, 200.0f, 10.0f);

    Feed_Normal_Angle(motor, 12.40f);
    assert(motor.Get_Total_Angle_Initialized());
    Assert_Near(motor.Get_Now_Total_Angle(), 12.40f);

    Feed_Normal_Angle(motor, -12.40f);
    Assert_Near(motor.Get_Now_Total_Angle(), 12.60f);

    Feed_Normal_Angle(motor, -12.10f);
    Assert_Near(motor.Get_Now_Total_Angle(), 12.90f);

    Class_Motor_DM_Normal reverse_motor;
    reverse_motor.Init(&hfdcan1, 0x11U, 0x01U, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
                       12.5f, 200.0f, 10.0f);
    Feed_Normal_Angle(reverse_motor, -12.40f);
    Feed_Normal_Angle(reverse_motor, 12.40f);
    Assert_Near(reverse_motor.Get_Now_Total_Angle(), -12.60f);
}

void Test_DM_Rx_Update_Count_Changes_On_Valid_Feedback()
{
    Class_Motor_DM_Normal motor;
    motor.Init(&hfdcan1, 0x11U, 0x01U, Motor_DM_Control_Method_NORMAL_MIT,
               12.5f, 45.0f, 10.0f);

    assert(motor.Get_Rx_Update_Count() == 0U);
    Feed_Normal_Angle(motor, 0.25f);
    assert(motor.Get_Rx_Update_Count() == 1U);
    Feed_Normal_Angle(motor, -0.50f);
    assert(motor.Get_Rx_Update_Count() == 2U);
}

void Test_Position_Speed_Mode_Preserves_Multiturn_Target_While_MIT_Stays_Bounded()
{
    Class_Motor_DM_Normal position_motor;
    position_motor.Init(&hfdcan1, 0x11U, 0x01U, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
                        12.5f, 200.0f, 10.0f);
    Feed_Normal_Angle(position_motor, 0.0f);
    position_motor.CAN_Send_Enter();
    CAN_Frames.clear();
    position_motor.Set_Control_Angle(79.464f);
    position_motor.Set_Control_Omega(1.0f);
    position_motor.TIM_Send_PeriodElapsedCallback();

    assert(CAN_Frames.size() == 1U);
    float transmitted_angle = 0.0f;
    std::memcpy(&transmitted_angle, CAN_Frames[0].Data.data(), sizeof(transmitted_angle));
    Assert_Near(transmitted_angle, 79.464f, 0.0001f);
    Assert_Near(position_motor.Get_Control_Angle(), 79.464f, 0.0001f);

    Class_Motor_DM_Normal mit_motor;
    mit_motor.Init(&hfdcan1, 0x11U, 0x01U, Motor_DM_Control_Method_NORMAL_MIT,
                   12.5f, 200.0f, 10.0f);
    Feed_Normal_Angle(mit_motor, 0.0f);
    mit_motor.CAN_Send_Enter();
    CAN_Frames.clear();
    mit_motor.Set_Control_Angle(79.464f);
    mit_motor.TIM_Send_PeriodElapsedCallback();
    Assert_Near(mit_motor.Get_Control_Angle(), 12.5f, 0.0001f);
}
}

std::uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, const std::uint16_t id,
                               std::uint8_t *data, const std::uint16_t length)
{
    assert(hfdcan == &hfdcan1);
    assert(length == 8U);

    Struct_CAN_Frame frame{};
    frame.ID = id;
    for (std::uint16_t i = 0; i < length; i++)
    {
        frame.Data[i] = data[i];
    }
    CAN_Frames.push_back(frame);
    return 1U;
}

int main()
{
    Test_Exit_Prevents_Automatic_Reenable();
    Test_Enter_Allows_Automatic_Reenable();
    Test_Normal_Feedback_Unwraps_Continuous_Angle_In_Both_Directions();
    Test_DM_Rx_Update_Count_Changes_On_Valid_Feedback();
    Test_Position_Speed_Mode_Preserves_Multiturn_Target_While_MIT_Stays_Bounded();
    return 0;
}
