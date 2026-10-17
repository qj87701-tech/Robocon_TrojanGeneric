#include <cmath>
#include <cstdint>
#include <cstring>

#include "test_assert.h"
#include "2_Device/Motor/Motor_VESC/dvc_motor_vesc.h"

bool init_finished = true;
FDCAN_HandleTypeDef hfdcan1{FDCAN1};
FDCAN_HandleTypeDef hfdcan2{FDCAN2};
FDCAN_HandleTypeDef hfdcan3{FDCAN3};
Struct_CAN_Manage_Object CAN1_Manage_Object{};
Struct_CAN_Manage_Object CAN2_Manage_Object{};
Struct_CAN_Manage_Object CAN3_Manage_Object{&hfdcan3, nullptr, {}, {}, 0U};
std::uint32_t Fake_Tick = 0U;

namespace
{
FDCAN_HandleTypeDef *Transmit_Handler = nullptr;
std::uint32_t Transmit_ID = 0U;
std::uint16_t Transmit_Length = 0U;
std::uint8_t Transmit_Buffer[8]{};

void Reset_Transmit()
{
    Transmit_Handler = nullptr;
    Transmit_ID = 0U;
    Transmit_Length = 0U;
    std::memset(Transmit_Buffer, 0, sizeof(Transmit_Buffer));
}

void Write_Int16_BE(std::uint8_t *buffer, const std::int16_t value)
{
    const std::uint16_t raw = static_cast<std::uint16_t>(value);
    buffer[0] = static_cast<std::uint8_t>(raw >> 8U);
    buffer[1] = static_cast<std::uint8_t>(raw);
}

void Write_Int32_BE(std::uint8_t *buffer, const std::int32_t value)
{
    const std::uint32_t raw = static_cast<std::uint32_t>(value);
    buffer[0] = static_cast<std::uint8_t>(raw >> 24U);
    buffer[1] = static_cast<std::uint8_t>(raw >> 16U);
    buffer[2] = static_cast<std::uint8_t>(raw >> 8U);
    buffer[3] = static_cast<std::uint8_t>(raw);
}

std::int32_t Read_Int32_BE(const std::uint8_t *buffer)
{
    const std::uint32_t raw = (static_cast<std::uint32_t>(buffer[0]) << 24U) |
                              (static_cast<std::uint32_t>(buffer[1]) << 16U) |
                              (static_cast<std::uint32_t>(buffer[2]) << 8U) |
                              static_cast<std::uint32_t>(buffer[3]);
    return static_cast<std::int32_t>(raw);
}

void Feed_Status1_And_Mark_Online(Class_Motor_VESC &motor, const std::uint8_t vesc_id)
{
    CAN3_Manage_Object.Rx_Header = {};
    CAN3_Manage_Object.Rx_Header.Identifier = (9U << 8U) | vesc_id;
    CAN3_Manage_Object.Rx_Header.IdType = FDCAN_EXTENDED_ID;
    CAN3_Manage_Object.Rx_Header.RxFrameType = FDCAN_DATA_FRAME;
    CAN3_Manage_Object.Rx_Header.DataLength = FDCAN_DLC_BYTES_8;
    std::memset(CAN3_Manage_Object.Rx_Buffer, 0, sizeof(CAN3_Manage_Object.Rx_Buffer));
    motor.CAN_RxCpltCallback();
    motor.TIM_100ms_Alive_PeriodElapsedCallback();
}

void Prepare_Frame(const std::uint8_t packet_id, const std::uint8_t vesc_id, const std::uint32_t data_length)
{
    CAN3_Manage_Object.Rx_Header = {};
    CAN3_Manage_Object.Rx_Header.Identifier = (static_cast<std::uint32_t>(packet_id) << 8U) | vesc_id;
    CAN3_Manage_Object.Rx_Header.IdType = FDCAN_EXTENDED_ID;
    CAN3_Manage_Object.Rx_Header.RxFrameType = FDCAN_DATA_FRAME;
    CAN3_Manage_Object.Rx_Header.DataLength = data_length;
    std::memset(CAN3_Manage_Object.Rx_Buffer, 0, sizeof(CAN3_Manage_Object.Rx_Buffer));
}

void Assert_Near(const float value, const float expected)
{
    assert(std::fabs(value - expected) < 1.0e-4f);
}

void Test_Init_Is_Disabled_And_Zeroed()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 5.0f);

    assert(motor.Get_Status() == Motor_VESC_Status_DISABLE);
    assert(!motor.Get_Motor_Enable());
    assert(motor.Get_Control_Method() == Motor_VESC_Control_Method_RPM);
    assert(motor.Get_Control_ERPM() == 0);
    assert(motor.Get_Control_Omega() == 0.0f);
    assert(motor.Get_Now_ERPM() == 0);
    assert(motor.Get_Now_Omega() == 0.0f);
}

void Test_Current_Mode_Uses_Ampere_Scaling_And_Limit()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 5.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_CURRENT);
    motor.Set_Control_Current(-8.0f);
    motor.Set_Motor_Enable(true);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((1U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == -5000);
    Assert_Near(motor.Get_Control_Current(), -5.0f);
}

void Test_Current_Brake_Mode_Uses_Positive_Ampere_Scaling_And_Limit()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 4.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_CURRENT_BRAKE);
    motor.Set_Control_Current_Brake(8.0f);
    motor.Set_Motor_Enable(true);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((2U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == 4000);
    Assert_Near(motor.Get_Control_Current_Brake(), 4.0f);

    motor.Set_Control_Current_Brake(-1.0f);
    motor.TIM_Send_PeriodElapsedCallback();
    assert(Read_Int32_BE(Transmit_Buffer) == 0);
}

void Test_Duty_Mode_Uses_Protocol_Scaling_And_Limit()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 5.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_DUTY);
    motor.Set_Control_Duty(1.5f);
    motor.Set_Motor_Enable(true);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((0U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == 100000);
    Assert_Near(motor.Get_Control_Duty(), 1.0f);
}

void Test_Position_Mode_Uses_Degree_Scaling_And_Limit()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 5.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_POSITION);
    motor.Set_Control_Position(450.0f);
    motor.Set_Motor_Enable(true);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((4U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == 360000000);
    Assert_Near(motor.Get_Control_Position(), 360.0f);
}

void Test_Current_Rel_Mode_Uses_Protocol_Scaling_And_Limit()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 5.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_CURRENT_REL);
    motor.Set_Control_Current_Rel(-1.5f);
    motor.Set_Motor_Enable(true);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((10U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == -100000);
    Assert_Near(motor.Get_Control_Current_Rel(), -1.0f);
}

void Test_Changing_Method_Clears_Previous_Control_Targets()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 5.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_CURRENT);
    motor.Set_Control_Current(3.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_DUTY);
    motor.Set_Motor_Enable(true);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((0U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == 0);
    Assert_Near(motor.Get_Control_Current(), 0.0f);
    Assert_Near(motor.Get_Control_Duty(), 0.0f);
}

void Test_Disabling_Clears_Non_RPM_Target_Before_Reenable()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000, 5.0f, 5.0f);
    motor.Set_Control_Method(Motor_VESC_Control_Method_DUTY);
    motor.Set_Control_Duty(0.5f);
    motor.Set_Motor_Enable(true);
    motor.Set_Motor_Enable(false);
    motor.Set_Motor_Enable(true);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((0U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == 0);
    Assert_Near(motor.Get_Control_Duty(), 0.0f);
}

void Test_Zero_One_And_Negative_Rad_Per_Second_Conversion()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);

    motor.Set_Control_Omega(0.0f);
    assert(motor.Get_Control_ERPM() == 0);
    motor.Set_Control_Omega(1.0f);
    assert(motor.Get_Control_ERPM() == 201);
    motor.Set_Control_Omega(-1.0f);
    assert(motor.Get_Control_ERPM() == -201);
}

void Test_Target_ERPM_Is_Clamped()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);

    motor.Set_Control_Omega(100.0f);
    assert(motor.Get_Control_ERPM() == 1000);
    motor.Set_Control_Omega(-100.0f);
    assert(motor.Get_Control_ERPM() == -1000);
}

void Test_Enabled_Before_Online_Sends_Set_RPM_Big_Endian()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    motor.Set_Motor_Enable(true);
    motor.Set_Control_Omega(1.0f);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_Handler == &hfdcan3);
    assert(Transmit_ID == ((3U << 8U) | 1U));
    assert(Transmit_Length == 4U);
    assert(Read_Int32_BE(Transmit_Buffer) == 201);
}

void Test_Disabled_Sends_Zero_Current_Not_Zero_RPM()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Feed_Status1_And_Mark_Online(motor, 1U);
    motor.Set_Motor_Enable(true);
    motor.Set_Control_Omega(1.0f);
    motor.Set_Motor_Enable(false);
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((1U << 8U) | 1U));
    assert(Transmit_Length == 4U);
    assert(Read_Int32_BE(Transmit_Buffer) == 0);
    assert(motor.Get_Control_ERPM() == 0);
    assert(motor.Get_Control_Omega() == 0.0f);
}

void Test_Status1_Decodes_ERPM_Current_Duty_And_Omega()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Prepare_Frame(9U, 1U, FDCAN_DLC_BYTES_8);
    Write_Int32_BE(&CAN3_Manage_Object.Rx_Buffer[0], -4200);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[4], -123);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[6], 456);

    motor.CAN_RxCpltCallback();

    assert(motor.Get_Now_ERPM() == -4200);
    Assert_Near(motor.Get_Now_Omega(), -20.94395065f);
    Assert_Near(motor.Get_Motor_Current(), -12.3f);
    Assert_Near(motor.Get_Duty(), 0.456f);
}

void Test_Status4_Decodes_Temperatures_Current_And_Position()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Prepare_Frame(16U, 1U, FDCAN_DLC_BYTES_8);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[0], 456);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[2], -123);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[4], 78);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[6], 4500);

    motor.CAN_RxCpltCallback();

    Assert_Near(motor.Get_MOS_Temperature(), 45.6f);
    Assert_Near(motor.Get_Motor_Temperature(), -12.3f);
    Assert_Near(motor.Get_Input_Current(), 7.8f);
    Assert_Near(motor.Get_PID_Position(), 90.0f);
}

void Test_Status5_Accepts_Eight_Bytes_And_Decodes_Voltage()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Prepare_Frame(27U, 1U, FDCAN_DLC_BYTES_8);
    Write_Int32_BE(&CAN3_Manage_Object.Rx_Buffer[0], -123456);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[4], 486);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[6], 0x1234);

    motor.CAN_RxCpltCallback();

    assert(motor.Get_Tachometer() == -123456);
    Assert_Near(motor.Get_Input_Voltage(), 48.6f);
}

void Test_Each_Status_Frame_Tracks_Its_Own_Freshness()
{
    const std::uint8_t packet_id[3] = {9U, 16U, 27U};
    const std::uint32_t length[3] = {FDCAN_DLC_BYTES_8, FDCAN_DLC_BYTES_8, FDCAN_DLC_BYTES_8};
    for (std::uint8_t i = 0U; i < 3U; i++)
    {
        Class_Motor_VESC motor;
        motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
        Fake_Tick = 10U + i;
        Prepare_Frame(packet_id[i], 1U, length[i]);
        motor.CAN_RxCpltCallback();
        motor.TIM_100ms_Alive_PeriodElapsedCallback();
        assert(motor.Get_Status() == Motor_VESC_Status_ENABLE);
        if (packet_id[i] == 9U)
        {
            assert(motor.Get_Status_1_Valid());
            assert(motor.Get_Status_1_Last_Update_Time() == Fake_Tick);
            assert(motor.Get_Status_1_Update_Count() == 1U);
        }
        else if (packet_id[i] == 16U)
        {
            assert(motor.Get_Status_4_Valid());
            assert(motor.Get_Status_4_Last_Update_Time() == Fake_Tick);
            assert(motor.Get_Status_4_Update_Count() == 1U);
        }
        else
        {
            assert(motor.Get_Status_5_Valid());
            assert(motor.Get_Status_5_Last_Update_Time() == Fake_Tick);
            assert(motor.Get_Status_5_Update_Count() == 1U);
        }
    }
}

void Test_Rejects_Standard_Remote_Unknown_And_Wrong_VESC_ID()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);

    Prepare_Frame(9U, 1U, FDCAN_DLC_BYTES_8);
    CAN3_Manage_Object.Rx_Header.IdType = FDCAN_STANDARD_ID;
    motor.CAN_RxCpltCallback();
    Prepare_Frame(9U, 1U, FDCAN_DLC_BYTES_8);
    CAN3_Manage_Object.Rx_Header.RxFrameType = FDCAN_REMOTE_FRAME;
    motor.CAN_RxCpltCallback();
    Prepare_Frame(8U, 1U, FDCAN_DLC_BYTES_8);
    motor.CAN_RxCpltCallback();
    Prepare_Frame(9U, 2U, FDCAN_DLC_BYTES_8);
    motor.CAN_RxCpltCallback();
    motor.TIM_100ms_Alive_PeriodElapsedCallback();

    assert(motor.Get_Status() == Motor_VESC_Status_DISABLE);
    assert(motor.Get_Now_ERPM() == 0);
}

void Test_Status1_And_Status4_Require_Eight_Bytes()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Prepare_Frame(9U, 1U, FDCAN_DLC_BYTES_6);
    motor.CAN_RxCpltCallback();
    Prepare_Frame(16U, 1U, FDCAN_DLC_BYTES_6);
    motor.CAN_RxCpltCallback();
    motor.TIM_100ms_Alive_PeriodElapsedCallback();
    assert(motor.Get_Status() == Motor_VESC_Status_DISABLE);
}

void Test_Status5_Rejects_Six_Bytes()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Prepare_Frame(27U, 1U, FDCAN_DLC_BYTES_6);
    Write_Int32_BE(&CAN3_Manage_Object.Rx_Buffer[0], 123456);
    Write_Int16_BE(&CAN3_Manage_Object.Rx_Buffer[4], 486);
    motor.CAN_RxCpltCallback();
    motor.TIM_100ms_Alive_PeriodElapsedCallback();

    assert(motor.Get_Status() == Motor_VESC_Status_DISABLE);
    assert(motor.Get_Tachometer() == 0);
    assert(motor.Get_Input_Voltage() == 0.0f);
}

void Test_Status_Frame_Expires_After_100ms()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Fake_Tick = 0U;
    Feed_Status1_And_Mark_Online(motor, 1U);
    motor.Set_Motor_Enable(true);
    motor.Set_Control_Omega(1.0f);

    Fake_Tick = 99U;
    motor.TIM_100ms_Alive_PeriodElapsedCallback();
    assert(motor.Get_Status() == Motor_VESC_Status_ENABLE);
    assert(motor.Get_Status_1_Valid());
    assert(motor.Get_Control_ERPM() == 201);
    Fake_Tick = 100U;
    motor.TIM_100ms_Alive_PeriodElapsedCallback();
    assert(motor.Get_Status() == Motor_VESC_Status_DISABLE);
    assert(!motor.Get_Status_1_Valid());
    assert(motor.Get_Motor_Enable());
    assert(motor.Get_Control_ERPM() == 201);
    assert(motor.Get_Control_Omega() == 1.0f);
}

void Test_Invalid_Frame_Does_Not_Refresh_Online_State_Or_Data()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Prepare_Frame(9U, 1U, FDCAN_DLC_BYTES_8);
    Write_Int32_BE(&CAN3_Manage_Object.Rx_Buffer[0], 2100);
    Fake_Tick = 0U;
    motor.CAN_RxCpltCallback();
    motor.TIM_100ms_Alive_PeriodElapsedCallback();
    Fake_Tick = 100U;
    motor.TIM_100ms_Alive_PeriodElapsedCallback();

    Prepare_Frame(27U, 1U, FDCAN_DLC_BYTES_6);
    Write_Int32_BE(&CAN3_Manage_Object.Rx_Buffer[0], 999999);
    motor.CAN_RxCpltCallback();
    motor.TIM_100ms_Alive_PeriodElapsedCallback();

    assert(motor.Get_Status() == Motor_VESC_Status_DISABLE);
    assert(motor.Get_Now_ERPM() == 2100);
    assert(motor.Get_Tachometer() == 0);
}

void Test_Offline_Status_Does_Not_Block_Enabled_RPM_Command()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Feed_Status1_And_Mark_Online(motor, 1U);
    motor.Set_Motor_Enable(true);
    motor.Set_Control_Omega(1.0f);
    Fake_Tick = 100U;
    motor.TIM_100ms_Alive_PeriodElapsedCallback();
    Reset_Transmit();

    motor.TIM_Send_PeriodElapsedCallback();

    assert(Transmit_ID == ((3U << 8U) | 1U));
    assert(Read_Int32_BE(Transmit_Buffer) == 201);
}

void Test_Reconnect_Preserves_The_Remote_Target()
{
    Class_Motor_VESC motor;
    motor.Init(&hfdcan3, 1U, 21U, 1.0f, 1000);
    Feed_Status1_And_Mark_Online(motor, 1U);
    motor.Set_Motor_Enable(true);
    motor.Set_Control_Omega(1.0f);
    Fake_Tick = 100U;
    motor.TIM_100ms_Alive_PeriodElapsedCallback();
    Fake_Tick = 101U;
    Feed_Status1_And_Mark_Online(motor, 1U);

    assert(motor.Get_Status() == Motor_VESC_Status_ENABLE);
    assert(motor.Get_Motor_Enable());
    assert(motor.Get_Control_ERPM() == 201);
    assert(motor.Get_Control_Omega() == 1.0f);
}

extern "C" std::uint32_t HAL_GetTick()
{
    return Fake_Tick;
}
}

void CAN_Init(FDCAN_HandleTypeDef *, CAN_Callback)
{
}

void CAN_Extended_Init(FDCAN_HandleTypeDef *, CAN_Callback)
{
}

std::uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *, std::uint16_t, std::uint8_t *, std::uint16_t)
{
    return HAL_OK;
}

std::uint8_t CAN_Transmit_Extended_Data(FDCAN_HandleTypeDef *hfdcan, std::uint32_t id, std::uint8_t *data, std::uint16_t length)
{
    Transmit_Handler = hfdcan;
    Transmit_ID = id;
    Transmit_Length = length;
    std::memcpy(Transmit_Buffer, data, length);
    return HAL_OK;
}

int main()
{
    Test_Init_Is_Disabled_And_Zeroed();
    Test_Current_Mode_Uses_Ampere_Scaling_And_Limit();
    Test_Current_Brake_Mode_Uses_Positive_Ampere_Scaling_And_Limit();
    Test_Duty_Mode_Uses_Protocol_Scaling_And_Limit();
    Test_Position_Mode_Uses_Degree_Scaling_And_Limit();
    Test_Current_Rel_Mode_Uses_Protocol_Scaling_And_Limit();
    Test_Changing_Method_Clears_Previous_Control_Targets();
    Test_Disabling_Clears_Non_RPM_Target_Before_Reenable();
    Test_Zero_One_And_Negative_Rad_Per_Second_Conversion();
    Test_Target_ERPM_Is_Clamped();
    Test_Enabled_Before_Online_Sends_Set_RPM_Big_Endian();
    Test_Disabled_Sends_Zero_Current_Not_Zero_RPM();
    Test_Status1_Decodes_ERPM_Current_Duty_And_Omega();
    Test_Status4_Decodes_Temperatures_Current_And_Position();
    Test_Status5_Accepts_Eight_Bytes_And_Decodes_Voltage();
    Test_Each_Status_Frame_Tracks_Its_Own_Freshness();
    Test_Rejects_Standard_Remote_Unknown_And_Wrong_VESC_ID();
    Test_Status1_And_Status4_Require_Eight_Bytes();
    Test_Status5_Rejects_Six_Bytes();
    Test_Status_Frame_Expires_After_100ms();
    Test_Invalid_Frame_Does_Not_Refresh_Online_State_Or_Data();
    Test_Offline_Status_Does_Not_Block_Enabled_RPM_Command();
    Test_Reconnect_Preserves_The_Remote_Target();
    return 0;
}
