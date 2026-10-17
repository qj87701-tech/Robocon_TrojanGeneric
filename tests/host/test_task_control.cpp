#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "test_assert.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Protocol/pro_sbus/pro_sbus.h"
#include "1_Middleware/Protocol/pro_upper/pro_upper.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "2_Device/IMU/HI06/dvc_imu_hi06.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Motor/Motor_VESC/dvc_motor_vesc.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "4_Task/tsk_4swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_config_and_callback/tsk_config_and_callback.h"
#include "4_Task/tsk_debug/tsk_debug.h"
#include "4_Task/tsk_remote/tsk_remote.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

GPIO_TypeDef Fake_GPIOF_Instance{0xF00DU};
FDCAN_HandleTypeDef hfdcan1{reinterpret_cast<void *>(0x10000U)};
FDCAN_HandleTypeDef hfdcan2{reinterpret_cast<void *>(0x20000U)};
FDCAN_HandleTypeDef hfdcan3{reinterpret_cast<void *>(0x30000U)};
TIM_HandleTypeDef htim5{TIM5};
UART_HandleTypeDef huart1{USART1};
UART_HandleTypeDef huart4{UART4};
UART_HandleTypeDef huart7{UART7};
UART_HandleTypeDef huart9{UART9};
UART_HandleTypeDef huart10{USART10};
Class_Timestamp SYS_Timestamp;
Fake_Motor_State &fake_motor_state = DM_J8006.Get_Fake_State();

int fake_sequence = 0;
int fake_timestamp_init_sequence = 0;
int fake_timestamp_overflow_count = 0;
TIM_HandleTypeDef *fake_timestamp_handler = nullptr;
std::uint64_t fake_timestamp_now = 0U;

void Call_TIM_Callback(TIM_HandleTypeDef *htim);

namespace
{
std::uint32_t Fake_Tick = 0U;
GPIO_PinState Key1_Level = GPIO_PIN_SET;
GPIO_PinState Key2_Level = GPIO_PIN_SET;
CAN_Callback Registered_CAN1_Callback = nullptr;
CAN_Callback Registered_CAN3_Callback = nullptr;
bool CAN1_Init_Saw_Init_Finished = true;
bool CAN3_Init_Saw_Init_Finished = true;
int CAN1_Init_Sequence = 0;
int CAN3_Init_Sequence = 0;
int CAN1_Standard_Init_Count = 0;
int CAN3_Extended_Init_Count = 0;
int TIM5_Start_Sequence = 0;
int TIM5_Start_Count = 0;
HAL_StatusTypeDef TIM5_Start_Result = HAL_OK;
HAL_StatusTypeDef UART_Transmit_Result = HAL_OK;
int Error_Handler_Count = 0;
int UART_Transmit_Count = 0;
std::string UART_Output;
int Vofa_Transmit_Count = 0;
std::vector<std::uint8_t> Vofa_Output;
int UART4_Receive_Start_Count = 0;
int UART7_Receive_Start_Count = 0;
int UART9_Receive_Start_Count = 0;
int UART10_Receive_Start_Count = 0;
bool Keep_Remote_Alive = false;

std::uint8_t Remote_Packet[31] = {
    0x0F, 0xC8, 0x40, 0x1F, 0xC2, 0x01, 0xF0, 0xFF,
    0xAA, 0xA8, 0xAA, 0xAA, 0xFF, 0x01, 0xD0, 0xFF,
    0x00, 0x08, 0xE0, 0x84, 0x49, 0x53, 0x05, 0xFF,
    0xAF, 0x05, 0x00, 0x00, 0x88, 0x00, 0x00};

void Reset_Fakes(const std::uint32_t initial_tick = 0U)
{
    Fake_Tick = initial_tick;
    Key1_Level = GPIO_PIN_SET;
    Key2_Level = GPIO_PIN_SET;
    Registered_CAN1_Callback = nullptr;
    Registered_CAN3_Callback = nullptr;
    CAN1_Init_Saw_Init_Finished = true;
    CAN3_Init_Saw_Init_Finished = true;
    CAN1_Init_Sequence = 0;
    CAN3_Init_Sequence = 0;
    CAN1_Standard_Init_Count = 0;
    CAN3_Extended_Init_Count = 0;
    TIM5_Start_Sequence = 0;
    TIM5_Start_Count = 0;
    TIM5_Start_Result = HAL_OK;
    UART_Transmit_Result = HAL_OK;
    Error_Handler_Count = 0;
    UART_Transmit_Count = 0;
    UART_Output.clear();
    Vofa_Transmit_Count = 0;
    Vofa_Output.clear();
    UART4_Receive_Start_Count = 0;
    UART7_Receive_Start_Count = 0;
    UART9_Receive_Start_Count = 0;
    UART10_Receive_Start_Count = 0;
    Keep_Remote_Alive = false;
    Remote_Packet[25] = 0x05U;
    Remote_Packet[26] = 0x00U;
    Remote_Packet[27] = 0x00U;
    fake_sequence = 0;
    fake_timestamp_init_sequence = 0;
    fake_timestamp_overflow_count = 0;
    fake_timestamp_handler = nullptr;
    fake_timestamp_now = 0U;
    fake_motor_state = {};
    DM_Chassis_LF.Get_Fake_State() = {};
    DM_Chassis_RF.Get_Fake_State() = {};
    DM_Chassis_LB.Get_Fake_State() = {};
    DM_Chassis_RB.Get_Fake_State() = {};
    VESC_Chassis_LF.Get_Fake_State() = {};
    VESC_Chassis_RF.Get_Fake_State() = {};
    VESC_Chassis_LB.Get_Fake_State() = {};
    VESC_Chassis_RB.Get_Fake_State() = {};
    Fake_NRF24L01 = {};
    Fake_HI06 = {};
}

void Run_At(const std::uint32_t tick)
{
    Fake_Tick = tick;
    Task_Loop();
}

void Advance_10ms()
{
    if (Keep_Remote_Alive)
    {
        assert(SBUS_Receive.Data_Process(Remote_Packet) == 1U);
    }
    Run_At(Fake_Tick + 10U);
}

void Enable_Debug_Remote()
{
    Keep_Remote_Alive = true;
    assert(SBUS_Receive.Data_Process(Remote_Packet) == 1U);
    Advance_10ms();
    assert(fake_motor_state.motor_enable);
}

void Set_Remote_Function(const std::uint8_t index, const bool pressed)
{
    assert(index < 16U);
    std::uint8_t &data = Remote_Packet[26U + index / 8U];
    const std::uint8_t mask = static_cast<std::uint8_t>(1U << (index % 8U));
    if (pressed)
    {
        data |= mask;
    }
    else
    {
        data &= static_cast<std::uint8_t>(~mask);
    }
    assert(SBUS_Receive.Data_Process(Remote_Packet) == 1U);
}

void Set_Key(const std::uint16_t pin, const GPIO_PinState state)
{
    if (pin == Key1_Pin)
    {
        Key1_Level = state;
    }
    else
    {
        assert(pin == Key2_Pin);
        Key2_Level = state;
    }
}

void Press_And_Release(const std::uint16_t pin)
{
    Set_Key(pin, GPIO_PIN_RESET);
    Advance_10ms();
    Advance_10ms();
    Set_Key(pin, GPIO_PIN_SET);
    Advance_10ms();
    Advance_10ms();
}

void Test_Initialization_Contract()
{
    Reset_Fakes();
    Task_Init();

    assert(init_finished);
    assert(fake_timestamp_handler == &htim5);
    assert(fake_motor_state.can_handler == &hfdcan1);
    assert(fake_motor_state.init_rx_id == 0x55U);
    assert(fake_motor_state.init_tx_id == 0x05U);
    assert(fake_motor_state.method == Motor_DM_Control_Method_NORMAL_OMEGA);
    assert(fake_motor_state.angle_max == 12.5f);
    assert(fake_motor_state.omega_max == 45.0f);
    assert(fake_motor_state.torque_max == 20.0f);
    assert(fake_motor_state.control_omega == 0.0f);
    assert(!CAN1_Init_Saw_Init_Finished);
    assert(!CAN3_Init_Saw_Init_Finished);
    assert(Registered_CAN1_Callback != nullptr);
    assert(Registered_CAN3_Callback != nullptr);
    assert(CAN1_Standard_Init_Count == 1);
    assert(CAN3_Extended_Init_Count == 1);
    assert(TIM5_Start_Count == 1);
    assert(Fake_HI06.Init_Count == 1);
    assert(UART4_Receive_Start_Count == 1);
    assert(UART4_Manage_Object.Callback_Function != nullptr);
    assert(UART7_Receive_Start_Count == 1);
    assert(UART9_Receive_Start_Count == 1);
    assert(UART9_Manage_Object.Callback_Function != nullptr);
    assert(UART10_Receive_Start_Count == 1);
    assert(Error_Handler_Count == 0);
    assert(!fake_motor_state.motor_enable);
    assert(fake_motor_state.enter_count == 0);
    assert(fake_motor_state.exit_count == 1);
    assert(DM_Chassis_LF.Get_Fake_State().can_handler == &hfdcan1);
    assert(DM_Chassis_RF.Get_Fake_State().can_handler == &hfdcan1);
    assert(DM_Chassis_LB.Get_Fake_State().can_handler == &hfdcan1);
    assert(DM_Chassis_RB.Get_Fake_State().can_handler == &hfdcan1);
    assert(VESC_Chassis_LF.Get_Fake_State().can_handler == &hfdcan3);
    assert(VESC_Chassis_RF.Get_Fake_State().can_handler == &hfdcan3);
    assert(VESC_Chassis_LB.Get_Fake_State().can_handler == &hfdcan3);
    assert(VESC_Chassis_RB.Get_Fake_State().can_handler == &hfdcan3);
    assert(DM_Chassis_LF.Get_Fake_State().exit_count == 1);
    assert(DM_Chassis_RF.Get_Fake_State().exit_count == 1);
    assert(DM_Chassis_LB.Get_Fake_State().exit_count == 1);
    assert(DM_Chassis_RB.Get_Fake_State().exit_count == 1);
    assert(fake_timestamp_init_sequence < fake_motor_state.init_sequence);
    assert(fake_motor_state.set_omega_sequence < CAN1_Init_Sequence);
    assert(CAN1_Init_Sequence < CAN3_Init_Sequence);
    assert(CAN3_Init_Sequence < TIM5_Start_Sequence);
}

void Test_UART4_Callback_Forwards_To_HI06()
{
    Reset_Fakes();
    Task_Init();
    std::uint8_t data[3] = {0x5AU, 0xA5U, 0x01U};

    UART4_Manage_Object.Callback_Function(data, sizeof(data));

    assert(Fake_HI06.UART_Callback_Count == 1);
    assert(Fake_HI06.UART_Buffer == data);
    assert(Fake_HI06.UART_Length == sizeof(data));
}

void Receive_Upper_Frame()
{
    std::uint8_t frame[32] = {0x55U, 0xaaU, 0xffU, 0x7fU, 0x00U, 0x80U, 0x18U, 0xfcU};
    frame[31] = 0x77U;
    assert(UART7_Manage_Object.Callback_Function != nullptr);
    UART7_Manage_Object.Callback_Function(frame, sizeof(frame));
}

void Test_UART7_Init_And_Foreground_Receive()
{
    Reset_Fakes();
    Task_Init();
    assert(Upper.Get_Status() == Upper_Status_OFFLINE);
    assert(!Upper.Get_Data().Valid && !Upper.Get_CRC_Check_Enable());
    Receive_Upper_Frame();
    assert(Upper.Get_Data().Frame_Count == 0U);
    Run_At(9U);
    assert(Upper.Get_Data().Frame_Count == 0U);
    Run_At(10U);
    assert(Upper.Get_Data().Frame_Count == 1U && Upper.Get_Data().Valid);
    assert(Upper.Get_Data().Speed_X == -32768 && Upper.Get_Data().Speed_Y == 32767 && Upper.Get_Data().Speed_W == -1000);
    assert(Remote_Chassis_Command.Speed_X == 0.0f && Remote_Chassis_Command.Speed_Y == 0.0f && Remote_Chassis_Command.Speed_W == 0.0f);
    assert(!fake_motor_state.motor_enable && !DM_Chassis_LF.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_LF.Get_Fake_State().control_omega == 0.0f);
}

void Test_Upper_Print_Not_Automatic()
{
    Reset_Fakes();
    Task_Init();
    Receive_Upper_Frame();
    Run_At(10U);
    Run_At(500U);
    assert(UART_Output.find("UPPER") == std::string::npos);
    Debug_UART10_Print_Upper_Data();
    assert(UART_Output.find("raw_xyw=-32768,32767,-1000") != std::string::npos);
    Debug_UART10_Print_Upper_Status_Data();
    assert(UART_Output.find("crc_enable=0") != std::string::npos);
}

void Test_HI06_Processes_Every_10ms()
{
    Reset_Fakes();
    Task_Init();
    assert(!fake_motor_state.motor_enable);

    Run_At(9U);
    assert(Fake_HI06.TIM_10ms_Count == 0);
    Run_At(10U);
    assert(Fake_HI06.TIM_10ms_Count == 1);
    Run_At(20U);
    assert(Fake_HI06.TIM_10ms_Count == 2);
}

void Test_VESC_And_Vofa_Send_Run_Every_10ms()
{
    Reset_Fakes();
    Task_Init();

    assert(UART9_Manage_Object.UART_Handler == &huart9);
    assert(UART9_Manage_Object.Callback_Function != nullptr);
    Run_At(9U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 0);
    Run_At(10U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(19U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(20U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 2);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_RB.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 2);
    assert(Vofa_Output.size() == 24U * sizeof(float) + sizeof(std::uint32_t));

    for (std::uint32_t tick = 21U; tick <= 1000U; ++tick)
    {
        Run_At(tick);
    }
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 100);
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_RB.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 100);
}

void Test_Enabled_VESC_Send_Runs_Every_10ms()
{
    Reset_Fakes();
    Task_Init();
    Remote_Packet[25] = 0x06U;
    Keep_Remote_Alive = true;

    for (int cycle = 1; cycle <= 100; ++cycle)
    {
        Advance_10ms();
        assert(VESC_Chassis_LF.Get_Fake_State().motor_enable);
        assert(VESC_Chassis_LF.Get_Fake_State().send_count == cycle);
    }
    assert(VESC_Chassis_RF.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_LB.Get_Fake_State().send_count == 0);
    assert(VESC_Chassis_RB.Get_Fake_State().send_count == 0);
    assert(Vofa_Transmit_Count == 100);
}

void Test_Vofa_Output_Contains_VESC_Waveform_Channels()
{
    Reset_Fakes();
    Task_Init();

    Fake_VESC_State &vesc = VESC_Chassis_LF.Get_Fake_State();
    vesc.control_omega = 6.28318531f;
    vesc.now_omega = -3.14159265f;
    vesc.motor_current = 8.5f;
    vesc.duty = 0.375f;
    vesc.input_current = 3.25f;
    vesc.input_voltage = 48.0f;
    vesc.mos_temperature = 52.25f;
    vesc.status_1_valid = true;
    vesc.status_4_valid = true;
    vesc.status_5_valid = true;
    vesc.status_5_update_count = 1U;
    Debug_Vofa_10ms_PeriodElapsedCallback();

    assert(Vofa_Transmit_Count == 1);
    assert(Vofa_Output.size() == 24U * sizeof(float) + sizeof(std::uint32_t));

    float channel[24]{};
    std::uint32_t frame_tail = 0U;
    std::memcpy(channel, Vofa_Output.data(), sizeof(channel));
    std::memcpy(&frame_tail, Vofa_Output.data() + sizeof(channel), sizeof(frame_tail));
    assert(std::fabs(channel[0] - 60.0f) < 0.001f);
    assert(std::fabs(channel[1] + 30.0f) < 0.001f);
    assert(std::fabs(channel[2] - 90.0f) < 0.001f);
    assert(std::fabs(channel[3] - 8.5f) < 0.0001f);
    assert(std::fabs(channel[4] - 37.5f) < 0.0001f);
    assert(std::fabs(channel[5] - 3.25f) < 0.0001f);
    assert(std::fabs(channel[6] - 48.0f) < 0.0001f);
    assert(std::fabs(channel[7] - 52.25f) < 0.0001f);
    assert(frame_tail == 0x7F800000U);
}

void Test_Vofa_Output_Uses_Fixed_10ms_Period_When_Feedback_Is_Invalid()
{
    Reset_Fakes();
    Task_Init();
    Fake_VESC_State &vesc = VESC_Chassis_LF.Get_Fake_State();

    vesc.status_1_valid = true;
    vesc.status_4_valid = true;
    vesc.status_5_valid = false;
    vesc.status_5_update_count = 1U;
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 1);
    float channel[24]{};
    std::memcpy(channel, Vofa_Output.data(), sizeof(channel));
    assert(std::isnan(channel[6]));

    vesc.status_5_valid = true;
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 2);
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 3);

    vesc.status_5_update_count = 2U;
    Debug_Vofa_10ms_PeriodElapsedCallback();
    assert(Vofa_Transmit_Count == 4);
}

void Test_USART9_MIT_Command_Applies_Before_Chassis_Control()
{
    Reset_Fakes();
    Task_Init();
    const std::uint8_t command[] = "dm_kp=8.5#";
    UART9_Manage_Object.Callback_Function(const_cast<std::uint8_t *>(command), sizeof(command) - 1U);

    Remote_Packet[25] = 0x06U;
    for (std::uint8_t i = 0U; i < 31U; i++)
    {
        Fake_NRF24L01.Rx_Packet[i] = Remote_Packet[i];
    }
    Fake_NRF24L01.Receive_Result = 1U;
    Run_At(10U);

    assert(std::fabs(DM_Chassis_LF.Get_Fake_State().k_p - 8.5f) < 0.0001f);
}

void Test_Key_Steps_Hold_And_Release()
{
    Reset_Fakes();
    Task_Init();
    Enable_Debug_Remote();

    Set_Key(Key1_Pin, GPIO_PIN_RESET);
    Advance_10ms();
    assert(fake_motor_state.control_omega == 0.0f);
    Advance_10ms();
    assert(fake_motor_state.control_omega == 1.0f);
    Advance_10ms();
    Advance_10ms();
    assert(fake_motor_state.control_omega == 1.0f);

    Set_Key(Key1_Pin, GPIO_PIN_SET);
    Advance_10ms();
    Advance_10ms();
    Set_Key(Key1_Pin, GPIO_PIN_RESET);
    Advance_10ms();
    Advance_10ms();
    assert(fake_motor_state.control_omega == 2.0f);

    Set_Key(Key1_Pin, GPIO_PIN_SET);
    Advance_10ms();
    Advance_10ms();
    Press_And_Release(Key2_Pin);
    assert(fake_motor_state.control_omega == 1.0f);
}

void Test_Speed_Clamps_At_Both_Limits()
{
    Reset_Fakes();
    Task_Init();
    Enable_Debug_Remote();

    for (int i = 0; i < 25; ++i)
    {
        Press_And_Release(Key1_Pin);
    }
    assert(fake_motor_state.control_omega == 20.0f);

    for (int i = 0; i < 45; ++i)
    {
        Press_And_Release(Key2_Pin);
    }
    assert(fake_motor_state.control_omega == -20.0f);
}

void Test_Dual_Key_Zero_Latch()
{
    Reset_Fakes();
    Task_Init();
    Enable_Debug_Remote();
    Press_And_Release(Key1_Pin);
    Press_And_Release(Key1_Pin);
    assert(fake_motor_state.control_omega == 2.0f);

    Key1_Level = GPIO_PIN_RESET;
    Key2_Level = GPIO_PIN_RESET;
    Advance_10ms();
    Advance_10ms();
    assert(fake_motor_state.control_omega == 0.0f);

    Key1_Level = GPIO_PIN_SET;
    Advance_10ms();
    Advance_10ms();
    Key1_Level = GPIO_PIN_RESET;
    Advance_10ms();
    Advance_10ms();
    assert(fake_motor_state.control_omega == 0.0f);

    Key1_Level = GPIO_PIN_SET;
    Key2_Level = GPIO_PIN_SET;
    Advance_10ms();
    Advance_10ms();
    Press_And_Release(Key1_Pin);
    assert(fake_motor_state.control_omega == 1.0f);
}

void Test_Motor_Enable_And_Timeout_Disable()
{
    Reset_Fakes();
    Task_Init();
    Enable_Debug_Remote();

    assert(fake_motor_state.enter_count == 1);
    fake_motor_state.control_omega = 6.0f;

    assert(SBUS_Receive.Data_Process(Remote_Packet) == 1U);
    Keep_Remote_Alive = false;
    for (int i = 0; i < 99; ++i)
    {
        Advance_10ms();
    }

    assert(SBUS_Receive.SBUS_Data.Online == 1U);
    assert(fake_motor_state.motor_enable);

    Advance_10ms();
    assert(SBUS_Receive.SBUS_Data.Online == 0U);
    assert(!fake_motor_state.motor_enable);
    assert(fake_motor_state.exit_count == 2);
    assert(fake_motor_state.control_omega == 0.0f);
}

void Test_Remote_Debug_Buttons_Trigger_Once()
{
    Reset_Fakes();
    Task_Init();
    Enable_Debug_Remote();

    Set_Remote_Function(0U, true);
    Advance_10ms();
    assert(fake_motor_state.control_omega == 1.0f);
    Advance_10ms();
    Advance_10ms();
    assert(fake_motor_state.control_omega == 1.0f);

    Set_Remote_Function(0U, false);
    Advance_10ms();
    Set_Remote_Function(0U, true);
    Advance_10ms();
    assert(fake_motor_state.control_omega == 2.0f);

    Set_Remote_Function(0U, false);
    Set_Remote_Function(1U, true);
    Advance_10ms();
    assert(fake_motor_state.control_omega == 1.0f);
}

void Test_Remote_Buttons_Require_Debug_Mode()
{
    Reset_Fakes();
    Task_Init();
    Remote_Packet[25] = 0x06U;
    Keep_Remote_Alive = true;
    assert(SBUS_Receive.Data_Process(Remote_Packet) == 1U);
    Advance_10ms();
    assert(fake_motor_state.motor_enable);

    Set_Remote_Function(0U, true);
    Advance_10ms();
    assert(fake_motor_state.control_omega == 0.0f);
}

void Test_Periodic_Output_Prints_Swerve_Control_Diagnostic_Every_100ms()
{
    Reset_Fakes();
    Task_Init();
    SBUS_Receive.SBUS_Data.Online = 1U;
    SBUS_Receive.SBUS_Data.Mode = 2U;
    SBUS_Receive.SBUS_Data.Motor_Enable = 1U;
    Remote_Chassis_Command.Online = 1U;
    Remote_Chassis_Command.Mode = 2U;
    Remote_Chassis_Command.Motor_Enable = 1U;
    Remote_Chassis_Command.Speed_X = 0.0f;
    Remote_Chassis_Command.Speed_Y = 1.0f;
    Remote_Chassis_Command.Speed_W = 0.0f;
    DM_Chassis_LF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_LF.Get_Fake_State().control_status = Motor_DM_Control_Status_ENABLE;
    DM_Chassis_LF.Get_Fake_State().now_angle = 0.0f;
    VESC_Chassis_LF.Get_Fake_State().status = Motor_VESC_Status_DISABLE;
    VESC_Chassis_LF.Get_Fake_State().now_erpm = -123;
    Chassis_Swerve_10ms_PeriodElapsedCallback();

    Debug_UART10_Print_Swerve_Data();
    assert(UART_Transmit_Count == 1);
    assert(UART_Output ==
           "SW RC=1 mode=2 motor_enable=1 speed=0.00,1.00,0.00 "
           "DM enable=1 online=1 control=1 target=0.00 target_omega=0.00 now=0.00 error=0.00 ratio=0.00 "
           "VESC enable=1 online=0 target_erpm=0 now_erpm=-123\r\n");
    assert(UART_Output.find("IMU ") == std::string::npos);

    UART_Transmit_Count = 0;
    UART_Output.clear();

    Run_At(50U);
    assert(UART_Transmit_Count == 0);
    Run_At(99U);
    assert(UART_Transmit_Count == 0);
    Run_At(100U);

    assert(UART_Transmit_Count == 1);
    assert(UART_Output.rfind("SW ", 0U) == 0U);
}

void Test_IMU_Data_Has_Independent_Print_Functions()
{
    Reset_Fakes();
    Task_Init();
    Fake_HI06.Status = HI06_Status_ONLINE;
    Fake_HI06.Data.Valid = 0x080019BDU;
    Fake_HI06.Data.Main_Status = 512U;
    Fake_HI06.Data.Accel_X = 1.25f;
    Fake_HI06.Data.Accel_Y = -2.50f;
    Fake_HI06.Data.Accel_Z = 3.75f;
    Fake_HI06.Data.Gyro_X = 0.50f;
    Fake_HI06.Data.Gyro_Y = -1.25f;
    Fake_HI06.Data.Gyro_Z = 2.00f;
    Fake_HI06.Data.Temperature = 27.50f;
    Fake_HI06.Data.Rx_Byte_Count = 82U;
    Fake_HI06.Data.Frame_Count = 3U;
    Fake_HI06.Data.CRC_Error_Count = 1U;
    Fake_HI06.Data.Invalid_Frame_Count = 2U;
    Fake_HI06.Data.Buffer_Overflow_Count = 4U;

    Debug_UART10_Print_IMU_Acceleration_Data();
    assert(UART_Output == "IMU online=1 acc=1.25,-2.50,3.75 temp=27.50\r\n");

    UART_Output.clear();
    Debug_UART10_Print_IMU_Gyroscope_Data();
    assert(UART_Output == "IMU online=1 gyr=0.50,-1.25,2.00 temp=27.50\r\n");

    UART_Output.clear();
    Debug_UART10_Print_IMU_Status_Data();
    assert(UART_Output == "IMU online=1 valid=0x080019BD status=512 rx=82 frame=3 crc_err=1 invalid=2 overflow=4\r\n");
}

void Test_Independent_IMU_Print_Skips_When_USART10_DMA_Is_Busy()
{
    Reset_Fakes();
    Task_Init();
    Fake_HI06.Status = HI06_Status_ONLINE;
    UART_Transmit_Result = HAL_BUSY;

    Debug_UART10_Print_IMU_Data();
    assert(UART_Transmit_Count == 1);
    assert(UART_Output.empty());
}

void Test_Motor_Data_Has_An_Independent_Print_Function()
{
    Reset_Fakes();
    Task_Init();
    fake_motor_state.motor_enable = true;
    fake_motor_state.status = Motor_DM_Status_ENABLE;
    fake_motor_state.control_status = Motor_DM_Control_Status_ENABLE;
    fake_motor_state.control_omega = -2.25f;
    fake_motor_state.now_omega = -0.50f;
    fake_motor_state.now_torque = 1.25f;
    fake_motor_state.mos_temperature = 36.0f;
    fake_motor_state.rotor_temperature = 35.0f;

    Debug_UART10_Print_Motor_Data();

    assert(UART_Transmit_Count == 1);
    assert(UART_Output ==
           "DM enable=1 status=1 control_status=1 target=-2.25 omega=-0.50 "
           "torque=1.25 mos=36.00 rotor=35.00\r\n");
}

void Test_Scheduler_Wrap_And_No_Catchup_Burst()
{
    Reset_Fakes();
    Task_Init();
    Run_At(9U);
    assert(fake_motor_state.send_count == 0);
    Run_At(10U);
    assert(fake_motor_state.send_count == 1);
    Run_At(100U);
    assert(fake_motor_state.send_count == 2);
    assert(fake_motor_state.alive_count == 1);
    Run_At(1100U);
    assert(fake_motor_state.send_count == 3);
    assert(fake_motor_state.alive_count == 2);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 3);
    assert(VESC_Chassis_LF.Get_Fake_State().alive_count == 2);
    assert(Vofa_Transmit_Count == 3);

    Reset_Fakes(std::numeric_limits<std::uint32_t>::max() - 5U);
    Task_Init();
    Run_At(3U);
    assert(fake_motor_state.send_count == 0);
    assert(Vofa_Transmit_Count == 0);
    Run_At(4U);
    assert(fake_motor_state.send_count == 1);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(13U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 1);
    assert(Vofa_Transmit_Count == 1);
    Run_At(14U);
    assert(VESC_Chassis_LF.Get_Fake_State().send_count == 2);
    assert(Vofa_Transmit_Count == 2);
}

void Test_CAN1_And_CAN3_Routing()
{
    Reset_Fakes();
    Task_Init();

    std::uint8_t buffer[8]{};
    FDCAN_RxHeaderTypeDef header{};
    header.IdType = FDCAN_STANDARD_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.Identifier = 0x55U;
    Registered_CAN1_Callback(header, buffer);
    assert(fake_motor_state.receive_count == 1);
    header.Identifier = 0x11U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_LF.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x22U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_RF.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x33U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_LB.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x44U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_RB.Get_Fake_State().receive_count == 1);
    header.Identifier = 0x56U;
    Registered_CAN1_Callback(header, buffer);
    assert(fake_motor_state.receive_count == 1);

    header.IdType = FDCAN_EXTENDED_ID;
    header.Identifier = (9U << 8U) | 1U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LF.Get_Fake_State().receive_count == 1);
    header.Identifier = (9U << 8U) | 2U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_RF.Get_Fake_State().receive_count == 1);
    header.Identifier = (16U << 8U) | 3U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LB.Get_Fake_State().receive_count == 1);
    header.Identifier = (27U << 8U) | 4U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_RB.Get_Fake_State().receive_count == 1);
    header.Identifier = (9U << 8U) | 5U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LF.Get_Fake_State().receive_count == 1);
    assert(VESC_Chassis_RF.Get_Fake_State().receive_count == 1);
    assert(VESC_Chassis_LB.Get_Fake_State().receive_count == 1);
    assert(VESC_Chassis_RB.Get_Fake_State().receive_count == 1);

    header.IdType = FDCAN_EXTENDED_ID;
    header.Identifier = 0x11U;
    Registered_CAN1_Callback(header, buffer);
    assert(DM_Chassis_LF.Get_Fake_State().receive_count == 1);
    header.IdType = FDCAN_STANDARD_ID;
    header.Identifier = 1U;
    Registered_CAN3_Callback(header, buffer);
    assert(VESC_Chassis_LF.Get_Fake_State().receive_count == 1);
}

void Test_Remote_Updates_Before_Chassis_Control()
{
    Reset_Fakes();
    Task_Init();

    Remote_Packet[25] = 0x06U;
    for (std::uint8_t i = 0U; i < 31U; i++)
    {
        Fake_NRF24L01.Rx_Packet[i] = Remote_Packet[i];
    }
    Fake_NRF24L01.Receive_Result = 1U;
    DM_Chassis_LF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_RF.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_LB.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    DM_Chassis_RB.Get_Fake_State().status = Motor_DM_Status_ENABLE;
    VESC_Chassis_LF.Get_Fake_State().status = Motor_VESC_Status_ENABLE;
    VESC_Chassis_RF.Get_Fake_State().status = Motor_VESC_Status_ENABLE;
    VESC_Chassis_LB.Get_Fake_State().status = Motor_VESC_Status_ENABLE;
    VESC_Chassis_RB.Get_Fake_State().status = Motor_VESC_Status_ENABLE;
    Run_At(10U);

    assert(Remote_Chassis_Command.Online == 1U);
    assert(Remote_Chassis_Command.Mode == 2U);
    assert(DM_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_LB.Get_Fake_State().motor_enable);
    assert(!DM_Chassis_RB.Get_Fake_State().motor_enable);
    assert(VESC_Chassis_LF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RF.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_LB.Get_Fake_State().motor_enable);
    assert(!VESC_Chassis_RB.Get_Fake_State().motor_enable);
    assert(DM_Chassis_LF.Get_Fake_State().control_omega == 0.0f);
    // 行进轮速度需要等待舵向对准，本测试只验证遥控命令先于底盘控制更新以及电机使能状态。
    // 舵向误差门控和VESC速度输出由底盘任务的独立测试覆盖。
}

void Test_TIM_Routing()
{
    Reset_Fakes();
    Task_Init();

    Call_TIM_Callback(&htim5);
    assert(fake_timestamp_overflow_count == 1);
    TIM_HandleTypeDef other_timer{reinterpret_cast<void *>(0x50000U)};
    Call_TIM_Callback(&other_timer);
    assert(fake_timestamp_overflow_count == 1);
}
}

extern "C" GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio_port, const std::uint16_t gpio_pin)
{
    if (gpio_port == GPIOG)
    {
        assert(gpio_pin == NRF_IRQ_Pin);
        return GPIO_PIN_SET;
    }

    assert(gpio_port == GPIOF);
    if (gpio_pin == Key1_Pin)
    {
        return Key1_Level;
    }
    assert(gpio_pin == Key2_Pin);
    return Key2_Level;
}

extern "C" std::uint32_t HAL_GetTick()
{
    return Fake_Tick;
}

extern "C" HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
{
    assert(htim == &htim5);
    ++TIM5_Start_Count;
    TIM5_Start_Sequence = ++fake_sequence;
    return TIM5_Start_Result;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                                std::uint8_t *data,
                                                const std::uint16_t size,
                                                const std::uint32_t timeout)
{
    (void)huart;
    (void)data;
    (void)size;
    (void)timeout;
    assert(false && "打印数据不应继续使用USART1阻塞发送");
    return HAL_ERROR;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                                    std::uint8_t *data,
                                                    const std::uint16_t size)
{
    if (huart == &huart9)
    {
        ++Vofa_Transmit_Count;
        Vofa_Output.assign(data, data + size);
        return UART_Transmit_Result;
    }

    assert(huart == &huart10);
    ++UART_Transmit_Count;
    if (UART_Transmit_Result == HAL_OK)
    {
        UART_Output.append(reinterpret_cast<const char *>(data), size);
    }
    return UART_Transmit_Result;
}

extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                                           std::uint8_t *,
                                                           const std::uint16_t size)
{
    assert(size == UART_BUFFER_SIZE);
    if (huart == &huart4)
    {
        assert(!init_finished);
        ++UART4_Receive_Start_Count;
    }
    else if (huart == &huart7)
    {
        ++UART7_Receive_Start_Count;
    }
    else if (huart == &huart9)
    {
        ++UART9_Receive_Start_Count;
    }
    else
    {
        assert(huart == &huart10);
        ++UART10_Receive_Start_Count;
    }
    return HAL_OK;
}

extern "C" void Error_Handler()
{
    ++Error_Handler_Count;
}

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, const CAN_Callback callback_function)
{
    assert(callback_function != nullptr);
    if (hfdcan == &hfdcan1)
    {
        Registered_CAN1_Callback = callback_function;
        CAN1_Init_Saw_Init_Finished = init_finished;
        CAN1_Init_Sequence = ++fake_sequence;
        ++CAN1_Standard_Init_Count;
    }
    else
    {
        assert(false && "CAN3必须使用扩展帧初始化");
    }
}

void CAN_Extended_Init(FDCAN_HandleTypeDef *hfdcan, const CAN_Callback callback_function)
{
    assert(hfdcan == &hfdcan3);
    assert(callback_function != nullptr);
    Registered_CAN3_Callback = callback_function;
    CAN3_Init_Saw_Init_Finished = init_finished;
    CAN3_Init_Sequence = ++fake_sequence;
    ++CAN3_Extended_Init_Count;
}

std::uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *, std::uint16_t, std::uint8_t *, std::uint16_t)
{
    return HAL_OK;
}

std::uint8_t CAN_Transmit_Extended_Data(FDCAN_HandleTypeDef *, std::uint32_t, std::uint8_t *, std::uint16_t)
{
    return HAL_OK;
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Call_TIM_Callback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PeriodElapsedCallback(htim);
}

int main()
{
    // 类入口保持10ms调度, 同一时刻不重复发送。
    Reset_Fakes();
    Class_Task_Control task;
    task.Init();
    Fake_Tick = 9U;
    task.Loop();
    assert(Vofa_Transmit_Count == 0);
    Fake_Tick = 10U;
    task.Loop();
    assert(Vofa_Transmit_Count == 1);
    task.Loop();
    assert(Vofa_Transmit_Count == 1);
    Test_Initialization_Contract();
    Test_UART4_Callback_Forwards_To_HI06();
    Test_UART7_Init_And_Foreground_Receive();
    Test_Upper_Print_Not_Automatic();
    Test_HI06_Processes_Every_10ms();
    Test_VESC_And_Vofa_Send_Run_Every_10ms();
    Test_Enabled_VESC_Send_Runs_Every_10ms();
    Test_Vofa_Output_Contains_VESC_Waveform_Channels();
    Test_Vofa_Output_Uses_Fixed_10ms_Period_When_Feedback_Is_Invalid();
    Test_USART9_MIT_Command_Applies_Before_Chassis_Control();
    Test_Key_Steps_Hold_And_Release();
    Test_Speed_Clamps_At_Both_Limits();
    Test_Dual_Key_Zero_Latch();
    Test_Periodic_Output_Prints_Swerve_Control_Diagnostic_Every_100ms();
    Test_IMU_Data_Has_Independent_Print_Functions();
    Test_Independent_IMU_Print_Skips_When_USART10_DMA_Is_Busy();
    Test_Motor_Enable_And_Timeout_Disable();
    Test_Remote_Debug_Buttons_Trigger_Once();
    Test_Remote_Buttons_Require_Debug_Mode();
    Test_Motor_Data_Has_An_Independent_Print_Function();
    Test_Scheduler_Wrap_And_No_Catchup_Burst();
    Test_CAN1_And_CAN3_Routing();
    Test_Remote_Updates_Before_Chassis_Control();
    Test_TIM_Routing();
    std::puts("Task control: existing safety/scheduling and upper integration tests passed");
    return 0;
}
