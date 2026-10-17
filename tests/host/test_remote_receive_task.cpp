#include <cmath>
#include <cstdio>
#include <cstdint>

#include "test_assert.h"
#include "2_Device/NRF24L01/dvc_nrf24l01.h"
#include "4_Task/tsk_remote/tsk_remote.h"

namespace
{
void Assert_Near(const float value, const float expected)
{
    if (std::fabs(value - expected) >= 1.0e-5f)
    {
        std::fprintf(stderr, "expected %.8f, got %.8f\n", expected, value);
        assert(false);
    }
}

void Pack_Remote_Data(const std::uint16_t channel_0, const std::uint16_t channel_1, const std::uint16_t channel_3, const std::uint8_t mode, const std::uint8_t motor_enable)
{
    std::uint16_t channel[16]{};
    channel[0] = channel_0;
    channel[1] = channel_1;
    channel[2] = 1000U;
    channel[3] = channel_3;
    for (std::uint8_t i = 4U; i < 16U; i++)
    {
        channel[i] = 1000U;
    }

    Fake_NRF24L01 = {};
    Fake_NRF24L01.Rx_Packet[0] = 0x0FU;
    Fake_NRF24L01.Rx_Packet[23] = 0xFFU;
    Fake_NRF24L01.Rx_Packet[24] = 0xAFU;
    Fake_NRF24L01.Rx_Packet[25] = static_cast<std::uint8_t>((mode & 0x03U) | ((motor_enable & 0x01U) << 2U));
    Fake_NRF24L01.Rx_Packet[28] = 0x88U;

    for (std::uint8_t channel_index = 0U; channel_index < 16U; channel_index++)
    {
        for (std::uint8_t bit_index = 0U; bit_index < 11U; bit_index++)
        {
            if ((channel[channel_index] & (1U << bit_index)) != 0U)
            {
                std::uint16_t packet_bit = static_cast<std::uint16_t>(channel_index * 11U + bit_index);
                Fake_NRF24L01.Rx_Packet[1U + packet_bit / 8U] |= static_cast<std::uint8_t>(1U << (packet_bit % 8U));
            }
        }
    }

    Fake_NRF24L01.Receive_Result = 1U;
}

void Receive_One_Packet()
{
    Remote_10ms_PeriodElapsedCallback();
    Fake_NRF24L01.Receive_Result = 0U;
}

void Test_Remote_Init_Loads_Safe_Command()
{
    Fake_NRF24L01 = {};
    Remote_Init();

    assert(Fake_NRF24L01.Init_Count == 1);
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);
    assert(Remote_Chassis_Command.Online == 0U);
    assert(Remote_Chassis_Command.Mode == 0U);
    assert(Remote_Chassis_Command.Motor_Enable == 0U);
}

void Test_Channel_Endpoints_Map_To_Minus_Max_Zero_And_Plus_Max()
{
    Remote_Init();
    Pack_Remote_Data(200U, 1000U, 1800U, 2U, 1U);
    Receive_One_Packet();

    Assert_Near(Remote_Chassis_Command.Speed_Y, -10.55575f);
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 23.45722389f);
    assert(Remote_Chassis_Command.Online == 1U);
    assert(Remote_Chassis_Command.Mode == 2U);
    assert(Remote_Chassis_Command.Motor_Enable == 1U);
}

void Test_Out_Of_Range_Channels_Clamp_And_Center_Dead_Zone_Clears()
{
    Remote_Init();
    Pack_Remote_Data(0U, 2047U, 1001U, 2U, 1U);
    Receive_One_Packet();

    Assert_Near(Remote_Chassis_Command.Speed_Y, -10.55575f);
    Assert_Near(Remote_Chassis_Command.Speed_X, 10.55575f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);
}

void Test_Channel_Zero_One_And_Three_Map_To_Y_X_And_W()
{
    Remote_Init();
    Pack_Remote_Data(1200U, 1400U, 1600U, 2U, 1U);
    Receive_One_Packet();

    Assert_Near(Remote_Chassis_Command.Speed_Y, 2.6389375f);
    Assert_Near(Remote_Chassis_Command.Speed_X, 5.277875f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 17.59291840f);
}

void Test_Remote_Gear_Range_Is_Mapped_Only_Once()
{
    Remote_Init();

    Pack_Remote_Data(1000U, 1000U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);

    Pack_Remote_Data(1000U, 1200U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 2.6389375f);

    Pack_Remote_Data(1000U, 1400U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 5.277875f);

    Pack_Remote_Data(1000U, 1600U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 7.9168125f);

    Pack_Remote_Data(1000U, 1800U, 1000U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 10.55575f);

    Pack_Remote_Data(1000U, 1000U, 1200U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 5.86430597f);

    Pack_Remote_Data(1000U, 1000U, 1400U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 11.72861195f);

    Pack_Remote_Data(1000U, 1000U, 1600U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 17.59291840f);

    Pack_Remote_Data(1000U, 1000U, 1800U, 2U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_W, 23.45722389f);
}

void Test_Non_Manual_Disable_And_Offline_Clear_All_Chassis_Speeds()
{
    Remote_Init();
    Pack_Remote_Data(1200U, 1400U, 1600U, 1U, 1U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);

    Pack_Remote_Data(1200U, 1400U, 1600U, 2U, 0U);
    Receive_One_Packet();
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);

    Pack_Remote_Data(1200U, 1400U, 1600U, 2U, 1U);
    Receive_One_Packet();
    for (std::uint8_t i = 0U; i < 100U; i++)
    {
        Remote_10ms_PeriodElapsedCallback();
    }
    Assert_Near(Remote_Chassis_Command.Speed_X, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_Y, 0.0f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 0.0f);
    assert(Remote_Chassis_Command.Online == 0U);
}

void Test_NRF_Polling_And_One_Second_Timeout_Are_Preserved()
{
    Remote_Init();
    Pack_Remote_Data(1000U, 1000U, 1000U, 2U, 1U);
    Receive_One_Packet();
    assert(Fake_NRF24L01.Receive_Count == 1);

    for (std::uint8_t i = 0U; i < 99U; i++)
    {
        Remote_10ms_PeriodElapsedCallback();
    }
    assert(Remote_Chassis_Command.Online == 1U);

    Remote_10ms_PeriodElapsedCallback();
    assert(Remote_Chassis_Command.Online == 0U);
    assert(Fake_NRF24L01.Receive_Count == 101);
}
}

int main()
{
    // 类入口仍向既有底盘指令快照输出, 不创建第二个接收器。
    Class_Remote_Control remote;
    remote.Init();
    Pack_Remote_Data(200U, 1000U, 1800U, 2U, 1U);
    remote.TIM_10ms_PeriodElapsedCallback();
    Assert_Near(Remote_Chassis_Command.Speed_Y, -10.55575f);
    Assert_Near(Remote_Chassis_Command.Speed_W, 23.45722389f);
    Test_Remote_Init_Loads_Safe_Command();
    Test_Channel_Endpoints_Map_To_Minus_Max_Zero_And_Plus_Max();
    Test_Out_Of_Range_Channels_Clamp_And_Center_Dead_Zone_Clears();
    Test_Channel_Zero_One_And_Three_Map_To_Y_X_And_W();
    Test_Remote_Gear_Range_Is_Mapped_Only_Once();
    Test_Non_Manual_Disable_And_Offline_Clear_All_Chassis_Speeds();
    Test_NRF_Polling_And_One_Second_Timeout_Are_Preserved();
    return 0;
}
