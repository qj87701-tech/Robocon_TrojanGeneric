#include <cassert>
#include <cstdint>

#include "test_assert.h"
#include "1_Middleware/Protocol/pro_sbus/pro_sbus.h"

namespace
{
constexpr std::uint16_t Expected_Channels[16] = {
    200U, 1000U, 1800U, 0U,
    2047U, 341U, 682U, 1365U,
    511U, 512U, 1023U, 1024U,
    1536U, 777U, 1234U, 42U};

constexpr std::uint8_t Valid_Packet[31] = {
    0x0F, 0xC8, 0x40, 0x1F, 0xC2, 0x01, 0xF0, 0xFF,
    0xAA, 0xA8, 0xAA, 0xAA, 0xFF, 0x01, 0xD0, 0xFF,
    0x00, 0x08, 0xE0, 0x84, 0x49, 0x53, 0x05, 0xFF,
    0xAF, 0x07, 0x81, 0x81, 0x88, 0x5A, 0xA5};

void Assert_Safe_Data(const Struct_SBUS_Data &data)
{
    for (std::uint8_t i = 0; i < 16U; i++)
    {
        assert(data.Channel[i] == 1000U);
        assert(data.Function[i] == 0U);
    }

    assert(data.Mode == 0U);
    assert(data.Motor_Enable == 0U);
    assert(data.Online == 0U);
}

void Test_Init_Loads_Safe_Data()
{
    Class_SBUS_Receive receiver;
    receiver.Init();

    Assert_Safe_Data(receiver.SBUS_Data);
}

void Test_Valid_Packet_Decodes_All_Data()
{
    Class_SBUS_Receive receiver;
    receiver.Init();

    assert(receiver.Data_Process(Valid_Packet) == 1U);

    for (std::uint8_t i = 0; i < 16U; i++)
    {
        assert(receiver.SBUS_Data.Channel[i] == Expected_Channels[i]);
    }

    assert(receiver.SBUS_Data.Mode == 3U);
    assert(receiver.SBUS_Data.Motor_Enable == 1U);
    assert(receiver.SBUS_Data.Function[0] == 1U);
    assert(receiver.SBUS_Data.Function[7] == 1U);
    assert(receiver.SBUS_Data.Function[8] == 1U);
    assert(receiver.SBUS_Data.Function[15] == 1U);
    assert(receiver.SBUS_Data.Function[1] == 0U);
    assert(receiver.SBUS_Data.Function[14] == 0U);
    assert(receiver.SBUS_Data.Online == 1U);
}

void Test_Invalid_Markers_Do_Not_Update_Or_Refresh()
{
    constexpr std::uint8_t Marker_Index[4] = {0U, 23U, 24U, 28U};

    for (std::uint8_t marker = 0; marker < 4U; marker++)
    {
        Class_SBUS_Receive receiver;
        receiver.Init();
        assert(receiver.Data_Process(Valid_Packet) == 1U);

        for (std::uint8_t i = 0; i < 5U; i++)
        {
            receiver.TIM_10ms_PeriodElapsedCallback();
        }

        std::uint8_t invalid_packet[31];
        for (std::uint8_t i = 0; i < 31U; i++)
        {
            invalid_packet[i] = Valid_Packet[i];
        }
        invalid_packet[Marker_Index[marker]] ^= 0x01U;

        assert(receiver.Data_Process(invalid_packet) == 0U);
        assert(receiver.SBUS_Data.Channel[0] == Expected_Channels[0]);
        assert(receiver.SBUS_Data.Motor_Enable == 1U);

        for (std::uint8_t i = 0; i < 94U; i++)
        {
            receiver.TIM_10ms_PeriodElapsedCallback();
        }
        assert(receiver.SBUS_Data.Online == 1U);

        receiver.TIM_10ms_PeriodElapsedCallback();
        Assert_Safe_Data(receiver.SBUS_Data);
    }
}

void Test_Timeout_Resets_Exactly_At_1s()
{
    Class_SBUS_Receive receiver;
    receiver.Init();
    assert(receiver.Data_Process(Valid_Packet) == 1U);

    for (std::uint8_t i = 0; i < 99U; i++)
    {
        receiver.TIM_10ms_PeriodElapsedCallback();
    }
    assert(receiver.SBUS_Data.Online == 1U);
    assert(receiver.SBUS_Data.Channel[15] == Expected_Channels[15]);

    receiver.TIM_10ms_PeriodElapsedCallback();
    Assert_Safe_Data(receiver.SBUS_Data);
}
}

int main()
{
    Test_Init_Loads_Safe_Data();
    Test_Valid_Packet_Decodes_All_Data();
    Test_Invalid_Markers_Do_Not_Update_Or_Refresh();
    Test_Timeout_Resets_Exactly_At_1s();
    return 0;
}
