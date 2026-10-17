#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "test_assert.h"
#include "1_Middleware/Protocol/pro_upper/pro_upper.h"
#include "stm32h7xx_hal.h"

namespace
{
std::uint32_t Test_Tick = 0U;
Class_Upper *Interrupt_Upper = nullptr;

std::uint16_t Reference_CRC(const std::uint8_t *data, const std::uint16_t length)
{
    std::uint16_t crc = 0xffffU;
    for (std::uint16_t i = 0U; i < length; ++i)
    {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
        for (unsigned int bit = 0U; bit < 8U; ++bit)
        {
            crc = static_cast<std::uint16_t>((crc << 1U) ^ ((crc & 0x8000U) != 0U ? 0x1021U : 0U));
        }
    }
    return crc;
}

std::array<std::uint8_t, 32> Make_Frame(const std::int16_t x = -32768, const std::int16_t y = 32767, const std::int16_t w = -1000)
{
    std::array<std::uint8_t, 32> frame{};
    frame[0] = 0x55U;
    frame[1] = 0xaaU;
    const std::int16_t speed[3] = {y, x, w};
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        const auto value = static_cast<std::uint16_t>(speed[i]);
        frame[2U + 2U * i] = static_cast<std::uint8_t>(value);
        frame[3U + 2U * i] = static_cast<std::uint8_t>(value >> 8U);
    }
    for (unsigned int i = 8U; i <= 28U; ++i) { frame[i] = static_cast<std::uint8_t>(i); }
    frame[8] = 0x55U;
    frame[9] = 0xaaU;
    frame[28] = 0x77U;
    const auto crc = Reference_CRC(frame.data(), 29U);
    frame[29] = static_cast<std::uint8_t>(crc);
    frame[30] = static_cast<std::uint8_t>(crc >> 8U);
    frame[31] = 0x77U;
    return frame;
}

void Feed_And_Process(Class_Upper &upper, std::uint8_t *data, const std::uint16_t length)
{
    upper.UART_RxCpltCallback(data, length);
    upper.TIM_10ms_PeriodElapsedCallback();
}

void Test_Init_And_Empty_Input()
{
    Class_Upper upper;
    upper.Init();
    assert(upper.Get_Status() == Upper_Status_OFFLINE);
    assert(!upper.Get_Data().Valid && !upper.Get_CRC_Check_Enable());
    auto frame = Make_Frame();
    upper.UART_RxCpltCallback(nullptr, 32U);
    upper.UART_RxCpltCallback(frame.data(), 0U);
    upper.TIM_10ms_PeriodElapsedCallback();
    assert(upper.Get_Data().Frame_Count == 0U && upper.Get_Data().Rx_Byte_Count == 0U);
    Feed_And_Process(upper, frame.data(), 32U);
    upper.Set_CRC_Check_Enable(true);
    upper.Init();
    assert(!upper.Get_Data().Valid && upper.Get_Data().Frame_Count == 0U);
    assert(!upper.Get_CRC_Check_Enable());
}

void Test_Raw_Signed_Speed_And_Fields()
{
    Class_Upper upper;
    upper.Init();
    auto frame = Make_Frame();
    Test_Tick = 20U;
    upper.UART_RxCpltCallback(frame.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 0U);
    upper.TIM_10ms_PeriodElapsedCallback();
    const auto &data = upper.Get_Data();
    assert(data.Speed_X == -32768 && data.Speed_Y == 32767 && data.Speed_W == -1000);
    assert(data.Data_Field[0] == 0x55U && data.Data_Field[1] == 0xaaU);
    assert(data.Extend_Data[8] == 0x77U);
    assert(std::memcmp(data.Raw_Data, frame.data(), 32U) == 0);
    assert(data.Valid && data.Frame_Count == 1U && data.Last_Rx_Tick == 20U);
    assert(upper.Get_Status() == Upper_Status_ONLINE);
    frame = Make_Frame(0, -1, 1234);
    Feed_And_Process(upper, frame.data(), 32U);
    assert(data.Speed_X == 0 && data.Speed_Y == -1 && data.Speed_W == 1234);
}

void Test_All_Split_Positions()
{
    auto frame = Make_Frame();
    for (std::uint16_t split = 1U; split < 32U; ++split)
    {
        Class_Upper upper;
        upper.Init();
        Feed_And_Process(upper, frame.data(), split);
        assert(!upper.Get_Data().Valid && upper.Get_Data().Frame_Count == 0U);
        Feed_And_Process(upper, frame.data() + split, static_cast<std::uint16_t>(32U - split));
        assert(upper.Get_Data().Frame_Count == 1U && upper.Get_Data().Speed_W == -1000);
    }
}

void Test_Multiple_Frames_And_Noise()
{
    Class_Upper upper;
    upper.Init();
    const auto first = Make_Frame();
    const auto second = Make_Frame(12, -34, 56);
    std::vector<std::uint8_t> stream{0U, 0x55U, 0x01U, 0x77U};
    stream.insert(stream.end(), first.begin(), first.end());
    stream.insert(stream.end(), second.begin(), second.end());
    Feed_And_Process(upper, stream.data(), static_cast<std::uint16_t>(stream.size()));
    assert(upper.Get_Data().Frame_Count == 2U && upper.Get_Data().Speed_X == 12);
    assert(upper.Get_Data().Speed_Y == -34 && upper.Get_Data().Speed_W == 56);
    assert(upper.Get_Data().Rx_Byte_Count == stream.size());
}

void Test_Embedded_Markers_And_Overlapping_Header()
{
    Class_Upper upper;
    upper.Init();
    auto frame = Make_Frame();
    std::vector<std::uint8_t> stream{0x55U};
    stream.insert(stream.end(), frame.begin(), frame.end());
    Feed_And_Process(upper, stream.data(), static_cast<std::uint16_t>(stream.size()));
    assert(upper.Get_Data().Frame_Count == 1U);
    assert(std::memcmp(upper.Get_Data().Raw_Data, frame.data(), 32U) == 0);
}

void Test_Bad_Tail_Then_Good_Frame()
{
    Class_Upper upper;
    upper.Init();
    auto bad = Make_Frame();
    auto good = Make_Frame(1, 2, 3);
    bad[31] = 0U;
    Feed_And_Process(upper, bad.data(), 32U);
    assert(!upper.Get_Data().Valid && upper.Get_Data().Invalid_Frame_Count > 0U);
    Feed_And_Process(upper, good.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 1U && upper.Get_Data().Speed_W == 3);
}

void Test_CRC_Known_Vector()
{
    const auto *vector = reinterpret_cast<const std::uint8_t *>("123456789");
    assert(Upper_CRC16_Calculate(vector, 9U) == 0x29b1U);
    assert(Upper_CRC16_Calculate(nullptr, 0U) == 0xffffU);
    assert(Upper_CRC16_Calculate(nullptr, 9U) == 0xffffU);
    assert(!Upper_CRC16_Check(nullptr, 32U));
    auto frame = Make_Frame();
    assert(Upper_CRC16_Check(frame.data(), 32U));
    assert(!Upper_CRC16_Check(frame.data(), 31U));
    assert(!Upper_CRC16_Check(frame.data(), 33U));
    frame[0] ^= 1U;
    assert(!Upper_CRC16_Check(frame.data(), 32U));
}

void Test_CRC_Disabled_And_Enabled()
{
    Class_Upper upper;
    upper.Init();
    auto good = Make_Frame();
    auto bad = good;
    bad[29] ^= 1U;
    Test_Tick = 0U;
    Feed_And_Process(upper, bad.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 1U && upper.Get_Data().CRC_Error_Count == 0U);
    upper.Set_CRC_Check_Enable(true);
    Test_Tick = 100U;
    Feed_And_Process(upper, good.data(), 32U);
    Test_Tick = 200U;
    Feed_And_Process(upper, bad.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 2U && upper.Get_Data().CRC_Error_Count > 0U);
    assert(upper.Get_Data().Last_Rx_Tick == 100U);
    Test_Tick = 600U;
    upper.TIM_10ms_PeriodElapsedCallback();
    assert(!upper.Get_Data().Valid && upper.Get_Status() == Upper_Status_OFFLINE);
    upper.Set_CRC_Check_Enable(false);
    Feed_And_Process(upper, bad.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 3U && upper.Get_Data().Valid);
}

void Test_CRC_Change_Drops_Pending_Data()
{
    Class_Upper upper;
    upper.Init();
    auto frame = Make_Frame();
    Feed_And_Process(upper, frame.data(), 12U);
    upper.UART_RxCpltCallback(frame.data() + 12U, 20U);
    upper.Set_CRC_Check_Enable(true);
    upper.TIM_10ms_PeriodElapsedCallback();
    assert(upper.Get_Data().Frame_Count == 0U);
    Feed_And_Process(upper, frame.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 1U);
    upper.UART_RxCpltCallback(frame.data(), 32U);
    upper.Set_CRC_Check_Enable(true); // 相同策略不能丢弃待处理帧
    upper.TIM_10ms_PeriodElapsedCallback();
    assert(upper.Get_Data().Frame_Count == 2U);
}

void Test_Timeout_Recovery_And_Tick_Wrap()
{
    Class_Upper upper;
    upper.Init();
    auto frame = Make_Frame();
    Test_Tick = std::numeric_limits<std::uint32_t>::max() - 100U;
    Feed_And_Process(upper, frame.data(), 32U);
    const auto received = Test_Tick;
    Test_Tick = received + 499U;
    Feed_And_Process(upper, frame.data(), 5U);
    assert(upper.Get_Data().Valid);
    Test_Tick = received + 500U;
    upper.TIM_10ms_PeriodElapsedCallback();
    assert(!upper.Get_Data().Valid && upper.Get_Status() == Upper_Status_OFFLINE);
    assert(upper.Get_Data().Last_Rx_Tick == received && upper.Get_Data().Speed_W == -1000);
    Feed_And_Process(upper, frame.data() + 5U, 27U);
    assert(upper.Get_Data().Valid && upper.Get_Data().Frame_Count == 2U);
}

void Overflow_During_Foreground()
{
    std::array<std::uint8_t, 1200> burst{};
    Interrupt_Upper->UART_RxCpltCallback(burst.data(), static_cast<std::uint16_t>(burst.size()));
}

void Test_Overflow_And_Interrupt_Recovery()
{
    Class_Upper upper;
    upper.Init();
    auto frame = Make_Frame();
    Feed_And_Process(upper, frame.data(), 12U);
    std::array<std::uint8_t, 1200> burst{};
    Feed_And_Process(upper, burst.data(), static_cast<std::uint16_t>(burst.size()));
    assert(upper.Get_Data().Buffer_Overflow_Count == 1U && upper.Get_Data().Frame_Count == 0U);
    Feed_And_Process(upper, frame.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 1U);
    upper.Init();
    upper.UART_RxCpltCallback(frame.data(), 32U);
    Interrupt_Upper = &upper;
    Test_DMB_Callback = Overflow_During_Foreground;
    upper.TIM_10ms_PeriodElapsedCallback();
    assert(upper.Get_Data().Frame_Count == 0U && upper.Get_Data().Buffer_Overflow_Count == 1U);
    Feed_And_Process(upper, frame.data(), 32U);
    assert(upper.Get_Data().Frame_Count == 1U && upper.Get_Data().Valid);
    Interrupt_Upper = nullptr;
}
}

extern "C" std::uint32_t HAL_GetTick() { return Test_Tick; }

int main()
{
    Test_Init_And_Empty_Input();
    Test_Raw_Signed_Speed_And_Fields();
    Test_All_Split_Positions();
    Test_Multiple_Frames_And_Noise();
    Test_Embedded_Markers_And_Overlapping_Header();
    Test_Bad_Tail_Then_Good_Frame();
    Test_CRC_Known_Vector();
    Test_CRC_Disabled_And_Enabled();
    Test_CRC_Change_Drops_Pending_Data();
    Test_Timeout_Recovery_And_Tick_Wrap();
    Test_Overflow_And_Interrupt_Recovery();
    std::puts("Upper protocol: 11 tests passed");
}
