#include <cassert>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "1_Middleware/Hipnuc/hipnuc_dec.h"
#include "2_Device/IMU/HI06/dvc_imu_hi06.h"
#include "stm32h7xx_hal.h"

namespace
{
constexpr float FLOAT_TOLERANCE = 0.0001f;
constexpr float PI = 3.14159265358979323846f;

std::uint32_t Test_Tick = 0U;
Class_HI06 *Overflow_Test_IMU = nullptr;
std::vector<std::uint8_t> Overflow_Test_Data;

void Overflow_During_Foreground_Drain()
{
    Overflow_Test_IMU->UART_RxCpltCallback(Overflow_Test_Data.data(), static_cast<std::uint16_t>(Overflow_Test_Data.size()));
}

template <typename PacketType>
std::vector<std::uint8_t> Build_Frame(const PacketType &packet)
{
    std::vector<std::uint8_t> frame(HIPNUC_HEADER_SIZE + sizeof(packet));
    frame[0] = HIPNUC_SYNC1;
    frame[1] = HIPNUC_SYNC2;
    frame[2] = static_cast<std::uint8_t>(sizeof(packet) & 0xffU);
    frame[3] = static_cast<std::uint8_t>((sizeof(packet) >> 8U) & 0xffU);
    std::memcpy(frame.data() + HIPNUC_HEADER_SIZE, &packet, sizeof(packet));

    std::uint16_t crc = hipnuc_crc16(0U, frame.data(), 4U);
    crc = hipnuc_crc16(crc, frame.data() + HIPNUC_HEADER_SIZE, sizeof(packet));
    frame[4] = static_cast<std::uint8_t>(crc & 0xffU);
    frame[5] = static_cast<std::uint8_t>((crc >> 8U) & 0xffU);
    return frame;
}

std::vector<std::uint8_t> Build_HI91_Frame(const hi91_t &packet)
{
    return Build_Frame(packet);
}

hi91_t Make_HI91_Packet(float yaw, std::int8_t temperature)
{
    hi91_t packet{};
    packet.tag = HIPNUC_ID_HI91;
    packet.main_status = HIPNUC_STATUS_STATIC;
    packet.temp = temperature;
    packet.acc[0] = 1.0f;
    packet.acc[1] = -0.5f;
    packet.acc[2] = 0.25f;
    packet.gyr[0] = 180.0f;
    packet.gyr[1] = -90.0f;
    packet.gyr[2] = 45.0f;
    packet.roll = 10.0f;
    packet.pitch = -20.0f;
    packet.yaw = yaw;
    return packet;
}

void Assert_Float_Near(float actual, float expected)
{
    assert(std::fabs(actual - expected) < FLOAT_TOLERANCE);
}

void Test_HI91_Split_Across_Callbacks()
{
    Class_HI06 imu;
    imu.Init();
    const std::vector<std::uint8_t> frame = Build_HI91_Frame(Make_HI91_Packet(30.0f, 27));

    imu.UART_RxCpltCallback(const_cast<std::uint8_t *>(frame.data()), 11U);
    imu.TIM_10ms_PeriodElapsedCallback();
    assert(imu.Get_Status() == HI06_Status_OFFLINE);
    assert(imu.Get_Data().Frame_Count == 0U);

    Test_Tick = 20U;
    imu.UART_RxCpltCallback(const_cast<std::uint8_t *>(frame.data() + 11U), static_cast<std::uint16_t>(frame.size() - 11U));
    imu.TIM_10ms_PeriodElapsedCallback();

    const Struct_HI06_Data &data = imu.Get_Data();
    assert(imu.Get_Status() == HI06_Status_ONLINE);
    assert(data.Frame_Count == 1U);
    assert(data.Main_Status == HIPNUC_STATUS_STATIC);
    assert(data.Temperature == 27.0f);
    Assert_Float_Near(data.Accel_X, 9.8f);
    Assert_Float_Near(data.Accel_Y, -4.9f);
    Assert_Float_Near(data.Accel_Z, 2.45f);
    Assert_Float_Near(data.Gyro_X, PI);
    Assert_Float_Near(data.Gyro_Y, -PI / 2.0f);
    Assert_Float_Near(data.Gyro_Z, PI / 4.0f);
    Assert_Float_Near(data.Roll, 10.0f * HIPNUC_DEG2RAD);
    Assert_Float_Near(data.Pitch, -20.0f * HIPNUC_DEG2RAD);
    Assert_Float_Near(data.Yaw, 30.0f * HIPNUC_DEG2RAD);
}

void Test_Multiple_HI91_Frames_Keep_The_Latest()
{
    Class_HI06 imu;
    imu.Init();
    const std::vector<std::uint8_t> first = Build_HI91_Frame(Make_HI91_Packet(30.0f, 27));
    const std::vector<std::uint8_t> second = Build_HI91_Frame(Make_HI91_Packet(-45.0f, 31));
    std::vector<std::uint8_t> frames = first;
    frames.insert(frames.end(), second.begin(), second.end());

    imu.UART_RxCpltCallback(frames.data(), static_cast<std::uint16_t>(frames.size()));
    imu.TIM_10ms_PeriodElapsedCallback();

    assert(imu.Get_Data().Frame_Count == 2U);
    assert(imu.Get_Data().Temperature == 31.0f);
    Assert_Float_Near(imu.Get_Data().Yaw, -45.0f * HIPNUC_DEG2RAD);
}

void Test_Fields_Follow_Official_Valid_Mask()
{
    Class_HI06 imu;
    imu.Init();
    std::vector<std::uint8_t> frame = Build_HI91_Frame(Make_HI91_Packet(30.0f, 27));

    imu.UART_RxCpltCallback(frame.data(), static_cast<std::uint16_t>(frame.size()));
    imu.TIM_10ms_PeriodElapsedCallback();

    const std::uint64_t valid = imu.Get_Data().Valid;
    assert((valid & HIPNUC_VALID_STATUS) != 0U);
    assert((valid & HIPNUC_VALID_ACC) != 0U);
    assert((valid & HIPNUC_VALID_GYR) != 0U);
    assert((valid & HIPNUC_VALID_ROLL_PITCH) != 0U);
    assert((valid & HIPNUC_VALID_YAW) != 0U);
    assert((valid & HIPNUC_VALID_TEMPERATURE) != 0U);
}

void Test_Bad_CRC_Does_Not_Update_Data()
{
    Class_HI06 imu;
    imu.Init();
    std::vector<std::uint8_t> frame = Build_HI91_Frame(Make_HI91_Packet(30.0f, 27));
    frame[HIPNUC_HEADER_SIZE + 10U] ^= 0x01U;

    imu.UART_RxCpltCallback(frame.data(), static_cast<std::uint16_t>(frame.size()));
    imu.TIM_10ms_PeriodElapsedCallback();

    assert(imu.Get_Status() == HI06_Status_OFFLINE);
    assert(imu.Get_Data().Frame_Count == 0U);
    assert(imu.Get_Data().CRC_Error_Count == 1U);
    assert(imu.Get_Data().Roll == 0.0f);
    assert(imu.Get_Data().Accel_X == 0.0f);
}

void Test_HI81_Is_Ignored_By_HI06()
{
    Class_HI06 imu;
    imu.Init();
    hi81_t packet{};
    packet.tag = HIPNUC_ID_HI81;
    packet.main_status = HIPNUC_STATUS_STATIC;
    std::vector<std::uint8_t> frame = Build_Frame(packet);

    Test_Tick = 100U;
    imu.UART_RxCpltCallback(frame.data(), static_cast<std::uint16_t>(frame.size()));
    imu.TIM_10ms_PeriodElapsedCallback();

    assert(imu.Get_Status() == HI06_Status_OFFLINE);
    assert(imu.Get_Data().Frame_Count == 0U);
    assert(imu.Get_Data().Last_Rx_Tick == 0U);
}

void Test_Ring_Overflow_Drops_Affected_Span_And_Recovers()
{
    Class_HI06 imu;
    imu.Init();
    std::vector<std::uint8_t> overflow_data(HI06_RX_BUFFER_SIZE + 100U, 0x55U);

    imu.UART_RxCpltCallback(overflow_data.data(), static_cast<std::uint16_t>(overflow_data.size()));
    assert(imu.Get_Data().Buffer_Overflow_Count == 1U);
    imu.TIM_10ms_PeriodElapsedCallback();

    std::vector<std::uint8_t> frame = Build_HI91_Frame(Make_HI91_Packet(60.0f, 29));
    Test_Tick = 200U;
    imu.UART_RxCpltCallback(frame.data(), static_cast<std::uint16_t>(frame.size()));
    imu.TIM_10ms_PeriodElapsedCallback();

    assert(imu.Get_Data().Buffer_Overflow_Count == 1U);
    assert(imu.Get_Status() == HI06_Status_ONLINE);
    assert(imu.Get_Data().Frame_Count == 1U);
    Assert_Float_Near(imu.Get_Data().Yaw, 60.0f * HIPNUC_DEG2RAD);
}

void Test_Exactly_One_Ring_Boundary_Does_Not_Write_Past_Storage()
{
    struct Struct_Guarded_HI06
    {
        std::uint32_t Before;
        Class_HI06 IMU;
        std::uint32_t After;
    } guarded{0x12345678U, {}, 0x87654321U};
    guarded.IMU.Init();
    std::array<std::uint8_t, HI06_RX_BUFFER_SIZE - 1U> boundary_data{};
    boundary_data.fill(0x33U);

    guarded.IMU.UART_RxCpltCallback(boundary_data.data(), static_cast<std::uint16_t>(boundary_data.size()));
    guarded.IMU.TIM_10ms_PeriodElapsedCallback();

    assert(guarded.Before == 0x12345678U);
    assert(guarded.After == 0x87654321U);
    assert(guarded.IMU.Get_Data().Buffer_Overflow_Count == 0U);
}

void Test_Offline_At_500ms_And_Tick_Wrap()
{
    Class_HI06 imu;
    imu.Init();
    std::vector<std::uint8_t> frame = Build_HI91_Frame(Make_HI91_Packet(30.0f, 27));
    Test_Tick = std::numeric_limits<std::uint32_t>::max() - 100U;
    imu.UART_RxCpltCallback(frame.data(), static_cast<std::uint16_t>(frame.size()));
    imu.TIM_10ms_PeriodElapsedCallback();
    assert(imu.Get_Status() == HI06_Status_ONLINE);

    Test_Tick = 398U;
    imu.TIM_10ms_PeriodElapsedCallback();
    assert(imu.Get_Status() == HI06_Status_ONLINE);

    Test_Tick = 399U;
    imu.TIM_10ms_PeriodElapsedCallback();
    assert(imu.Get_Status() == HI06_Status_OFFLINE);
    Assert_Float_Near(imu.Get_Data().Yaw, 30.0f * HIPNUC_DEG2RAD);
}

void Test_Overflow_During_Foreground_Drain_Discards_Affected_Span()
{
    Class_HI06 imu;
    imu.Init();
    std::vector<std::uint8_t> affected_frame = Build_HI91_Frame(Make_HI91_Packet(75.0f, 30));
    imu.UART_RxCpltCallback(affected_frame.data(), static_cast<std::uint16_t>(affected_frame.size()));

    Overflow_Test_IMU = &imu;
    Overflow_Test_Data.assign(HI06_RX_BUFFER_SIZE, 0x55U);
    Test_DMB_Callback = Overflow_During_Foreground_Drain;
    imu.TIM_10ms_PeriodElapsedCallback();

    assert(imu.Get_Data().Buffer_Overflow_Count == 1U);
    assert(imu.Get_Data().Frame_Count == 0U);
    assert(imu.Get_Status() == HI06_Status_OFFLINE);

    std::vector<std::uint8_t> recovery_frame = Build_HI91_Frame(Make_HI91_Packet(-60.0f, 28));
    imu.UART_RxCpltCallback(recovery_frame.data(), static_cast<std::uint16_t>(recovery_frame.size()));
    imu.TIM_10ms_PeriodElapsedCallback();
    assert(imu.Get_Data().Frame_Count == 1U);
    assert(imu.Get_Status() == HI06_Status_ONLINE);
    Assert_Float_Near(imu.Get_Data().Yaw, -60.0f * HIPNUC_DEG2RAD);
}
}

extern "C" std::uint32_t HAL_GetTick()
{
    return Test_Tick;
}

int main()
{
    Test_HI91_Split_Across_Callbacks();
    Test_Multiple_HI91_Frames_Keep_The_Latest();
    Test_Fields_Follow_Official_Valid_Mask();
    Test_Bad_CRC_Does_Not_Update_Data();
    Test_HI81_Is_Ignored_By_HI06();
    Test_Ring_Overflow_Drops_Affected_Span_And_Recovers();
    Test_Exactly_One_Ring_Boundary_Does_Not_Write_Past_Storage();
    Test_Offline_At_500ms_And_Tick_Wrap();
    Test_Overflow_During_Foreground_Drain_Discards_Affected_Span();
    return 0;
}
