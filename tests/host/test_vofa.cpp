#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "2_Device/Plotter/Vofa/dvc_vofa.h"

bool init_finished = false;

Struct_UART_Manage_Object UART1_Manage_Object{};
Struct_UART_Manage_Object UART2_Manage_Object{};
Struct_UART_Manage_Object UART3_Manage_Object{};
Struct_UART_Manage_Object UART4_Manage_Object{};
Struct_UART_Manage_Object UART5_Manage_Object{};
Struct_UART_Manage_Object UART6_Manage_Object{};
Struct_UART_Manage_Object UART7_Manage_Object{};
Struct_UART_Manage_Object UART8_Manage_Object{};
Struct_UART_Manage_Object UART9_Manage_Object{};
Struct_UART_Manage_Object UART10_Manage_Object{};

namespace
{
UART_HandleTypeDef UART9_Handler{UART9};
UART_HandleTypeDef *Transmit_UART = nullptr;
std::uint8_t Transmit_Data[64]{};
std::uint16_t Transmit_Length = 0U;

void Reset_Fakes()
{
    UART9_Manage_Object = {};
    Transmit_UART = nullptr;
    std::memset(Transmit_Data, 0, sizeof(Transmit_Data));
    Transmit_Length = 0U;
}

void Test_Vofa_Uses_UART9_And_Sends_JustFloat_Frame()
{
    Reset_Fakes();
    Class_Vofa_UART vofa;
    float channel_0 = 1.25f;
    float channel_1 = -2.50f;

    UART9_Manage_Object.UART_Handler = &UART9_Handler;
    vofa.Init(&UART9_Handler);
    vofa.Set_Data(2, static_cast<const void *>(&channel_0), static_cast<const void *>(&channel_1));
    vofa.TIM_1ms_Write_PeriodElapsedCallback();

    assert(Transmit_UART == &UART9_Handler);
    assert(Transmit_Length == 2U * sizeof(float) + sizeof(std::uint32_t));

    float output_0 = 0.0f;
    float output_1 = 0.0f;
    std::uint32_t frame_tail = 0U;
    std::memcpy(&output_0, Transmit_Data, sizeof(float));
    std::memcpy(&output_1, Transmit_Data + sizeof(float), sizeof(float));
    std::memcpy(&frame_tail, Transmit_Data + 2U * sizeof(float), sizeof(frame_tail));
    assert(output_0 == 1.25f);
    assert(output_1 == -2.50f);
    assert(frame_tail == 0x7F800000U);
}

void Test_Vofa_Receives_Variable_Assignment()
{
    Reset_Fakes();
    Class_Vofa_UART vofa;
    const char *variable_list[] = {"speed_kp", "speed_ki"};
    std::uint8_t command[] = "speed_ki=-1.25#";

    vofa.Init(&UART9_Handler, 2U, variable_list);
    vofa.UART_RxCpltCallback(command, sizeof(command) - 1U);

    assert(vofa.Get_Variable_Index() == 1);
    assert(std::fabs(vofa.Get_Variable_Value() + 1.25f) < 0.0001f);
}
}

std::uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, std::uint8_t *data, const std::uint16_t length)
{
    assert(length <= sizeof(Transmit_Data));
    Transmit_UART = huart;
    std::memcpy(Transmit_Data, data, length);
    Transmit_Length = length;
    return HAL_OK;
}

int main()
{
    Test_Vofa_Uses_UART9_And_Sends_JustFloat_Frame();
    Test_Vofa_Receives_Variable_Assignment();
    return 0;
}
