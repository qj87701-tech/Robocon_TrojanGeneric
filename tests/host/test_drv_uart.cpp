#include <cassert>
#include <cstdint>

#include "1_Middleware/Driver/UART/drv_uart.h"

Class_Timestamp SYS_Timestamp;
bool init_finished = false;

int fake_sequence = 0;
int fake_timestamp_init_sequence = 0;
int fake_timestamp_overflow_count = 0;
TIM_HandleTypeDef *fake_timestamp_handler = nullptr;
std::uint64_t fake_timestamp_now = 0U;

namespace
{
UART_HandleTypeDef UART4_Handler{UART4};
UART_HandleTypeDef UART7_Handler{UART7};
UART_HandleTypeDef UART9_Handler{UART9};
UART_HandleTypeDef UART10_Handler{USART10};

UART_HandleTypeDef *Receive_UART = nullptr;
std::uint8_t *Receive_Buffer = nullptr;
std::uint16_t Receive_Length = 0U;
int Receive_Start_Count = 0;

UART_HandleTypeDef *Transmit_UART = nullptr;
std::uint8_t *Transmit_Buffer = nullptr;
std::uint16_t Transmit_Length = 0U;

std::uint8_t *Callback_Buffer = nullptr;
std::uint16_t Callback_Length = 0U;
int Callback_Count = 0;

void Reset_Fakes()
{
    init_finished = false;
    fake_timestamp_now = 0U;
    Receive_UART = nullptr;
    Receive_Buffer = nullptr;
    Receive_Length = 0U;
    Receive_Start_Count = 0;
    Transmit_UART = nullptr;
    Transmit_Buffer = nullptr;
    Transmit_Length = 0U;
    Callback_Buffer = nullptr;
    Callback_Length = 0U;
    Callback_Count = 0;
}

void Test_Callback(std::uint8_t *buffer, const std::uint16_t length)
{
    Callback_Buffer = buffer;
    Callback_Length = length;
    ++Callback_Count;
}

void Test_UART7_UART9_UART10_Init_Starts_Idle_DMA()
{
    Reset_Fakes();

    UART_Init(&UART7_Handler, Test_Callback);
    assert(UART7_Manage_Object.UART_Handler == &UART7_Handler);
    assert(UART7_Manage_Object.Callback_Function == Test_Callback);
    assert(Receive_UART == &UART7_Handler);
    assert(Receive_Buffer == UART7_Manage_Object.Rx_Buffer_0);
    assert(Receive_Length == UART_BUFFER_SIZE);

    UART_Init(&UART9_Handler, nullptr);
    assert(UART9_Manage_Object.UART_Handler == &UART9_Handler);
    assert(UART9_Manage_Object.Callback_Function == nullptr);
    assert(Receive_UART == &UART9_Handler);
    assert(Receive_Buffer == UART9_Manage_Object.Rx_Buffer_0);

    UART_Init(&UART10_Handler, Test_Callback);
    assert(UART10_Manage_Object.UART_Handler == &UART10_Handler);
    assert(UART10_Manage_Object.Callback_Function == Test_Callback);
    assert(Receive_UART == &UART10_Handler);
    assert(Receive_Buffer == UART10_Manage_Object.Rx_Buffer_0);
    assert(Receive_Start_Count == 3);
}

void Test_USART10_Rx_Event_Swaps_Buffer_And_Calls_Callback()
{
    Reset_Fakes();
    UART_Init(&UART10_Handler, Test_Callback);
    UART10_Manage_Object.Rx_Buffer_0[0] = 0x5AU;
    fake_timestamp_now = 123456U;
    init_finished = true;

    HAL_UARTEx_RxEventCallback(&UART10_Handler, 1U);

    assert(UART10_Manage_Object.Rx_Buffer_Ready == UART10_Manage_Object.Rx_Buffer_0);
    assert(UART10_Manage_Object.Rx_Buffer_Active == UART10_Manage_Object.Rx_Buffer_1);
    assert(UART10_Manage_Object.Rx_Time_Stamp == 123456U);
    assert(Receive_Buffer == UART10_Manage_Object.Rx_Buffer_1);
    assert(Callback_Count == 1);
    assert(Callback_Buffer == UART10_Manage_Object.Rx_Buffer_0);
    assert(Callback_Buffer[0] == 0x5AU);
    assert(Callback_Length == 1U);
}

void Test_UART4_Rx_Event_Swaps_Buffer_And_Calls_Callback()
{
    Reset_Fakes();
    UART_Init(&UART4_Handler, Test_Callback);
    assert(UART4_Manage_Object.UART_Handler == &UART4_Handler);
    assert(UART4_Manage_Object.Callback_Function == Test_Callback);
    assert(Receive_Buffer == UART4_Manage_Object.Rx_Buffer_0);
    UART4_Manage_Object.Rx_Buffer_0[0] = 0x5AU;
    UART4_Manage_Object.Rx_Buffer_0[1] = 0xA5U;
    fake_timestamp_now = 654321U;
    init_finished = true;

    HAL_UARTEx_RxEventCallback(&UART4_Handler, 2U);

    assert(UART4_Manage_Object.Rx_Buffer_Ready == UART4_Manage_Object.Rx_Buffer_0);
    assert(UART4_Manage_Object.Rx_Buffer_Active == UART4_Manage_Object.Rx_Buffer_1);
    assert(UART4_Manage_Object.Rx_Time_Stamp == 654321U);
    assert(Receive_Buffer == UART4_Manage_Object.Rx_Buffer_1);
    assert(Callback_Count == 1);
    assert(Callback_Buffer == UART4_Manage_Object.Rx_Buffer_0);
    assert(Callback_Length == 2U);
}

void Test_Rx_Event_Before_Init_Finished_Only_Restarts_DMA()
{
    Reset_Fakes();
    UART_Init(&UART7_Handler, Test_Callback);
    std::uint8_t *active_buffer = UART7_Manage_Object.Rx_Buffer_Active;

    HAL_UARTEx_RxEventCallback(&UART7_Handler, 8U);

    assert(UART7_Manage_Object.Rx_Buffer_Active == active_buffer);
    assert(Receive_Buffer == active_buffer);
    assert(Receive_Start_Count == 2);
    assert(Callback_Count == 0);
}

void Test_UART_Error_Reinitializes_First_Buffer()
{
    Reset_Fakes();
    UART_Init(&UART9_Handler, nullptr);
    UART9_Manage_Object.Rx_Buffer_Active = UART9_Manage_Object.Rx_Buffer_1;

    HAL_UART_ErrorCallback(&UART9_Handler);

    assert(UART9_Manage_Object.Rx_Buffer_Active == UART9_Manage_Object.Rx_Buffer_0);
    assert(Receive_UART == &UART9_Handler);
    assert(Receive_Buffer == UART9_Manage_Object.Rx_Buffer_0);
    assert(Receive_Start_Count == 2);
}

void Test_UART_Transmit_Data_Uses_DMA()
{
    Reset_Fakes();
    std::uint8_t data[3] = {1U, 2U, 3U};

    assert(UART_Transmit_Data(&UART10_Handler, data, sizeof(data)) == HAL_OK);
    assert(Transmit_UART == &UART10_Handler);
    assert(Transmit_Buffer == data);
    assert(Transmit_Length == sizeof(data));
}
}

extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                                           std::uint8_t *data,
                                                           const std::uint16_t size)
{
    Receive_UART = huart;
    Receive_Buffer = data;
    Receive_Length = size;
    ++Receive_Start_Count;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart,
                                                    std::uint8_t *data,
                                                    const std::uint16_t size)
{
    Transmit_UART = huart;
    Transmit_Buffer = data;
    Transmit_Length = size;
    return HAL_OK;
}

int main()
{
    Test_UART7_UART9_UART10_Init_Starts_Idle_DMA();
    Test_USART10_Rx_Event_Swaps_Buffer_And_Calls_Callback();
    Test_UART4_Rx_Event_Swaps_Buffer_And_Calls_Callback();
    Test_Rx_Event_Before_Init_Finished_Only_Restarts_DMA();
    Test_UART_Error_Reinitializes_First_Buffer();
    Test_UART_Transmit_Data_Uses_DMA();
    return 0;
}
