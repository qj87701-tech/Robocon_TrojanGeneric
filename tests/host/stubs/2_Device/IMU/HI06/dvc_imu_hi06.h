#ifndef TESTS_HOST_STUBS_DVC_IMU_HI06_H
#define TESTS_HOST_STUBS_DVC_IMU_HI06_H

#include "2_Device/IMU/HI06/HI06_Define.h"

struct Fake_HI06_State
{
    int Init_Count;
    int UART_Callback_Count;
    int TIM_10ms_Count;
    uint8_t *UART_Buffer;
    uint16_t UART_Length;
    Enum_HI06_Status Status;
    Struct_HI06_Data Data;
};

inline Fake_HI06_State Fake_HI06{};

class Class_HI06
{
public:
    void Init()
    {
        Fake_HI06.Init_Count++;
    }

    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
    {
        Fake_HI06.UART_Callback_Count++;
        Fake_HI06.UART_Buffer = Buffer;
        Fake_HI06.UART_Length = Length;
    }

    void TIM_10ms_PeriodElapsedCallback()
    {
        Fake_HI06.TIM_10ms_Count++;
    }

    Enum_HI06_Status Get_Status() const
    {
        return Fake_HI06.Status;
    }

    const Struct_HI06_Data &Get_Data() const
    {
        return Fake_HI06.Data;
    }
};

inline Class_HI06 HI06;

#endif
