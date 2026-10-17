#include <cassert>

#include "test_assert.h"
#include "2_Device/BSP/Key/bsp_key.h"

namespace
{
GPIO_TypeDef Fake_GPIO_Port{0x1234U};
GPIO_PinState Fake_Pin_State = GPIO_PIN_SET;

void Set_Pin(const GPIO_PinState state)
{
    Fake_Pin_State = state;
}

void Test_Debounced_Press_Release_And_Hold()
{
    Set_Pin(GPIO_PIN_SET);
    Class_Key key;
    key.Init(&Fake_GPIO_Port, GPIO_PIN_2, GPIO_PIN_SET);
    assert(key.Get_Key_Status() == BSP_Key_Status_FREE);

    Set_Pin(GPIO_PIN_RESET);
    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_FREE);
    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_TRIG_FREE_PRESSED);
    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_PRESSED);
    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_PRESSED);

    Set_Pin(GPIO_PIN_SET);
    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_PRESSED);
    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_TRIG_PRESSED_FREE);
    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_FREE);
}

void Test_Bounce_Does_Not_Trigger()
{
    Set_Pin(GPIO_PIN_SET);
    Class_Key key;
    key.Init(&Fake_GPIO_Port, GPIO_PIN_2, GPIO_PIN_SET);

    for (int i = 0; i < 6; ++i)
    {
        Set_Pin((i % 2 == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        key.TIM_10ms_PeriodElapsedCallback();
        assert(key.Get_Key_Status() == BSP_Key_Status_FREE);
    }
}

void Test_Pressed_During_Init_Does_Not_Trigger()
{
    Set_Pin(GPIO_PIN_RESET);
    Class_Key key;
    key.Init(&Fake_GPIO_Port, GPIO_PIN_4, GPIO_PIN_SET);
    assert(key.Get_Key_Status() == BSP_Key_Status_PRESSED);

    key.TIM_10ms_PeriodElapsedCallback();
    assert(key.Get_Key_Status() == BSP_Key_Status_PRESSED);
}
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio_port, const std::uint16_t gpio_pin)
{
    assert(gpio_port == &Fake_GPIO_Port);
    assert((gpio_pin == GPIO_PIN_2) || (gpio_pin == GPIO_PIN_4));
    return Fake_Pin_State;
}

int main()
{
    Test_Debounced_Press_Release_And_Hold();
    Test_Bounce_Does_Not_Trigger();
    Test_Pressed_During_Init_Does_Not_Trigger();
    return 0;
}
