/**
 * @file bsp_key.h
 * @brief 板载按键
 * @version 0.1
 * @date 2026-09-30 0.1 新建文档
 */

#ifndef BSP_KEY_H
#define BSP_KEY_H

/* Includes ------------------------------------------------------------------*/

#include "gpio.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 按键状态
 */
enum Enum_BSP_Key_Status
{
    BSP_Key_Status_FREE = 0,
    BSP_Key_Status_TRIG_FREE_PRESSED,
    BSP_Key_Status_TRIG_PRESSED_FREE,
    BSP_Key_Status_PRESSED,
};

/**
 * @brief 板载按键
 */
class Class_Key
{
public:
    void Init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState key_free_state = GPIO_PIN_SET);

    inline Enum_BSP_Key_Status Get_Key_Status() const;

    void TIM_10ms_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 按键所在GPIO端口
    GPIO_TypeDef *GPIO_Port;
    // 按键所在GPIO引脚
    uint16_t GPIO_Pin;
    // 按键未按下时GPIO状态
    GPIO_PinState Key_Free_State;

    // 内部变量

    // 已经确认稳定的GPIO状态
    GPIO_PinState Stable_GPIO_State;
    // 等待确认的GPIO状态
    GPIO_PinState Candidate_GPIO_State;
    // 等待确认状态的连续计数
    uint8_t Candidate_Count = 0;

    // 读变量

    // 按键状态
    Enum_BSP_Key_Status Key_Status = BSP_Key_Status_FREE;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations -------------------------------------------*/

/**
 * @brief 获取按键状态
 *
 * @return Enum_BSP_Key_Status 按键状态
 */
inline Enum_BSP_Key_Status Class_Key::Get_Key_Status() const
{
    return (Key_Status);
}

#endif

