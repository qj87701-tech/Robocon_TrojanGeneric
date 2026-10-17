/**
 * @file bsp_key.cpp
 * @brief 板载按键
 * @version 0.1
 * @date 2026-09-30 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_key.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化按键
 *
 * @param gpio_port 按键所在GPIO端口
 * @param gpio_pin 按键所在GPIO引脚
 * @param key_free_state 按键未按下时GPIO状态
 */
void Class_Key::Init(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, GPIO_PinState key_free_state)
{
    GPIO_Port = gpio_port;
    GPIO_Pin = gpio_pin;
    Key_Free_State = key_free_state;

    Stable_GPIO_State = HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin);
    Candidate_GPIO_State = Stable_GPIO_State;
    Candidate_Count = 0;

    if (Stable_GPIO_State == Key_Free_State)
    {
        Key_Status = BSP_Key_Status_FREE;
    }
    else
    {
        Key_Status = BSP_Key_Status_PRESSED;
    }
}

/**
 * @brief 每10ms读取并处理一次按键状态
 *
 * @note 连续两次读到相同的新状态后才确认状态变化, 用于按键消抖
 */
void Class_Key::TIM_10ms_PeriodElapsedCallback()
{
    GPIO_PinState now_gpio_state = HAL_GPIO_ReadPin(GPIO_Port, GPIO_Pin);

    // 默认保持当前稳定状态
    if (Stable_GPIO_State == Key_Free_State)
    {
        Key_Status = BSP_Key_Status_FREE;
    }
    else
    {
        Key_Status = BSP_Key_Status_PRESSED;
    }

    // 本次读取与稳定状态相同, 不需要继续消抖
    if (now_gpio_state == Stable_GPIO_State)
    {
        Candidate_GPIO_State = Stable_GPIO_State;
        Candidate_Count = 0;
        return;
    }

    // 第一次读到新的状态
    if (now_gpio_state != Candidate_GPIO_State)
    {
        Candidate_GPIO_State = now_gpio_state;
        Candidate_Count = 1;
        return;
    }

    Candidate_Count++;
    if (Candidate_Count < 2)
    {
        return;
    }

    GPIO_PinState pre_gpio_state = Stable_GPIO_State;
    Stable_GPIO_State = Candidate_GPIO_State;
    Candidate_Count = 0;

    if (pre_gpio_state == Key_Free_State && Stable_GPIO_State != Key_Free_State)
    {
        Key_Status = BSP_Key_Status_TRIG_FREE_PRESSED;
    }
    else if (pre_gpio_state != Key_Free_State && Stable_GPIO_State == Key_Free_State)
    {
        Key_Status = BSP_Key_Status_TRIG_PRESSED_FREE;
    }
}

