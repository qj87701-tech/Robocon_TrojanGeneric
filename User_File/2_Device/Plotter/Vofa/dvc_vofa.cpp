/**
 * @file dvc_vofa.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief vofa
 * @version 0.1
 * @date 2025-09-22 0.1 新建
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_vofa.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief vofa初始化
 *
 * @param huart 指定的UART
 * @param __Rx_Variable_Assignment_Num 接收指令字典的数量
 * @param __Rx_Variable_Assignment_List 接收指令字典列表
 * @param __Frame_Tail JustFloat帧尾标识符
 */
void Class_Vofa_UART::Init(const UART_HandleTypeDef *huart, const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const uint32_t &__Frame_Tail)
{
    if (huart->Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == UART8)
    {
        UART_Manage_Object = &UART8_Manage_Object;
    }
    else if (huart->Instance == UART9)
    {
        UART_Manage_Object = &UART9_Manage_Object;
    }
    else if (huart->Instance == USART10)
    {
        UART_Manage_Object = &UART10_Manage_Object;
    }

    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = __Rx_Variable_Assignment_List;
    Frame_Tail = __Frame_Tail;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data UART接收数据
 * @param Length UART接收数据长度, Byte
 */
void Class_Vofa_UART::UART_RxCpltCallback(const uint8_t *Rx_Data, const uint16_t &Length)
{
    Data_Process(Rx_Data, Length);
}

/**
 * @brief 将JustFloat数据发送到VOFA
 */
void Class_Vofa_UART::TIM_1ms_Write_PeriodElapsedCallback()
{
    if (UART_Manage_Object == nullptr || UART_Manage_Object->UART_Handler == nullptr || Data_Number == 0U)
    {
        return;
    }

    Output();
    UART_Transmit_Data(UART_Manage_Object->UART_Handler, Tx_Buffer, Data_Number * sizeof(float) + sizeof(uint32_t));
}

/**
 * @brief 数据处理过程
 *
 * @param Rx_Data VOFA接收指令数据
 * @param Length VOFA接收指令长度, Byte
 */
void Class_Vofa_UART::Data_Process(const uint8_t *Rx_Data, const uint16_t &Length)
{
    uint16_t value_index = Judge_Variable_Name(Rx_Data, Length);
    Judge_Variable_Value(Rx_Data, Length, value_index);
}

/**
 * @brief 判断指令变量名
 *
 * @param Rx_Data VOFA接收指令数据
 * @param Length VOFA接收指令长度, Byte
 * @return uint16_t 数值在接收数据中的起始位置
 */
uint16_t Class_Vofa_UART::Judge_Variable_Name(const uint8_t *Rx_Data, const uint16_t &Length)
{
    char variable_name[VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH]{};
    uint16_t equal_index = 0U;

    Variable_Index = -1;
    while (equal_index < Length && Rx_Data[equal_index] != '=' && equal_index < VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH - 1U)
    {
        variable_name[equal_index] = static_cast<char>(Rx_Data[equal_index]);
        equal_index++;
    }

    if (equal_index >= Length || Rx_Data[equal_index] != '=' || Rx_Variable_List == nullptr)
    {
        return (Length);
    }

    for (uint8_t i = 0U; i < Rx_Variable_Num; i++)
    {
        if (Rx_Variable_List[i] != nullptr && strcmp(variable_name, Rx_Variable_List[i]) == 0)
        {
            Variable_Index = i;
            break;
        }
    }
    return (equal_index + 1U);
}

/**
 * @brief 判断指令变量数值
 *
 * @param Rx_Data VOFA接收指令数据
 * @param Length VOFA接收指令长度, Byte
 * @param Value_Index 数值在接收数据中的起始位置
 */
void Class_Vofa_UART::Judge_Variable_Value(const uint8_t *Rx_Data, const uint16_t &Length, uint16_t Value_Index)
{
    if (Variable_Index < 0 || Value_Index >= Length)
    {
        return;
    }

    float sign = 1.0f;
    float integer = 0.0f;
    float decimal = 0.0f;
    float decimal_scale = 0.1f;
    bool decimal_part = false;

    if (Rx_Data[Value_Index] == '-')
    {
        sign = -1.0f;
        Value_Index++;
    }

    for (uint16_t i = Value_Index; i < Length && Rx_Data[i] != '#'; i++)
    {
        if (Rx_Data[i] == '.')
        {
            decimal_part = true;
            continue;
        }
        if (Rx_Data[i] < '0' || Rx_Data[i] > '9')
        {
            Variable_Index = -1;
            return;
        }

        float digit = static_cast<float>(Rx_Data[i] - '0');
        if (decimal_part)
        {
            decimal += digit * decimal_scale;
            decimal_scale *= 0.1f;
        }
        else
        {
            integer = integer * 10.0f + digit;
        }
    }

    Variable_Value = sign * (integer + decimal);
}

/**
 * @brief 将JustFloat通道数据和帧尾写入UART发送缓冲区
 */
void Class_Vofa_UART::Output()
{
    memset(Tx_Buffer, 0, sizeof(Tx_Buffer));
    for (uint8_t i = 0U; i < Data_Number; i++)
    {
        memcpy(Tx_Buffer + i * sizeof(float), Data[i], sizeof(float));
    }
    memcpy(Tx_Buffer + Data_Number * sizeof(float), &Frame_Tail, sizeof(Frame_Tail));
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
