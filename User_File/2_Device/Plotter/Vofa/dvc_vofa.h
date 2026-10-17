/**
 * @file dvc_vofa.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief vofa
 * @version 0.1
 * @date 2025-09-22 0.1 新建
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef DVC_VOFA_H
#define DVC_VOFA_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/UART/drv_uart.h"
#include <stdarg.h>

/* Exported macros -----------------------------------------------------------*/

#define VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH (100)
#define VOFA_DATA_MAX_NUMBER (24)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Reusable, vofa工具, JustFloat模式
 */
class Class_Vofa_UART
{
public:
    void Init(const UART_HandleTypeDef *huart, const uint8_t &__Rx_Variable_Assignment_Num = 0, const char **__Rx_Variable_Assignment_List = nullptr, const uint32_t &__Frame_Tail = 0x7f800000);

    inline int32_t Get_Variable_Index() const;

    inline float Get_Variable_Value() const;

    inline void Set_Data(const int &Number, ...);

    void UART_RxCpltCallback(const uint8_t *Rx_Data, const uint16_t &Length);

    void TIM_1ms_Write_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;
    // 接收指令字典的数量
    uint8_t Rx_Variable_Num = 0U;
    // 接收指令字典列表指针
    const char **Rx_Variable_List = nullptr;
    // 数据包尾标
    uint32_t Frame_Tail = 0x7f800000U;

    // 内部变量

    // 发送缓冲区
    uint8_t Tx_Buffer[UART_BUFFER_SIZE]{};
    // 需要绘图的各个变量数据地址, VOFA最多显示24个通道
    const void *Data[VOFA_DATA_MAX_NUMBER]{};
    // 当前发送的数据数量
    uint8_t Data_Number = 0U;
    // 当前接收的指令在指令字典中的编号
    int32_t Variable_Index = -1;
    // 当前接收的指令在指令字典中的值
    float Variable_Value = 0.0f;

    // 内部函数

    void Data_Process(const uint8_t *Rx_Data, const uint16_t &Length);

    uint16_t Judge_Variable_Name(const uint8_t *Rx_Data, const uint16_t &Length);

    void Judge_Variable_Value(const uint8_t *Rx_Data, const uint16_t &Length, uint16_t Value_Index);

    void Output();
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前接收的指令在指令字典中的编号
 *
 * @return int32_t 指令编号, 未匹配时为-1
 */
inline int32_t Class_Vofa_UART::Get_Variable_Index() const
{
    return (Variable_Index);
}

/**
 * @brief 获取当前接收的指令在指令字典中的值
 *
 * @return float 指令数值
 */
inline float Class_Vofa_UART::Get_Variable_Value() const
{
    return (Variable_Value);
}

/**
 * @brief 添加被发送的数据
 *
 * @param Number 添加的数据数量
 * @param ... 每个数据的指针, 调用时转换为const void指针
 */
inline void Class_Vofa_UART::Set_Data(const int &Number, ...)
{
    Data_Number = Number > VOFA_DATA_MAX_NUMBER ? VOFA_DATA_MAX_NUMBER : (Number < 0 ? 0U : static_cast<uint8_t>(Number));

    va_list data_ptr;
    va_start(data_ptr, Number);
    for (uint8_t i = 0U; i < Data_Number; i++)
    {
        Data[i] = va_arg(data_ptr, const void *);
    }
    va_end(data_ptr);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
