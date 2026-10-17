/**
 * @file pro_sbus.cpp
 * @brief NRF24L01遥控数据接收协议
 * @version 0.1
 * @date 2026-10-03 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "pro_sbus.h"

/* Private macros ------------------------------------------------------------*/

#define SBUS_FRAME_HEAD 0x0F
#define SBUS_FRAME_FLAG 0xFF
#define SBUS_FRAME_TAIL 0xAF
#define SBUS_CUSTOM_FLAG 0x88

#define SBUS_CHANNEL_MIDDLE 1000
#define SBUS_ALIVE_COUNT_MAX 100

/* Exported variables --------------------------------------------------------*/

Class_SBUS_Receive SBUS_Receive;

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 遥控数据恢复为安全值
 */
void Class_SBUS_Receive::Data_Reset()
{
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUM; i++)
    {
        SBUS_Data.Channel[i] = SBUS_CHANNEL_MIDDLE;
        SBUS_Data.Function[i] = 0;
    }

    SBUS_Data.Mode = 0;
    SBUS_Data.Motor_Enable = 0;
    SBUS_Data.Online = 0;
    Alive_Count = 0;
}

/**
 * @brief 初始化SBUS接收数据
 */
void Class_SBUS_Receive::Init()
{
    Data_Reset();
}

/**
 * @brief 校验并解析31字节遥控数据
 *
 * @param data NRF24L01接收数据
 * @return 0 数据无效
 * @return 1 数据有效
 */
uint8_t Class_SBUS_Receive::Data_Process(const uint8_t *data)
{
    if (data[0] != SBUS_FRAME_HEAD || data[23] != SBUS_FRAME_FLAG || data[24] != SBUS_FRAME_TAIL || data[28] != SBUS_CUSTOM_FLAG)
    {
        return 0;
    }

    // 16路SBUS通道, 每路数据占11位
    SBUS_Data.Channel[0] = ((data[1] >> 0 | data[2] << 8) & 0x07FF);
    SBUS_Data.Channel[1] = ((data[2] >> 3 | data[3] << 5) & 0x07FF);
    SBUS_Data.Channel[2] = ((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
    SBUS_Data.Channel[3] = ((data[5] >> 1 | data[6] << 7) & 0x07FF);
    SBUS_Data.Channel[4] = ((data[6] >> 4 | data[7] << 4) & 0x07FF);
    SBUS_Data.Channel[5] = ((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
    SBUS_Data.Channel[6] = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
    SBUS_Data.Channel[7] = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
    SBUS_Data.Channel[8] = ((data[12] >> 0 | data[13] << 8) & 0x07FF);
    SBUS_Data.Channel[9] = ((data[13] >> 3 | data[14] << 5) & 0x07FF);
    SBUS_Data.Channel[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF);
    SBUS_Data.Channel[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF);
    SBUS_Data.Channel[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF);
    SBUS_Data.Channel[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF);
    SBUS_Data.Channel[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF);
    SBUS_Data.Channel[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF);

    // 串口屏模式和电机使能
    SBUS_Data.Mode = data[25] & 0x03;
    SBUS_Data.Motor_Enable = (data[25] >> 2) & 0x01;

    // 串口屏功能按钮1~16
    for (uint8_t i = 0; i < 8; i++)
    {
        SBUS_Data.Function[i] = (data[26] >> i) & 0x01;
        SBUS_Data.Function[i + 8] = (data[27] >> i) & 0x01;
    }

    Alive_Count = 0;
    SBUS_Data.Online = 1;

    return 1;
}

/**
 * @brief 每10ms检查一次遥控器在线状态
 */
void Class_SBUS_Receive::TIM_10ms_PeriodElapsedCallback()
{
    if (SBUS_Data.Online == 0)
    {
        return;
    }

    Alive_Count++;
    if (Alive_Count >= SBUS_ALIVE_COUNT_MAX)
    {
        Data_Reset();
    }
}
