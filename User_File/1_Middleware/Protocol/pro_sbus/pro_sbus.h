/**
 * @file pro_sbus.h
 * @brief NRF24L01遥控数据接收协议
 * @version 0.1
 * @date 2026-10-03 0.1 新建文档
 */

#ifndef __PRO_SBUS_H
#define __PRO_SBUS_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Exported macros -----------------------------------------------------------*/

#define SBUS_CHANNEL_NUM 16

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint16_t Channel[SBUS_CHANNEL_NUM];
    uint8_t Mode;
    uint8_t Motor_Enable;
    uint8_t Function[16];
    uint8_t Online;
} Struct_SBUS_Data;

class Class_SBUS_Receive
{
private:
    uint8_t Alive_Count;

    void Data_Reset();

public:
    Struct_SBUS_Data SBUS_Data;

    void Init();
    uint8_t Data_Process(const uint8_t *data);
    void TIM_10ms_PeriodElapsedCallback();
};

/* Exported variables --------------------------------------------------------*/

extern Class_SBUS_Receive SBUS_Receive;

#endif
