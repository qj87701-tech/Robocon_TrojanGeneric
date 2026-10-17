/**
 * @file HI06_Define.h
 * @author jiang
 * @brief 超核电子HI06配置
 * @version 0.1
 * @date 2026-10-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef HI06_DEFINE_H
#define HI06_DEFINE_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

// UART4接收环形缓冲区大小
#define HI06_RX_BUFFER_SIZE 1024U
// 超过500ms没有收到有效HI91数据则认为IMU离线
#define HI06_OFFLINE_TIMEOUT 500U

/* Exported types ------------------------------------------------------------*/

/**
 * @brief HI06状态
 *
 */
enum Enum_HI06_Status
{
    HI06_Status_DISABLE = 0,
    HI06_Status_OFFLINE,
    HI06_Status_ONLINE,
};

/**
 * @brief HI06数据, 姿态与角速度单位rad、rad/s, 加速度单位m/s^2
 *
 */
struct Struct_HI06_Data
{
    uint64_t Valid;
    uint16_t Main_Status;
    float Roll;
    float Pitch;
    float Yaw;
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    float Temperature;
    uint32_t Last_Rx_Tick;
    uint32_t Rx_Byte_Count;
    uint32_t Frame_Count;
    uint32_t CRC_Error_Count;
    uint32_t Invalid_Frame_Count;
    uint32_t Buffer_Overflow_Count;
};

#endif
