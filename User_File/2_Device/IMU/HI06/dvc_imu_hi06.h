/**
 * @file dvc_imu_hi06.h
 * @author jiang
 * @brief 超核电子HI06配置与操作
 * @version 0.1
 * @date 2026-10-05
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef DVC_IMU_HI06_H
#define DVC_IMU_HI06_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Hipnuc/hipnuc_dec.h"
#include "2_Device/IMU/HI06/HI06_Define.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 超核电子HI06类
 *
 */
class Class_HI06
{
public:
    void Init();

    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void TIM_10ms_PeriodElapsedCallback();

    inline Enum_HI06_Status Get_Status() const;
    inline const Struct_HI06_Data &Get_Data() const;

protected:
    Enum_HI06_Status Status = HI06_Status_DISABLE;
    Struct_HI06_Data Data{};
    hipnuc_raw_t Raw_Data{};
    hipnuc_sample_t Sample_Data{};

    uint8_t Rx_Buffer[HI06_RX_BUFFER_SIZE]{};
    volatile uint16_t Rx_Write_Index = 0U;
    volatile uint16_t Rx_Read_Index = 0U;
    volatile uint8_t Rx_Overflow = 0U;

    void Buffer_Overflow_Process();
    void Data_Process();
};

/* Exported variables --------------------------------------------------------*/

extern Class_HI06 HI06;

/* Exported function declarations -------------------------------------------*/

/**
 * @brief 获取HI06状态
 *
 * @return Enum_HI06_Status HI06状态
 */
inline Enum_HI06_Status Class_HI06::Get_Status() const
{
    return Status;
}

/**
 * @brief 获取HI06数据
 *
 * @return const Struct_HI06_Data& HI06数据
 */
inline const Struct_HI06_Data &Class_HI06::Get_Data() const
{
    return Data;
}

#endif
