/**
 * @file dvc_imu_hi06.cpp
 * @author jiang
 * @brief 超核电子HI06配置与操作
 * @version 0.1
 * @date 2026-10-05
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_imu_hi06.h"

#include "stm32h7xx_hal.h"
#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

Class_HI06 HI06;

/**
 * @brief HI06初始化
 *
 */
void Class_HI06::Init()
{
    Status = HI06_Status_OFFLINE;
    std::memset(&Data, 0, sizeof(Data));
    std::memset(&Raw_Data, 0, sizeof(Raw_Data));
    std::memset(&Sample_Data, 0, sizeof(Sample_Data));
    std::memset(Rx_Buffer, 0, sizeof(Rx_Buffer));
    Rx_Write_Index = 0U;
    Rx_Read_Index = 0U;
    Rx_Overflow = 0U;
}

/**
 * @brief UART接收完成回调函数, 只缓存字节
 *
 * @param Buffer 接收数据指针
 * @param Length 接收数据长度
 */
void Class_HI06::UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == nullptr)
    {
        return;
    }

    uint8_t overflow_in_callback = 0U;
    for (uint16_t i = 0U; i < Length; i++)
    {
        const uint16_t next_write_index = static_cast<uint16_t>((Rx_Write_Index + 1U) % HI06_RX_BUFFER_SIZE);
        if (next_write_index == Rx_Read_Index)
        {
            overflow_in_callback = 1U;
            Rx_Overflow = 1U;
            break;
        }

        Rx_Buffer[Rx_Write_Index] = Buffer[i];
        __DMB();
        Rx_Write_Index = next_write_index;
        Data.Rx_Byte_Count++;
    }

    if (overflow_in_callback != 0U)
    {
        Data.Buffer_Overflow_Count++;
    }
}

/**
 * @brief 丢弃发生溢出的接收区段并清除残帧
 *
 */
void Class_HI06::Buffer_Overflow_Process()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Rx_Read_Index = Rx_Write_Index;
    Rx_Overflow = 0U;
    __DMB();
    __set_PRIMASK(primask);

    const uint32_t frame_count = Raw_Data.frame_count;
    const uint32_t crc_error_count = Raw_Data.crc_error_count;
    const uint32_t invalid_count = Raw_Data.invalid_count;
    std::memset(&Raw_Data, 0, sizeof(Raw_Data));
    Raw_Data.frame_count = frame_count;
    Raw_Data.crc_error_count = crc_error_count;
    Raw_Data.invalid_count = invalid_count;
}

/**
 * @brief 处理一帧HI91数据
 *
 */
void Class_HI06::Data_Process()
{
    if ((Sample_Data.valid & HIPNUC_VALID_STATUS) != 0U)
    {
        Data.Main_Status = Sample_Data.main_status;
    }
    if ((Sample_Data.valid & HIPNUC_VALID_ROLL_PITCH) != 0U)
    {
        Data.Roll = Sample_Data.roll;
        Data.Pitch = Sample_Data.pitch;
    }
    if ((Sample_Data.valid & HIPNUC_VALID_YAW) != 0U)
    {
        Data.Yaw = Sample_Data.yaw;
    }
    if ((Sample_Data.valid & HIPNUC_VALID_ACC) != 0U)
    {
        Data.Accel_X = Sample_Data.acc[0];
        Data.Accel_Y = Sample_Data.acc[1];
        Data.Accel_Z = Sample_Data.acc[2];
    }
    if ((Sample_Data.valid & HIPNUC_VALID_GYR) != 0U)
    {
        Data.Gyro_X = Sample_Data.gyr[0];
        Data.Gyro_Y = Sample_Data.gyr[1];
        Data.Gyro_Z = Sample_Data.gyr[2];
    }
    if ((Sample_Data.valid & HIPNUC_VALID_TEMPERATURE) != 0U)
    {
        Data.Temperature = Sample_Data.temperature;
    }

    Data.Valid = Sample_Data.valid;
    Data.Last_Rx_Tick = HAL_GetTick();
    Data.Frame_Count++;
    Status = HI06_Status_ONLINE;
}

/**
 * @brief 10ms周期处理HI06接收数据
 *
 */
void Class_HI06::TIM_10ms_PeriodElapsedCallback()
{
    if (Rx_Overflow != 0U)
    {
        Buffer_Overflow_Process();
    }

    while (Rx_Read_Index != Rx_Write_Index)
    {
        __DMB();
        if (Rx_Overflow != 0U)
        {
            Buffer_Overflow_Process();
            break;
        }

        const uint8_t rx_data = Rx_Buffer[Rx_Read_Index];
        Rx_Read_Index = static_cast<uint16_t>((Rx_Read_Index + 1U) % HI06_RX_BUFFER_SIZE);
        const int decode_result = hipnuc_input(&Raw_Data, rx_data);

        if (Rx_Overflow != 0U)
        {
            Buffer_Overflow_Process();
            break;
        }
        if (decode_result > 0 && hipnuc_sample_from_raw(&Raw_Data, &Sample_Data) > 0 && Sample_Data.source == HIPNUC_SOURCE_HI91)
        {
            Data_Process();
        }
    }

    Data.CRC_Error_Count = Raw_Data.crc_error_count;
    Data.Invalid_Frame_Count = Raw_Data.invalid_count;

    if (Status == HI06_Status_ONLINE && static_cast<uint32_t>(HAL_GetTick() - Data.Last_Rx_Tick) >= HI06_OFFLINE_TIMEOUT)
    {
        Status = HI06_Status_OFFLINE;
    }
}
