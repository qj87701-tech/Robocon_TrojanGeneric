/**
 * @file pro_upper.h
 * @author jiang
 * @brief UART7上位机32字节协议与原始数据封装
 * @version 0.1
 * @date 2026-10-13 0.1 新建文档
 */

#ifndef PRO_UPPER_H
#define PRO_UPPER_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/

#define UPPER_FRAME_LENGTH 32U
#define UPPER_FRAME_HEAD_H 0x55U
#define UPPER_FRAME_HEAD_L 0xAAU
#define UPPER_FRAME_TAIL 0x77U
#define UPPER_SPEED_Y_INDEX 2U
#define UPPER_SPEED_X_INDEX 4U
#define UPPER_SPEED_W_INDEX 6U
#define UPPER_DATA_FIELD_INDEX 8U
#define UPPER_DATA_FIELD_NUMBER 12U
#define UPPER_EXTEND_DATA_INDEX 20U
#define UPPER_EXTEND_DATA_NUMBER 9U
#define UPPER_CRC_INDEX 29U
#define UPPER_FRAME_TAIL_INDEX 31U
#define UPPER_CRC_POLYNOMIAL 0x1021U
#define UPPER_CRC_INITIAL 0xFFFFU
#define UPPER_RX_BUFFER_SIZE 1024U
#define UPPER_OFFLINE_TIMEOUT 500U // ms, 仅用于通信诊断, 后续按上位机发送周期调整

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 上位机通信状态, 不参与电机使能判断
 */
enum Enum_Upper_Status
{
    Upper_Status_DISABLE = 0,
    Upper_Status_OFFLINE,
    Upper_Status_ONLINE,
};

/**
 * @brief 最近完整帧及通信统计, 速度保留原始整数, 尚未约定物理单位
 * @note CRC默认关闭时, Valid仅代表长度及帧头帧尾符合格式
 */
struct Struct_Upper_Data
{
    int16_t Speed_X = 0;
    int16_t Speed_Y = 0;
    int16_t Speed_W = 0;
    uint8_t Data_Field[UPPER_DATA_FIELD_NUMBER]{};
    uint8_t Extend_Data[UPPER_EXTEND_DATA_NUMBER]{};
    uint8_t Raw_Data[UPPER_FRAME_LENGTH]{};
    bool Valid = false;
    uint32_t Last_Rx_Tick = 0U;
    uint32_t Rx_Byte_Count = 0U;
    uint32_t Frame_Count = 0U;
    uint32_t Invalid_Frame_Count = 0U;
    uint32_t CRC_Error_Count = 0U;
    uint32_t Buffer_Overflow_Count = 0U;
};

/**
 * @brief 上位机协议类, ISR只缓存, 10ms前台解析, 不执行接收数据中的控制动作
 */
class Class_Upper
{
public:
    void Init();
    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    void TIM_10ms_PeriodElapsedCallback();
    void Set_CRC_Check_Enable(bool Enable);

    inline const Struct_Upper_Data &Get_Data() const { return Data; }
    inline Enum_Upper_Status Get_Status() const { return Status; }
    inline bool Get_CRC_Check_Enable() const { return CRC_Check_Enable; }

protected:
    Enum_Upper_Status Status = Upper_Status_DISABLE;
    Struct_Upper_Data Data{};
    bool CRC_Check_Enable = false;
    uint8_t Rx_Buffer[UPPER_RX_BUFFER_SIZE]{};
    volatile uint16_t Rx_Write_Index = 0U;
    volatile uint16_t Rx_Read_Index = 0U;
    volatile uint8_t Rx_Overflow = 0U;
    uint8_t Frame_Buffer[UPPER_FRAME_LENGTH]{};
    uint16_t Frame_Length = 0U;

    void Buffer_Reset();
    void Frame_Byte_Process(uint8_t Byte);
    void Frame_Resync();
    void Data_Process();
    static int16_t Read_Int16(const uint8_t *Buffer);
};

/* Exported variables --------------------------------------------------------*/

extern Class_Upper Upper;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief CRC-16/CCITT-FALSE计算, 无反射、无最终异或
 * @param Buffer 待计算字节, 空指针返回初值
 * @param Length 字节数
 * @return uint16_t CRC, 标准向量123456789结果为0x29B1
 */
uint16_t Upper_CRC16_Calculate(const uint8_t *Buffer, uint16_t Length);

/**
 * @brief 检查32字节报文的CRC字段, 不检查帧头帧尾
 * @param Frame 报文字节, 下标29为CRC低字节, 30为高字节
 * @param Length 必须为32字节
 * @return bool 字节0~28的CRC是否匹配
 */
bool Upper_CRC16_Check(const uint8_t *Frame, uint16_t Length);

#endif
