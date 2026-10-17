/**
 * @file pro_upper.cpp
 * @author jiang
 * @brief UART7上位机原始数据接收与固定长度组帧
 * @version 0.1
 * @date 2026-10-13 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "pro_upper.h"

#include "stm32h7xx_hal.h"
#include <cstring>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

Class_Upper Upper;

/* Function prototypes -------------------------------------------------------*/

uint16_t Upper_CRC16_Calculate(const uint8_t *Buffer, uint16_t Length)
{
    uint16_t crc = UPPER_CRC_INITIAL;
    if (Buffer == nullptr) { return crc; }
    for (uint16_t i = 0U; i < Length; i++)
    {
        crc ^= static_cast<uint16_t>(Buffer[i]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = static_cast<uint16_t>((crc << 1U) ^ ((crc & 0x8000U) != 0U ? UPPER_CRC_POLYNOMIAL : 0U));
        }
    }
    return crc;
}

bool Upper_CRC16_Check(const uint8_t *Frame, uint16_t Length)
{
    if (Frame == nullptr || Length != UPPER_FRAME_LENGTH) { return false; }
    const uint16_t received_crc = static_cast<uint16_t>(Frame[UPPER_CRC_INDEX] | (static_cast<uint16_t>(Frame[UPPER_CRC_INDEX + 1U]) << 8U));
    return Upper_CRC16_Calculate(Frame, UPPER_CRC_INDEX) == received_crc;
}

/**
 * @brief 初始化协议对象, 应先于UART7接收注册调用
 */
void Class_Upper::Init()
{
    Status = Upper_Status_OFFLINE;
    Data = {};
    CRC_Check_Enable = false;
    Buffer_Reset();
}

/**
 * @brief 清除待处理字节及残帧, 临界区只更新索引, 不清除已发布数据和统计
 */
void Class_Upper::Buffer_Reset()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Rx_Read_Index = Rx_Write_Index;
    Rx_Overflow = 0U;
    Frame_Length = 0U;
    __DMB();
    __set_PRIMASK(primask);
}

/**
 * @brief 设置CRC接收校验策略, 切换时丢弃旧策略下尚未处理的字节
 * @param Enable true开启, false关闭; 默认关闭
 */
void Class_Upper::Set_CRC_Check_Enable(bool Enable)
{
    if (CRC_Check_Enable == Enable) { return; }
    Buffer_Reset();
    CRC_Check_Enable = Enable;
}

/**
 * @brief UART7接收回调, 只复制字节, 不解析、不打印
 * @param Buffer DMA提供的接收区, 返回前复制到本对象缓存
 * @param Length 本次接收字节数, 不代表协议帧边界
 */
void Class_Upper::UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == nullptr || Length == 0U) { return; }
    Data.Rx_Byte_Count += Length;
    if (Rx_Overflow != 0U) { return; }
    for (uint16_t i = 0U; i < Length; i++)
    {
        const uint16_t next_index = static_cast<uint16_t>((Rx_Write_Index + 1U) % UPPER_RX_BUFFER_SIZE);
        if (next_index == Rx_Read_Index)
        {
            Rx_Overflow = 1U;
            Data.Buffer_Overflow_Count++;
            break;
        }
        Rx_Buffer[Rx_Write_Index] = Buffer[i];
        __DMB();
        Rx_Write_Index = next_index;
    }
}

/**
 * @brief 小端有符号16位读取, 不缩放, 显式处理负数避免实现相关转换
 */
int16_t Class_Upper::Read_Int16(const uint8_t *Buffer)
{
    const uint16_t raw = static_cast<uint16_t>(Buffer[0] | (static_cast<uint16_t>(Buffer[1]) << 8U));
    const int32_t value = raw >= 0x8000U ? static_cast<int32_t>(raw) - 65536 : raw;
    return static_cast<int16_t>(value);
}

/**
 * @brief 发布完整帧, 短临界区确保溢出检查与发布之间不插入缺字节区段
 */
void Class_Upper::Data_Process()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (Rx_Overflow == 0U)
    {
        Data.Speed_Y = Read_Int16(Frame_Buffer + UPPER_SPEED_Y_INDEX);
        Data.Speed_X = Read_Int16(Frame_Buffer + UPPER_SPEED_X_INDEX);
        Data.Speed_W = Read_Int16(Frame_Buffer + UPPER_SPEED_W_INDEX);
        std::memcpy(Data.Data_Field, Frame_Buffer + UPPER_DATA_FIELD_INDEX, sizeof(Data.Data_Field));
        std::memcpy(Data.Extend_Data, Frame_Buffer + UPPER_EXTEND_DATA_INDEX, sizeof(Data.Extend_Data));
        std::memcpy(Data.Raw_Data, Frame_Buffer, sizeof(Data.Raw_Data));
        Data.Last_Rx_Tick = HAL_GetTick();
        Data.Frame_Count++;
        Data.Valid = true;
        Status = Upper_Status_ONLINE;
    }
    __set_PRIMASK(primask);
}

/**
 * @brief 失败候选帧重新同步, 保留内部下一帧头或末尾单字节55
 */
void Class_Upper::Frame_Resync()
{
    for (uint16_t i = 1U; i + 1U < Frame_Length; i++)
    {
        if (Frame_Buffer[i] == UPPER_FRAME_HEAD_H && Frame_Buffer[i + 1U] == UPPER_FRAME_HEAD_L)
        {
            Frame_Length = static_cast<uint16_t>(Frame_Length - i);
            std::memmove(Frame_Buffer, Frame_Buffer + i, Frame_Length);
            return;
        }
    }
    Frame_Length = Frame_Buffer[Frame_Length - 1U] == UPPER_FRAME_HEAD_H ? 1U : 0U;
    if (Frame_Length == 1U) { Frame_Buffer[0] = UPPER_FRAME_HEAD_H; }
}

/**
 * @brief 固定长度字节流组帧, 载荷中的55AA及77不提前结束报文
 */
void Class_Upper::Frame_Byte_Process(uint8_t Byte)
{
    if (Frame_Length == 0U)
    {
        if (Byte == UPPER_FRAME_HEAD_H) { Frame_Buffer[Frame_Length++] = Byte; }
        return;
    }
    if (Frame_Length == 1U && Byte != UPPER_FRAME_HEAD_L)
    {
        Frame_Length = Byte == UPPER_FRAME_HEAD_H ? 1U : 0U;
        return;
    }
    Frame_Buffer[Frame_Length++] = Byte;
    if (Frame_Length != UPPER_FRAME_LENGTH) { return; }
    if (Frame_Buffer[UPPER_FRAME_TAIL_INDEX] != UPPER_FRAME_TAIL)
    {
        Data.Invalid_Frame_Count++;
        Frame_Resync();
        return;
    }
    if (CRC_Check_Enable && !Upper_CRC16_Check(Frame_Buffer, UPPER_FRAME_LENGTH))
    {
        Data.CRC_Error_Count++;
        Frame_Resync();
        return;
    }
    Data_Process();
    Frame_Length = 0U;
}

/**
 * @brief 10ms前台解析及超时诊断, 每次最多处理进入时已有的缓存字节
 */
void Class_Upper::TIM_10ms_PeriodElapsedCallback()
{
    if (Rx_Overflow != 0U) { Buffer_Reset(); }
    const uint16_t pending_length = static_cast<uint16_t>((Rx_Write_Index + UPPER_RX_BUFFER_SIZE - Rx_Read_Index) % UPPER_RX_BUFFER_SIZE);
    for (uint16_t i = 0U; i < pending_length; i++)
    {
        __DMB();
        if (Rx_Overflow != 0U) { Buffer_Reset(); break; }
        const uint8_t byte = Rx_Buffer[Rx_Read_Index];
        Rx_Read_Index = static_cast<uint16_t>((Rx_Read_Index + 1U) % UPPER_RX_BUFFER_SIZE);
        Frame_Byte_Process(byte);
        if (Rx_Overflow != 0U) { Buffer_Reset(); break; }
    }
    if (Status == Upper_Status_ONLINE && static_cast<uint32_t>(HAL_GetTick() - Data.Last_Rx_Tick) >= UPPER_OFFLINE_TIMEOUT)
    {
        Status = Upper_Status_OFFLINE;
        Data.Valid = false;
    }
}
