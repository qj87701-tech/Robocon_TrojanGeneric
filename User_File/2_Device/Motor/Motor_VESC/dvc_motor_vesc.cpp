/**
 * @file dvc_motor_vesc.cpp
 * @brief VESC航模电机CAN驱动
 * @version 0.3
 * @date 2026-10-06 0.1 新建文档
 * @date 2026-10-07 0.2 增加六种VESC CAN控制方式
 * @date 2026-10-08 0.3 修正STATUS_5长度并增加状态报文超时管理
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_motor_vesc.h"

/* Private macros ------------------------------------------------------------*/

#define MOTOR_VESC_PACKET_SET_DUTY 0U
#define MOTOR_VESC_PACKET_SET_CURRENT 1U
#define MOTOR_VESC_PACKET_SET_CURRENT_BRAKE 2U
#define MOTOR_VESC_PACKET_SET_RPM 3U
#define MOTOR_VESC_PACKET_SET_POSITION 4U
#define MOTOR_VESC_PACKET_SET_CURRENT_REL 10U
#define MOTOR_VESC_PACKET_STATUS 9U
#define MOTOR_VESC_PACKET_STATUS_4 16U
#define MOTOR_VESC_PACKET_STATUS_5 27U
#define MOTOR_VESC_STATUS_TIMEOUT_MS 100U
#define MOTOR_VESC_PI 3.14159265358979323846f

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

static void Motor_VESC_Int32_To_BE(int32_t Value, uint8_t *Buffer);
static int16_t Motor_VESC_BE_To_Int16(const uint8_t *Buffer);
static int32_t Motor_VESC_BE_To_Int32(const uint8_t *Buffer);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化VESC电机
 *
 * @param hcan 绑定的CAN总线
 * @param __VESC_ID VESC节点ID
 * @param __Pole_Pairs 电机极对数, 用于机械转速与ERPM换算
 * @param __Gearbox_Rate 电机到轮组的传动比
 * @param __ERPM_Max 最大电气转速, ERPM
 * @param __Current_Max 最大电机电流, A
 * @param __Current_Brake_Max 最大制动电流, A
 */
void Class_Motor_VESC::Init(FDCAN_HandleTypeDef *hcan, uint8_t __VESC_ID, uint8_t __Pole_Pairs, float __Gearbox_Rate, int32_t __ERPM_Max, float __Current_Max, float __Current_Brake_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }

    VESC_ID = __VESC_ID;
    Pole_Pairs = __Pole_Pairs;
    Gearbox_Rate = __Gearbox_Rate;
    ERPM_Max = __ERPM_Max;
    Current_Max = __Current_Max;
    Current_Brake_Max = __Current_Brake_Max;
    Motor_Enable = false;
    Motor_VESC_Status = Motor_VESC_Status_DISABLE;
    Control_Method = Motor_VESC_Control_Method_RPM;
    Rx_Data = {};
    Status_1_State = {};
    Status_4_State = {};
    Status_5_State = {};
    Reset_Control_Targets();
}

/**
 * @brief CAN接收回调, 解析当前VESC节点的状态报文
 */
void Class_Motor_VESC::CAN_RxCpltCallback()
{
    Data_Process();
}

/**
 * @brief 每100ms检查一次在线状态
 */
void Class_Motor_VESC::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Status_1_State.Valid = Get_Rx_Frame_Valid(Status_1_State);
    Status_4_State.Valid = Get_Rx_Frame_Valid(Status_4_State);
    Status_5_State.Valid = Get_Rx_Frame_Valid(Status_5_State);
    Motor_VESC_Status = (Status_1_State.Valid || Status_4_State.Valid || Status_5_State.Valid) ? Motor_VESC_Status_ENABLE : Motor_VESC_Status_DISABLE;
}

/**
 * @brief 周期发送VESC控制报文, 由底盘10ms周期调用
 */
void Class_Motor_VESC::TIM_Send_PeriodElapsedCallback()
{
    Output();
}

/**
 * @brief 处理VESC扩展帧状态报文并更新电机反馈数据
 */
void Class_Motor_VESC::Data_Process()
{
    if (CAN_Manage_Object == nullptr)
    {
        return;
    }

    const FDCAN_RxHeaderTypeDef &header = CAN_Manage_Object->Rx_Header;
    if (header.IdType != FDCAN_EXTENDED_ID || header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return;
    }

    uint8_t packet_id = static_cast<uint8_t>((header.Identifier >> 8U) & 0xFFU);
    uint8_t vesc_id = static_cast<uint8_t>(header.Identifier & 0xFFU);
    if (vesc_id != VESC_ID)
    {
        return;
    }

    const uint8_t *buffer = CAN_Manage_Object->Rx_Buffer;
    Struct_Motor_VESC_Rx_Frame_State *frame_state = nullptr;
    switch (packet_id)
    {
    case (MOTOR_VESC_PACKET_STATUS):
    {
        if (header.DataLength != FDCAN_DLC_BYTES_8)
        {
            return;
        }
        Rx_Data.Now_ERPM = Motor_VESC_BE_To_Int32(&buffer[0]);
        if (Pole_Pairs == 0U || Gearbox_Rate <= 0.0f)
        {
            Rx_Data.Now_Omega = 0.0f;
        }
        else
        {
            Rx_Data.Now_Omega = static_cast<float>(Rx_Data.Now_ERPM) / (static_cast<float>(Pole_Pairs) * Gearbox_Rate) * 2.0f * MOTOR_VESC_PI / 60.0f;
        }
        Rx_Data.Motor_Current = static_cast<float>(Motor_VESC_BE_To_Int16(&buffer[4])) / 10.0f;
        Rx_Data.Duty = static_cast<float>(Motor_VESC_BE_To_Int16(&buffer[6])) / 1000.0f;
        frame_state = &Status_1_State;
        break;
    }
    case (MOTOR_VESC_PACKET_STATUS_4):
    {
        if (header.DataLength != FDCAN_DLC_BYTES_8)
        {
            return;
        }
        Rx_Data.MOS_Temperature = static_cast<float>(Motor_VESC_BE_To_Int16(&buffer[0])) / 10.0f;
        Rx_Data.Motor_Temperature = static_cast<float>(Motor_VESC_BE_To_Int16(&buffer[2])) / 10.0f;
        Rx_Data.Input_Current = static_cast<float>(Motor_VESC_BE_To_Int16(&buffer[4])) / 10.0f;
        Rx_Data.PID_Position = static_cast<float>(Motor_VESC_BE_To_Int16(&buffer[6])) / 50.0f;
        frame_state = &Status_4_State;
        break;
    }
    case (MOTOR_VESC_PACKET_STATUS_5):
    {
        if (header.DataLength != FDCAN_DLC_BYTES_8)
        {
            return;
        }
        Rx_Data.Tachometer = Motor_VESC_BE_To_Int32(&buffer[0]);
        Rx_Data.Input_Voltage = static_cast<float>(Motor_VESC_BE_To_Int16(&buffer[4])) / 10.0f;
        frame_state = &Status_5_State;
        break;
    }
    default:
    {
        return;
    }
    }

    frame_state->Last_Update_Time = HAL_GetTick();
    frame_state->Update_Count++;
    frame_state->Valid = true;
}

/**
 * @brief 判断指定状态报文是否仍在100ms有效期内
 *
 * @param Frame_State 状态报文的新鲜度信息
 * @return bool true为有效, false为超时或尚未接收
 */
bool Class_Motor_VESC::Get_Rx_Frame_Valid(const Struct_Motor_VESC_Rx_Frame_State &Frame_State) const
{
    return (Frame_State.Valid && static_cast<uint32_t>(HAL_GetTick() - Frame_State.Last_Update_Time) < MOTOR_VESC_STATUS_TIMEOUT_MS);
}

/**
 * @brief 获取STATUS报文有效状态
 */
bool Class_Motor_VESC::Get_Status_1_Valid() const
{
    return (Get_Rx_Frame_Valid(Status_1_State));
}

/**
 * @brief 获取STATUS_4报文有效状态
 */
bool Class_Motor_VESC::Get_Status_4_Valid() const
{
    return (Get_Rx_Frame_Valid(Status_4_State));
}

/**
 * @brief 获取STATUS_5报文有效状态
 */
bool Class_Motor_VESC::Get_Status_5_Valid() const
{
    return (Get_Rx_Frame_Valid(Status_5_State));
}

/**
 * @brief 按当前控制方式发送VESC控制报文, 失能时发送零电流
 */
void Class_Motor_VESC::Output()
{
    if (CAN_Manage_Object == nullptr)
    {
        return;
    }

    uint8_t tx_data[4];
    uint32_t packet_id;
    if (Motor_Enable)
    {
        switch (Control_Method)
        {
        case (Motor_VESC_Control_Method_CURRENT):
        {
            packet_id = MOTOR_VESC_PACKET_SET_CURRENT;
            Motor_VESC_Int32_To_BE(static_cast<int32_t>(roundf(Control_Current * 1000.0f)), tx_data);
            break;
        }
        case (Motor_VESC_Control_Method_CURRENT_BRAKE):
        {
            packet_id = MOTOR_VESC_PACKET_SET_CURRENT_BRAKE;
            Motor_VESC_Int32_To_BE(static_cast<int32_t>(roundf(Control_Current_Brake * 1000.0f)), tx_data);
            break;
        }
        case (Motor_VESC_Control_Method_DUTY):
        {
            packet_id = MOTOR_VESC_PACKET_SET_DUTY;
            Motor_VESC_Int32_To_BE(static_cast<int32_t>(roundf(Control_Duty * 100000.0f)), tx_data);
            break;
        }
        case (Motor_VESC_Control_Method_POSITION):
        {
            packet_id = MOTOR_VESC_PACKET_SET_POSITION;
            Motor_VESC_Int32_To_BE(static_cast<int32_t>(roundf(Control_Position * 1000000.0f)), tx_data);
            break;
        }
        case (Motor_VESC_Control_Method_CURRENT_REL):
        {
            packet_id = MOTOR_VESC_PACKET_SET_CURRENT_REL;
            Motor_VESC_Int32_To_BE(static_cast<int32_t>(roundf(Control_Current_Rel * 100000.0f)), tx_data);
            break;
        }
        case (Motor_VESC_Control_Method_RPM):
        default:
        {
            packet_id = MOTOR_VESC_PACKET_SET_RPM;
            Motor_VESC_Int32_To_BE(Control_ERPM, tx_data);
            break;
        }
        }
    }
    else
    {
        Motor_Enable = false;
        Reset_Control_Targets();
        packet_id = MOTOR_VESC_PACKET_SET_CURRENT;
        Motor_VESC_Int32_To_BE(0, tx_data);
    }

    CAN_Transmit_Extended_Data(CAN_Manage_Object->CAN_Handler, (packet_id << 8U) | VESC_ID, tx_data, 4U);
}

/**
 * @brief 清空全部VESC控制目标, 防止使能或模式切换时沿用旧指令
 */
void Class_Motor_VESC::Reset_Control_Targets()
{
    Control_Omega = 0.0f;
    Control_ERPM = 0;
    Control_Current = 0.0f;
    Control_Current_Brake = 0.0f;
    Control_Duty = 0.0f;
    Control_Position = 0.0f;
    Control_Current_Rel = 0.0f;
}

/**
 * @brief 将int32转换为大端字节流
 *
 * @param Value 待转换的有符号32位数据
 * @param Buffer 输出缓冲区, 长度至少为4字节
 */
static void Motor_VESC_Int32_To_BE(int32_t Value, uint8_t *Buffer)
{
    uint32_t raw = static_cast<uint32_t>(Value);
    Buffer[0] = static_cast<uint8_t>(raw >> 24U);
    Buffer[1] = static_cast<uint8_t>(raw >> 16U);
    Buffer[2] = static_cast<uint8_t>(raw >> 8U);
    Buffer[3] = static_cast<uint8_t>(raw);
}

/**
 * @brief 从大端字节流读取int16
 *
 * @param Buffer 输入缓冲区, 长度至少为2字节
 * @return int16_t 有符号16位数据
 */
static int16_t Motor_VESC_BE_To_Int16(const uint8_t *Buffer)
{
    uint16_t raw = (static_cast<uint16_t>(Buffer[0]) << 8U) | static_cast<uint16_t>(Buffer[1]);
    return (static_cast<int16_t>(raw));
}

/**
 * @brief 从大端字节流读取int32
 *
 * @param Buffer 输入缓冲区, 长度至少为4字节
 * @return int32_t 有符号32位数据
 */
static int32_t Motor_VESC_BE_To_Int32(const uint8_t *Buffer)
{
    uint32_t raw = (static_cast<uint32_t>(Buffer[0]) << 24U) | (static_cast<uint32_t>(Buffer[1]) << 16U) | (static_cast<uint32_t>(Buffer[2]) << 8U) | static_cast<uint32_t>(Buffer[3]);
    return (static_cast<int32_t>(raw));
}
