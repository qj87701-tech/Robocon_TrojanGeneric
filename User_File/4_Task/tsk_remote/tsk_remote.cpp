/**
 * @file tsk_remote.cpp
 * @brief 遥控器接收和底盘指令换算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_remote.h"

#include <math.h>

/* Private variables ---------------------------------------------------------*/

static Class_Remote_Control Remote_Control;

/* Exported variables --------------------------------------------------------*/

Class_NRF24L01 NRF24L01;
Struct_Remote_Chassis_Command Remote_Chassis_Command;

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 将遥控通道值换算为底盘速度
 *
 * @param Channel 遥控通道值, 200~1800
 * @return float 底盘目标平移速度, m/s
 */
float Class_Remote_Control::Channel_To_Speed(uint16_t Channel)
{
    if (Channel < Parameter.Channel_Min)
    {
        Channel = Parameter.Channel_Min;
    }
    else if (Channel > Parameter.Channel_Max)
    {
        Channel = Parameter.Channel_Max;
    }

    float speed = (static_cast<float>(Channel) - Parameter.Channel_Middle) * Parameter.Translation_Speed_Max / (Parameter.Channel_Max - Parameter.Channel_Middle);
    if (fabsf(speed) < Parameter.Speed_Dead_Zone)
    {
        speed = 0.0f;
    }

    return speed;
}

/**
 * @brief 将遥控旋转通道值换算为底盘角速度
 *
 * @param Channel 遥控通道值, 200~1800
 * @return float 底盘目标角速度, rad/s
 */
float Class_Remote_Control::Channel_To_Angular_Speed(uint16_t Channel)
{
    if (Channel < Parameter.Channel_Min)
    {
        Channel = Parameter.Channel_Min;
    }
    else if (Channel > Parameter.Channel_Max)
    {
        Channel = Parameter.Channel_Max;
    }

    float speed = (static_cast<float>(Channel) - Parameter.Channel_Middle) * Parameter.Angular_Speed_Max / (Parameter.Channel_Max - Parameter.Channel_Middle);
    if (fabsf(speed) < Parameter.Angular_Speed_Dead_Zone)
    {
        speed = 0.0f;
    }

    return speed;
}

/**
 * @brief 根据遥控在线、模式和使能状态更新底盘控制指令
 */
void Class_Remote_Control::Update_Chassis_Command()
{
    Remote_Chassis_Command.Online = SBUS_Receive.SBUS_Data.Online;
    Remote_Chassis_Command.Mode = SBUS_Receive.SBUS_Data.Mode;
    Remote_Chassis_Command.Motor_Enable = SBUS_Receive.SBUS_Data.Motor_Enable;

    if (Remote_Chassis_Command.Online == 0 || Remote_Chassis_Command.Mode != REMOTE_MODE_MANUAL || Remote_Chassis_Command.Motor_Enable == 0)
    {
        Remote_Chassis_Command.Speed_X = 0.0f;
        Remote_Chassis_Command.Speed_Y = 0.0f;
        Remote_Chassis_Command.Speed_W = 0.0f;
        return;
    }

    Remote_Chassis_Command.Speed_Y = Channel_To_Speed(SBUS_Receive.SBUS_Data.Channel[0]);
    Remote_Chassis_Command.Speed_X = Channel_To_Speed(SBUS_Receive.SBUS_Data.Channel[1]);
    Remote_Chassis_Command.Speed_W = Channel_To_Angular_Speed(SBUS_Receive.SBUS_Data.Channel[3]);
}

/**
 * @brief 初始化遥控器接收
 */
void Class_Remote_Control::Init()
{
    SBUS_Receive.Init();
    NRF24L01.NRF24L01_Init();
    Update_Chassis_Command();
}

/**
 * @brief 每10ms接收NRF24L01数据、解析SBUS并更新底盘指令
 */
void Class_Remote_Control::TIM_10ms_PeriodElapsedCallback()
{
    SBUS_Receive.TIM_10ms_PeriodElapsedCallback();

    if (NRF24L01.NRF24L01_Receive() == 1)
    {
        SBUS_Receive.Data_Process(NRF24L01.NRF24L01_data.NRF24L01_RxPacket);
    }

    Update_Chassis_Command();
}

/* 兼容原有任务入口 ----------------------------------------------------------*/

void Remote_Init()
{
    Remote_Control.Init();
}

void Remote_10ms_PeriodElapsedCallback()
{
    Remote_Control.TIM_10ms_PeriodElapsedCallback();
}
