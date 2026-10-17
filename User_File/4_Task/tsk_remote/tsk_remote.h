/**
 * @file tsk_remote.h
 * @brief 遥控器接收和底盘指令换算
 * @version 0.1
 * @date 2026-10-05 0.1 新建文档
 */

#ifndef TSK_REMOTE_H
#define TSK_REMOTE_H

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Protocol/pro_sbus/pro_sbus.h"
#include "2_Device/NRF24L01/dvc_nrf24l01.h"

/* Exported macros -----------------------------------------------------------*/

#define REMOTE_MODE_MANUAL 2

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 遥控器底盘控制指令
 */
struct Struct_Remote_Chassis_Command
{
    float Speed_X;
    float Speed_Y;
    float Speed_W;
    uint8_t Online;
    uint8_t Mode;
    uint8_t Motor_Enable;
};

/**
 * @brief 遥控通道与速度换算参数, 默认值与原工程一致
 */
struct Struct_Remote_Parameter
{
    uint16_t Channel_Min = 200U;              // 通道下限
    uint16_t Channel_Middle = 1000U;          // 通道中值
    uint16_t Channel_Max = 1800U;             // 通道上限
    float Translation_Speed_Max = 10.55575f;  // 平移速度上限, m/s
    float Angular_Speed_Max = 23.45722389f;   // 旋转速度上限, rad/s
    float Speed_Dead_Zone = 0.05f;            // 平移死区, m/s
    float Angular_Speed_Dead_Zone = 0.12f;    // 旋转死区, rad/s
};

/**
 * @brief 10ms遥控接收与指令换算, 复用全局NRF24L01和SBUS_Receive
 * @note Remote_Chassis_Command保留为各任务读取的指令快照
 */
class Class_Remote_Control
{
public:
    void Init();

    void TIM_10ms_PeriodElapsedCallback();

private:
    float Channel_To_Speed(uint16_t Channel);
    float Channel_To_Angular_Speed(uint16_t Channel);
    void Update_Chassis_Command();

    const Struct_Remote_Parameter Parameter{};
};

/* Exported variables --------------------------------------------------------*/

extern Class_NRF24L01 NRF24L01;
extern Struct_Remote_Chassis_Command Remote_Chassis_Command;

/* Exported function declarations --------------------------------------------*/

void Remote_Init();

void Remote_10ms_PeriodElapsedCallback();

#endif
