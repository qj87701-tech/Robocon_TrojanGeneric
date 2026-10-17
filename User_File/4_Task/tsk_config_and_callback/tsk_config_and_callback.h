/**
 * @file tsk_config_and_callback.h
 * @brief 存放用户定义的回调函数以及任务
 * @version 0.1
 * @date 2026-09-30 0.1 新建文档
 */

#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/

#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Protocol/pro_upper/pro_upper.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "2_Device/BSP/Key/bsp_key.h"
#include "2_Device/IMU/HI06/dvc_imu_hi06.h"
#include "4_Task/tsk_motor_init/motor_init.h"
#include "4_Task/tsk_4swerve_chassis/tsk_chassis.h"
#include "4_Task/tsk_debug/tsk_debug.h"
#include "4_Task/tsk_remote/tsk_remote.h"
#include "fdcan.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裸机任务与按键调速参数, 不改变现有调度顺序
 */
struct Struct_Task_Parameter
{
    float Control_Omega_Step = 1.0f;    // J8006按键调速步进, rad/s
    float Control_Omega_Min = -20.0f;   // J8006调速下限, rad/s
    float Control_Omega_Max = 20.0f;    // J8006调速上限, rad/s
    uint8_t Remote_Mode_Debug = 1U;     // 遥控按键调试模式
    uint32_t Task_10ms_Period = 10U;    // 控制与VOFA调度周期, ms
    uint32_t Task_100ms_Period = 100U;  // 在线检查与诊断周期, ms
    uint32_t Task_500ms_Period = 500U;  // 预留低频调度周期, ms
    uint32_t Task_1s_Period = 1000U;    // 预留低频调度周期, ms
};

/**
 * @brief 裸机前台任务调度, 硬件接收回调仍使用普通函数入口
 * @note 全工程仅使用一个调度对象; 不引入RTOS和补发追赶
 */
class Class_Task_Control
{
public:
    void Init();

    void Loop();

private:
    void Task10ms_Callback();
    void Task100ms_Callback();
    void Task500ms_Callback();
    void Task1s_Callback();

    const Struct_Task_Parameter Parameter{};

    Class_Key Key1;                            // 板载按键1
    Class_Key Key2;                            // 板载按键2
    uint32_t Task_10ms_Previous_Tick = 0U;     // 上次调度时刻, ms
    uint32_t Task_100ms_Previous_Tick = 0U;
    uint32_t Task_500ms_Previous_Tick = 0U;
    uint32_t Task_1s_Previous_Tick = 0U;
    bool Dual_Key_Latched = false;             // 双按键清零后等待全部松开
    bool Remote_Key1_Previous_Status = false;  // 遥控按键边沿检测
    bool Remote_Key2_Previous_Status = false;
};

/* Exported variables --------------------------------------------------------*/

// HAL接收与驱动初始化使用的兼容标志
extern bool init_finished;

extern "C" {
#endif

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化用户设备、通信总线和裸机任务时间基准
 */
void Task_Init();

/**
 * @brief 裸机前台循环, 按时间基准调度各周期回调
 */
void Task_Loop();

#ifdef __cplusplus
}
#endif

#endif
