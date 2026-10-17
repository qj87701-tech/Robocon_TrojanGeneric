/**
 * @file tsk_config_and_callback.cpp
 * @brief 存放用户定义的回调函数以及任务
 * @version 0.1
 * @date 2026-09-30 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

/* Private variables ---------------------------------------------------------*/

static Class_Task_Control Task_Control;

// 全局初始化完成标志位, 保留HAL和驱动兼容入口
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

static void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

static void CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

static void UART4_Callback(uint8_t *Buffer, uint16_t Length);

static void UART7_Callback(uint8_t *Buffer, uint16_t Length);

static void UART9_Vofa_Callback(uint8_t *Buffer, uint16_t Length);

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1接收回调函数
 *
 * @param Header CAN接收报文头
 * @param Buffer CAN接收数据
 */
static void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    if (Header.IdType != FDCAN_STANDARD_ID || Header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return;
    }

    switch (Header.Identifier)
    {
    case (DM_J8006_MASTER_ID):
    {
        DM_J8006.CAN_RxCpltCallback();
        break;
    }
    case (DM_CHASSIS_LF_MASTER_ID):
    {
        DM_Chassis_LF.CAN_RxCpltCallback();
        break;
    }
    case (DM_CHASSIS_RF_MASTER_ID):
    {
        DM_Chassis_RF.CAN_RxCpltCallback();
        break;
    }
    case (DM_CHASSIS_LB_MASTER_ID):
    {
        DM_Chassis_LB.CAN_RxCpltCallback();
        break;
    }
    case (DM_CHASSIS_RB_MASTER_ID):
    {
        DM_Chassis_RB.CAN_RxCpltCallback();
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief CAN3接收回调函数
 *
 * @param Header CAN接收报文头
 * @param Buffer CAN接收数据
 */
static void CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    if (Header.IdType != FDCAN_EXTENDED_ID || Header.RxFrameType != FDCAN_DATA_FRAME)
    {
        return;
    }

    switch (Header.Identifier & 0xffU)
    {
    case (VESC_CHASSIS_LF_ID):
    {
        VESC_Chassis_LF.CAN_RxCpltCallback();
        break;
    }
    case (VESC_CHASSIS_RF_ID):
    {
        VESC_Chassis_RF.CAN_RxCpltCallback();
        break;
    }
    case (VESC_CHASSIS_LB_ID):
    {
        VESC_Chassis_LB.CAN_RxCpltCallback();
        break;
    }
    case (VESC_CHASSIS_RB_ID):
    {
        VESC_Chassis_RB.CAN_RxCpltCallback();
        break;
    }
    default:
    {
        break;
    }
    }
}

/**
 * @brief UART4接收回调函数
 *
 * @param Buffer 接收数据指针
 * @param Length 接收数据长度
 */
static void UART4_Callback(uint8_t *Buffer, uint16_t Length)
{
    HI06.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief UART7接收上位机字节, 协议对象只缓存, 由10ms前台完成解析
 * @param Buffer 本次DMA接收区
 * @param Length 本次接收字节数, 不代表32字节帧边界
 */
static void UART7_Callback(uint8_t *Buffer, uint16_t Length)
{
    Upper.UART_RxCpltCallback(Buffer, Length);
}

/**
 * @brief USART9接收VOFA调参指令
 *
 * @param Buffer 接收数据指针
 * @param Length 接收数据长度, Byte
 */
static void UART9_Vofa_Callback(uint8_t *Buffer, uint16_t Length)
{
    Debug_UART9_RxCpltCallback(Buffer, Length);
}

/**
 * @brief 每10ms调用一次
 */
void Class_Task_Control::Task10ms_Callback()
{
    HI06.TIM_10ms_PeriodElapsedCallback();
    Upper.TIM_10ms_PeriodElapsedCallback();

    Remote_10ms_PeriodElapsedCallback();

    // USART9调参命令只在前台应用, 并先于本周期舵轮控制生效
    Debug_10ms_PeriodElapsedCallback();

    Key1.TIM_10ms_PeriodElapsedCallback();
    Key2.TIM_10ms_PeriodElapsedCallback();

    Enum_BSP_Key_Status key1_status = Key1.Get_Key_Status();
    Enum_BSP_Key_Status key2_status = Key2.Get_Key_Status();

    bool remote_key1_status = SBUS_Receive.SBUS_Data.Function[0] != 0;
    bool remote_key2_status = SBUS_Receive.SBUS_Data.Function[1] != 0;
    bool remote_key1_trigger = remote_key1_status && !Remote_Key1_Previous_Status;
    bool remote_key2_trigger = remote_key2_status && !Remote_Key2_Previous_Status;

    Remote_Key1_Previous_Status = remote_key1_status;
    Remote_Key2_Previous_Status = remote_key2_status;

    // 遥控器在线并打开电机使能时使能电机
    bool motor_enable = SBUS_Receive.SBUS_Data.Online != 0 && SBUS_Receive.SBUS_Data.Motor_Enable != 0;
    if (motor_enable != DM_J8006.Get_Motor_Enable())
    {
        if (motor_enable)
        {
            DM_J8006.CAN_Send_Enter();
        }
        else
        {
            DM_J8006.Set_Control_Omega(0.0f);
            DM_J8006.CAN_Send_Exit();
            Dual_Key_Latched = false;
        }
    }

    // 电机失能时不响应调速按键
    if (!motor_enable)
    {
        DM_J8006.TIM_Send_PeriodElapsedCallback();
        // Chassis_10ms_PeriodElapsedCallback();
        Chassis_Swerve_10ms_PeriodElapsedCallback();
        Chassis_VESC_10ms_PeriodElapsedCallback();
        Debug_Vofa_10ms_PeriodElapsedCallback();
        return;
    }

    bool remote_key_enable = SBUS_Receive.SBUS_Data.Mode == Parameter.Remote_Mode_Debug;
    bool key1_down = key1_status == BSP_Key_Status_TRIG_FREE_PRESSED || key1_status == BSP_Key_Status_PRESSED || (remote_key_enable && remote_key1_status);
    bool key2_down = key2_status == BSP_Key_Status_TRIG_FREE_PRESSED || key2_status == BSP_Key_Status_PRESSED || (remote_key_enable && remote_key2_status);
    bool key1_trigger = key1_status == BSP_Key_Status_TRIG_FREE_PRESSED || (remote_key_enable && remote_key1_trigger);
    bool key2_trigger = key2_status == BSP_Key_Status_TRIG_FREE_PRESSED || (remote_key_enable && remote_key2_trigger);

    // 两个按键同时按下时将目标速度清零
    if (key1_down && key2_down)
    {
        DM_J8006.Set_Control_Omega(0.0f);
        Dual_Key_Latched = true;
    }
    // 双按键清零后, 等待两个按键全部松开
    else if (Dual_Key_Latched)
    {
        if (!key1_down && !key2_down)
        {
            Dual_Key_Latched = false;
        }
    }
    // KEY1每按下一次, 目标速度增加1rad/s
    else if (key1_trigger)
    {
        float control_omega = DM_J8006.Get_Control_Omega() + Parameter.Control_Omega_Step;
        if (control_omega > Parameter.Control_Omega_Max)
        {
            control_omega = Parameter.Control_Omega_Max;
        }
        DM_J8006.Set_Control_Omega(control_omega);
    }
    // KEY2每按下一次, 目标速度减少1rad/s
    else if (key2_trigger)
    {
        float control_omega = DM_J8006.Get_Control_Omega() - Parameter.Control_Omega_Step;
        if (control_omega < Parameter.Control_Omega_Min)
        {
            control_omega = Parameter.Control_Omega_Min;
        }
        DM_J8006.Set_Control_Omega(control_omega);
    }

    // 发送达妙电机控制报文
    DM_J8006.TIM_Send_PeriodElapsedCallback();
    // Chassis_10ms_PeriodElapsedCallback();
    Chassis_Swerve_10ms_PeriodElapsedCallback();
    Chassis_VESC_10ms_PeriodElapsedCallback();
    Debug_Vofa_10ms_PeriodElapsedCallback();
}

/**
 * @brief 每100ms调用一次
 */
void Class_Task_Control::Task100ms_Callback()
{
    DM_J8006.TIM_100ms_Alive_PeriodElapsedCallback();
    Chassis_100ms_PeriodElapsedCallback();
    Debug_UART10_Print_Swerve_Data();
}

/**
 * @brief 每500ms调用一次
 */
void Class_Task_Control::Task500ms_Callback()
{
    // Debug_UART10_Print_IMU_Data();
    // 上位机仅接收和诊断; 按需开启一种打印, 与其他USART10输出错开
    // Debug_UART10_Print_Upper_Data();
    // Debug_UART10_Print_Upper_Status_Data();
}

/**
 * @brief 每1s调用一次
 */
void Class_Task_Control::Task1s_Callback()
{
    // Debug_UART10_Print_Remote_Data();
    // Debug_UART10_Print_Motor_Data();
}

/**
 * @brief 初始化任务
 */
void Class_Task_Control::Init()
{
    init_finished = false;
    Dual_Key_Latched = false;
    Remote_Key1_Previous_Status = false;
    Remote_Key2_Previous_Status = false;

    // 时间戳
    SYS_Timestamp.Init(&htim5);

    // HI06和UART4空闲中断DMA接收
    HI06.Init();
    UART_Init(&huart4, UART4_Callback);

    // UART7上位机原始数据接收, 初始化后默认不校验CRC, 不参与底盘控制
    Upper.Init();
    UART_Init(&huart7, UART7_Callback);
    // 其余UART空闲中断DMA接收
    UART_Init(&huart9, UART9_Vofa_Callback);
    UART_Init(&huart10, nullptr);

    // USART9 VOFA调试任务, 默认观察左前轮DM ID 1和VESC ID 1
    Debug_Init();

    // 遥控数据和NRF24L01
    Remote_Init();

    // 电机初始化
    Motor_Init();

    // 底盘初始化
    Chassis_Init();

    // 板载按键, 松开为高电平
    Key1.Init(Key1_GPIO_Port, Key1_Pin, GPIO_PIN_SET);
    Key2.Init(Key2_GPIO_Port, Key2_Pin, GPIO_PIN_SET);

    // 电机CAN
    CAN_Init(&hfdcan1, CAN1_Callback);
    CAN_Extended_Init(&hfdcan3, CAN3_Callback);

    // TIM5用于时间戳溢出计时
    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    uint32_t now_tick = HAL_GetTick();
    Task_10ms_Previous_Tick = now_tick;
    Task_100ms_Previous_Tick = now_tick;
    Task_500ms_Previous_Tick = now_tick;
    Task_1s_Previous_Tick = now_tick;

    // 标记初始化完成
    init_finished = true;

    // 默认失能达妙电机
    DM_J8006.CAN_Send_Exit();
    DM_Chassis_LF.CAN_Send_Exit();
    DM_Chassis_RF.CAN_Send_Exit();
    DM_Chassis_LB.CAN_Send_Exit();
    DM_Chassis_RB.CAN_Send_Exit();
}

/**
 * @brief 前台循环任务
 */
void Class_Task_Control::Loop()
{
    uint32_t now_tick = HAL_GetTick();

    if ((uint32_t)(now_tick - Task_10ms_Previous_Tick) >= Parameter.Task_10ms_Period)
    {
        Task_10ms_Previous_Tick = now_tick;
        Task10ms_Callback();
    }

    if ((uint32_t)(now_tick - Task_100ms_Previous_Tick) >= Parameter.Task_100ms_Period)
    {
        Task_100ms_Previous_Tick = now_tick;
        Task100ms_Callback();
    }

    if ((uint32_t)(now_tick - Task_500ms_Previous_Tick) >= Parameter.Task_500ms_Period)
    {
        Task_500ms_Previous_Tick = now_tick;
        Task500ms_Callback();
    }

    if ((uint32_t)(now_tick - Task_1s_Previous_Tick) >= Parameter.Task_1s_Period)
    {
        Task_1s_Previous_Tick = now_tick;
        Task1s_Callback();
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim 定时器句柄
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
    }
}

/* 兼容Core/main.c中的C入口 ---------------------------------------------------*/

void Task_Init()
{
    Task_Control.Init();
}

void Task_Loop()
{
    Task_Control.Loop();
}
