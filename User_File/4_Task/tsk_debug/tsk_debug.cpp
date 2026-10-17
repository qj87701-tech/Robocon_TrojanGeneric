/**
 * @file tsk_debug.cpp
 * @brief 调试数据输出任务
 * @version 0.1
 * @date 2026-10-11 0.1 新建文档
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_debug.h"

#include <math.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/

static Class_Debug_Control Debug_Control;

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USART9 VOFA调试任务
 * @note 默认观察左前轮DM ID 1和VESC ID 1
 */
void Class_Debug_Control::Init()
{
    Debug_Swerve_Wheel = Swerve_Wheel_LF;
    Debug_Pending_Command_Index = -1;
    Debug_Pending_Command_Value = 0.0f;
    Debug_Pending_Command_Update_Count = 0U;
    Debug_Applied_Command_Update_Count = 0U;

    Debug_Vofa_UART9.Init(&huart9, DEBUG_VOFA_COMMAND_NUMBER, Debug_Vofa_Command_Name);
    Debug_Vofa_UART9.Set_Data(DEBUG_VOFA_DATA_NUMBER,
                             static_cast<const void *>(&Debug_Vofa_Data[0]),
                             static_cast<const void *>(&Debug_Vofa_Data[1]),
                             static_cast<const void *>(&Debug_Vofa_Data[2]),
                             static_cast<const void *>(&Debug_Vofa_Data[3]),
                             static_cast<const void *>(&Debug_Vofa_Data[4]),
                             static_cast<const void *>(&Debug_Vofa_Data[5]),
                             static_cast<const void *>(&Debug_Vofa_Data[6]),
                             static_cast<const void *>(&Debug_Vofa_Data[7]),
                             static_cast<const void *>(&Debug_Vofa_Data[8]),
                             static_cast<const void *>(&Debug_Vofa_Data[9]),
                             static_cast<const void *>(&Debug_Vofa_Data[10]),
                             static_cast<const void *>(&Debug_Vofa_Data[11]),
                             static_cast<const void *>(&Debug_Vofa_Data[12]),
                             static_cast<const void *>(&Debug_Vofa_Data[13]),
                             static_cast<const void *>(&Debug_Vofa_Data[14]),
                             static_cast<const void *>(&Debug_Vofa_Data[15]),
                             static_cast<const void *>(&Debug_Vofa_Data[16]),
                             static_cast<const void *>(&Debug_Vofa_Data[17]),
                             static_cast<const void *>(&Debug_Vofa_Data[18]),
                             static_cast<const void *>(&Debug_Vofa_Data[19]),
                             static_cast<const void *>(&Debug_Vofa_Data[20]),
                             static_cast<const void *>(&Debug_Vofa_Data[21]),
                             static_cast<const void *>(&Debug_Vofa_Data[22]),
                             static_cast<const void *>(&Debug_Vofa_Data[23]));
}

/**
 * @brief USART9接收VOFA调参指令
 *
 * @param Buffer 接收数据指针
 * @param Length 接收数据长度, Byte
 */
void Class_Debug_Control::UART9_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    if (!Command_Is_Complete(Buffer, Length))
    {
        return;
    }

    Debug_Vofa_UART9.UART_RxCpltCallback(Buffer, Length);
    int32_t variable_index = Debug_Vofa_UART9.Get_Variable_Index();
    if (variable_index < 0 || variable_index >= static_cast<int32_t>(DEBUG_VOFA_COMMAND_NUMBER))
    {
        return;
    }

    Debug_Pending_Command_Index = variable_index;
    Debug_Pending_Command_Value = Debug_Vofa_UART9.Get_Variable_Value();
    Debug_Pending_Command_Update_Count++;
}

/**
 * @brief 检查USART9调参指令是否包含完整变量名、数值和帧尾
 */
bool Class_Debug_Control::Command_Is_Complete(const uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == nullptr || Length < 4U || Buffer[Length - 1U] != '#')
    {
        return (false);
    }

    uint16_t equal_index = 0U;
    while (equal_index < Length && Buffer[equal_index] != '=')
    {
        equal_index++;
    }
    if (equal_index == 0U || equal_index + 1U >= Length - 1U)
    {
        return (false);
    }

    uint16_t value_index = equal_index + 1U;
    if (Buffer[value_index] == '-')
    {
        value_index++;
    }

    bool has_digit = false;
    bool has_decimal_point = false;
    for (uint16_t i = value_index; i < Length - 1U; i++)
    {
        if (Buffer[i] >= '0' && Buffer[i] <= '9')
        {
            has_digit = true;
            continue;
        }
        if (Buffer[i] == '.' && !has_decimal_point)
        {
            has_decimal_point = true;
            continue;
        }
        return (false);
    }
    return (has_digit);
}

/**
 * @brief 每10ms在前台应用一条USART9暂存的MIT调参指令
 * @note UART接收回调只暂存数据, 不在中断上下文修改电机控制量
 */
void Class_Debug_Control::TIM_10ms_PeriodElapsedCallback()
{
    uint32_t update_count_before;
    uint32_t update_count_after;
    int32_t variable_index;
    float variable_value;
    do
    {
        update_count_before = Debug_Pending_Command_Update_Count;
        if (update_count_before == Debug_Applied_Command_Update_Count)
        {
            return;
        }
        variable_index = Debug_Pending_Command_Index;
        variable_value = Debug_Pending_Command_Value;
        update_count_after = Debug_Pending_Command_Update_Count;
    } while (update_count_before != update_count_after);

    Debug_Applied_Command_Update_Count = update_count_after;
    switch (variable_index)
    {
    case (Debug_Vofa_Command_DM_K_P): Chassis_Swerve_Set_MIT_K_P(Debug_Swerve_Wheel, variable_value); break;
    case (Debug_Vofa_Command_DM_K_D): Chassis_Swerve_Set_MIT_K_D(Debug_Swerve_Wheel, variable_value); break;
    case (Debug_Vofa_Command_DM_Torque_Feedforward): Chassis_Swerve_Set_MIT_Feedforward_Torque(Debug_Swerve_Wheel, variable_value); break;
    default: break;
    }
}

/**
 * @brief 选择VOFA观察的舵轮轮组
 *
 * @param Wheel 舵轮轮组编号
 */
void Class_Debug_Control::Set_Swerve_Wheel(Enum_Swerve_Wheel Wheel)
{
    if (Wheel >= Swerve_Wheel_Number)
    {
        return;
    }

    Debug_Swerve_Wheel = Wheel;
}

/**
 * @brief 每10ms发送一次当前轮组的固定24通道快照, 发送频率100Hz
 * @note USART9使用VOFA JustFloat协议, I0~I23含义见Debug_Update_Vofa_Data右侧注释
 * @note 无效连续反馈使用NaN, 避免旧数据被误认为实时数据
 */
void Class_Debug_Control::Vofa_TIM_10ms_PeriodElapsedCallback()
{
    Update_Vofa_Data();
    Debug_Vofa_UART9.TIM_1ms_Write_PeriodElapsedCallback();
}

/**
 * @brief 更新当前轮组的24个VOFA通道快照
 * @note I0~I7为VESC速度和电气量; I8~I18为DM6220 MIT调参量; I19~I23为状态及回零诊断
 * @note DM位置与角速度使用rad、rad/s; VESC转速为机械rpm, 不是ERPM
 */
void Class_Debug_Control::Update_Vofa_Data()
{
    Class_Motor_DM_Normal *dm = Debug_DM_Motor[Debug_Swerve_Wheel];                                      // 当前观察轮组的DM6220
    Class_Motor_VESC *vesc = Debug_VESC_Motor[Debug_Swerve_Wheel];                                       // 当前观察轮组的VESC
    const Struct_Chassis_Swerve_Diagnostic &diagnostic = Chassis_Swerve_Diagnostic[Debug_Swerve_Wheel];  // 当前轮组舵向诊断

    bool dm_feedback_valid = (dm->Get_Status() == Motor_DM_Status_ENABLE);                                 // DM软件在线判断, 无效连续反馈填NaN
    Debug_Vofa_Data[0]   = vesc->Get_Control_Omega() * DEBUG_RAD_S_TO_RPM;                                 // I0  VESC目标机械转速(rpm)
    Debug_Vofa_Data[1]   = vesc->Get_Status_1_Valid() ? vesc->Get_Now_Omega() * DEBUG_RAD_S_TO_RPM : NAN;  // I1  VESC实际机械转速(rpm)
    Debug_Vofa_Data[2]   = vesc->Get_Status_1_Valid() ? Debug_Vofa_Data[0] - Debug_Vofa_Data[1] : NAN;     // I2  VESC速度误差: 目标-实际(rpm)
    Debug_Vofa_Data[3]   = vesc->Get_Status_1_Valid() ? vesc->Get_Motor_Current() : NAN;                   // I3  VESC电机侧电流(A), 不是直接读取FOC iq
    Debug_Vofa_Data[4]   = vesc->Get_Status_1_Valid() ? vesc->Get_Duty() * 100.0f : NAN;                   // I4  VESC占空比(%)
    Debug_Vofa_Data[5]   = vesc->Get_Status_4_Valid() ? vesc->Get_Input_Current() : NAN;                   // I5  VESC电源输入电流(A)
    Debug_Vofa_Data[6]   = vesc->Get_Status_5_Valid() ? vesc->Get_Input_Voltage() : NAN;                   // I6  VESC母线输入电压(V)
    Debug_Vofa_Data[7]   = vesc->Get_Status_4_Valid() ? vesc->Get_MOS_Temperature() : NAN;                 // I7  VESC MOS温度(℃)

    Debug_Vofa_Data[8]   = dm->Get_Control_Angle();                                                              // I8  DM6220目标位置(rad)
    Debug_Vofa_Data[9]   = dm_feedback_valid ? dm->Get_Now_Total_Angle() : NAN;                                  // I9  DM6220实际连续位置(rad)
    Debug_Vofa_Data[10]  = dm_feedback_valid ? Debug_Vofa_Data[8] - Debug_Vofa_Data[9] : NAN;                    // I10 DM6220位置误差: 目标-实际(rad)
    Debug_Vofa_Data[11]  = dm->Get_Control_Omega();                                                              // I11 DM6220 MIT目标角速度(rad/s)
    Debug_Vofa_Data[12]  = dm_feedback_valid ? dm->Get_Now_Omega() : NAN;                                        // I12 DM6220实际角速度(rad/s)
    Debug_Vofa_Data[13]  = dm->Get_Control_Torque();                                                             // I13 DM6220 MIT前馈转矩(N·m)
    Debug_Vofa_Data[14]  = dm_feedback_valid ? dm->Get_Now_Torque() : NAN;                                       // I14 DM6220反馈转矩(N·m)
    Debug_Vofa_Data[15]  = dm->Get_K_P();                                                                        // I15 DM6220 MIT位置增益Kp
    Debug_Vofa_Data[16]  = dm->Get_K_D();                                                                        // I16 DM6220 MIT速度阻尼增益Kd
    Debug_Vofa_Data[17]  = dm_feedback_valid ? dm->Get_Now_MOS_Temperature() - DEBUG_CELSIUS_TO_KELVIN : NAN;    // I17 DM6220 MOS温度(℃)
    Debug_Vofa_Data[18]  = dm_feedback_valid ? dm->Get_Now_Rotor_Temperature() - DEBUG_CELSIUS_TO_KELVIN : NAN;  // I18 DM6220电机温度(℃)

    Debug_Vofa_Data[19]  = static_cast<float>(dm->Get_Control_Status());              // I19 DM6220驱动器反馈状态: 0失能/1使能/8~14故障
    Debug_Vofa_Data[20]  = static_cast<float>(dm_feedback_valid);                     // I20 DM6220软件在线(0/1), 不等同于驱动器使能
    Debug_Vofa_Data[21]  = static_cast<float>(diagnostic.Steer_Target_Enable);        // I21 DM6220舵向目标使能(0/1)
    Debug_Vofa_Data[22]  = static_cast<float>(diagnostic.Steer_Homing_Status);        // I22 回零状态: 0失能/1等反馈/2回零/3就绪/4超时
    Debug_Vofa_Data[23]  = static_cast<float>(diagnostic.Steer_Homing_Elapsed_Time);  // I23 DM6220回零耗时(ms)
}

/**
 * @brief USART10选择需要打印的HI06数据
 * @note 默认打印姿态; 加速度、角速度和状态可单独选择, 不宜同周期连续调用多种打印
 */
void Class_Debug_Control::UART10_Print_IMU_Data()
{
    UART10_Print_IMU_Attitude_Data();
    // UART10_Print_IMU_Acceleration_Data();
    // UART10_Print_IMU_Gyroscope_Data();
    // UART10_Print_IMU_Status_Data();
}

/**
 * @brief USART10打印上位机原始数据, 不进行缩放或物理单位换算
 * @note online: 在线0/1; valid: 数据有效0/1; raw_xyw: 原始X/Y/W有符号整数, 未除以1000
 * @note field: 帧下标8~19的12字节; extend: 下标20~28的9字节, 均按两位十六进制输出
 * @note DMA忙时丢弃本次打印, 不阻塞; 应与其他USART10输出错开调用
 */
void Class_Debug_Control::UART10_Print_Upper_Data()
{
    char (&tx_data)[2][DEBUG_UPPER_UART10_BUFFER_SIZE] = UART10_Upper_Data_Tx_Data;                                                        // 双缓冲, 不覆盖DMA正在发送的数据
    uint8_t &tx_index = UART10_Upper_Data_Tx_Index;                                                                                        // 当前用于准备打印数据的缓冲区
    const Struct_Upper_Data &data = Upper.Get_Data();                                                                                      // 最近完整上位机帧及其有效状态
    char *buffer = tx_data[tx_index];                                                                                                      // 本次打印文本的写入位置
    int tx_length = snprintf(buffer, sizeof(tx_data[tx_index]), "UPPER online=%u valid=%u raw_xyw=%d,%d,%d field=",                        // 在线/有效标志、原始X/Y/W及数据区标题
                             static_cast<unsigned int>(Upper.Get_Status() == Upper_Status_ONLINE), static_cast<unsigned int>(data.Valid),  // online: 在线0/1; valid: 最近数据有效0/1
                             static_cast<int>(data.Speed_X), static_cast<int>(data.Speed_Y), static_cast<int>(data.Speed_W));              // raw_xyw: 原始X/Y/W整数, 未除以1000
    if (tx_length <= 0 || tx_length >= static_cast<int>(sizeof(tx_data[tx_index]))) { return; }                                            // 格式化失败或截断时不发送

    for (uint16_t i = 0U; i < UPPER_DATA_FIELD_NUMBER + UPPER_EXTEND_DATA_NUMBER; i++)                                               // 依次打印12个数据字节和9个扩展字节
    {
        const uint8_t value = i < UPPER_DATA_FIELD_NUMBER ? data.Data_Field[i] : data.Extend_Data[i - UPPER_DATA_FIELD_NUMBER];      // field: 帧8~19字节; extend: 帧20~28字节
        const char *format = i == 0U ? "%02X" : (i == UPPER_DATA_FIELD_NUMBER ? " extend=%02X" : " %02X");                           // 字节以两位十六进制显示, 扩展区单独标识
        const int remaining = static_cast<int>(sizeof(tx_data[tx_index])) - tx_length;                                               // 文本缓冲区剩余容量
        const int written = snprintf(buffer + tx_length, static_cast<size_t>(remaining), format, static_cast<unsigned int>(value));  // 追加当前原始字节
        if (written <= 0 || written >= remaining) { return; }                                                                        // 防止输出截断或越界
        tx_length += written;                                                                                                        // 累计本次打印文本长度
    }
    const int remaining = static_cast<int>(sizeof(tx_data[tx_index])) - tx_length;                                                   // 为行尾预留容量
    const int written = snprintf(buffer + tx_length, static_cast<size_t>(remaining), "\r\n");                                        // 每条数据打印完后换行
    if (written <= 0 || written >= remaining) { return; }                                                                            // 容量不足时不发送残缺行
    tx_length += written;                                                                                                            // 发送长度包含换行符
    if (UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(buffer), static_cast<uint16_t>(tx_length)) == HAL_OK)               // USART10非阻塞DMA发送, 忙时跳过
    {
        tx_index ^= 1U;                                                                                                              // 发送成功后切换缓冲区, 避免覆盖在途数据
    }
}

/**
 * @brief USART10打印上位机接收状态, CRC关闭时不将帧格式有效解释为校验通过
 * @note online/valid/crc_enable: 在线、数据有效、CRC启用标志(0/1); age_ms: 数据年龄(ms), 无完整帧为NA
 * @note rx/frame: 接收字节数/成功帧数; invalid/crc_err/overflow: 格式错误/CRC错误/缓存溢出次数
 */
void Class_Debug_Control::UART10_Print_Upper_Status_Data()
{
    char (&tx_data)[2][DEBUG_UPPER_UART10_BUFFER_SIZE] = UART10_Upper_Status_Data_Tx_Data;                                                                                                                     // 状态打印独立双缓冲
    uint8_t &tx_index = UART10_Upper_Status_Data_Tx_Index;                                                                                                                                                     // 当前准备文本的缓冲区下标
    char age[16];                                                                                                                                                                                              // age_ms: 最近完整帧距今的毫秒数
    const Struct_Upper_Data &data = Upper.Get_Data();                                                                                                                                                          // 最近数据及接收统计
    if (data.Frame_Count == 0U)                                                                                                                                                                                // 从未收到完整帧
    {
        snprintf(age, sizeof(age), "NA");                                                                                                                                                                      // NA表示没有可计算年龄的数据
    }
    else
    {
        snprintf(age, sizeof(age), "%lu", static_cast<unsigned long>(static_cast<uint32_t>(HAL_GetTick() - data.Last_Rx_Tick)));                                                                               // age_ms: 数据年龄(ms), 兼容时基回绕
    }
    const int tx_length = snprintf(tx_data[tx_index], sizeof(tx_data[tx_index]),
                                  "UPPER online=%u valid=%u age_ms=%s crc_enable=%u rx=%lu frame=%lu invalid=%lu crc_err=%lu overflow=%lu\r\n",                                                                // 各字段按下面参数顺序输出
                                  static_cast<unsigned int>(Upper.Get_Status() == Upper_Status_ONLINE), static_cast<unsigned int>(data.Valid), age,                                                            // 在线0/1、数据有效0/1、数据年龄(ms)
                                  static_cast<unsigned int>(Upper.Get_CRC_Check_Enable()), static_cast<unsigned long>(data.Rx_Byte_Count),                                                                     // CRC校验启用0/1、累计接收字节数
                                  static_cast<unsigned long>(data.Frame_Count), static_cast<unsigned long>(data.Invalid_Frame_Count),                                                                          // 接收成功帧数、帧格式错误次数
                                  static_cast<unsigned long>(data.CRC_Error_Count), static_cast<unsigned long>(data.Buffer_Overflow_Count));                                                                   // CRC错误次数、接收缓存溢出次数
    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data[tx_index])) && UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data[tx_index]), static_cast<uint16_t>(tx_length)) == HAL_OK)  // 文本完整时DMA发送, 忙时跳过
    {
        tx_index ^= 1U;                                                                                                                                                                                        // 仅发送成功才切换缓冲区
    }
}

/**
 * @brief USART10打印HI06姿态角和温度
 * @note online: 在线0/1; rpy_deg: 横滚Roll、俯仰Pitch、偏航Yaw(°); temp: 温度(℃)
 */
void Class_Debug_Control::UART10_Print_IMU_Attitude_Data()
{
    char roll[16];
    char pitch[16];
    char yaw[16];
    char temperature[16];
    // DMA发送期间保持当前缓冲区不变, 另一缓冲区用于准备下一帧
    char (&tx_data)[2][96] = UART10_IMU_Attitude_Data_Tx_Data;
    uint8_t &tx_index = UART10_IMU_Attitude_Data_Tx_Index;
    const Struct_HI06_Data &data = HI06.Get_Data();

    UART10_Float_To_String(data.Roll * DEBUG_RAD_TO_DEG, roll);
    UART10_Float_To_String(data.Pitch * DEBUG_RAD_TO_DEG, pitch);
    UART10_Float_To_String(data.Yaw * DEBUG_RAD_TO_DEG, yaw);
    UART10_Float_To_String(data.Temperature, temperature);

    int tx_length = snprintf(tx_data[tx_index], sizeof(tx_data[tx_index]),
                             "IMU online=%u rpy_deg=%s,%s,%s temp=%s\r\n",
                             static_cast<unsigned int>(HI06.Get_Status() == HI06_Status_ONLINE),
                             roll, pitch, yaw, temperature);

    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data[tx_index])) && UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data[tx_index]), static_cast<uint16_t>(tx_length)) == HAL_OK)
    {
        tx_index ^= 1U;
    }
}

/**
 * @brief USART10打印HI06三轴加速度和温度
 * @note online: 在线0/1; acc: X、Y、Z轴加速度(m/s²); temp: 温度(℃)
 */
void Class_Debug_Control::UART10_Print_IMU_Acceleration_Data()
{
    char accel_x[16];
    char accel_y[16];
    char accel_z[16];
    char temperature[16];
    char (&tx_data)[2][96] = UART10_IMU_Acceleration_Data_Tx_Data;
    uint8_t &tx_index = UART10_IMU_Acceleration_Data_Tx_Index;
    const Struct_HI06_Data &data = HI06.Get_Data();

    UART10_Float_To_String(data.Accel_X, accel_x);
    UART10_Float_To_String(data.Accel_Y, accel_y);
    UART10_Float_To_String(data.Accel_Z, accel_z);
    UART10_Float_To_String(data.Temperature, temperature);

    int tx_length = snprintf(tx_data[tx_index], sizeof(tx_data[tx_index]),
                             "IMU online=%u acc=%s,%s,%s temp=%s\r\n",
                             static_cast<unsigned int>(HI06.Get_Status() == HI06_Status_ONLINE),
                             accel_x, accel_y, accel_z, temperature);

    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data[tx_index])) && UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data[tx_index]), static_cast<uint16_t>(tx_length)) == HAL_OK)
    {
        tx_index ^= 1U;
    }
}

/**
 * @brief USART10打印HI06三轴角速度和温度
 * @note online: 在线0/1; gyr: X、Y、Z轴角速度(rad/s); temp: 温度(℃)
 */
void Class_Debug_Control::UART10_Print_IMU_Gyroscope_Data()
{
    char gyro_x[16];
    char gyro_y[16];
    char gyro_z[16];
    char temperature[16];
    char (&tx_data)[2][96] = UART10_IMU_Gyroscope_Data_Tx_Data;
    uint8_t &tx_index = UART10_IMU_Gyroscope_Data_Tx_Index;
    const Struct_HI06_Data &data = HI06.Get_Data();

    UART10_Float_To_String(data.Gyro_X, gyro_x);
    UART10_Float_To_String(data.Gyro_Y, gyro_y);
    UART10_Float_To_String(data.Gyro_Z, gyro_z);
    UART10_Float_To_String(data.Temperature, temperature);

    int tx_length = snprintf(tx_data[tx_index], sizeof(tx_data[tx_index]),
                             "IMU online=%u gyr=%s,%s,%s temp=%s\r\n",
                             static_cast<unsigned int>(HI06.Get_Status() == HI06_Status_ONLINE),
                             gyro_x, gyro_y, gyro_z, temperature);

    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data[tx_index])) && UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data[tx_index]), static_cast<uint16_t>(tx_length)) == HAL_OK)
    {
        tx_index ^= 1U;
    }
}

/**
 * @brief USART10打印HI06接收和解析状态
 * @note online: 在线0/1; valid: 有效数据字段位掩码(十六进制), 不是布尔值; status: HI06主状态字
 * @note rx/frame: 缓存接收字节数/成功帧数; crc_err/invalid/overflow: CRC错误/无效帧/缓存溢出次数
 */
void Class_Debug_Control::UART10_Print_IMU_Status_Data()
{
    char (&tx_data)[2][160] = UART10_IMU_Status_Data_Tx_Data;
    uint8_t &tx_index = UART10_IMU_Status_Data_Tx_Index;
    const Struct_HI06_Data &data = HI06.Get_Data();

    int tx_length = snprintf(tx_data[tx_index], sizeof(tx_data[tx_index]),
                             "IMU online=%u valid=0x%08lX status=%u rx=%lu frame=%lu crc_err=%lu invalid=%lu overflow=%lu\r\n",
                             static_cast<unsigned int>(HI06.Get_Status() == HI06_Status_ONLINE),
                             static_cast<unsigned long>(static_cast<uint32_t>(data.Valid)),
                             static_cast<unsigned int>(data.Main_Status),
                             static_cast<unsigned long>(data.Rx_Byte_Count),
                             static_cast<unsigned long>(data.Frame_Count),
                             static_cast<unsigned long>(data.CRC_Error_Count),
                             static_cast<unsigned long>(data.Invalid_Frame_Count),
                             static_cast<unsigned long>(data.Buffer_Overflow_Count));

    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data[tx_index])) && UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data[tx_index]), static_cast<uint16_t>(tx_length)) == HAL_OK)
    {
        tx_index ^= 1U;
    }
}

/**
 * @brief USART10打印遥控数据
 * @note online/motor_enable: 遥控在线/电机使能指令(0/1); mode: 遥控模式, 当前1为调试、2为手动
 * @note ch: Channel[0~3]的原始通道值, 当前中值1000; function: Function[0~15]的功能位顺序
 */
void Class_Debug_Control::UART10_Print_Remote_Data()
{
    // DMA发送未完成前缓冲区必须保持有效
    char (&tx_data)[160] = UART10_Remote_Data_Tx_Data;
    int tx_length = snprintf(tx_data, sizeof(tx_data),
                             "RC online=%u mode=%u motor_enable=%u ch=%u,%u,%u,%u function=%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u\r\n",
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Online),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Mode),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Motor_Enable),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Channel[0]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Channel[1]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Channel[2]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Channel[3]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[0]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[1]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[2]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[3]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[4]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[5]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[6]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[7]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[8]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[9]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[10]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[11]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[12]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[13]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[14]),
                             static_cast<unsigned int>(SBUS_Receive.SBUS_Data.Function[15]));

    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data)))
    {
        UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data), static_cast<uint16_t>(tx_length));
    }
}

/**
 * @brief USART10打印DM_J8006速度控制测试电机数据, 不是舵向DM6220
 * @note enable: 软件使能指令0/1; status: 软件在线0/1; control_status: 驱动器反馈状态(0失能/1使能/8~14故障)
 * @note target/omega: 目标/实际角速度(rad/s); torque: 反馈转矩(N·m)
 * @note mos/rotor: 当前直接输出驱动内部温度(K), 未转换为℃, 与VOFA温度通道不同
 */
void Class_Debug_Control::UART10_Print_Motor_Data()
{
    char control_omega[16];
    char now_omega[16];
    char now_torque[16];
    char mos_temperature[16];
    char rotor_temperature[16];
    // DMA发送未完成前缓冲区必须保持有效
    char (&tx_data)[180] = UART10_Motor_Data_Tx_Data;

    UART10_Float_To_String(DM_J8006.Get_Control_Omega(), control_omega);
    UART10_Float_To_String(DM_J8006.Get_Now_Omega(), now_omega);
    UART10_Float_To_String(DM_J8006.Get_Now_Torque(), now_torque);
    UART10_Float_To_String(DM_J8006.Get_Now_MOS_Temperature(), mos_temperature);
    UART10_Float_To_String(DM_J8006.Get_Now_Rotor_Temperature(), rotor_temperature);

    int tx_length = snprintf(tx_data, sizeof(tx_data),
                             "DM enable=%u status=%u control_status=%u target=%s omega=%s torque=%s mos=%s rotor=%s\r\n",
                             static_cast<unsigned int>(DM_J8006.Get_Motor_Enable()),
                             static_cast<unsigned int>(DM_J8006.Get_Status()),
                             static_cast<unsigned int>(DM_J8006.Get_Control_Status()),
                             control_omega, now_omega, now_torque, mos_temperature, rotor_temperature);

    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data)))
    {
        UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data), static_cast<uint16_t>(tx_length));
    }
}

/**
 * @brief USART10打印左前舵轮控制链路诊断数据
 * @note RC/motor_enable: 遥控在线/使能指令(0/1); mode: 遥控模式; speed: X、Y平移速度(m/s)及W角速度(rad/s)
 * @note DM enable/online/control: 使能指令/软件在线/驱动器反馈状态; target/target_omega: 目标位置(rad)/MIT目标角速度(rad/s)
 * @note DM now: 协议反馈位置(rad), 非VOFA I9连续位置; error: 舵向诊断角度误差(rad); ratio: 舵向误差限速比例(0~1)
 * @note VESC enable/online: 使能指令/软件在线(0/1); target_erpm/now_erpm: 目标/实际电气转速, U8机械rpm=ERPM/21
 * @note 当前固定打印左前轮DM ID 1和VESC ID 1, 不随VOFA观察轮组选择切换
 */
void Class_Debug_Control::UART10_Print_Swerve_Data()
{
    char speed_x[16];
    char speed_y[16];
    char speed_w[16];
    char dm_target_angle[16];
    char dm_target_omega[16];
    char dm_now_angle[16];
    char dm_angle_error[16];
    char steer_speed_ratio[16];
    // DMA发送未完成前缓冲区必须保持有效
    char (&tx_data)[320] = UART10_Swerve_Data_Tx_Data;

    UART10_Float_To_String(Remote_Chassis_Command.Speed_X, speed_x);
    UART10_Float_To_String(Remote_Chassis_Command.Speed_Y, speed_y);
    UART10_Float_To_String(Remote_Chassis_Command.Speed_W, speed_w);
    UART10_Float_To_String(DM_Chassis_LF.Get_Control_Angle(), dm_target_angle);
    UART10_Float_To_String(DM_Chassis_LF.Get_Control_Omega(), dm_target_omega);
    UART10_Float_To_String(DM_Chassis_LF.Get_Now_Angle(), dm_now_angle);
    UART10_Float_To_String(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Angle_Error, dm_angle_error);
    UART10_Float_To_String(Chassis_Swerve_Diagnostic[Swerve_Wheel_LF].Steer_Speed_Ratio, steer_speed_ratio);

    int tx_length = snprintf(tx_data, sizeof(tx_data),
                             "SW RC=%u mode=%u motor_enable=%u speed=%s,%s,%s DM enable=%u online=%u control=%u target=%s target_omega=%s now=%s error=%s ratio=%s VESC enable=%u online=%u target_erpm=%ld now_erpm=%ld\r\n",
                             static_cast<unsigned int>(Remote_Chassis_Command.Online),
                             static_cast<unsigned int>(Remote_Chassis_Command.Mode),
                             static_cast<unsigned int>(Remote_Chassis_Command.Motor_Enable),
                             speed_x, speed_y, speed_w,
                             static_cast<unsigned int>(DM_Chassis_LF.Get_Motor_Enable()),
                             static_cast<unsigned int>(DM_Chassis_LF.Get_Status()),
                             static_cast<unsigned int>(DM_Chassis_LF.Get_Control_Status()),
                             dm_target_angle, dm_target_omega, dm_now_angle, dm_angle_error, steer_speed_ratio,
                             static_cast<unsigned int>(VESC_Chassis_LF.Get_Motor_Enable()),
                             static_cast<unsigned int>(VESC_Chassis_LF.Get_Status()),
                             static_cast<long>(VESC_Chassis_LF.Get_Control_ERPM()),
                             static_cast<long>(VESC_Chassis_LF.Get_Now_ERPM()));

    if (tx_length > 0 && tx_length < static_cast<int>(sizeof(tx_data)))
    {
        UART_Transmit_Data(&huart10, reinterpret_cast<uint8_t *>(tx_data), static_cast<uint16_t>(tx_length));
    }
}

/**
 * @brief 保留两位小数转换为字符串
 *
 * @param Value 浮点数
 * @param String 字符串
 */
void Class_Debug_Control::UART10_Float_To_String(float Value, char *String)
{
    bool negative = Value < 0.0f;
    if (negative)
    {
        Value = -Value;
    }

    uint32_t value_100 = static_cast<uint32_t>(Value * 100.0f + 0.5f);
    snprintf(String, 16, "%s%lu.%02lu", negative ? "-" : "", static_cast<unsigned long>(value_100 / 100U), static_cast<unsigned long>(value_100 % 100U));
}

/* 兼容原有任务入口 ----------------------------------------------------------*/

void Debug_Init()
{
    Debug_Control.Init();
}

void Debug_UART9_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    Debug_Control.UART9_RxCpltCallback(Buffer, Length);
}

void Debug_10ms_PeriodElapsedCallback()
{
    Debug_Control.TIM_10ms_PeriodElapsedCallback();
}

void Debug_Vofa_10ms_PeriodElapsedCallback()
{
    Debug_Control.Vofa_TIM_10ms_PeriodElapsedCallback();
}

void Debug_Set_Swerve_Wheel(Enum_Swerve_Wheel Wheel)
{
    Debug_Control.Set_Swerve_Wheel(Wheel);
}

void Debug_UART10_Print_Remote_Data()
{
    Debug_Control.UART10_Print_Remote_Data();
}

void Debug_UART10_Print_Motor_Data()
{
    Debug_Control.UART10_Print_Motor_Data();
}

void Debug_UART10_Print_Swerve_Data()
{
    Debug_Control.UART10_Print_Swerve_Data();
}

void Debug_UART10_Print_IMU_Data()
{
    Debug_Control.UART10_Print_IMU_Data();
}

void Debug_UART10_Print_IMU_Attitude_Data()
{
    Debug_Control.UART10_Print_IMU_Attitude_Data();
}

void Debug_UART10_Print_IMU_Acceleration_Data()
{
    Debug_Control.UART10_Print_IMU_Acceleration_Data();
}

void Debug_UART10_Print_IMU_Gyroscope_Data()
{
    Debug_Control.UART10_Print_IMU_Gyroscope_Data();
}

void Debug_UART10_Print_IMU_Status_Data()
{
    Debug_Control.UART10_Print_IMU_Status_Data();
}

void Debug_UART10_Print_Upper_Data()
{
    Debug_Control.UART10_Print_Upper_Data();
}

void Debug_UART10_Print_Upper_Status_Data()
{
    Debug_Control.UART10_Print_Upper_Status_Data();
}
