#include <cstdint>
#include <cstring>

#include "test_assert.h"
#include "../../User_File/1_Middleware/Driver/CAN/drv_can.h"

bool init_finished = true;
int fake_sequence = 0;
int fake_timestamp_init_sequence = 0;
int fake_timestamp_overflow_count = 0;
TIM_HandleTypeDef *fake_timestamp_handler = nullptr;
std::uint64_t fake_timestamp_now = 0U;
Class_Timestamp SYS_Timestamp;

FDCAN_HandleTypeDef hfdcan1{FDCAN1};
FDCAN_HandleTypeDef hfdcan2{FDCAN2};
FDCAN_HandleTypeDef hfdcan3{FDCAN3};

namespace
{
FDCAN_HandleTypeDef *Configured_Handler = nullptr;
FDCAN_FilterTypeDef Configured_Filter{};
std::uint32_t Global_Standard_Action = 0U;
std::uint32_t Global_Extended_Action = 0U;
std::uint32_t Remote_Standard_Action = 0U;
std::uint32_t Remote_Extended_Action = 0U;
std::uint32_t Notification_Mask = 0U;
int Start_Count = 0;

FDCAN_HandleTypeDef *Transmit_Handler = nullptr;
FDCAN_TxHeaderTypeDef Transmit_Header{};
std::uint8_t *Transmit_Data = nullptr;

void Callback_A(FDCAN_RxHeaderTypeDef &, std::uint8_t *)
{
}

void Reset_Fakes()
{
    Configured_Handler = nullptr;
    Configured_Filter = {};
    Global_Standard_Action = 0xFFFFFFFFU;
    Global_Extended_Action = 0xFFFFFFFFU;
    Remote_Standard_Action = 0xFFFFFFFFU;
    Remote_Extended_Action = 0xFFFFFFFFU;
    Notification_Mask = 0U;
    Start_Count = 0;
    Transmit_Handler = nullptr;
    Transmit_Header = {};
    Transmit_Data = nullptr;
}

void Test_CAN_Init_Keeps_Standard_All_Pass_Filter()
{
    Reset_Fakes();
    CAN_Init(&hfdcan1, Callback_A);

    assert(CAN1_Manage_Object.CAN_Handler == &hfdcan1);
    assert(CAN1_Manage_Object.Callback_Function == Callback_A);
    assert(Configured_Handler == &hfdcan1);
    assert(Configured_Filter.IdType == FDCAN_STANDARD_ID);
    assert(Configured_Filter.FilterIndex == 0U);
    assert(Configured_Filter.FilterType == FDCAN_FILTER_MASK);
    assert(Configured_Filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0);
    assert(Configured_Filter.FilterID1 == 0U);
    assert(Configured_Filter.FilterID2 == 0U);
    assert(Global_Standard_Action == FDCAN_REJECT);
    assert(Global_Extended_Action == FDCAN_REJECT);
    assert((Notification_Mask & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U);
    assert(Start_Count == 1);
}

void Test_CAN_Extended_Init_Uses_Extended_All_Pass_Filter()
{
    Reset_Fakes();
    CAN_Extended_Init(&hfdcan3, Callback_A);

    assert(CAN3_Manage_Object.CAN_Handler == &hfdcan3);
    assert(CAN3_Manage_Object.Callback_Function == Callback_A);
    assert(Configured_Handler == &hfdcan3);
    assert(Configured_Filter.IdType == FDCAN_EXTENDED_ID);
    assert(Configured_Filter.FilterIndex == 0U);
    assert(Configured_Filter.FilterType == FDCAN_FILTER_MASK);
    assert(Configured_Filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0);
    assert(Configured_Filter.FilterID1 == 0U);
    assert(Configured_Filter.FilterID2 == 0U);
    assert(Global_Standard_Action == FDCAN_REJECT);
    assert(Global_Extended_Action == FDCAN_REJECT);
    assert(Remote_Standard_Action == FDCAN_REJECT_REMOTE);
    assert(Remote_Extended_Action == FDCAN_REJECT_REMOTE);
    assert(Start_Count == 1);
}

void Test_Standard_Transmit_Remains_Standard()
{
    Reset_Fakes();
    std::uint8_t data[8]{};
    CAN_Transmit_Data(&hfdcan1, 0x123U, data, 8U);

    assert(Transmit_Handler == &hfdcan1);
    assert(Transmit_Header.Identifier == 0x123U);
    assert(Transmit_Header.IdType == FDCAN_STANDARD_ID);
    assert(Transmit_Header.DataLength == 8U);
    assert(Transmit_Data == data);
}

void Assert_Extended_Transmit_DLC(const std::uint16_t length, const std::uint32_t expected_dlc)
{
    Reset_Fakes();
    std::uint8_t data[8]{};
    CAN_Transmit_Extended_Data(&hfdcan3, 0x1ABCDEU, data, length);

    assert(Transmit_Handler == &hfdcan3);
    assert(Transmit_Header.Identifier == 0x1ABCDEU);
    assert(Transmit_Header.IdType == FDCAN_EXTENDED_ID);
    assert(Transmit_Header.TxFrameType == FDCAN_DATA_FRAME);
    assert(Transmit_Header.DataLength == expected_dlc);
    assert(Transmit_Header.FDFormat == FDCAN_CLASSIC_CAN);
    assert(Transmit_Data == data);
}

void Test_Extended_Transmit_Uses_29_Bit_ID_And_DLC()
{
    Assert_Extended_Transmit_DLC(4U, FDCAN_DLC_BYTES_4);
    Assert_Extended_Transmit_DLC(6U, FDCAN_DLC_BYTES_6);
    Assert_Extended_Transmit_DLC(8U, FDCAN_DLC_BYTES_8);
}
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *filter)
{
    Configured_Handler = hfdcan;
    Configured_Filter = *filter;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, std::uint32_t non_matching_std, std::uint32_t non_matching_ext, std::uint32_t reject_remote_std, std::uint32_t reject_remote_ext)
{
    Global_Standard_Action = non_matching_std;
    Global_Extended_Action = non_matching_ext;
    Remote_Standard_Action = reject_remote_std;
    Remote_Extended_Action = reject_remote_ext;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, std::uint32_t active_it, std::uint32_t)
{
    Notification_Mask = active_it;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *)
{
    ++Start_Count;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *)
{
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *header, std::uint8_t *data)
{
    Transmit_Handler = hfdcan;
    Transmit_Header = *header;
    Transmit_Data = data;
    return HAL_OK;
}

extern "C" HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, std::uint32_t, FDCAN_RxHeaderTypeDef *, std::uint8_t *)
{
    return HAL_ERROR;
}

int main()
{
    Test_CAN_Init_Keeps_Standard_All_Pass_Filter();
    Test_CAN_Extended_Init_Uses_Extended_All_Pass_Filter();
    Test_Standard_Transmit_Remains_Standard();
    Test_Extended_Transmit_Uses_29_Bit_ID_And_DLC();
    return 0;
}
