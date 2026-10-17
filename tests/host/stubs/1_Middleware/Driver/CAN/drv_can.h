#ifndef TESTS_HOST_STUBS_DRV_CAN_H
#define TESTS_HOST_STUBS_DRV_CAN_H

#include "fdcan.h"

#include <cstdint>

typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef &header, std::uint8_t *buffer);

struct Struct_CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;
    FDCAN_RxHeaderTypeDef Rx_Header;
    std::uint8_t Rx_Buffer[64];
    std::uint64_t Rx_Timestamp;
};

extern bool init_finished;
extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback callback_function);
void CAN_Extended_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback callback_function);
std::uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, std::uint16_t id, std::uint8_t *data, std::uint16_t length);
std::uint8_t CAN_Transmit_Extended_Data(FDCAN_HandleTypeDef *hfdcan, std::uint32_t id, std::uint8_t *data, std::uint16_t length);

#endif
