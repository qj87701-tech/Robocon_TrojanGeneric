#ifndef TESTS_HOST_MOTOR_STUBS_DRV_CAN_H
#define TESTS_HOST_MOTOR_STUBS_DRV_CAN_H

#include <cstdint>

#include "fdcan.h"

typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef &header, std::uint8_t *buffer);

struct Struct_CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;
    FDCAN_RxHeaderTypeDef Rx_Header;
    std::uint8_t Rx_Buffer[64];
    std::uint64_t Rx_Timestamp;
};

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;

extern std::uint8_t CAN1_0x3fe_Tx_Data[];
extern std::uint8_t CAN1_0x4fe_Tx_Data[];
extern std::uint8_t CAN2_0x3fe_Tx_Data[];
extern std::uint8_t CAN2_0x4fe_Tx_Data[];
extern std::uint8_t CAN3_0x3fe_Tx_Data[];
extern std::uint8_t CAN3_0x4fe_Tx_Data[];

std::uint8_t CAN_Transmit_Data(FDCAN_HandleTypeDef *hfdcan, std::uint16_t id,
                               std::uint8_t *data, std::uint16_t length);

#endif
