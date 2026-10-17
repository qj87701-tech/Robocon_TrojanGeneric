#ifndef __DVC_NRF24L01_H
#define __DVC_NRF24L01_H

#include "main.h"
#include "nrf24l01_define.h"

/*外部可调用全局数组***********/
#define NRF24L01_TX_PACKET_WIDTH 31 // 发送数据包宽度，范围：1~32字节
#define NRF24L01_RX_PACKET_WIDTH 31 // 接收通道0数据包宽度，范围：1~32字节

typedef struct
{
    uint8_t NRF24L01_TxAddress[5];
    uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];
    uint8_t NRF24L01_RxAddress[5];
    uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];

} NRF24L01_Type;

/***********外部可调用全局数组*/

/*函数声明*********************/

/*********************函数声明*/

class Class_NRF24L01
{
private:
    /*SPI实现*/
    void NRF24L01_W_CE(uint8_t BitValue);
    void NRF24L01_W_CSN(uint8_t BitValue);
    void NRF24L01_W_SCK(uint8_t BitValue);
    void NRF24L01_W_MOSI(uint8_t BitValue);
    void NRF24L01_GPIO_Init(void);
    uint8_t NRF24L01_R_MISO(void);
    uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte);
    /*指令实现*/
    uint8_t NRF24L01_ReadReg(uint8_t RegAddress);
    void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
    void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Data);
    void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);
    void NRF24L01_ReadRxPayload(uint8_t *DataArray, uint8_t Count);
    void NRF24L01_WriteTxPayload(uint8_t *DataArray, uint8_t Count);
    void NRF24L01_FlushTx(void);
    void NRF24L01_FlushRx(void);
    uint8_t NRF24L01_ReadStatus(void);

    /*功能函数*/
    void NRF24L01_PowerDown(void);
    void NRF24L01_StandbyI(void);
    void NRF24L01_Rx(void);
    void NRF24L01_Tx(void);

public:
    NRF24L01_Type NRF24L01_data;

    void NRF24L01_Init(void);
    uint8_t NRF24L01_Send(void);
    uint8_t NRF24L01_Receive(void);
    void NRF24L01_UpdateRxAddress(void);
};

#endif /* __DVC_NRF24L01_H */

