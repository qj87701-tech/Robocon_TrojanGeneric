#ifndef TESTS_HOST_STUBS_DVC_NRF24L01_H
#define TESTS_HOST_STUBS_DVC_NRF24L01_H

#include <cstdint>

#define NRF24L01_RX_PACKET_WIDTH 31

typedef struct
{
    std::uint8_t NRF24L01_TxAddress[5];
    std::uint8_t NRF24L01_TxPacket[NRF24L01_RX_PACKET_WIDTH];
    std::uint8_t NRF24L01_RxAddress[5];
    std::uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];
} NRF24L01_Type;

struct Fake_NRF24L01_State
{
    std::uint8_t Rx_Packet[NRF24L01_RX_PACKET_WIDTH]{};
    std::uint8_t Receive_Result = 0U;
    int Init_Count = 0;
    int Receive_Count = 0;
};

inline Fake_NRF24L01_State Fake_NRF24L01;

class Class_NRF24L01
{
public:
    NRF24L01_Type NRF24L01_data{};

    void NRF24L01_Init()
    {
        Fake_NRF24L01.Init_Count++;
    }

    std::uint8_t NRF24L01_Receive()
    {
        Fake_NRF24L01.Receive_Count++;

        if (Fake_NRF24L01.Receive_Result == 1U)
        {
            for (std::uint8_t i = 0; i < NRF24L01_RX_PACKET_WIDTH; i++)
            {
                NRF24L01_data.NRF24L01_RxPacket[i] = Fake_NRF24L01.Rx_Packet[i];
            }
        }

        return Fake_NRF24L01.Receive_Result;
    }
};

#endif
