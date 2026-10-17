#ifndef TESTS_HOST_STUBS_STM32H7XX_HAL_H
#define TESTS_HOST_STUBS_STM32H7XX_HAL_H

#include <cstdint>

struct GPIO_TypeDef
{
    std::uint32_t Marker;
};

enum GPIO_PinState
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET,
};

constexpr std::uint16_t GPIO_PIN_2 = 0x0004U;
constexpr std::uint16_t GPIO_PIN_4 = 0x0010U;
constexpr std::uint16_t GPIO_PIN_13 = 0x2000U;

struct FDCAN_HandleTypeDef
{
    void *Instance;
};

struct FDCAN_RxHeaderTypeDef
{
    std::uint32_t Identifier;
    std::uint32_t IdType;
    std::uint32_t RxFrameType;
    std::uint32_t DataLength;
    std::uint32_t ErrorStateIndicator;
    std::uint32_t BitRateSwitch;
    std::uint32_t FDFormat;
    std::uint32_t RxTimestamp;
    std::uint32_t FilterIndex;
    std::uint32_t IsFilterMatchingFrame;
};

struct FDCAN_TxHeaderTypeDef
{
    std::uint32_t Identifier;
    std::uint32_t IdType;
    std::uint32_t TxFrameType;
    std::uint32_t DataLength;
    std::uint32_t ErrorStateIndicator;
    std::uint32_t BitRateSwitch;
    std::uint32_t FDFormat;
    std::uint32_t TxEventFifoControl;
    std::uint32_t MessageMarker;
};

struct FDCAN_FilterTypeDef
{
    std::uint32_t IdType;
    std::uint32_t FilterIndex;
    std::uint32_t FilterType;
    std::uint32_t FilterConfig;
    std::uint32_t FilterID1;
    std::uint32_t FilterID2;
};

struct TIM_HandleTypeDef
{
    void *Instance;
};

struct UART_HandleTypeDef
{
    void *Instance;
};

enum HAL_StatusTypeDef
{
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
};

#define TIM5 (reinterpret_cast<void *>(0x40000U))
#define USART1 (reinterpret_cast<void *>(0x50000U))
#define USART2 (reinterpret_cast<void *>(0x50001U))
#define USART3 (reinterpret_cast<void *>(0x50002U))
#define UART4 (reinterpret_cast<void *>(0x50003U))
#define UART5 (reinterpret_cast<void *>(0x50004U))
#define USART6 (reinterpret_cast<void *>(0x50005U))
#define UART7 (reinterpret_cast<void *>(0x50006U))
#define UART8 (reinterpret_cast<void *>(0x50007U))
#define UART9 (reinterpret_cast<void *>(0x50008U))
#define USART10 (reinterpret_cast<void *>(0x50009U))
#define FDCAN1 (reinterpret_cast<void *>(0x10000U))
#define FDCAN2 (reinterpret_cast<void *>(0x20000U))
#define FDCAN3 (reinterpret_cast<void *>(0x30000U))
#define HAL_MAX_DELAY 0xFFFFFFFFU
#define RESET 0U

#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 1U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_REMOTE_FRAME 1U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_FILTER_MASK 0U
#define FDCAN_FILTER_TO_RXFIFO0 0U
#define FDCAN_REJECT 0U
#define FDCAN_FILTER_REMOTE 0U
#define FDCAN_REJECT_REMOTE 1U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE (1UL << 0)
#define FDCAN_IT_BUS_OFF (1UL << 1)
#define FDCAN_IT_ERROR_PASSIVE (1UL << 2)
#define FDCAN_IT_ARB_PROTOCOL_ERROR (1UL << 3)
#define FDCAN_IT_DATA_PROTOCOL_ERROR (1UL << 4)
#define FDCAN_DLC_BYTES_0 0x00000000U
#define FDCAN_DLC_BYTES_1 0x00010000U
#define FDCAN_DLC_BYTES_2 0x00020000U
#define FDCAN_DLC_BYTES_3 0x00030000U
#define FDCAN_DLC_BYTES_4 0x00040000U
#define FDCAN_DLC_BYTES_5 0x00050000U
#define FDCAN_DLC_BYTES_6 0x00060000U
#define FDCAN_DLC_BYTES_7 0x00070000U
#define FDCAN_DLC_BYTES_8 0x00080000U

inline std::uint32_t __get_PRIMASK()
{
    return 0U;
}

inline void __disable_irq()
{
}

inline void __set_PRIMASK(std::uint32_t)
{
}

using Test_DMB_Hook = void (*)();
inline Test_DMB_Hook Test_DMB_Callback = nullptr;

inline void __DMB()
{
    if (Test_DMB_Callback != nullptr)
    {
        Test_DMB_Hook callback = Test_DMB_Callback;
        Test_DMB_Callback = nullptr;
        callback();
    }
}

#ifdef __cplusplus
extern "C" {
#endif

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *gpio_port, std::uint16_t gpio_pin);
std::uint32_t HAL_GetTick();
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, std::uint8_t *data,
                                    std::uint16_t size, std::uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, std::uint8_t *data,
                                        std::uint16_t size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart,
                                               std::uint8_t *data,
                                               std::uint16_t size);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, std::uint16_t size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, FDCAN_FilterTypeDef *filter);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan, std::uint32_t non_matching_std, std::uint32_t non_matching_ext, std::uint32_t reject_std_remote, std::uint32_t reject_ext_remote);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *hfdcan, std::uint32_t active_it, std::uint32_t buffer_indexes);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *header, std::uint8_t *data);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, std::uint32_t rx_location, FDCAN_RxHeaderTypeDef *header, std::uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif
