#ifndef TESTS_HOST_STUBS_GPIO_H
#define TESTS_HOST_STUBS_GPIO_H

#include "stm32h7xx_hal.h"

extern GPIO_TypeDef Fake_GPIOF_Instance;
inline GPIO_TypeDef Fake_GPIOG_Instance{0x600DU};

#define GPIOF (&Fake_GPIOF_Instance)
#define GPIOG (&Fake_GPIOG_Instance)
#define Key1_GPIO_Port GPIOF
#define Key1_Pin GPIO_PIN_2
#define Key2_GPIO_Port GPIOF
#define Key2_Pin GPIO_PIN_4
#define NRF_IRQ_GPIO_Port GPIOG
#define NRF_IRQ_Pin GPIO_PIN_13

#ifdef __cplusplus
extern "C" {
#endif

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
