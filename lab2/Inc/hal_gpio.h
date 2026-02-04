#include "stm32f072xb.h"
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <stm32f0xx_hal_rcc.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void My_HAL_GPIOx_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_RCC_GPIOC_CLK_ENABLE(void);
void HAL_RCC_GPIOA_CLK_ENABLE(void);
void EXTI_Setup(EXTI_TypeDef *EXTI0, SYSCFG_TypeDef *EXTICR);