#include "stm32f072xb.h"
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <hal_gpio.h>

/**
 * General GPIO Init function
 */
void My_HAL_GPIOx_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    for (uint32_t pinNum = 0; pinNum < 16; pinNum++) {
        //checks if current pinNum is user passed Pin to modify
        if (GPIO_Init->Pin & (1U << pinNum)) {

            // Setting MODER Reg
            GPIOx->MODER &= ~(3U << (pinNum * 2)); //clears pin mode
            GPIOx->MODER |= (GPIO_Init->Mode << (pinNum * 2)); //sets pin to passed Mode

            //Setting OTYPER Reg
            if ((GPIO_Init->Mode & GPIO_MODE_OUTPUT_PP) == GPIO_MODE_OUTPUT_PP) {
                GPIOx->OTYPER &= ~(1U << pinNum); // Reset pin reg bit for push-pull
            }
            else {
                GPIOx->OTYPER |= (1U << pinNum); //set pin reg bit for open-drain
            }

            //setting PUPDR reg
            GPIOx->PUPDR &= ~(3U << (pinNum * 2)); //clears pin mode
            GPIOx->PUPDR |= (GPIO_Init->Pull << (pinNum * 2)); //sets pin to passed pull-up or pull-down mode

            //Setting OSPEEDR Reg
            GPIOx->OSPEEDR &= ~(3U << (pinNum * 2)); // Reset pin reg bits
            GPIOx->OSPEEDR |= (GPIO_Init->Speed << (pinNum * 2)); //Set pin reg bits to passed speed

            int AFRreg;
            //Setting Alternate Function
            if (pinNum >= 8) { // Checking what reg (high or low) to modify
                AFRreg = 1;
            } else {AFRreg = 0;}
            GPIOx->AFR[AFRreg] &= ~(8U << pinNum); // Clear 4 reg bits
            GPIOx->AFR[AFRreg] |= (GPIO_Init->Alternate << pinNum); // Set alternate bits in proper reg
        }
    }
}

void HAL_RCC_GPIOC_CLK_ENABLE(void)
{
    RCC->AHBENR |= (1 << 19); // Enable GPIOC clock bit (bit 19)
}

void HAL_RCC_GPIOA_CLK_ENABLE(void)
{
    RCC->AHBENR |= (1 << 17); // Enable GPIOA clock bit (bit 17)
}

void RCC_TIM23_CLK_Enable(void) 
{
    RCC->APB1ENR |= 0x1; // Sets bits 1:0 (TIM2 and TIM3)
    RCC->APB1ENR |= (1 << 1); // Sets bits 1:0 (TIM2 and TIM3)
}

/*
void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
}
*/

GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if (GPIOx->IDR & GPIO_Pin) {
        return GPIO_PIN_SET; // IDR reg is 1 (pin is high)
    } else {
        return GPIO_PIN_RESET; // IDR reg is 0 (pin is low)
    }
}


void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    if (PinState == GPIO_PIN_SET) {
        GPIOx->BSRR |= GPIO_Pin; // Set the ODR reg for the passed pin
    } else {
        GPIOx->BSRR &= ~GPIO_Pin; // Reset the ODR reg for the passed pin
    }
}


void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR ^= GPIO_Pin; // Toggle the output data register for the passed pin(s)
}

void Timer2_Setup(TIM_TypeDef *TIMx) {
    TIMx->PSC = 0x1F3F; // set PSC to 7,999 (make clk 1KHz)
    TIMx->ARR = 0xFA; // set ARR to 250
    TIMx->DIER |= 0x1; //enable UEV interrupt
    TIMx->CR1 |= 0x1; //enable time clk
}

