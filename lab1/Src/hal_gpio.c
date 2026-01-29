#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <hal_gpio.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    // Put PC6,7,8,9(LEDs Red, Orange, Green, Blue) in output mode (01)
    GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18); // Sets MODER[x][0] to 1 for PC6,7,8,9
    GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19)); // Sets MODER[x][1] to 0 for PC6,7,8,9  
    GPIOA->MODER &= ~0x3; // Sets MODER[x] to 00 for PA0 (User Button)

    // Put PC6,7,8,9(LEDs Red, Orange, Green, Blue) in Push-pull output type (0)
    GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)); // Sets OTYPER[x] to 0 for PC6,7,8,9

    // Set PC6,7,8,9(LEDs Red, Orange, Green, Blue) speed to low speed (x0)
    GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18)); // Sets OSPEEDR[x][0] to 0 for PC6,7,8,9
    GPIOA->OSPEEDR &= ~0x1; // Sets OSPEEDR[x][0] to 0 (Low Speed) for PA0 (User Button)

    // Disable pull-up/pull-down resistors for PC6,7,8,9(LEDs Red, Orange, Green, Blue) (00)
    GPIOC->PUPDR &= ~((3 << 12) | (3 << 14) | (3 << 16) | (3 << 18)); // Sets PUPDR[x] to 0 for PC6,7,8,9
    //Setting User Button to pull-down resistor (10)
    GPIOA->PUPDR &= ~0x1; // Sets PUPDR[x][0] to 0 for PA0 (User Button)
    GPIOA->PUPDR |= (1 << 1); // Sets PUPDR[x][1] to 1 for PA0 (User Button)

    // Set LEDs initial state: Orange ON, Red, Green, and Blue OFF
    GPIOC->ODR |= (1 << 9); //set PC9 high (Green LEDs)
    GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8)); //set PC6, PC7, and PC8 low (Red, Green, and Blue LEDs)
}
void HAL_RCC_GPIOC_CLK_ENABLE(void)
{
    RCC->AHBENR |= (1 << 19); // Enable GPIOC clock bit (bit 19)
}
/*
void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
}
*/

/*
GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return -1;
}
*/

/*
void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
}
*/


void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR ^= GPIO_Pin; // Toggle the output data register for the passed pin(s)
}

