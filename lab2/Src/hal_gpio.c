#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <hal_gpio.h>

/**
 * LED gpio init functio
 */
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
    GPIOA->PUPDR &= ~0x1; // Sets PUPDR[0][0] to 0 for PA0 (User Button)
    GPIOA->PUPDR |= (1 << 1); // Sets PUPDR[0][1] to 1 for PA0 (User Button)

    // Set LEDs initial state: Orange ON, Red, Green, and Blue OFF
    GPIOC->ODR |= (1 << 9); //set PC9 high (Green LEDs)
    GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8)); //set PC6, PC7, and PC8 low (Red, Green, and Blue LEDs)
}

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

void EXTI_Setup(EXTI_TypeDef *EXTI0, SYSCFG_TypeDef *EXTICR) {
    EXTI0->IMR |= 0x1; //unmask interupt generation for line0
    EXTI0->RTSR |= 0x1; //enable rising trigger detection for line0
    SYSCFG->EXTICR[0] |= ~(0xF); //set multiplexer of EXTI0 to PA0
}

