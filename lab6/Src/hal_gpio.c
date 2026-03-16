#include "stm32f072xb.h"
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <hal_gpio.h>

/**
 * General GPIO Init function
 */
void My_HAL_GPIOx_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    for (uint32_t pinNum = 0; pinNum < 16; pinNum++) {
        //checks if current pinNum is user passed Pin to modify
        if (GPIO_Init->Pin & (1U << pinNum)) {

            // Setting MODER Reg
            GPIOx->MODER &= ~(3U << (pinNum * 2)); //clears pin mode
            GPIOx->MODER |= (GPIO_Init->Mode << (pinNum * 2)); //sets pin to passed Mode

           //Setting OTYPER Reg (check for both regular output and alternate function push-pull)
            if ((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP)) {
                GPIOx->OTYPER &= ~(1U << pinNum); // Push-pull
            } else {
                GPIOx->OTYPER |= (1U << pinNum); // Open-drain
            }

            //setting PUPDR reg
            GPIOx->PUPDR &= ~(3U << (pinNum * 2)); //clears pin mode
            GPIOx->PUPDR |= (GPIO_Init->Pull << (pinNum * 2)); //sets pin to passed pull-up or pull-down mode

            //Setting OSPEEDR Reg
            GPIOx->OSPEEDR &= ~(3U << (pinNum * 2)); // Reset pin reg bits
            GPIOx->OSPEEDR |= (GPIO_Init->Speed << (pinNum * 2)); //Set pin reg bits to passed speed

            uint32_t AFRreg;
            uint32_t AFRPin = pinNum;
            //Setting Alternate Function
            if (pinNum >= 8) { // Checking what reg (high or low) to modify
                AFRreg = 1;
                AFRPin -= 8;
            } else {AFRreg = 0;}
            GPIOx->AFR[AFRreg] &= ~(0xF << (AFRPin * 4)); // Clear 4 reg bits
            GPIOx->AFR[AFRreg] |= (GPIO_Init->Alternate << (AFRPin * 4)); // Set alternate bits in proper reg
        }
    }
}

void HAL_RCC_GPIOC_CLK_ENABLE(void)
{
    RCC->AHBENR |= (1 << 19); // Enable GPIOC clock bit (bit 19)
}

void HAL_RCC_GPIOB_CLK_ENABLE(void)
{
    RCC->AHBENR |= (1 << 18); // Enable GPIOB clock bit (bit 18)
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

void RCC_USART3_CLK_ENABLE(void)
{
    RCC->APB1ENR |= (1 << 18); // Sets bit 18 (USART3) to 1
}

void RCC_ADC_CLK_ENABLE(void) 
{
    RCC->APB2ENR |= (1 << 9);
}

void RCC_DAC_CLK_ENABLE(void)
{
    RCC->APB1ENR |= (1 << 29);
}

void EXTI_Setup(EXTI_TypeDef *EXTI0, SYSCFG_TypeDef *EXTICR) {
    EXTI0->IMR |= 0x1; //unmask interupt generation for line0
    EXTI0->RTSR |= 0x1; //enable rising trigger detection for line0
    SYSCFG->EXTICR[0] |= ~(0xF); //set multiplexer of EXTI0 to PA0
}

void USART_Setup(USART_TypeDef *USARTx, uint32_t baudRate) {
  USARTx->CR1 &= ~(1 << 15); //Set OVER8 = 0
  USARTx->CR1 |= (0x3 << 2); // Set bits 2 & 3 (TX and RX enable)
  USARTx->CR1 |= (1 << 5); // set bit 5 (receive register not empty interupt)

  uint32_t clk = HAL_RCC_GetHCLKFreq();
  USARTx->BRR = clk / baudRate; // Set Baud rate to ~115,200 (divide 8MHz by 69)

  USARTx->CR1 |= 0x1; //enable USART3
}

void I2Cx_Setup(I2C_TypeDef *I2Cx) {
    I2Cx->TIMINGR = (0x1 << 28) // PRESC
                  | (0x4 << 20) // SCLDEL
                  | (0x2 << 16) // SCADEL
                  | (0x0F << 8) // SCLH
                  | (0x13);     // SCLL
}

void Init_I2C_Transaction(I2C_TypeDef *I2Cx, int RW, uint8_t Addr, uint8_t NumBytes) {
    //clear NBYTES, SADD field, Reset AUTOEND, STOP, START, and R/W
    I2Cx->CR2 &= ~((0xFF << 16) | (0x7F << 1) | (1U << 25) | (1U << 14) | (1U << 13) | (1U << 10)); 
    
    I2Cx->CR2 |= ((Addr & 0x7F) << 1); // Set SADD (7-bit address goes in bits 7:1)
    I2Cx->CR2 |= ((NumBytes & 0xFF) << 16); // Set NBYTES (8 bit value)
    if (RW) I2Cx->CR2 |= (1U << 10);   // Set read/write bit (Write = 0, Read = 1)
    I2Cx->CR2 |= (1U << 13); // Set START bit
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
        GPIOx->BSRR = GPIO_Pin; // Set the ODR reg for the passed pin
    } else {
        GPIOx->BSRR = ((uint32_t)GPIO_Pin << 16); // Write to the upper 16 bits to reset
    }
}

void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->ODR ^= GPIO_Pin; // Toggle the output data register for the passed pin(s)
}

void TIM2_Setup(TIM_TypeDef *TIMx) {
    TIMx->PSC = 0x1F3F; // set PSC to 7,999 (make clk 1KHz)
    TIMx->ARR = 0xFA; // set ARR to 250
    TIMx->DIER |= 0x1; //enable UEV interrupt
    TIMx->CR1 |= 0x1; //enable time clk
}

void TIM3_Setup(TIM_TypeDef *TIMx) {
    TIMx->PSC = 0x63;
    TIMx->ARR = 0x64;
    
    // Clear and set channel modes properly
    TIMx->CCMR1 &= ~(0x3 | (0x3 << 8));     // Clear CC1S and CC2S (output mode)
    TIMx->CCMR1 &= ~(0x7 << 4);             // Clear OC1M
    TIMx->CCMR1 |= (0x7 << 4);              // Set OC1M to 111 (PWM Mode 2)
    TIMx->CCMR1 &= ~(0x7 << 12);            // Clear OC2M
    TIMx->CCMR1 |= (0x6 << 12);             // Set OC2M to 110 (PWM Mode 1)
    TIMx->CCMR1 |= (1 << 3) | (1 << 11);    // Enable preload for both channels
    
    TIMx->CCER |= (0x1) | (1 << 4);         // Enable output for channels 1 and 2
    
    TIMx->CCR1 = 20;  // 20% duty cycle
    TIMx->CCR2 = 80;  // 20% duty cycle
    
    TIMx->CR1 |= 0x1;         // Enable timer
}

void ADC_Configuration(void) {
    ADC1->CFGR1 |= (1 << 13); // set to continuous conversion mode
    ADC1->CFGR1 &= ~(0x3 << 10); // clear hardware trigger bits (software only)
    ADC1->CFGR1 &= ~(0x3 << 3); // Clear resolution bits
    ADC1->CFGR1 |= (1 << 4); // set 4:3 to 10 (8 bit resoltuion)
    ADC1->CFGR1 &= ~(1 << 5); // clear bit 5 (right aligned data)
}

void DAC_Configuration(void) {
    DAC1->CR |= 1; // Enable DAC channel 1
    DAC1->CR |= (1 << 2); // Enable Trigger 
    DAC1->CR |= (0x7 << 3); // Set trigger selection to software
}

