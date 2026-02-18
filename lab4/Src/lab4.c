#include "main.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <assert.h>
#include <stdio.h>
#include "hal_gpio.h"
#include "core_cm0.h"
#include "stm32f0xx_hal_gpio_ex.h"
#include "stm32f0xx_hal_rcc.h"

void sendChar(char c);
void sendString(const char *str);
void SystemClock_Config(void);

volatile uint8_t receiveReg;
volatile uint8_t newData = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_RCC_GPIOA_CLK_ENABLE();
  RCC->APB2ENR |= 0x1; //enables SYSCFG clk
  RCC_USART3_CLK_ENABLE();

  // Setup PB10 and PB11 into Tx and RX mode 
  GPIO_InitTypeDef iniStr = {GPIO_PIN_10 | GPIO_PIN_11,
                          GPIO_MODE_AF_PP,
                          GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW,
                          GPIO_AF4_USART3};
  // Setup ALL LEDs
  GPIO_InitTypeDef iniStr2 = {GPIO_PIN_8 | GPIO_PIN_7 |GPIO_PIN_6 | GPIO_PIN_9,
                          GPIO_MODE_OUTPUT_PP,
                          GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW};
  My_HAL_GPIOx_Init(GPIOB, &iniStr);
  My_HAL_GPIOx_Init(GPIOC, &iniStr2);
  USART_Setup(USART3, 115200);

  assert(((GPIOB->MODER >> (10*2)) & 0x3) == 0x2); //assert PB10 is in alternate mode (10)
  assert(((GPIOB->MODER >> (11*2)) & 0x3) == 0x2); //assert PB11 is in alternate mode (10)

  assert(((GPIOB->AFR[1] >> 8) & 0xF) == 0x4); //assert PC10 is in AF4 mode
  assert(((GPIOB->AFR[1] >> 12) & 0xF) == 0x4); //assert PC11 is in AF4 mode

  assert(((USART3->CR1 >> 15) & 0x1) == 0x0); // check USART3 OVER8 = 0
  assert(((USART3->CR1 >> 2) & 0x3) == 0x3); // check TX and RX are enabled
  assert(((USART3->CR1) & 0x1) == 0x1); // check USART3 is enabled
  assert((USART3->BRR) == 0x45); // check USART3 Baud rate divdes by 69

  EXTI_Setup(EXTI, SYSCFG); // Setup PA0 (User Button) for interupts
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_EnableIRQ(USART3_4_IRQn);
  NVIC_SetPriority(EXTI0_1_IRQn,2);
  NVIC_SetPriority(USART3_4_IRQn, 1);

  char errorMsgLED[] = "ERROR: Key Pressed Not Routed To An LED\r\n ";
  char errorMsgNum[] = "ERROR: Number Pressed Not Available Option\r\n ";
  char CMDMsg[] = "Enter Command:\r\n";
  uint16_t GPIO_PIN;
  const char *selectedLED;
  const char *selectedCMD;
  while (1)
  {
    sendString(CMDMsg);
    while(newData == 0) {} //wait for rx reg to be populated with LED color
    uint8_t key = receiveReg;
    newData = 0;
    switch (key) {
      case 'b': 
        GPIO_PIN = GPIO_PIN_7;
        selectedLED = "blue";
        break;
      case 'o': 
        GPIO_PIN = GPIO_PIN_8;
        selectedLED = "orange";
        break;
      case 'g': 
        GPIO_PIN = GPIO_PIN_9;
        selectedLED = "green";
        break;
      case 'r': 
        GPIO_PIN = GPIO_PIN_6;
        selectedLED = "red";
        break;
      default: 
        sendString(errorMsgLED);
        continue;
    }
  
    while(newData == 0) {} //wait for rx reg to be populated with num command
    uint8_t num = receiveReg;
    newData = 0;
    switch (num) {
      case '0': 
        GPIOC->ODR &= ~GPIO_PIN;
        selectedCMD = "turned off";
        break;
      case '1': 
        GPIOC->ODR |= GPIO_PIN;
        selectedCMD = "turned on";
        break;
      case '2': 
        My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN);
        selectedCMD = "toggled";
        break;
      default: 
        sendString(errorMsgNum);
        continue;
    }

    char buffer[64];
    sprintf(buffer, "Successfully %s %s LED\r\n", selectedCMD, selectedLED);
    sendString(buffer);
  }
  return -1;
}

void sendChar(char c) {
  while(((USART3->ISR >> 7) & 0x1) == 0x0) {} //wait for tx reg to be empty
  USART3->TDR = c;
}

void sendString(const char *str) {
  while(*str != 0) {
    sendChar(*str);
    str++;
  }
}

void EXTI0_1_IRQHandler() {
  char string[] = "User Button Pressed\r\n ";
  sendString(string);
  My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
  NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
  EXTI->PR = 0x1;
}

void USART3_4_IRQHandler() {
  receiveReg = USART3->RDR;
  newData = 1;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
