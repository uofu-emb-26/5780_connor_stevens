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

void SystemClock_Config(void);

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
  // Setup LEDs Green and Orange 
  GPIO_InitTypeDef iniStr2 = {GPIO_PIN_8,
                          GPIO_MODE_OUTPUT_PP,
                          GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW};
  My_HAL_GPIOx_Init(GPIOB, &iniStr);
  My_HAL_GPIOx_Init(GPIOC, &iniStr2);

  USART3->CR1 &= ~(1 << 15); //Set OVER8 = 0

  uint32_t clk = HAL_RCC_GetHCLKFreq();
  uint32_t baudRate = 115200;
  USART3->BRR = clk / baudRate; // Set Baud rate to ~115,200 (divide 8MHz by 69)

  USART3->CR1 |= (0x3 << 2); // Set bits 2 & 3 (TX and RX enable) to 1 
  USART3->CR1 |= 0x1; //enable USART3

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
  NVIC_SetPriority(EXTI0_1_IRQn, 1);
  
  while (1)
  {
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
  char string[] = "Hello World";
  sendString(string);
  My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
  NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
  EXTI->PR = 0x1;
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
