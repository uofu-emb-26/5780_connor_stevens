#include "main.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <assert.h>
#include <stdio.h>
#include "hal_gpio.h"
#include "core_cm0.h"
#include "stm32f0xx_hal_gpio_ex.h"

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
  HAL_RCC_GPIOC_CLK_ENABLE(); //enable gpioc clock
  RCC_TIM23_CLK_Enable(); //enable TIM2 and TIM3 clk

  // Setup LEDs Green and Orange 
  GPIO_InitTypeDef iniStr = {GPIO_PIN_8 | GPIO_PIN_9,
                          GPIO_MODE_OUTPUT_PP,
                          GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW};
  // Setup LEDs Green and Orange 
  GPIO_InitTypeDef iniStr2 = {GPIO_PIN_6 | GPIO_PIN_7,
                          GPIO_MODE_AF_PP,
                          GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW,
                          GPIO_AF0_TIM3};
  My_HAL_GPIOx_Init(GPIOC, &iniStr);
  My_HAL_GPIOx_Init(GPIOC, &iniStr2);

  TIM2_Setup(TIM2); //setup clock to interupt 4 times a second
  TIM3_Setup(TIM3);

  assert(((GPIOC->MODER >> (6*2)) & 0x3) == 0x2); //assert PC6 (Blue LED) is in alternate mode (10)
  assert(((GPIOC->MODER >> (7*2)) & 0x3) == 0x2); //assert PC7 (Red LED) is in alternate mode (10)

  assert(((GPIOC->AFR[0] >> 28) & 0xF) == 0x0); //assert PC7 is in AF0 mode
  assert(((GPIOC->AFR[0] >> 24) & 0xF) == 0x0); //assert PC6 is in AF0 mode

  // Check CCMR1: CC1S = 00, CC2S = 00 (bits 0-1 and 8-9)
  assert((TIM3->CCMR1 & 0x3) == 0);         // CC1S
  assert((TIM3->CCMR1 & (0x3 << 8)) == 0);  // CC2S

  // Check CCMR1: OC1M = 111 (bits 4-6), OC2M = 110 (bits 12-14)
  assert((TIM3->CCMR1 & (0x7 << 4)) == (0x7 << 4));     // OC1M = 111
  assert((TIM3->CCMR1 & (0x7 << 12)) == (0x6 << 12));   // OC2M = 110

  // Check CCMR1: OC1PE = 1 (bit 3), OC2PE = 1 (bit 11)
  assert((TIM3->CCMR1 & (1 << 3)) != 0);   // OC1PE
  assert((TIM3->CCMR1 & (1 << 11)) != 0);  // OC2PE

  // Check CCER: CC1E = 1 (bit 0), CC2E = 1 (bit 4)
  assert((TIM3->CCER & 0x1) != 0);         // CC1E
  assert((TIM3->CCER & (1 << 4)) != 0);    // CC2E

  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_SetPriority(TIM2_IRQn, 1);

  GPIOC->ODR |= (1 << 9); //set PC9 high (Green LED)
  GPIOC->ODR &= ~(1 << 8); //set PC8 low (Orange LED)

  while (1)
  {
  }
  return -1;
}

void TIM2_IRQHandler(void) 
{
  My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9); // toggle green and orange LED
  TIM2->SR &= ~(0x1);
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
