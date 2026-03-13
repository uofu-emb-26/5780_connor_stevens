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
  #include <stdlib.h>

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
  RCC_ADC_CLK_ENABLE();

  //Setup ALL LEDs and PC0
  GPIO_InitTypeDef iniStr1 = {GPIO_PIN_8 | GPIO_PIN_7 |GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_0,
                            GPIO_MODE_OUTPUT_PP,
                            GPIO_NOPULL,
                            GPIO_SPEED_FREQ_LOW};
  My_HAL_GPIOx_Init(GPIOC, &iniStr1);
  GPIOC->MODER |= 0x3; // Set PC0 to analog mode (11)

  assert(((GPIOC->MODER >> (6*2)) & 0x3) == 0x1); //assert PC6 is in output mode (01)
  assert(((GPIOC->MODER >> (7*2)) & 0x3) == 0x1); //assert PC7 is in output mode (01)
  assert(((GPIOC->MODER >> (8*2)) & 0x3) == 0x1); //assert PC8 is in output mode (01)
  assert(((GPIOC->MODER >> (9*2)) & 0x3) == 0x1); //assert PC9 is in output mode (01)
  assert(((GPIOC->MODER) & 0x3) == 0x3); //assert PC0 is in analog mode (11)
  assert(((GPIOC->PUPDR) & 0x3) == 0x0); //assert PC0 is in no pull/push (00)


  ADC_Configuration();
  ADC1->CHSELR |= (1 << 10); // set ADC channel to 10

  assert(((ADC1->CFGR1 >> 13) & 0x1) == 0x1); // check continuous conversion mode
  assert(((ADC1->CFGR1 >> 10) & 0x3) == 0x0); // check hardware trigger disabled (00)
  assert(((ADC1->CFGR1 >> 3) & 0x3) == 0x2); // check 8 bit resolution (10)
  assert(((ADC1->CHSELR >> 10) & 0x1) == 0x1); // check channel 10 is enabled

  ADC1->CR |= (1 << 31); // Start calibration process
  while (((ADC1->CR >> 31) & 0x1) == 0x0) {} // Wait for calibration process to complete

  ADC1->CR |= 0x1; // Enable ADC
  while (((ADC1->ISR) & 0x1) == 0x0) {} // Wait for ADC to enable

  ADC1->CR |= (1 << 2); // Start ADC conversion

  while (1)
  {
    HAL_Delay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
  }
  return -1;
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
