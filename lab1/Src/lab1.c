#include "main.h"
#include "stm32f0xx_hal.h"
#include <assert.h>
#include <hal_gpio.h>

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

  HAL_Init(); //reset of all peripherals, Initializes the Flash interface and the Systick.
  SystemClock_Config(); //config system clock

  HAL_RCC_GPIOC_CLK_ENABLE(); //enable GPIOC clock
  HAL_RCC_GPIOA_CLK_ENABLE(); //enable GPIOA clock
  assert(((RCC->AHBENR >> 19) & 0x1) == 1); //assert GPIOC clock is enabled
  GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7, 
                              GPIO_MODE_OUTPUT_PP, 
                              GPIO_NOPULL, 
                              GPIO_SPEED_FREQ_LOW};
  My_HAL_GPIO_Init(GPIOC, &initStr); //initialize GPIOC pins 6, 7, 8, and 9
  assert(((GPIOC->MODER >> (8*2)) & 0x3) == 0x1); //assert PC8 (Orange LED) is in output mode (01)
  assert(((GPIOC->MODER >> (9*2)) & 0x3) == 0x1); //assert PC9 (Green LED) is in output mode (01)
  assert(((GPIOC->MODER >> (6*2)) & 0x3) == 0x1); //assert PC6 (Blue LED) is in output mode (01)
  assert(((GPIOC->MODER >> (7*2)) & 0x3) == 0x1); //assert PC7 (Red LED) is in output mode (01)
  assert(((GPIOC->ODR >> 9) & 0x1) == 0x1); //assert Green LED is enabled
  assert(((GPIOC->ODR >> 6) & 0x1) == 0x0); //assert Red LED is disabled
  assert(((GPIOC->ODR >> 7) & 0x1) == 0x0); //assert Blue LED is disabled
  assert(((GPIOC->ODR >> 8) & 0x1) == 0x0); //assert Orange LED is disabled

  uint32_t debouncer = 0;
  int cycle = 0;
  int buttonEn = 1; // button starts enabled
  while (1)
  {
    debouncer = (debouncer << 1);
    //sets low bit to one if button is pressed
    if (My_HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
      debouncer |= 0x1;
    }
      if (debouncer == 0x7FFFFFFF && buttonEn) //button has been pressed, execute one cycle
    {
      buttonEn = 0; // disable until release
      switch (cycle)
      {
        case 0:
          My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_9); // Green OFF & Red ON
          break;
        case 1:
          My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_6); // Red OFF & Orange ON
          break;
        case 2:
          My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8); // Orange OFF & Blue ON
          break;
        case 3:
          My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9 | GPIO_PIN_7); // Blue OFF & Green ON
          break;
      }
      if (cycle == 3) {cycle = 0;}
      else {cycle++;}
    }

    // Button released, renable button
    if (debouncer == 0x00000000)
    {
        buttonEn = 1;
    }
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
