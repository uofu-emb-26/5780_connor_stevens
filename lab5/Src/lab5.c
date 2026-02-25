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
volatile uint8_t receiveReg;
volatile uint8_t newData = 0;
void sendChar(char c);
void sendString(const char *str);

typedef enum {
    I2C_PASS = 0,
    I2C_NACK = 1
} I2C_Status;

I2C_Status writeOp(uint8_t Byte);
I2C_Status readOp(uint8_t *Byte);
void NACKF_Error(void);



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
  HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_RCC_GPIOC_CLK_ENABLE();
  RCC->APB1ENR |= (1 << 22); // Enable I2C2 CLK
  RCC_USART3_CLK_ENABLE();

  // Setup I2C SDA
  GPIO_InitTypeDef iniStr = {GPIO_PIN_11,
                            GPIO_MODE_AF_OD,
                            GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW,
                      GPIO_AF1_I2C2};

  GPIO_InitTypeDef iniStr3 = {GPIO_PIN_4 | GPIO_PIN_5,
                        GPIO_MODE_AF_PP,
                        GPIO_NOPULL,
                        GPIO_SPEED_FREQ_LOW,
                        GPIO_AF1_USART3};
  //Setup ALL LEDs and PC0
  GPIO_InitTypeDef iniStr2 = {GPIO_PIN_8 | GPIO_PIN_7 |GPIO_PIN_6 | GPIO_PIN_9 |GPIO_PIN_0,
                          GPIO_MODE_OUTPUT_PP,
                          GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW};

  My_HAL_GPIOx_Init(GPIOB, &iniStr);
  iniStr.Pin = GPIO_PIN_13; // set to PB13
  iniStr.Alternate = GPIO_AF5_I2C2; //set AF to 5
  My_HAL_GPIOx_Init(GPIOB, &iniStr);
  My_HAL_GPIOx_Init(GPIOC, &iniStr2);
  iniStr2.Pin = GPIO_PIN_14; // Set inistr3 to only pin 14 for PB14 init
  My_HAL_GPIOx_Init(GPIOB, &iniStr2);
  My_HAL_GPIOx_Init(GPIOC, &iniStr3); //set PC4 and PC5
  USART_Setup(USART3, 115200);

  GPIOB->ODR |= (1 << 14); // Set PB14
  GPIOC->ODR |= 0x1; //Set PC0

  assert(((GPIOB->MODER >> (11*2)) & 0x3) == 0x2); //assert PB11 is in alternate mode (10)
  assert(((GPIOB->MODER >> (13*2)) & 0x3) == 0x2); //assert PB13 is in alternate mode (10)
  assert(((GPIOB->MODER >> (14*2)) & 0x3) == 0x1); //assert PB13 is in output mode (01)
  assert(((GPIOC->MODER) & 0x3) == 0x1); //assert PC0 is in output mode (01)

  assert(((GPIOB->OTYPER >> 11) & 0x1) == 0x1); //assert PB11 open-drain mode(1)
  assert(((GPIOB->OTYPER >> 13) & 0x1) == 0x1); //assert PB13 open-drain mode(1)
  assert(((GPIOB->OTYPER >> 14) & 0x1) == 0x0); //assert PB14 push-pull mode(1)
  assert(((GPIOB->OTYPER >> 13) & 0x1) == 0x1); //assert PC0 push-pull mode(1) 

  assert(((GPIOB->PUPDR >> (13*2)) & 0x3) == 0x0); //assert PB13 is in no pull (00)
  assert(((GPIOB->PUPDR >> (11*2)) & 0x3) == 0x0); //assert PB11 is in no pull (00)
  assert(((GPIOB->AFR[1] >> 12) & 0xF) == 0x1); //assert PB11 is in AF1 mode
  assert(((GPIOB->AFR[1] >> 20) & 0xF) == 0x5); //assert PB13 is in AF5 mode

  assert(((GPIOB->ODR >> 14) & 0x1) == 0x1); //assert PB14 is high
  assert(((GPIOC->ODR) & 0x1) == 0x1); //assert PC0 is high

  I2Cx_Setup(I2C2);
  I2C2->CR1 |= 0x1; // Enable peripheral

  Init_I2C_Transaction(I2C2, 0, 0x69, 0x1); //Initialize Write Transmit
  writeOp(0x0F); // Write WHO_AM_I Address
  Init_I2C_Transaction(I2C2, 1, 0x69, 0x1); //Initialize Read Transmit

  uint8_t receivedByte;
  if (readOp(&receivedByte) == I2C_PASS) { // Attempt to Read
    if (receivedByte == 0xD4) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    } else {HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);}
  }
  I2C2->CR2 |= (1 << 14); // Set STOP

  char msg[] = "Test USART\r\n";

  while (1)
  {
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    sendString(msg);
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

I2C_Status writeOp(uint8_t Byte) {
  // Wait for TXIS or NACKF Flags set
  while((((I2C2->ISR >> 1) & 0x1) == 0x0) || (((I2C2->ISR >> 4) & 0x1) == 0x0)) {} 
  if (((I2C2->ISR >> 1) & 0x1) == 0x1) {
    I2C2->TXDR = Byte;
    while (((I2C2->ISR >> 6) & 0x1) == 0); // Wait for TC flag to be set
    char buffer[64];
    sprintf(buffer, "Successfully Sent Byte: %u\r\n", Byte);
    sendString(buffer);
    return I2C_PASS;
  }
  else {
    return I2C_NACK;
  }
}

I2C_Status readOp(uint8_t *Byte) {
  // Wait for RXNE or NACKF Flags set
  while((((I2C2->ISR >> 2) & 0x1) == 0x0) || (((I2C2->ISR >> 4) & 0x1) == 0x0)) {} 
  if (((I2C2->ISR >> 2) & 0x1) == 0x1) {
    *Byte = I2C2->RXDR;
    while (((I2C2->ISR >> 6) & 0x1) == 0); // Wait for TC flag to be set
    char buffer[64];
    sprintf(buffer, "Successfully Received Byte: %s\r\n", Byte);
    sendString(buffer);
    return I2C_PASS;
  }
  else {
    return I2C_NACK;
  }
}

void NACKF_Error(void) {
  char string[] = "NACKF flag set!\r\n";
  sendString(string);
  I2C2->ICR |= (1<<4);
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
