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
  char* int_to_str(int num);
  void GyroscopeInit(void);
  void WhoAmIRegRead(void);



  typedef enum {
      I2C_PASS = 0,
      I2C_NACK = 1
  } I2C_Status;

  I2C_Status writeOp(uint8_t Byte, uint8_t isLastByte);
  I2C_Status readOp(uint8_t *Byte, uint8_t isLastByte);
  void NACKF_Error(const char *msg);

  char newFlash[] = "<----------New Flash------------>\r\n\n";
  char enterW[] = "Entering WriteOp \r\n";
  char enterR[] = "Entering ReadOp \r\n";
  char pastWhileW[] = "Exited WriteOp While Loop \r\n";
  char pastWhileR[] = "Exited ReadOp While Loop \r\n";
  char waitingTC[] = "Waiting for TC to be SET \r\n";
  char SetupPass[] = "Gyroscope setup successfull\r\n";
  char SetupFail[] = "Gyroscope setup unsuccessfull\r\n";


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
    // Setup USART pins
    GPIO_InitTypeDef iniStr3 = {GPIO_PIN_4 | GPIO_PIN_5,
                          GPIO_MODE_AF_PP,
                          GPIO_NOPULL,
                          GPIO_SPEED_FREQ_LOW,
                          GPIO_AF1_USART3};
    //Setup ALL LEDs and PC0
    GPIO_InitTypeDef iniStr2 = {GPIO_PIN_8 | GPIO_PIN_7 |GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_0,
                            GPIO_MODE_OUTPUT_PP,
                            GPIO_NOPULL,
                            GPIO_SPEED_FREQ_LOW};
    GPIO_InitTypeDef iniStr4 = {
                              GPIO_PIN_15,
                              GPIO_MODE_OUTPUT_OD,  // Open-drain
                              GPIO_NOPULL,
                              GPIO_SPEED_FREQ_LOW};

    My_HAL_GPIOx_Init(GPIOB, &iniStr4);
    GPIOB->ODR |= (1 << 15); // Set high so it's not pulling the line down

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
    assert(((GPIOB->MODER >> (14*2)) & 0x3) == 0x1); //assert PB14 is in output mode (01)
    assert(((GPIOC->MODER) & 0x3) == 0x1); //assert PC0 is in output mode (01)

    assert(((GPIOB->OTYPER >> 11) & 0x1) == 0x1); //assert PB11 open-drain mode(1)
    assert(((GPIOB->OTYPER >> 13) & 0x1) == 0x1); //assert PB13 open-drain mode(1)
    assert(((GPIOB->OTYPER >> 14) & 0x1) == 0x0); //assert PB14 push-pull mode(0)
    assert(((GPIOC->OTYPER) & 0x1) == 0x0); //assert PC0 push-pull mode(0) 

    assert(((GPIOB->PUPDR >> (13*2)) & 0x3) == 0x0); //assert PB13 is in no pull (00)
    assert(((GPIOB->PUPDR >> (11*2)) & 0x3) == 0x0); //assert PB11 is in no pull (00)
    assert(((GPIOB->AFR[1] >> 12) & 0xF) == 0x1); //assert PB11 is in AF1 mode
    assert(((GPIOB->AFR[1] >> 20) & 0xF) == 0x5); //assert PB13 is in AF5 mode

    assert(((GPIOB->ODR >> 14) & 0x1) == 0x1); //assert PB14 is high
    assert(((GPIOC->ODR) & 0x1) == 0x1); //assert PC0 is high


    I2Cx_Setup(I2C2);
    I2C2->CR1 |= 0x1;   // Re-enable
    sendString(newFlash);
    GyroscopeInit();
  //  WhoAmIRegRead();
    I2C2->CR2 |= (1 << 14);

    uint8_t Xlow;
    uint8_t XHigh;
    int16_t XFull;
    uint8_t Ylow;
    uint8_t YHigh;
    int16_t YFull;
    int8_t XCount = 0;
    int8_t YCount = 0;
    while (1)
    {
      HAL_Delay(100);
      I2C2->ICR |= (1U << 5) | (1U << 4);  // clear STOP and NACK flag

      // Read X-Axis Data (0xA8 is the address of OUT_X_L with MSB SET)
      Init_I2C_Transaction(I2C2, 0, 0x69, 0x1); //Initialize Write Transmit 

      if (writeOp(0xA8, 1) == I2C_NACK) { // Write OUT_X_L Addr
        NACKF_Error("Write 0xA8");
        continue;
      } 
      Init_I2C_Transaction(I2C2, 1, 0x69, 0x4); //Initialize Read Transmit
      if (readOp(&Xlow, 0) == I2C_NACK) { // Attempt to Read
        NACKF_Error("Read Xlow");
        continue;
      } 
      else if (readOp(&XHigh, 0) == I2C_NACK) { // Attempt to Read
        NACKF_Error("Read XHigh");
        continue;
      }
      else if (readOp(&Ylow, 0) == I2C_NACK) { // Attempt to Read
        NACKF_Error("Read Ylow");
        continue;
      }
      else if (readOp(&YHigh, 1) == I2C_NACK) { // Attempt to Read
        NACKF_Error("Read YHigh");
        continue;
      }
      else {
        XFull = (int16_t)((uint16_t)Xlow | ((uint16_t)XHigh << 8));
        YFull = (int16_t)((uint16_t)Ylow | ((uint16_t)YHigh << 8));
        XCount = (XFull > 0 && (XFull > 100 || XFull < 100)) ? XCount + 1 : XCount - 1;
        YCount = (YFull > 0 && (XFull > 100 || XFull < 100)) ? YCount + 1 : YCount - 1;
        char buffer[128];
        sprintf(buffer, "X: %d | Y: %d\r\n", XFull, YFull);
        sendString(buffer);
        I2C2->CR2 |= (1 << 14); // STOP transaction and restart
      }
      if (XCount == 5) {
        XCount = 0;
        GPIOC->ODR |= GPIO_PIN_9;// Heading in Green LED direction
        GPIOC->ODR &= ~GPIO_PIN_8;
        GPIOC->ODR &= ~GPIO_PIN_7;
        GPIOC->ODR &= ~GPIO_PIN_6;
      }
      else if (XCount == -5) {
        XCount = 0;
        GPIOC->ODR |= GPIO_PIN_8;// Heading in Orange LED direction
        GPIOC->ODR &= ~GPIO_PIN_9;
        GPIOC->ODR &= ~GPIO_PIN_7;
        GPIOC->ODR &= ~GPIO_PIN_6;
      }
      if (YCount == 5) {
        YCount = 0;
        GPIOC->ODR |= GPIO_PIN_6; // Heading in BLUE LED direction
        GPIOC->ODR &= ~GPIO_PIN_7;
        GPIOC->ODR &= ~GPIO_PIN_8;
        GPIOC->ODR &= ~GPIO_PIN_9;
      }
      else if (YCount == -5) {
        YCount = 0;
        GPIOC->ODR |= GPIO_PIN_7; // Heading in RED LED direction
        GPIOC->ODR &= ~GPIO_PIN_6;
        GPIOC->ODR &= ~GPIO_PIN_8;
        GPIOC->ODR &= ~GPIO_PIN_9;
      }
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

  I2C_Status writeOp(uint8_t Byte, uint8_t isLastByte) {
 //   sendString(enterW);
    // Wait for TXIS or NACKF Flags set
    while((((I2C2->ISR >> 1) & 0x1) == 0x0) && (((I2C2->ISR >> 4) & 0x1) == 0x0)) {}
//    sendString(pastWhileW);
    if (((I2C2->ISR >> 1) & 0x1) == 0x1) {
      I2C2->TXDR = Byte;
      if (isLastByte) {
 //       sendString(waitingTC);
        while (((I2C2->ISR >> 6) & 0x1) == 0) {} // Wait for TC flag to be set
      }
      // char buffer[64];
      // sprintf(buffer, "Successfully Sent Byte: 0x%02X\r\n", Byte);
      // sendString(buffer);
      return I2C_PASS;
    }
    else {
      return I2C_NACK;
    }
  }

  I2C_Status readOp(uint8_t *Byte, uint8_t isLastByte) {
//    sendString(enterR);
    // Wait for RXNE or NACKF Flags set
    while((((I2C2->ISR >> 2) & 0x1) == 0x0) && (((I2C2->ISR >> 4) & 0x1) == 0x0)) {} 
//    sendString(pastWhileR);
    if (((I2C2->ISR >> 2) & 0x1) == 0x1) {
      *Byte = I2C2->RXDR;
      if (isLastByte) {
//        sendString(waitingTC);
        while (((I2C2->ISR >> 6) & 0x1) == 0) {} // Wait for TC flag to be set
      }
      // char buffer[64];
      // sprintf(buffer, "Successfully Received Byte: 0x%02X\r\n", *Byte);
      // sendString(buffer);
      return I2C_PASS;
    }
    else {
      return I2C_NACK;
    }
  }

  void GyroscopeInit() {
    Init_I2C_Transaction(I2C2, 0, 0x69, 0x2); //Initialize Write Transmit
    if (writeOp(0x20, 0) == I2C_NACK) { // Write CTRL_REG1 ADDR
      NACKF_Error("Write CTRL_REG1 Addr");
      return;
    } 
    if (writeOp(0x0B, 1) == I2C_NACK) { // enable X and Y and Set sensor into Normal mode
      NACKF_Error("Write Gyro Settings");
      return;
    }  
    Init_I2C_Transaction(I2C2, 0, 0x69, 0x1); //Initialize Write Transmit
    if (writeOp(0x20, 1) == I2C_NACK) { // Write CTRL_REG1 ADDR
      NACKF_Error("Write CTRL_REG1 Addr for Read");
      return;
    } 
    Init_I2C_Transaction(I2C2, 1, 0x69, 0x1); //Initialize Read Transmit
    uint8_t CTRL_REG1;
    if (readOp(&CTRL_REG1, 1) == I2C_PASS) { // Attempt to Read
      if (CTRL_REG1 == 0x0B) { // Check if correct bits are enabled
        sendString(SetupPass);
      } else {
        sendString(SetupFail);
      } 
    } else {
      NACKF_Error("readOp CTRL_REG1");
      return;
    }
  }

  void WhoAmIRegRead() {
    Init_I2C_Transaction(I2C2, 0, 0x69, 0x1); //Initialize Write Transmit
    if (writeOp(0x20, 1) == I2C_NACK) { // Write WHO_AM_I Addr
      NACKF_Error("Write WHO_AM_I Addr");
      return;
    } 
    Init_I2C_Transaction(I2C2, 1, 0x69, 0x1); //Initialize Read Transmit
    uint8_t receivedByte;
    if (readOp(&receivedByte, 1) == I2C_PASS) { // Attempt to Read
      if (receivedByte == 0xD3) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // Toggle green LED, PASS
      } else {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggel red LED, fail
      } 
    }
    else {
      NACKF_Error("readOp WHO_AM_I reg");
      return;
    }
  }

  void NACKF_Error(const char *msg) {
    char buffer[64];
    sprintf(buffer, "NACKF flag set for %s\r\n", msg);
    sendString(buffer);
    I2C2->ICR |= (1 << 4); // clear NACK flag
    I2C2->ICR |= (1 << 5);  // clear STOP flag
    I2C2->CR2 |= (1 << 14);// send STOP code to the bus
  }

  char* int_to_str(int num) {
      // 'static' keeps this memory alive for the life of the program
      // 12 bytes is enough for -2147483648 and a null terminator
      static char buffer[12]; 
      int i = 10;
      int is_negative = 0;

      buffer[11] = '\0'; // Null terminator at the very end

      if (num == 0) {
          buffer[i--] = '0';
      } else {
          if (num < 0) {
              is_negative = 1;
              num = -num;
          }
          while (num > 0 && i > 0) {
              buffer[i--] = (num % 10) + '0';
              num /= 10;
          }
          if (is_negative) buffer[i--] = '-';
      }

      // Return the pointer to where the string actually starts
      return &buffer[i + 1];
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
