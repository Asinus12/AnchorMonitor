#include "bb_init.h"




/*********************************************************/
/*********************************************************/
/*********************************************************/
/* Init LED on PC13 */
void initStatusLED(){
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // GPIOC port clock enable 
  GPIOC->CRH &= ~GPIO_CRH_CNF13; // 00: General purpose output push-pull
  GPIOC->CRH |= GPIO_CRH_MODE13; // 11: Output mode, max speed 50 MHz.
}


/*********************************************************/
/*********************************************************/
/*********************************************************/
/* Init sonar indicator led on PB11 */ 
void initSonarLED(){
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // GPIOC port clock enable 
  GPIOB->CRH &= ~GPIO_CRH_CNF11; // 00: General purpose output push-pull
  GPIOB->CRH |= GPIO_CRH_MODE11; // 11: Output mode, max speed 50 MHz.
}


/*********************************************************/
/*********************************************************/
/*********************************************************/
/* Init USART2 on PA2:tx PA3:RX */
void initUsart2(){
     
  // USART1 PA09 : TX, PA10 : RX conflict spi  
  // USART2 PA02 : TX, PA03 : RX interrupt  
  // USART3 PB10 : TX, PB11 : RX  
  
  /* USART2_Init */
  uint32_t tmpreg = 0x00, apbclock = 0x00;
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  RCC_ClocksTypeDef RCC_ClocksStatus;

  // enable clocks 
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; 

  // pa2 tx 
  GPIOA->CRL |= GPIO_CRL_CNF2_1; // pushpull
  GPIOA->CRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_MODE2_0; //output mode 50mhz

  // pa3 rx 
  GPIOA->CRL |= GPIO_CRL_CNF3_0; // floating
  GPIOA->CRL |= 0x00000000; // input 

  // configure CR1 register
  USART2->CR1 &= 0xE9F3; // CR1_CLEAR_Mask;
  USART2->CR1 |= USART_WordLength_8b|USART_Parity_No|USART_Mode_Rx | USART_Mode_Tx;

  // configure CR2 register
  USART2->CR2 &= 0xCFFF; // CR2_STOP_CLEAR_Mask;
  USART2->CR2 |= USART_StopBits_1;

  // configure CR3 register
  USART2->CR3 &= 0xFCFF;
  USART2->CR3 |= USART_HardwareFlowControl_None;

  // configure baud rate register 
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  apbclock = RCC_ClocksStatus.PCLK1_Frequency; // if usart2, else .PCLK2_
  integerdivider = ((25 * apbclock) / (4 * (9600))); // 4 normal, 2 oversampling
  tmpreg = (integerdivider / 100) << 4;
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x0F); // 0x0F normal, 0x07 oversampling

  // Write to baudrate register
  USART2->BRR = (uint16_t)tmpreg;

  // enable usart (CR1_UE_Set)
  USART2->CR1 |= 0x2000; 

  // configure interrupts
  *(__IO uint32_t*)0x4000440C  |= (((uint32_t)0x01) << (USART_IT_RXNE & 0x001F)); // 0x001f = ITMASK

  // enable interrupts
  NVIC->ISER[((uint32_t)(USART2_IRQn) >> 5)] = (1 << ((uint32_t)(USART2_IRQn) & 0x1F)); /* enable interrupt */


}


/*********************************************************/
/*********************************************************/
/*********************************************************/
/* Init TIM3 CH4 38kHz PWM output on PB1 - 40khz PWM Sonar */ 
void initPWMOutput(){

  /***********************************************************/
  /**** INIT TIM3_CHANNEL_4 PWM OUTPUT on PB1 ****************/
  /***********************************************************/

  // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 

  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

  // init_GPIOx_pin(GPIOB, GPIO_Pin_1, GPIO_Mode_AF_PP, RCC_APB2Periph_GPIOB);

  /* -----------------------------------------------------------------------
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
  ----------------------------------------------------------------------- */

  // uint16_t prescaler = (SystemCoreClock / 24000000) - 1;
  
  // // Time base configuration 
  // TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  // TIM_TimeBaseStructure.TIM_Period = 600;
  // TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  // TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  // TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  // TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  // // PWM1 Mode configuration: Channel4 on PB1 
  // TIM_OCInitTypeDef  TIM_OCInitStructure;
  // TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  // TIM_OCInitStructure.TIM_Pulse = 300;
  // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  // TIM_OC4Init(TIM3, &TIM_OCInitStructure); // TIM_OC2Init, TIM_OC3Init

  // TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); // TIM_OC2PreloadConfig, ..

  // TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  // TIM_Cmd(TIM3, DISABLE);
}


/*********************************************************/
/*********************************************************/
/*********************************************************/
/* Init SPI1 on PA7-MOSI, PA6-MISO, PA5-CLK, PA4-SoftSlave */
void initSPI(){

    /* SPI clock enabled */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    
    /* GPIOA port clock enable */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* PA4 software slave select */
    GPIOA->CRL &= ~GPIO_CRL_CNF4; // 00: General purpose output push-pull
    GPIOA->CRL |= GPIO_CRL_MODE4; // 11: Output mode, max speed 50 MHz.

    /* PA5 SPI clock */
    GPIOA->CRL &= ~GPIO_CRL_CNF5;
    GPIOA->CRL |= GPIO_CRL_CNF5_1; // 10: Alternate function output Push-pull
    GPIOA->CRL |= GPIO_CRL_MODE5; // 11: Output mode, max speed 50 MHz.

    /* PA6 SPI MISO */
    GPIOA->CRL &= ~GPIO_CRL_CNF6;
    GPIOA->CRL |= GPIO_CRL_CNF6_1; // 10: Alternate function output Push-pull
    GPIOA->CRL |= GPIO_CRL_MODE6; // 11: Output mode, max speed 50 MHz.

    /* PA7 SPI MOSI */
    GPIOA->CRL &= ~GPIO_CRL_CNF7;
    GPIOA->CRL |= GPIO_CRL_CNF7_1; // 10: Alternate function output Push-pull
    GPIOA->CRL |= GPIO_CRL_MODE7; // 11: Output mode, max speed 50 MHz.

    /* Remap AFIO */
    AFIO->MAPR &= ~AFIO_MAPR_SPI1_REMAP; // 0: No remap (NSS/PA4, SCK/PA5, MISO/PA6, MOSI/PA7)


    // CR2 reg bit 2(SSOE): 0- SS output is disabled in master mode and the cell can work in multimaster configuration
    //                      1- SS output is enabled in master mode and when the cell is enabled, cell can NOT work in multimaster configuration
    SPI1->CR2 |= SPI_CR2_SSOE; // SS output enable

    // CLEAR CR1
    //SPIx->CR1 = 0; // Clear unwanted values from register

    // CR1 reg bit 11(DFF): 0-8bit frame format, 1-16bit frame format
    SPI1->CR1 |= SPI_CR1_DFF; // 8-bit data frame format, SPI_CR1_DFF;  // 16-bit data frame format
    
    // CR1 reg bit 2(MSTR): 0-Slave
    //                      1-Master
    SPI1->CR1 |= SPI_CR1_MSTR; // Master device

    // CR1 reg bit 1 (CPOL): 0-clk to 0 when idle
    //                       1-clk to 1 when idle 
    SPI1->CR1 |= SPI_CR1_CPOL; // Clock polarity: high when idle

    //CR1 reg bit 0 (CPHA): 0-The first clock transition is the first data capture edge
    //                      1-The second clock transition is the first data capture edge
    SPI1->CR1 |= SPI_CR1_CPHA; // Capture data on 2nd edge (rising edge)

    // Bits 5:3 BR[2:0]: Baud rate control
    // 000: fPCLK/2
    // 001: fPCLK/4
    // 010: fPCLK/8
    // 011: fPCLK/16
    // 100: fPCLK/32
    // 101: fPCLK/64
    // 110: fPCLK/128
    // 111: fPCLK/256

    //SPIx->CR1 |= 0 << SPI_CR1_BR_1|SPI_CR1_BR_; // precekiri
    SPI1->CR1 |= SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_SPE; // Peripheral enabled

        
    
}


