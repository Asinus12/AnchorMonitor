/************************************************
 * @dbg.c 
 * 
 * Debugging file for Anchor monitor 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "dbg.h"





// Initializing functions 
void bb_initStatusLED(){
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // GPIOC port clock enable 
  GPIOC->CRH &= ~GPIO_CRH_CNF13; // 00: General purpose output push-pull
  GPIOC->CRH |= GPIO_CRH_MODE13; // 11: Output mode, max speed 50 MHz.

  GPIOC->ODR |= (1 << 13); // set bit, inv logic

}
void bb_initUSART2(){
     
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
void USART1_Init(void) // conflicting with spi
{
  
    USART_InitTypeDef usart1_init_struct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO,  ENABLE);
    /* Init serial debugging on USART1 (PA9:TX, PA10:RX interrupt) */
    init_GPIOx_pin(GPIOA, GPIO_Pin_9, GPIO_Mode_AF_PP, RCC_APB2Periph_GPIOA);
    init_GPIOx_pin(GPIOA, GPIO_Pin_10, GPIO_Mode_IN_FLOATING, RCC_APB2Periph_GPIOA);

    usart1_init_struct.USART_BaudRate = 9600;
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;
    usart1_init_struct.USART_StopBits = USART_StopBits_1;
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart1_init_struct);

    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);

    USART1_PutString(USART1,(uint8_t*) "Init USART1: PA9-tx, PA10-rx\n\r");
}
void USART2_Init(void)
{
    
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 ,  ENABLE);
    // init_GPIOx_pin(GPIOA, GPIO_Pin_2, GPIO_Mode_AF_PP, RCC_APB2Periph_GPIOA);
    // init_GPIOx_pin(GPIOA, GPIO_Pin_3, GPIO_Mode_IN_FLOATING, RCC_APB2Periph_GPIOA);
    // PA9-tx, PA10-rx USART1 dela
    // PB6-tx, PB7-rx USART1 remapped
    // PB10-tx PB11-rx USART3
    // PA2-tx, PA3-rx
   
    USART_InitTypeDef usart1_init_struct;
    usart1_init_struct.USART_BaudRate = 9600;
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;
    usart1_init_struct.USART_StopBits = USART_StopBits_1;
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usart1_init_struct);

    USART_Cmd(USART2, ENABLE);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART2_IRQn);

}
void USART3_Init(void)
{
    // USART1: PB6 tx1, PB7 rx1

    USART_InitTypeDef usart1_init_struct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 ,  ENABLE);

    init_GPIOx_pin(GPIOB, GPIO_Pin_10, GPIO_Mode_AF_PP, RCC_APB2Periph_GPIOB);
    init_GPIOx_pin(GPIOB, GPIO_Pin_11, GPIO_Mode_IN_FLOATING, RCC_APB2Periph_GPIOB);


    // PA9-tx, PA10-rx USART1 dela
    // PB6-tx, PB7-rx USART1 remapped
    // PB10-tx PB11-rx USART3
    // PA2-tx, PA3-rx
   
    usart1_init_struct.USART_BaudRate = 9600;
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;
    usart1_init_struct.USART_StopBits = USART_StopBits_1;
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usart1_init_struct);

    USART_Cmd(USART3, ENABLE);

    // USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    // NVIC_EnableIRQ(USART2_IRQn);

    USART1_PutString(USART3, (uint8_t*) "Init USART3 \n\r");
}

// tx rx functions 
void USART1_PutChar(USART_TypeDef* USARTx, uint8_t ch)
{
  while(!(USARTx->SR & USART_SR_TXE));
  USARTx->DR = ch;
}
void USART1_PutString(USART_TypeDef* USARTx, uint8_t * str)
{
  while(*str != 0)
  {
    USART1_PutChar(USARTx, *str);
    str++;
  }
}

// print and draw functions 
void terminalPrintLogo(){
  USART1_PutString(USART2, "\n\r==================================================\n\r");
  USART1_PutString(USART2, "\n\r                      /|");
  USART1_PutString(USART2, "\n\r                     / |");
  USART1_PutString(USART2, "\n\r                    /  |\\");
  USART1_PutString(USART2, "\n\r                   /   | \\");
  USART1_PutString(USART2, "\n\r                  /____|__\\");
  USART1_PutString(USART2, "\n\r~~~~~~~~~~~~~~~~~~~\\_____/~~~~~~~~~~~~~~~~~~~~~~~\n\r");
  USART1_PutString(USART2, "\n\r Anchor Monitor (R)                v1.0  2020     \n\r");
  USART1_PutString(USART2, "\n\r__________________________________________________");
  USART1_PutString(USART2, "\n\r==================================================");
  USART1_PutString(USART2, "\n\r");
}
void terminalPrintCommands(){
  USART1_PutString(USART2, "\n\r********************");
  USART1_PutString(USART2, "\n\r***** COMMANDS *****");
  USART1_PutString(USART2, "\n\r********************");
  USART1_PutString(USART2, "\n\rLED W 08 .......... Blinks led 8/2 times, 0xFF error");
  USART1_PutString(USART2, "\n\rSON W C ............... send pattern C(1010) via sonar");
  USART1_PutString(USART2, "\n\rSPI R 0F ........... returns value(0x6a) at WHO AM I");
  USART1_PutString(USART2, "\n\rSON W FF ................. sends FF via sonar (PB11)");
  USART1_PutString(USART2, "\n\r\n\rEnter command ...\n\r");
  // S:start, P:stop, W:write, R:read, N:number_of_times, D:data
}
void terminalPrintRegisters(){
    /* GPIO registers */
  // bitPrint2((uint32_t*) 0x4001100C, " GPIOC ODR after set");   
  // bitPrint2((uint32_t*) 0x40021000, " RCC_CR");
  // bitPrint2((uint32_t*) 0x40021004, " RCC_CFGR");
  // bitPrint2((uint32_t*) 0x4002101C, " RCC_APB1ENR");
  // bitPrint2((uint32_t*) 0x40021018, " RCC_APB2ENR");
  // bitPrint2((uint32_t*) 0x40021014, " RCC_AHBENR");
}
void BitPrint(volatile uint32_t* x, char* comment ){

    int i=0;

    for (i = BIT_PRINT_REG_SIZE - 1; i >= 0; i--)
    {
       USART1_PutChar(USART2, (uint8_t) *x & (1u << i) ? '1' : '0');
       if((i==28) || (i==24) || (i==20) || (i==16) || (i==12) || (i==8) || (i==4))
      USART1_PutChar(USART2, (uint8_t)'.');
    }

    USART1_PutString(USART2, (uint8_t*) comment);
    USART1_PutChar(USART2, (uint8_t) '\n');
    USART1_PutChar(USART2, (uint8_t) '\r');
}

// data functions
int extractCommand(const char* buffer, int i){

    // Command format: AAA B CC
    // AAA -  ids: STM, SPI, LED, SON, GYR
    // B -  R:x W:y
    // CC - uint8 data in hex (aa,ff,..)

    int noerr = 1; // no errors
    char* p = CmdID_names[i];
    char* b = buffer;

    while(*p != '\0') {
        if(*p++ != *b++){
            noerr = 0; // error!
        }
    }
    if(noerr){
        sCMD.ID =i;
        // skip presledek
        b++;
        // get RW 
        if(*b == 'R') sCMD.rw = 1;
        if(*b == 'W') sCMD.rw = 0;
        // skip presledek 
        b++;
        // 0xXY
        sCMD.data = (int) strtol(b, NULL, 10);
    }
    return 0;

  
    
}
int extractCommands(){
    int i =0;
    while(i < COUNT){
        extractCommand(&sDbgMsg.message,i++);
    }
    
}
int checkCommand( char* buffer, char *original){
    int retval = 1; // ok 

    
    while(*original != '\0') {
        if(*original++ != *buffer++){
            retval = 0; //error
        }
    }
    return retval;
}
void reset_dbg_msg(){
    sDbgMsg.message[0] = '\0';
    sDbgMsg.parse = eReset;
    sDbgMsg.index = 0;
}