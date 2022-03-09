/***************************************************************************
			                   /|
			                  / |			 
		                   /  |\
		                  /   | \
		                 /____|__\
~~~~~~~~~~~~~~~~~~~~~~\_____/~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 ____  _     ____  _    ____  ____    _     ____  _     _  _____  ____  ____ 
/  _ \/ \  //   _\/ \ //  _ \/  __\  / \__//  _ \/ \  // \/__ __\/  _ \/  __\
| / \|| |\ ||  /  | |_|| / \||  \/|  | |\/|| / \|| |\ || |  / \  | / \||  \/|
| |-||| | \||  \_ | | || \_/||    /  | |  || \_/|| | \|| |  | |  | \_/||    /
\_/ \|\_/  \\____/\_/ \\____/\_/\_\  \_/  \\____/\_/  \\_/  \_/  \____/\_/\_\

=============================================================================

  AUTHOR: Blaz Bogataj 2021
    STM32F103C8: "bluepill": F1xx Medium-density: 20 KiB SRAM, 64 KiB flash 1Kib Page
  SWD: PA.13-SWDIO, PA.14-SWDCK     
    C Language Version:
    - Type info gcc in bash  
    -std=c90 -std=c99 -std=c11 

  PITFALS: 
    - spi did not read data when probing with osciloscope (1x)
    - spi presluhi med MISO in MOSI in CLK linijo, nedefiniran
    - pr enmu bluepillu je blo treba povezat grounde, true
    - FAKOFF SPI zastavca je povzrocila HardFault, Zdej delay namest flaga

    - If gyro is not working, REFLASH! 
    - if OLED is not working, disconnect all, especially UART!!
  NOTES:
    - in oled init sequence : changed to 0xa5 - Output ignores RAM content
    - EXTI_GenerateSWInterrupt(EXTI_Line0); // simulates interrupt
    - decimal [6] -> [inv bits, +1] -> [-6] in twos complement 


  TODO:
    - proper 3v supply  
****************************************************************************/
#include "dbg.h" 
#include "oled.h"
#include "gyro.h"
#include "fonts.h"
#include "sonar.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "misc.h"



// classic noop delay
void Delay(int value){
       while(value--)
      __asm("nop");
}

// to be removed 
int init_GPIOx_pin (GPIO_TypeDef* portx, uint16_t pin, GPIOMode_TypeDef mode, uint32_t clk_bus){

  GPIO_InitTypeDef   GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( clk_bus , ENABLE);
  GPIO_InitStructure.GPIO_Pin =  pin;
  GPIO_InitStructure.GPIO_Mode = mode;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(portx, &GPIO_InitStructure);

  return 0;
}

// Configure PB.09 in interrupt mode - free
/*
void EXTI9_5_Config(void)
{
  // Init pin 
  init_GPIOx_pin(GPIOB, GPIO_Pin_9, GPIO_Mode_IN_FLOATING, RCC_APB2Periph_GPIOB);

  // Enable AFIO clock 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  // Connect EXTI9 Line to PB.09 pin 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

  // Configure EXTI9 line 
  EXTI_InitTypeDef   EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line9;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI9_5 Interrupt to the lowest priority 
  NVIC_InitTypeDef   NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
*/
// Init input capture TIM3-ch2 on PA7, Conflict with SPI1 
void InCapt_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void init_input_capture(){
   // TIM3 clock enable 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  // init pins
  init_GPIOx_pin(GPIOA, GPIO_Pin_1, GPIO_Mode_IN_FLOATING, RCC_APB2Periph_GPIOA);
  
  // NVIC configuration 
  InCapt_NVIC_Configuration();

  /* TIM2 configuration: PWM Input mode ------------------------
     The external signal is connected to TIM2 CH2 pin (PA.01), 
     The Rising edge is used as active edge,
     The TIM3 CCR2 is used to compute the frequency value 
     The TIM3 CCR1 is used to compute the duty cycle value
  ------------------------------------------------------------ */
  TIM_ICInitTypeDef TIM_ICInitStructure;

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

  /* Select the TIM2 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

  /* Enable the Master/Slave Mode */
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
}




int main(void)
{   

  SysTick_Config (SysTick_LOAD_RELOAD_Msk - 1); // 5Hz ISR
  bb_initSonarLED(); // PB11
  bb_initStatusLED(); // PC13
  bb_initUSART2(); // PA2:tx PA3:rx
  i2cm_init(I2C1, 100000); // PB6:SCL, PB7:SDA for OLED
  Delay(2000000);
  SSD1306_init_seq();
  init_input_capture();

  //bb_initSPI(SPI1);// PA:4-SS, 5-CLK, 6-MISO, 7-MOSI for Gyro
  //EXTI0_Config(); // PA.00 EXTI for touch controller, zbrisu ponesrec ??


  /******************* USERSPACE: *************************/

  // display 
  SSD1306_ON();
  
  USART1_PutString(USART2, "Speedometer app\n\r");


  Delay(2000000);





   SSD1306_ClearScreen(BLACK);
   SSD1306_Puts("lime:", &Font_7x10, WHITE, 1, 28);
   SSD1306_UpdateScreen();
   
  while(1){

    GPIO_ResetBits(GPIOB, GPIO_Pin_11); 
    Delay(40000);
    GPIO_SetBits(GPIOB, GPIO_Pin_11); 
    Delay(40000);

  }



  while(1);       // End of superloop
  return 0; // End of Main 
}           // End of program 
////////////// End of file 

