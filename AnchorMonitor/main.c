/***************************************************************************
                                   |\
                                   | \
 ==================================   \
 ==================================   /
                                   | /
                                   |/
 _____                     _                      _            
/  ___|                   | |                    | |           
\ `--. _ __   ___  ___  __| | ___  _ __ ___   ___| |_ ___ _ __ 
 `--. \ '_ \ / _ \/ _ \/ _` |/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
/\__/ / |_) |  __/  __/ (_| | (_) | | | | | |  __/ ||  __/ |   
\____/| .__/ \___|\___|\__,_|\___/|_| |_| |_|\___|\__\___|_|   
      | |                                                      
      |_|                                            

=============================================================================

  AUTHOR: Blaz Bogataj 2021
  BOARD : STM32F103C8V "bluepill"
  SWD : PA13-SWDIO, PA14-SWDCK     
  UART : PA2-Tx, PA3-Rx
  I2C :
  TIM2 INPUT CAPTURE :
****************************************************************************/
#include "dbg.h" 
#include "oled.h"
//#include "gyro.h"
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
  Delay(2000000);

  i2cm_init(I2C1, 100000); // PB6:SCL, PB7:SDA for OLED
  SSD1306_init_seq();
  init_input_capture();

  //bb_initSPI(SPI1);// PA:4-SS, 5-CLK, 6-MISO, 7-MOSI for Gyro
  //EXTI0_Config(); // PA.00 EXTI for touch controller, zbrisu ponesrec ??


  /******************* USERSPACE: *************************/

  // display 
    

      
  USART1_PutString(USART2, "   _____                     _                      _            \n\r");
  USART1_PutString(USART2, "  /  ___|                   | |                    | |            \n\r");
  USART1_PutString(USART2, "  \\ `--. _ __   ___  ___  __| | ___  _ __ ___   ___| |_ ___ _ __  \n\r");
  USART1_PutString(USART2, "   `--. \\ '_ \\ / _ \\/ _ \\/ _` |/ _ \\| '_ ` _ \\ / _ \\ __/ _ \\ '__|  \n\r");
  USART1_PutString(USART2, "  /\\__/ / |_) |  __/  __/ (_| | (_) | | | | | |  __/ ||  __/ |    \n\r");
  USART1_PutString(USART2, "  \\____/| .__/ \\___|\\___|\\__,_|\\___/|_| |_| |_|\\___|\\__\\___|_|    \n\r");
  USART1_PutString(USART2, "        | |                                                       \n\r");
  USART1_PutString(USART2, "        |_|                                                       \n\r");
  USART1_PutString(USART2, "__________________________________________________________________\n\r");
  USART1_PutString(USART2, "Set terminal to: Force local echo and Force local line editing \n\r");
  USART1_PutString(USART2, "WDT X 100 ... sets width to 100mm \n\r");
  USART1_PutString(USART2, "UNT 1 ... sets units to m/s \n\r");
  USART1_PutString(USART2, "UNT 0 ... sets units to fps \n\r");


  // print initial screen 
  SSD1306_ON();
  SSD1306_ClearScreen(BLACK);
  SSD1306_Puts("Width:    mm", &Font_7x10, WHITE, 1, 9);
  SSD1306_Puts("Speed: 320", &Font_7x10, WHITE, 1, 28);
  SSD1306_Puts("Units: fps", &Font_7x10, WHITE, 1, 46);
  SSD1306_UpdateScreen();

   
   char wdt[4];

  while(1){

    if(eUART == sDbgMsg.parse){
      extractCommands();
      switch(sCMD.ID){
          case WDT:
            __itoa(sCMD.data,wdt,10);
            SSD1306_Puts(wdt, &Font_7x10, WHITE, 50, 9);
          break;
          case UNT:
            if(sCMD.rw == 1)
              SSD1306_Puts("fps", &Font_7x10, WHITE, 57, 46);
            else 
              SSD1306_Puts("m/s", &Font_7x10, WHITE, 57, 46);
          break;
          default:
            sCMD.data = 0x00;
            sCMD.ID = eReset;
            sCMD.rw = 0; 
          break;
      }
      SSD1306_UpdateScreen();
      reset_dbg_msg();
    }
  }

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

