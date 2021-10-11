/************************************************
 * @sonar.c 
 * 
 * Sonar file for Anchor monitor 
 * 
 * AUTHOR: Blaz Bogataj, 2021 
 ************************************************/
#include "sonar.h"


// Init TIM3 CH4 38kHz PWM output on PB1 - 40khz PWM Sonar gre v system.c/sonar.c 
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
void bb_initSonarLED(){
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // GPIOC port clock enable 
  GPIOB->CRH &= ~GPIO_CRH_CNF11; // 00: General purpose output push-pull
  GPIOB->CRH |= GPIO_CRH_MODE11; // 11: Output mode, max speed 50 MHz.
}
void sonar_send( char hexval, int delay ){

    char pktmask = 0x81;
    char packet = pktmask | (hexval << 2);
    char mask = 0x1, i = 8; //size in bits

    while(i-->0){
        if((mask << i) & packet){
            GPIOB->ODR |= 1 << 11;
        }
        else{
            GPIO_ResetBits(GPIOB, GPIO_Pin_11); 
        }
      Delay(delay);
    }
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); 
}



