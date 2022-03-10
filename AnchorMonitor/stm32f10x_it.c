/**
  ******************************************************************************
  * @file    TIM/PWM_Input/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "dbg.h"
#include "oled.h"
#include "stdbool.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"


__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;


void NMI_Handler(void)
{
}


void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}




void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}




void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}




void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}




void DebugMon_Handler(void)
{}




void SVC_Handler(void)
{}




void PendSV_Handler(void)
{}




void SysTick_Handler(void)
{

    display.refresh_sig = 1;

    if(0xff == sStatusLED.blink){
      GPIOC->ODR &= ~(1 << 13);
    }
     else if(sStatusLED.blink){
      GPIOC->ODR ^= 1 << 13;
      sStatusLED.blink--;
    }
    else{
      GPIOC->ODR |= 1 << 13;
    }
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

// void EXTI0_IRQHandler(void)
// {
//   uint8_t pt1x, pt1y, pt2x, pt2y;
//   char touchBuffer[8];
//   // PA0
//   if(EXTI_GetITStatus(EXTI_Line0) != RESET)
//   {
//     //  Delay(100);
//     //  i2cm_Start(I2C1, 0x26,1,1000);
//     //  Delay(5);
//     //  i2cm_ReadBuffAndStop(I2C1,touchBuffer, 8, 1000);
//     //  Delay(100);

//     // USART1_PutString(USART2, (uint8_t*) "hello ext0-pa0\n\r");
//     // USART1_PutString(USART2, (uint8_t*) touchBuffer);



    
//     EXTI_ClearITPendingBit(EXTI_Line0);
//   }
// }

// void EXTI9_5_IRQHandler(void)
// {
//   if(EXTI_GetITStatus(EXTI_Line9) != RESET)
//   {    
//     //USART1_PutString(USART2, (uint8_t*) "hello ext9-pb9\n\r");
//     EXTI_ClearITPendingBit(EXTI_Line9);
//   }
// }


void USART2_IRQHandler(void)
{
  char ch = 0x00;
  
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    // get character
    ch = USART_ReceiveData(USART2);

    // if not busy 
    if(!sDbgMsg.parse){
      // if ch = enter, terminate and send signal
      if(ch == '\r'){
        sDbgMsg.message[sDbgMsg.index] = '\0';
        sDbgMsg.parse = eUART; // cmd id 
        USART1_PutString(USART2, (uint8_t*) "\r\n");
      }
      // save and print back rx ch 
      else{
        sDbgMsg.message[sDbgMsg.index++] = ch;
        USART1_PutChar(USART2, ch); 
      }
      // if buffer overflow, clear buffer
      if(sDbgMsg.index > DBG_MSG_LENGTH - 1){ 
        sDbgMsg.index = 0;
        USART1_PutString(USART2, (uint8_t*) "\r          ");
        USART1_PutString(USART2, (uint8_t*) "          \r");
      }
    }
  }
}

void TIM2_IRQHandler(void)
{

  char itoabuffer[10];
  // Clear TIM3 Capture compare interrupt pending bit 
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Trigger);

  // Get the Input Capture value 
  IC2Value = TIM_GetCapture2(TIM2);

  if (IC2Value != 0)
  {
    // Frequency and Duty cycle computation 
    DutyCycle = (TIM_GetCapture1(TIM2) * 100) / IC2Value;
    Frequency = SystemCoreClock / IC2Value;
    // if((Frequency > 1000) && (Frequency < 1300)){
    //    GPIO_ResetBits(GPIOC, GPIO_Pin_13); 
    // }
    __itoa(Frequency, itoabuffer, 10);
    USART1_PutString(USART2, itoabuffer);
    USART1_PutString(USART2, "\n\r");

    SSD1306_Puts("Speed:", &Font_7x10, WHITE, 1, 28);
    SSD1306_Puts("      ", &Font_7x10, WHITE, 50, 28);
    SSD1306_Puts(itoabuffer, &Font_7x10, WHITE, 50, 28);
    SSD1306_UpdateScreen();



  }
  else
  {
    DutyCycle = 0;
    Frequency = 0;
  }

}



/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
