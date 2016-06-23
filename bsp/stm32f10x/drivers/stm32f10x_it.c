/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
#include <board.h>
#include <rtthread.h>

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void rt_hw_irq(unsigned short intstat);

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while(1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while(1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while(1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

//void SysTick_Handler(void)
//{
//    // definition in boarc.c
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
extern struct rt_device uart_device[];

/*******************************************************************************
* Function Name  : DMA1_Channel2_IRQHandler
* Description    : This function handles DMA1 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//  void DMA1_Channel2_IRQHandler(void)
//  {
//      extern void rt_hw_serial_dma_tx_isr(struct rt_device * device);
//
//      /* enter interrupt */
//      rt_interrupt_enter();
//
//      if(DMA_GetITStatus(DMA1_IT_TC2))
//      {
//          /* transmission complete, invoke serial dma tx isr */
//          rt_hw_serial_dma_tx_isr(&uart_device[2]);
//      }
//
//      /* clear DMA flag */
//      DMA_ClearFlag(DMA1_FLAG_TC2 | DMA1_FLAG_TE2);
//
//      /* leave interrupt */
//      rt_interrupt_leave();
//  }


/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void USART1_IRQHandler(void)
//{
//    extern void rt_hw_serial_isr(struct rt_device * device);
//
//    /* enter interrupt */
//    rt_interrupt_enter();
//
//    rt_hw_serial_isr(&uart_device[0]);
//
//    /* leave interrupt */
//    rt_interrupt_leave();
//}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void USART2_IRQHandler(void)
//{
//    extern void rt_hw_serial_isr(struct rt_device * device);
//
//    /* enter interrupt */
//    rt_interrupt_enter();
//
//    rt_hw_serial_isr(&uart_device[1]);
//
//    /* leave interrupt */
//    rt_interrupt_leave();
//}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void USART3_IRQHandler(void)
//{
//    extern void rt_hw_serial_isr(struct rt_device * device);
//
//    /* enter interrupt */
//    rt_interrupt_enter();
//
//    rt_hw_serial_isr(&uart_device[2]);
//
//    /* leave interrupt */
//    rt_interrupt_leave();
//}
//  #endif
void TAMPER_IRQHandler(void)
{

}
void  RTC_IRQHandler(void)
{

}
void   FLASH_IRQHandler(void)
{

}
void   RCC_IRQHandler(void)
{

}
void  DMA1_Channel1_IRQHandler(void)
{

}



void  DMA1_Channel4_IRQHandler(void)
{

}




void  DMA1_Channel7_IRQHandler(void)
{

}

void WWDG_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_kprintf("WWDG_IRQHandler\n");


    /* leave interrupt */
    rt_interrupt_leave();

}

void CAN1_SCE_IRQHandler(void)
{

}

void  TIM1_BRK_IRQHandler
(void)
{
    rt_hw_irq(TIM1_BRK_IRQn);

}

void  TIM1_UP_IRQHandler
(void)
{
    rt_hw_irq(TIM1_UP_IRQn);

}

void  TIM1_TRG_COM_IRQHandler
(void)
{
    rt_hw_irq(TIM1_TRG_COM_IRQn);

}

void  TIM1_CC_IRQHandler
(void)
{
    rt_hw_irq(TIM1_CC_IRQn);

}

void  TIM2_IRQHandler(void)
{
    rt_hw_irq(TIM2_IRQn);

}

void  TIM3_IRQHandler(void)
{
    rt_hw_irq(TIM3_IRQn);

}


void USBWakeUp_IRQHandler(void)
{

}

void TIM8_BRK_IRQHandler
()
{
}

void   TIM8_UP_IRQHandler
()
{
}

void   TIM8_TRG_COM_IRQHandler
()
{
}

void   TIM8_CC_IRQHandler
()
{
}


void  FSMC_IRQHandler()
{
}

void  TIM5_IRQHandler()
{
    rt_hw_irq(TIM5_IRQn);
}

void  SPI3_IRQHandler()
{
}




void TIM6_IRQHandler()
{
    rt_hw_irq(TIM6_IRQn);
}

void  TIM7_IRQHandler()
{
    rt_hw_irq(TIM7_IRQn);
}

void  DMA2_Channel1_IRQHandler
()
{
}

void   DMA2_Channel2_IRQHandler
()
{
}


void  DMA2_Channel4_5_IRQHandler
()
{
}

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
