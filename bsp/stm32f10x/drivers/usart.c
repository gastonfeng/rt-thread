/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2013-05-13     aozima       update for kehong-lingtai.
 * 2015-01-31     armink       make sure the serial transmit complete in putc()
 * 2016-05-13     armink       add DMA Rx mode
 */

#include<mcu_def.h>
#include "stm32f10x.h"
//	#include "usart.h"
#include "board.h"
#include <rtdevice.h>
#include <model.h>

/* USART1 */
#define UART1_GPIO_TX        GPIO_Pin_9
#define UART1_GPIO_RX        GPIO_Pin_10
#define UART1_GPIO           GPIOA

/* USART2 */
#define UART2_GPIO_TX        GPIO_Pin_2
#define UART2_GPIO_RX        GPIO_Pin_3
#define UART2_GPIO           GPIOA

/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_TX        GPIO_Pin_10
#define UART3_GPIO_RX        GPIO_Pin_11
#define UART3_GPIO           GPIOB

/* USART4 */
#define UART4_GPIO_TX        GPIO_Pin_10
#define UART4_GPIO_RX        GPIO_Pin_11
#define UART4_GPIO           GPIOC

#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))


/* STM32 uart driver */
//struct uart_hw_def
//{
//    USART_TypeDef * *uart_device;
//    IRQn_Type irq;
//    struct stm32_uart_dma {
//        /* dma channel */
//        DMA_Channel_TypeDef *rx_ch;
//        /* dma global flag */
//        uint32_t rx_gl_flag;
//        /* dma irq channel */
//        uint8_t rx_irq_ch;
//        /* last receive index */
//        rt_size_t last_recv_len;
//    } dma;
//    io_def tx_en;   //发送允许
//    io_def rx_en;   //接收允许
//};

//static void DMA_Configuration(struct rt_serial_device *serial);

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct uart_hw_def* uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct uart_hw_def *)serial->parent.user_data;

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8)
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
    else if (cfg->data_bits == DATA_BITS_9)
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }

    if (cfg->stop_bits == STOP_BITS_1)
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    }
    else if (cfg->stop_bits == STOP_BITS_2)
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }

    if (cfg->parity == RT_PARITY_NONE)
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    else if (cfg->parity == RT_PARITY_ODD)
    {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    }
    else if (cfg->parity == RT_PARITY_EVEN)
    {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(((USART_TypeDef *)uart->uart_device), &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(((USART_TypeDef *)uart->uart_device), ENABLE);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct uart_hw_def* uart;
    rt_uint32_t ctrl_arg = (rt_uint32_t)(arg);

    RT_ASSERT(serial != RT_NULL);
    uart = (struct uart_hw_def *)serial->parent.user_data;

    switch (cmd)
    {
        /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        serial->parent.flag&=~RT_DEVICE_FLAG_ACTIVATED;
        UART_DISABLE_IRQ(uart->irq);
        /* disable interrupt */
        USART_ITConfig(((USART_TypeDef *)uart->uart_device), USART_IT_RXNE, DISABLE);
        break;
        /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        UART_ENABLE_IRQ(uart->irq);
        serial->parent.flag|=RT_DEVICE_FLAG_ACTIVATED;
        /* enable interrupt */
        USART_ITConfig(((USART_TypeDef *)uart->uart_device), USART_IT_RXNE, ENABLE);
        break;
        /* USART config */
    case RT_DEVICE_CTRL_CONFIG :
////        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) {
////            DMA_Configuration(serial);
////        }
        break;
    }
    return RT_EOK;
}

static int stm32_putc(struct rt_serial_device *serial, char c)
{
    struct uart_hw_def* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct uart_hw_def *)serial->parent.user_data;

    while (!(((USART_TypeDef *)uart->uart_device)->SR & USART_FLAG_TXE));
    ((USART_TypeDef *)uart->uart_device)->DR = c;

    return 1;
}

static int stm32_getc(struct rt_serial_device *serial)
{
    int ch;
    struct uart_hw_def* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct uart_hw_def *)serial->parent.user_data;

    ch = -1;
    if (((USART_TypeDef *)uart->uart_device)->SR & USART_FLAG_RXNE)
    {
        ch = ((USART_TypeDef *)uart->uart_device)->DR & 0xff;
    }

    return ch;
}

#if dma
/**
 * Serial port receive idle process. This need add to uart idle ISR.
 *
 * @param serial serial device
 */
static void dma_uart_rx_idle_isr(struct rt_serial_device *serial)
{
    struct uart_hw_def *uart = (struct uart_hw_def *) serial->parent.user_data;
    rt_size_t recv_total_len, recv_len;
    /* disable dma, stop receive data */
    DMA_Cmd(uart->dma.rx_ch, DISABLE);

    recv_total_len = serial->config.bufsz - DMA_GetCurrDataCounter(uart->dma.rx_ch);
    if (recv_total_len > uart->dma.last_recv_len)
    {
        recv_len = recv_total_len - uart->dma.last_recv_len;
    }
    else
    {
        recv_len = recv_total_len;
    }
    uart->dma.last_recv_len = recv_total_len;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    /* read a data for clear receive idle interrupt flag */
    USART_ReceiveData(((USART_TypeDef *)uart->uart_device));
    DMA_ClearFlag(uart->dma.rx_gl_flag);
    DMA_Cmd(uart->dma.rx_ch, ENABLE);
}

/**
 * DMA receive done process. This need add to DMA receive done ISR.
 *
 * @param serial serial device
 */
static void dma_rx_done_isr(struct rt_serial_device *serial)
{
    struct uart_hw_def *uart = (struct uart_hw_def *) serial->parent.user_data;
    rt_size_t recv_total_len, recv_len;
    /* disable dma, stop receive data */
    DMA_Cmd(uart->dma.rx_ch, DISABLE);

    recv_total_len = serial->config.bufsz - DMA_GetCurrDataCounter(uart->dma.rx_ch);
    if (recv_total_len > uart->dma.last_recv_len)
    {
        recv_len = recv_total_len - uart->dma.last_recv_len;
    }
    else
    {
        recv_len = recv_total_len;
    }
    uart->dma.last_recv_len = recv_total_len;

    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));

    DMA_ClearFlag(uart->dma.rx_gl_flag);
    /* reload */
    DMA_SetCurrDataCounter(uart->dma.rx_ch, serial->config.bufsz);
    DMA_Cmd(uart->dma.rx_ch, ENABLE);
}

#endif
/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial)
{
    struct uart_hw_def *uart = (struct uart_hw_def *) serial->parent.user_data;

    RT_ASSERT(uart != RT_NULL);

    if(USART_GetITStatus(((USART_TypeDef *)uart->uart_device), USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(((USART_TypeDef *)uart->uart_device), USART_IT_RXNE);
    }
    if(USART_GetITStatus(((USART_TypeDef *)uart->uart_device), USART_IT_IDLE) != RESET)
    {
//        dma_uart_rx_idle_isr(serial);
    }
    if (USART_GetITStatus(((USART_TypeDef *)uart->uart_device), USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(((USART_TypeDef *)uart->uart_device), USART_IT_TC);
    }
    if (USART_GetFlagStatus(((USART_TypeDef *)uart->uart_device), USART_FLAG_ORE) == SET)
    {
        stm32_getc(serial);
        USART_ClearITPendingBit(((USART_TypeDef *)uart->uart_device), USART_FLAG_ORE);
    }
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    stm32_putc,
    stm32_getc,
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */

struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

//    dma_rx_done_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART2 device driver structure */

struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

//    dma_rx_done_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
/* UART3 device driver structure */

struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

//    dma_rx_done_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
/* UART4 device driver structure */

struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

//    dma_rx_done_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART4 */


struct rt_serial_device serial5;

void UART5_IRQHandler()
{
    rt_interrupt_enter();

    uart_isr(&serial5);

    /* leave interrupt */
    rt_interrupt_leave();
}

static void RCC_Configuration(void)
{
#if defined(RT_USING_UART1)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif /* RT_USING_UART4 */
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#if defined(RT_USING_UART1)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_RX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART4 */
}

static void NVIC_Configuration(const struct uart_hw_def* uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#if dma
static void DMA_Configuration(struct rt_serial_device *serial)
{
    struct uart_hw_def *uart = (struct uart_hw_def *) serial->parent.user_data;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* enable transmit idle interrupt */
    USART_ITConfig(((USART_TypeDef *)uart->uart_device), USART_IT_IDLE , ENABLE);

    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    /* rx dma config */
    DMA_DeInit(uart->dma.rx_ch);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(((USART_TypeDef *)uart->uart_device)->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) rx_fifo->buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = serial->config.bufsz;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(uart->dma.rx_ch, &DMA_InitStructure);
    DMA_ClearFlag(uart->dma.rx_gl_flag);
    DMA_ITConfig(uart->dma.rx_ch, DMA_IT_TC, ENABLE);
    USART_DMACmd(((USART_TypeDef *)uart->uart_device), USART_DMAReq_Rx, ENABLE);
    DMA_Cmd(uart->dma.rx_ch, ENABLE);

    /* rx dma interrupt config */
    NVIC_InitStructure.NVIC_IRQChannel = uart->dma.rx_irq_ch;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
#endif

void rt_hw_usart_init(struct rt_serial_device *serial,const char *dev_name,unsigned int flag,const struct uart_hw_def *uart)
{
    USART_ClockInitTypeDef USART_ClockInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
//      struct uart_hw_def* uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    USART_TypeDef *uart_device=((USART_TypeDef *)uart->uart_device);

//      RCC_Configuration();
//      GPIO_Configuration();


    switch((int)uart_device)
    {
    case(int)USART1:
        /* uart init */
        /* Enable USART1 and GPIOA clocks */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
        USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
        USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
        USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
        USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
        USART_ClockInit(uart_device, &USART_ClockInitStructure);


        /* Configure USART1 Rx (PA.10) as input floating */
        GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

        /* Configure USART1 Tx (PA.09) as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);


        break;
    case(int)USART2:


#if (defined(STM32F10X_LD) || defined(STM32F10X_MD) || defined(STM32F10X_CL))
        /* Enable AFIO and GPIOD clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);
        USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
        USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
        USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
        USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
        USART_ClockInit(uart_device, &USART_ClockInitStructure);

        /* Enable the USART2 Pins Software Remapping */
        GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#else
        /* Enable AFIO and GPIOA clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
#endif

        /* Enable USART2 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

        /* Configure USART2 Rx as input floating */
        GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

        /* Configure USART2 Tx as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

        break;
    case(int)USART3:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOB, ENABLE);
        /* Enable USART3 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
        USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
        USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
        USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
        USART_ClockInit(uart_device, &USART_ClockInitStructure);

        /* DMA clock enable */
        //      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        /* Configure USART3 Rx as input floating */
        GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

        /* Configure USART3 Tx as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

        break;
    case(int)UART4:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOC, ENABLE);
        /* Enable USART3 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

        /* Configure USART3 Rx as input floating */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        /* Configure USART3 Tx as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        break;
    case(int)UART5:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
        /* Enable USART3 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);


        /* Configure USART3 Rx as input floating */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* Configure USART3 Tx as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        break;
    }
    NVIC_Configuration(uart);

    serial->ops    = &stm32_uart_ops;
    serial->config = config;
//   dev->uart=uart;

    /* register UART1 device */
    rt_hw_serial_register(serial, dev_name,RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX|flag,(void *)uart);
    serial->lock=rt_sem_create("lock", 1, RT_IPC_FLAG_FIFO);
}
void rs485_to_tx(void *dev)
{
    struct rt_serial_device *serial=(struct rt_serial_device *)dev;
    struct uart_hw_def *hw=(struct uart_hw_def *)serial->parent.user_data;
    hw_gpio_out_init(hw->tx_en);
    hw_gpio_out(hw->tx_en, 1);
    rt_thread_delay(RT_TICK_PER_SECOND / 1000);
}

void rs422_to_tx(void *dev)
{
    struct rt_serial_device *serial=(struct rt_serial_device *)dev;
    struct uart_hw_def *hw=(struct uart_hw_def *)serial->parent.user_data;
    hw_gpio_out_init(hw->tx_en);
    hw_gpio_out(hw->tx_en, 1);
    hw_gpio_out_init(hw->rx_en);
    hw_gpio_out(hw->rx_en, 1);
    rt_thread_delay(RT_TICK_PER_SECOND / 1000);
}

void rs485_to_rx(void *dev)
{
    struct rt_serial_device *serial=(struct rt_serial_device *)dev;
    struct uart_hw_def *hw=(struct uart_hw_def *)serial->parent.user_data;
#ifdef STM32
    while (!(((USART_TypeDef *)hw->uart_device)->SR & USART_FLAG_TXE));
#else
    while(!(hw->uart_device->ustat & USTAT_EMPTY));
#endif
    rt_thread_delay(RT_TICK_PER_SECOND / 1000);
    hw_gpio_out(hw->tx_en, 0);
}

void rs422_to_rx(void *dev)
{
    struct rt_serial_device *serial=(struct rt_serial_device *)dev;
    struct uart_hw_def *hw=(struct uart_hw_def *)serial->parent.user_data;
#ifdef STM32
    while (!(((USART_TypeDef *)hw->uart_device)->SR & USART_FLAG_TXE));
#else
    while(!(hw->uart_device->ustat & USTAT_EMPTY));
#endif
    rt_thread_delay(2);
    hw_gpio_out(hw->tx_en, 0);
    hw_gpio_out(hw->rx_en, 0);
}

