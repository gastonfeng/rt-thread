#include<mcu_def.h>
#include <model.h>
#include <s3c2416.h>
#include <rthw.h>
#include <serial.h>

#define USTAT_RCV_READY   0x01    /* receive data ready */
#define USTAT_TXB_EMPTY   0x02    /* tx buffer empty */

#define UART0   ((uartport *)&U0BASE)
#define UART1   ((uartport *)&U1BASE)
#define UART2   ((uartport *)&U2BASE)

uart_device_t com_1,com_2,com_3;
#define INTUART2      (15)
#define INTLCD        (16)
#define INTDMA0       (17)
#define INTDMA1       (18)
#define INTDMA2       (19)
#define INTDMA3       (20)
#define INTSDI        (21)
#define INTSPI0       (22)
#define INTUART1      (23)
//#define INTNOTUSED24  (24)
#define INTNIC  (24)
#define INTUSBD       (25)
#define INTUSBH       (26)
#define INTIIC        (27)
#define INTUART0      (28)

#define BIT_SUB_TXD2  (0x1<<7)
#define BIT_SUB_RXD2  (0x1<<6)
#define BIT_SUB_ERR1  (0x1<<5)
#define BIT_SUB_TXD1  (0x1<<4)
#define BIT_SUB_RXD1  (0x1<<3)
#define BIT_SUB_ERR0  (0x1<<2)
#define BIT_SUB_TXD0  (0x1<<1)
#define BIT_SUB_RXD0  (0x1<<0)

extern unsigned int PCLK;

static rt_err_t s3c2440_configure(struct uart_device *serial, struct serial_configure *cfg)
{
    struct uart_hw_def* uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct uart_hw_def *)serial->parent.user_data;
    uartport *uart_device=(uartport *)uart->uart_device;
    uart_device->UFCON = 0x0;
    /* disable the flow control */
    uart_device->UMCON = 0x0;
    /* Normal,No parity,1 stop,8 bit */
    switch (cfg->data_bits)
    {
    case DATA_BITS_5:
        uart_device->ULCON &= ~0x3;
        uart_device->ULCON |= 0x0;
        break;
    case DATA_BITS_6:
        uart_device->ULCON &= ~0x3;
        uart_device->ULCON |= 0x1;
        break;
    case DATA_BITS_7:
        uart_device->ULCON &= ~0x3;
        uart_device->ULCON |= 0x2;
        break;
    case DATA_BITS_8:
    default:
        uart_device->ULCON &= ~0x3;
        uart_device->ULCON |= 0x3;
        break;
    }

    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        uart_device->ULCON &= ~0x4;
        uart_device->ULCON |= 0x0;
        break;
    case STOP_BITS_2:
    default:
        uart_device->ULCON &= ~0x4;
        uart_device->ULCON |= 0x4;
        break;
    }

    switch (cfg->parity)
    {
    case RT_PARITY_ODD:
        uart_device->ULCON &= ~0x38;
        uart_device->ULCON |= 0x20;
        break;
    case RT_PARITY_EVEN:
        uart_device->ULCON &= ~0x38;
        uart_device->ULCON |= 0x28;
        break;
    default:
        uart_device->ULCON &= ~0x38;
        uart_device->ULCON |= 0x0;
        break;
    }
    /*
     * tx=level,rx=edge,disable timeout int.,enable rx error int.,
     * normal,interrupt or polling
     */
    uart_device->UCON = (1<<8)  | (1<<7) | (1<<2) | (1<<0);;
    /* Set uart0 bps */
    uart_device->UBRDIV = (rt_int32_t)(PCLK/ (cfg->baud_rate* 16)) - 1;
    uart_device->UDIVSLOT = 0x0888;

    delay_us(100);

    /* output PCLK to UART0/1, PWMTIMER */
    switch((int)uart_device)
    {
    case(int)0x50000000:

//            CLKCON |= 1<<(10);
        /* UART0 port configure */
        GPHCON_REG &= ~(0x0f);
        GPHCON_REG |= (0x02 << 0 | 0x02 << 2);	   ///Enable TXD0 RXD0,GPH0-1
        GPHPU_REG &=~ 0x3;
        /* install uart0 isr */
        break;
    case(int)0x50004000:
//            CLKCON |= 1<<(11);
        /* UART0 port configure */
        GPHCON_REG &= ~(0x0f0);
        GPHCON_REG |= (0x02 << 4 | 0x02 << 6);		//GPH2-3
        /* PULLUP is disable */
        GPHPU_REG &=~ 0xc;
        /* install uart0 isr */
        break;
    case(int)0x50008000:

//            CLKCON |= 1<<(12);
        /* UART0 port configure */
        GPHCON_REG &= ~(0x0f00);		//GPH4-5
        GPHCON_REG |= (0x02 << 8 | 0x02 << 10);
        /* PULLUP is disable */
        GPHPU_REG &= ~0x30;
        /* install uart0 isr */
        break;
    }
    return RT_EOK;
}

static rt_err_t s3c2440_control(struct uart_device *serial, int cmd, void *arg)
{
    struct uart_hw_def* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct uart_hw_def *)serial->parent.user_data;
    uartport *uart_device=(uartport *)uart->uart_device;

    switch(cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        serial->parent.flag&=~RT_DEVICE_FLAG_ACTIVATED;
        rt_hw_interrupt_mask(uart->irq);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        rt_hw_interrupt_umask(uart->irq);
        serial->parent.flag|=RT_DEVICE_FLAG_ACTIVATED;
//        if((int)uart_device==0x50008000)
//            chk_uart2();
        break;
    }

    return RT_EOK;
}

static int s3c2440_putc(struct uart_device *serial, char c)
{
    struct uart_hw_def* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct uart_hw_def *)serial->parent.user_data;
    uartport *uart_device=(uartport *)uart->uart_device;

    while(!(uart_device->UTRSTAT & USTAT_TXB_EMPTY));
    uart_device->UTXH = (c & 0xFF);

    return 1;
}

static int s3c2440_getc(struct uart_device *serial)
{
    int ch;
    struct uart_hw_def* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct uart_hw_def *)serial->parent.user_data;
    uartport *uart_device=(uartport *)uart->uart_device;

    ch = -1;
    if(uart_device->UTRSTAT & USTAT_RCV_READY)
    {
        ch = uart_device->URXH & 0xff;
    }

    return ch;
}

static const struct rt_uart_ops s3c2440_uart_ops =
{
    s3c2440_configure,
    s3c2440_control,
    s3c2440_putc,
    s3c2440_getc,
};

/**
 * This function will handle serial
 */
static void rt_serial0_handler(int vector, void *param)
{
//	    INTSUBMSK_REG |= (BIT_SUB_RXD0);
    uart_device_t *uart=param;

    rt_hw_serial_isr(uart,RT_SERIAL_EVENT_RX_IND);

    SUBSRCPND_REG |= BIT_SUB_RXD0;

    /* Unmask sub interrupt (RXD0) */
//	    INTSUBMSK_REG  &= ~(BIT_SUB_RXD0);
}

/**
 * This function will handle serial
 */
static void rt_serial1_handler(int vector, void *param)
{
//	    INTSUBMSK_REG |= (BIT_SUB_RXD1);
    uart_device_t *uart=param;

    rt_hw_serial_isr(uart,RT_SERIAL_EVENT_RX_IND);

    SUBSRCPND_REG |= BIT_SUB_RXD1;

    /* Unmask sub interrupt (RXD0) */
//	    INTSUBMSK_REG  &= ~(BIT_SUB_RXD1);
}

/**
 * This function will handle serial
 */
static void rt_serial2_handler(int vector, void *param)
{
//	    INTSUBMSK_REG |= (BIT_SUB_RXD2);
    uart_device_t *uart=param;

	int uerstat=UERSTAT2_REG;

    rt_hw_serial_isr(uart,RT_SERIAL_EVENT_RX_IND);

    SUBSRCPND_REG |= BIT_SUB_RXD2;

    /* Unmask sub interrupt (RXD0) */
//	    INTSUBMSK_REG  &= ~(BIT_SUB_RXD2);
}

void rs485_to_tx(uart_device_t *dev)
{
    const struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    hw_gpio_out_init(hw->tx_en);
    hw_gpio_out(hw->tx_en, 1);
    rt_thread_delay(RT_TICK_PER_SECOND / 1000);
}

void rs422_to_tx(uart_device_t *dev)
{
    const  struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    hw_gpio_out_init(hw->tx_en);
    hw_gpio_out(hw->tx_en, 1);
    hw_gpio_out_init(hw->rx_en);
    hw_gpio_out(hw->rx_en, 1);
    rt_thread_delay(RT_TICK_PER_SECOND / 1000);
}

#define USTAT_EMPTY     0x02    /* tx buffer empty */

void rs485_to_rx(uart_device_t *dev)
{
    const struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    uartport *uart_device=(uartport *)hw->uart_device;
#ifdef STM32
    while(!(uart_device->SR & USART_FLAG_TC));
#else
    while(!(uart_device->UTRSTAT & USTAT_EMPTY));
#endif
    rt_thread_delay(2);
    hw_gpio_out(hw->tx_en, 0);
}

void rs422_to_rx(uart_device_t *dev)
{
    const struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    uartport *uart_device=(uartport *)hw->uart_device;
#ifdef STM32
    while(!(uart_device->SR & USART_FLAG_TC));
#else
    while(!(uart_device->UTRSTAT & USTAT_EMPTY));
#endif
    rt_thread_delay(2);
    hw_gpio_out(hw->tx_en, 0);
    hw_gpio_out(hw->rx_en, 0);
}

/**
 * This function will handle init uart
 */
void rt_hw_usart_init(uart_device_t *dev,const char *dev_name,unsigned int flag,struct uart_hw_def *uart)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    uart_device_t *serial;
    serial=(uart_device_t *)dev;
    uartport *uart_device=(uartport *)uart->uart_device;


    /* FIFO enable, Tx/Rx FIFO clear */
    uart_device->UFCON = 0x0;
    /* disable the flow control */
    uart_device->UMCON = 0x0;
    /* Normal,No parity,1 stop,8 bit */
    uart_device->ULCON = 0x3;
    /*
     * tx=level,rx=edge,disable timeout int.,enable rx error int.,
     * normal,interrupt or polling
     */
    uart_device->UCON = (1<<8) | (1<<7) | (1<<2) | (1<<0);;
    /* Set uart0 bps */
    uart_device->UBRDIV = (rt_int32_t)(PCLK/ (115200 * 16)) - 1;
    uart_device->UDIVSLOT = 0x0888;
    /* output PCLK to UART0/1, PWMTIMER */

    delay_us(100);

    switch((int)uart_device)
    {
    case(int)0x50000000:

//            CLKCON |= 1<<(10);
        /* UART0 port configure */
        GPHCON_REG &= ~(0x0f);
        GPHCON_REG |= (0x02 << 0 | 0x02 << 2);	   ///Enable TXD0 RXD0,GPH0-1
        GPHPU_REG &=~ 0x3;
        /* install uart0 isr */
        INTSUBMSK_REG &= ~(BIT_SUB_RXD0);
        rt_hw_interrupt_install(INTUART0, rt_serial0_handler, (void *)dev, "UART0");
        rt_hw_interrupt_mask(INTUART0);
        uart->irq=INTUART0;
        break;
    case(int)0x50004000:
//            CLKCON |= 1<<(11);
        GPHCON_REG &= ~(0x0f0);
        GPHCON_REG |= (0x02 << 4 | 0x02 << 6);		//GPH2-3
        /* PULLUP is disable */
        GPHPU_REG &=~ 0xc;
        /* install uart0 isr */
        INTSUBMSK_REG &= ~(BIT_SUB_RXD1);
        rt_hw_interrupt_install(INTUART1, rt_serial1_handler, dev, "UART1");
        rt_hw_interrupt_mask(INTUART1);
        uart->irq=INTUART1;
        break;
    case(int)0x50008000:

//            CLKCON |= 1<<(12);
        GPHCON_REG &= ~(0x0f00);		//GPH4-5
        GPHCON_REG |= (0x02 << 8 | 0x02 << 10);
        /* PULLUP is disable */
        GPHPU_REG &= ~0x30;
        /* install uart0 isr */
        INTSUBMSK_REG &= ~(BIT_SUB_RXD2);
        rt_hw_interrupt_install(INTUART2, rt_serial2_handler, dev, "UART2");
        rt_hw_interrupt_mask(INTUART2);
        uart->irq=INTUART2;
//          rt_kprintf("GPHCON_REG=0x%x,GPHPU_REG=0x%x,CLKCON=0x%x,uart->user_data->uart_device->ucon=0x%x,uart->user_data->uart_device->ubrd=0x%x,PCLK=%d",GPHCON_REG,GPHPU_REG,CLKCON,uart->user_data->uart_device->ucon,uart->user_data->uart_device->ubrd,PCLK);
//        while(1)
//        {
////        GPHDAT|=0xc0;
////        GPHDAT&=~0xc0;
//            while(!(uart->user_data->uart_device->ustat & USTAT_TXB_EMPTY));
//            uart->user_data->uart_device->utxh = (0xaa& 0xFF);
//        }
        break;
    }

    serial->ops    = &s3c2440_uart_ops;
    serial->config = config;
    dev->uart=uart;

    /* register uart0 */
    rt_hw_serial_register(dev, dev_name,RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | flag,uart);
    serial->lock=rt_sem_create("lock", 1, RT_IPC_FLAG_FIFO);

}

extern struct rt_irq_desc isr_table[];

void chk_uart2(void)
{
    //io
    RT_ASSERT((GPHCON_REG&0xf00)==0xa00);
	RT_ASSERT((GPHPU_REG &0x30)==0);

    //irq
    if((INTMSK_REG&BIT_UART2)==0)
        rt_kprintf("BIT_UART2 enable\n");
    else
        rt_kprintf("BIT_UART2 disable\n");
    RT_ASSERT((INTSUBMSK_REG&BIT_SUB_RXD2)==0);
    RT_ASSERT(isr_table[15].handler==rt_serial2_handler);

    //reg
    RT_ASSERT(UFCON2_REG==0);
    RT_ASSERT(UMCON2_REG==0);
    RT_ASSERT(UCON2_REG == (1<<8)  | (1<<7) | (1<<2) | (1<<0));
    RT_ASSERT(UBRDIV2_REG == (rt_int32_t)(PCLK/ (38400 * 16)) - 1);
    RT_ASSERT(UDIVSLOT2_REG== 0x0888);

    //clk
    RT_ASSERT((PCLKCON_REG &0x4)==1);

	rt_kprintf("SRCPND_REG=0x%x,UTRSTAT2_REG=0x%x,UERSTAT2_REG=0x%x,UFSTAT2_REG=0x%x,UMSTAT2_REG=0x%x",SRCPND_REG&BIT_UART2,UTRSTAT2_REG,UERSTAT2_REG,UFSTAT2_REG,UMSTAT2_REG);
}
