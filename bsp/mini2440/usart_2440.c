#include <mcu_def.h>
#include <model.h>
#include <product_config.h>
#include <s3c24x0.h>

#define USTAT_RCV_READY   0x01    /* receive data ready */
#define USTAT_TXB_EMPTY   0x02    /* tx buffer empty */

#define UART0   ((uartport *)&U0BASE)
#define UART1   ((uartport *)&U1BASE)
#define UART2   ((uartport *)&U2BASE)

uart_device_t com_1,com_2,com_3;


static rt_err_t s3c2440_configure(struct uart_device *serial, struct serial_configure *cfg)
{
    struct uart_hw_def* uart;
    volatile int i;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct uart_hw_def *)serial->parent.user_data;
    uartport *uart_device=(uartport *)uart->uart_device;
    uart_device->ufcon = 0x0;
    /* disable the flow control */
    uart_device->umcon = 0x0;
    /* Normal,No parity,1 stop,8 bit */
    uart_device->ulcon = 0x3;
    /*
     * tx=level,rx=edge,disable timeout int.,enable rx error int.,
     * normal,interrupt or polling
     */
    uart_device->ucon = 0x145;
    /* Set uart0 bps */
    uart_device->ubrd = (rt_int32_t)(get_PCLK()/ (cfg->baud_rate* 16)) - 1;
    /* output PCLK to UART0/1, PWMTIMER */
    switch((int)uart_device)
    {
        case(int)UART0:

            CLKCON |= 1<<(10);
            for(i = 0; i < 100; i++);
            /* UART0 port configure */
            GPHCON |= 0xA0;
            /* PULLUP is disable */
            GPHUP &=~ 0xc;
            /* install uart0 isr */
            break;
        case(int)UART1:
            CLKCON |= 1<<(11);
            for(i = 0; i < 100; i++);
            /* UART0 port configure */
            GPHCON |= 0xA00;
            /* PULLUP is disable */
            GPHUP &=~ 0x30;
            /* install uart0 isr */
            break;
        case(int)UART2:

            CLKCON |= 1<<(12);
            for(i = 0; i < 100; i++);
            /* UART0 port configure */
            GPHCON |= 0xA000;
            /* PULLUP is disable */
            GPHUP &= ~0xc0;
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

    while(!(uart_device->ustat & USTAT_TXB_EMPTY));
    uart_device->utxh = (c & 0xFF);

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
    if(uart_device->ustat & USTAT_RCV_READY)
    {
        ch = uart_device->urxh & 0xff;
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
    INTSUBMSK_REG |= (BIT_SUB_RXD0);
    uart_device_t *uart=param;

    rt_hw_serial_isr(uart,RT_SERIAL_EVENT_RX_IND);

    SUBSRCPND_REG |= BIT_SUB_RXD0;

    /* Unmask sub interrupt (RXD0) */
    INTSUBMSK_REG  &= ~(BIT_SUB_RXD0);
}

/**
 * This function will handle serial
 */
static void rt_serial1_handler(int vector, void *param)
{
    INTSUBMSK_REG |= (BIT_SUB_RXD1);
    uart_device_t *uart=param;

    rt_hw_serial_isr(uart,RT_SERIAL_EVENT_RX_IND);

    SUBSRCPND_REG |= BIT_SUB_RXD1;

    /* Unmask sub interrupt (RXD0) */
    INTSUBMSK_REG  &= ~(BIT_SUB_RXD1);
}

/**
 * This function will handle serial
 */
static void rt_serial2_handler(int vector, void *param)
{
    INTSUBMSK_REG |= (BIT_SUB_RXD2);
    uart_device_t *uart=param;

    rt_hw_serial_isr(uart,RT_SERIAL_EVENT_RX_IND);

    SUBSRCPND_REG |= BIT_SUB_RXD2;

    /* Unmask sub interrupt (RXD0) */
    INTSUBMSK_REG  &= ~(BIT_SUB_RXD2);
}

void rs485_to_tx(uart_device_t *dev)
{
    struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    hw_gpio_out_init(hw->tx_en);
    hw_gpio_out(hw->tx_en, 1);
    rt_thread_delay(RT_TICK_PER_SECOND / 1000);
}

void rs422_to_tx(uart_device_t *dev)
{
    struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    hw_gpio_out_init(hw->tx_en);
    hw_gpio_out(hw->tx_en, 1);
    hw_gpio_out_init(hw->rx_en);
    hw_gpio_out(hw->rx_en, 1);
    rt_thread_delay(RT_TICK_PER_SECOND / 1000);
}

#define USTAT_EMPTY     0x02    /* tx buffer empty */

void rs485_to_rx(uart_device_t *dev)
{
    struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    uartport *uart_device=(uartport *)hw->uart_device;
#ifdef STM32
    while(!(uart_device->SR & USART_FLAG_TC));
#else
    while(!(uart_device->ustat & USTAT_EMPTY));
#endif
    rt_thread_delay(2);
    hw_gpio_out(hw->tx_en, 0);
}

void rs422_to_rx(uart_device_t *dev)
{
    struct uart_hw_def *hw=((uart_device_t *)dev)->uart;
    uartport *uart_device=(uartport *)hw->uart_device;
#ifdef STM32
    while(!(uart_device->SR & USART_FLAG_TC));
#else
    while(!(uart_device->ustat & USTAT_EMPTY));
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
    volatile int i;


    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    uart_device_t *serial;
    serial=(uart_device_t *)dev;
    uartport *uart_device=(uartport *)uart->uart_device;


    /* FIFO enable, Tx/Rx FIFO clear */
    uart_device->ufcon = 0x0;
    /* disable the flow control */
    uart_device->umcon = 0x0;
    /* Normal,No parity,1 stop,8 bit */
    uart_device->ulcon = 0x3;
    /*
     * tx=level,rx=edge,disable timeout int.,enable rx error int.,
     * normal,interrupt or polling
     */
    uart_device->ucon = 0x145;
    /* Set uart0 bps */
    uart_device->ubrd = (rt_int32_t)(get_PCLK()/ (115200 * 16)) - 1;
    /* output PCLK to UART0/1, PWMTIMER */
    switch((int)uart_device)
    {
        case(int)UART0:

            CLKCON |= 1<<(10);
            for(i = 0; i < 100; i++);
            /* UART0 port configure */
            GPHCON |= 0xA0;
            /* PULLUP is disable */
            GPHUP &=~ 0xc;
            /* install uart0 isr */
            INTSUBMSK_REG &= ~(BIT_SUB_RXD0);
            rt_hw_interrupt_install(INTUART0, rt_serial0_handler, dev, "UART0");
            rt_hw_interrupt_mask(INTUART0);
            uart->irq=INTUART0;
            break;
        case(int)UART1:
            CLKCON |= 1<<(11);
            for(i = 0; i < 100; i++);
            /* UART0 port configure */
            GPHCON |= 0xA00;
            /* PULLUP is disable */
            GPHUP &=~ 0x30;
            /* install uart0 isr */
            INTSUBMSK_REG &= ~(BIT_SUB_RXD1);
            rt_hw_interrupt_install(INTUART1, rt_serial1_handler, dev, "UART1");
            rt_hw_interrupt_mask(INTUART1);
            uart->irq=INTUART1;
            break;
        case(int)UART2:

            CLKCON |= 1<<(12);
            for(i = 0; i < 100; i++);
            /* UART0 port configure */
            GPHCON |= 0xA000;
            /* PULLUP is disable */
            GPHUP &= ~0xc0;
            /* install uart0 isr */
            INTSUBMSK_REG &= ~(BIT_SUB_RXD2);
            rt_hw_interrupt_install(INTUART2, rt_serial2_handler, dev, "UART2");
            rt_hw_interrupt_mask(INTUART2);
            uart->irq=INTUART2;
//          rt_kprintf("GPHCON=0x%x,GPHUP=0x%x,CLKCON=0x%x,uart->user_data->uart_device->ucon=0x%x,uart->user_data->uart_device->ubrd=0x%x,PCLK=%d",GPHCON,GPHUP,CLKCON,uart->user_data->uart_device->ucon,uart->user_data->uart_device->ubrd,PCLK);
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

