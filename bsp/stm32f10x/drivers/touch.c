#include<mcu_def.h>
#include <stdbool.h>
#ifdef STM32
#include "stm32f10x.h"
#endif

#ifdef LPC178X
#include "LPC177x_8x.h"
#endif

//#include "board.h"
#include "touch.h"
//#include "product_config.h"

#include <rtthread.h>
#include <rtgui/event.h>
#include <rtgui/kbddef.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>


#define HOLD_TIME   RT_TICK_PER_SECOND/10
#define QUERY_TIME  RT_TICK_PER_SECOND/20
/*
MISO PA6
MOSI PA7
CLK  PA5
CS   PC4
*/

//  #define   CS_0()          GPIO_ResetBits(GPIOG,GPIO_Pin_11)
//  #define   CS_1()          GPIO_SetBits(GPIOG,GPIO_Pin_11)

/*
7  6 - 4  3      2     1-0
s  A2-A0 MODE SER/DFR PD1-PD0
*/
#define TOUCH_MSR_Y  0x90   //读X轴坐标指令 addr:1
#define TOUCH_MSR_X  0xD0   //读Y轴坐标指令 addr:3

struct rtgui_touch_device
{
    struct rt_device parent;

    rt_timer_t poll_timer;
    rt_uint16_t x, y;

    rt_bool_t calibrating;
    rt_touch_calibration_func_t calibration_func;

    rt_uint16_t min_x, max_x;
    rt_uint16_t min_y, max_y;
    const spi_io_def *chip;
	const lcd_t *lcd;
};
static struct rtgui_touch_device *touch_dev = RT_NULL;
extern const lcd_t lcd;
//  extern unsigned char SPI_WriteByte(unsigned char data);
//rt_inline void EXTI_Enable(rt_uint32_t enable);

//  struct rt_semaphore spi1_lock;

//  void rt_hw_spi1_baud_rate(uint16_t SPI_BaudRatePrescaler)
//  {
//      SPI2->CR1 &= ~SPI_BaudRatePrescaler_256;
//      SPI2->CR1 |= SPI_BaudRatePrescaler;
//  }

void touch_set_pin(const io_def p, unsigned char v)
{
    hw_gpio_out(p, v);
}

unsigned char touch_get_pin(const io_def p)
{
    return    hw_gpio_in(p);
}



unsigned char ads7843_WriteByte(unsigned char data)
{
    char buf, readbyte = 0;
    volatile char i, j;
#if 0
    if(touch_dev->chip->spi != RT_NULL)
    {
        //Wait until the transmit buffer is empty
        while(SPI_I2S_GetFlagStatus(touch_dev->chip->spi, SPI_I2S_FLAG_TXE) == RESET);
        // Send the byte
        SPI_I2S_SendData(touch_dev->chip->spi, data);

        //Wait until a data is received
        while(SPI_I2S_GetFlagStatus(touch_dev->chip->spi, SPI_I2S_FLAG_RXNE) == RESET);
        // Get the received data
        readbyte = SPI_I2S_ReceiveData(touch_dev->chip->spi);
        rt_kprintf("\ntouch don't use SPI . %s  L%d\n",__FILE__,__LINE__);
    }
    else
#endif
    {

        for(i = 0; i < 8; i++)
        {
            readbyte = readbyte << 1;
            buf = (data >> (7 - i)) & 0x1; //MSB在前,LSB在后
            touch_set_pin(touch_dev->chip->mosi, buf); //时钟上升沿锁存DIN

            for(j = 0; j < 25; j++); //200ns

            touch_set_pin(touch_dev->chip->clk, 1); //开始发送命令字

            for(j = 0; j < 25; j++); //200ns

            if(touch_get_pin(touch_dev->chip->miso))
            {
                readbyte++;
            }
            touch_set_pin(touch_dev->chip->clk, 0); //时钟脉冲，一共8个
        }
    }

    return readbyte;
}




#define FILTER_TIMES_TOUCH 10
static void rtgui_touch_calculate()
{
    volatile int i, j;
    //      rt_kprintf("touch_dev up: %s,%d\n",__FILE__,__LINE__);
    if(touch_dev != RT_NULL)
    {
        //          rt_kprintf("touch up: %s,%d\n",__FILE__,__LINE__);
        //          rt_sem_take(&spi1_lock, RT_WAITING_FOREVER);
        //          rt_kprintf("touch up: %s,%d\n",__FILE__,__LINE__);
        /* SPI1 configure */
        //          rt_hw_spi1_baud_rate(SPI_BaudRatePrescaler_64);/* 72M/64=1.125M */

        //读取触摸值
        {
            rt_uint16_t tmpx[10];
            rt_uint16_t tmpy[10];
            //              unsigned int i;

            /* From the datasheet:
             * When the very first CLK after the control byte comes in, the
             * DOUT of ADS7843 is not valid. So we could only get 7bits from
             * the first SPI_WriteByte. And the got the following 5 bits from
             * another SPI_WriteByte.(aligned MSB)
             */
            //                  rt_kprintf("touch up: %s,%d\n",__FILE__,__LINE__);
            for(i = 0; i < FILTER_TIMES_TOUCH; i++)
            {
                //                  for(j = 0; j < 250; j++); //200ns
                touch_set_pin(touch_dev->chip->cs, 0);
                for(j = 0; j < 100; j++); //200ns
                ads7843_WriteByte(TOUCH_MSR_X);

                j = 0;
                if(!touch_dev->chip->busy.port)
                {
                    for(j = 0; j < 1000; j++);
                }
                else
                {
                    while(!touch_get_pin(touch_dev->chip->busy))
                    {
                        if(j++ > 50000)
                        {
                            break;
                        }
                    };
                }
                tmpx[i] = (ads7843_WriteByte(0x00) & 0x7F) << 5;
                tmpx[i] |= (ads7843_WriteByte(TOUCH_MSR_Y) >> 3) & 0x1F;
//                  for(j = 0; j < 20; j++); //200ns

                j = 0;
                while(!touch_get_pin(touch_dev->chip->busy))
                {
                    if(j++ > 50000)
                    {
                        break;
                    }
                };

                tmpy[i] = (ads7843_WriteByte(0x00) & 0x7F) << 5;
                tmpy[i] |= (ads7843_WriteByte(0x00) >> 3) & 0x1F;

                ads7843_WriteByte(1 << 7);   /* 打开中断 */
//                                      for(j = 0; j < 250; j++); //200ns
                touch_set_pin(touch_dev->chip->cs, 1);
                if(touch_get_pin(touch_dev->chip->irq) != 0)
                {
                    return;
                }
                //                  rt_kprintf("\n\r x-y: %d,%d\n\r",tmpx[i],tmpy[i]);
            }

            //              rt_kprintf("touch_dev up: %s,%d\n",__FILE__,__LINE__);
            //去最高值与最低值,再取平均值
            {
                rt_uint32_t min_x = 0xFFFF, min_y = 0xFFFF;
                rt_uint32_t max_x = 0, max_y = 0;
                rt_uint32_t total_x = 0;
                rt_uint32_t total_y = 0;
//                  unsigned int i;

                for(i = 0; i < FILTER_TIMES_TOUCH; i++)
                {
                    if(tmpx[i] < min_x)
                    {
                        min_x = tmpx[i];
                    }
                    if(tmpx[i] > max_x)
                    {
                        max_x = tmpx[i];
                    }
                    total_x += tmpx[i];

                    if(tmpy[i] < min_y)
                    {
                        min_y = tmpy[i];
                    }
                    if(tmpy[i] > max_y)
                    {
                        max_y = tmpy[i];
                    }
                    total_y += tmpy[i];
                }
                total_x = total_x - min_x - max_x;
                total_y = total_y - min_y - max_y;
                if(rtgui_touch_swap)
                {
                    touch_dev->y = total_x / (FILTER_TIMES_TOUCH - 2);
                    touch_dev->x = total_y / (FILTER_TIMES_TOUCH - 2);
                }
                else
                {
                    touch_dev->x = total_x / (FILTER_TIMES_TOUCH - 2);
                    touch_dev->y = total_y / (FILTER_TIMES_TOUCH - 2);
                }
            }//去最高值与最低值,再取平均值
        }//读取触摸值
        //          rt_kprintf("touch up: %s,%d\n",__FILE__,__LINE__);

        //          rt_sem_release(&spi1_lock);
        //          rt_kprintf("\n\r x-y: %d,%d\n\r",touch->x,touch->y);

        /* if it's not in calibration status  */
        if(touch_dev->calibrating != RT_TRUE)
        {
            if(touch_dev->max_x > touch_dev->min_x)
            {
                touch_dev->x = (touch_dev->x - touch_dev->min_x) * lcd.w / (touch_dev->max_x - touch_dev->min_x);
            }
            else if(touch_dev->max_x < touch_dev->min_x)
            {
                touch_dev->x = (touch_dev->min_x - touch_dev->x) * lcd.w / (touch_dev->min_x - touch_dev->max_x);
            }

            if(touch_dev->max_y > touch_dev->min_y)
            {
                touch_dev->y = (touch_dev->y - touch_dev->min_y) * lcd.h / (touch_dev->max_y - touch_dev->min_y);
            }
            else if(touch_dev->max_y < touch_dev->min_y)
            {
                touch_dev->y = (touch_dev->min_y - touch_dev->y) * lcd.h / (touch_dev->min_y - touch_dev->max_y);
            }

            // normalize the data
            if(touch_dev->x & 0x8000)
            {
                touch_dev->x = 0;
            }
            else if(touch_dev->x > lcd.w)
            {
                touch_dev->x = lcd.w - 1;
            }

            if(touch_dev->y & 0x8000)
            {
                touch_dev->y = 0;
            }
            else if(touch_dev->y > lcd.h)
            {
                touch_dev->y = lcd.h - 1;
            }
            //              rt_kprintf("\n\r x-y: %d,%d\n\r",touch->x,touch->y);
            //              rt_kprintf("touch up: %s,%d\n",__FILE__,__LINE__);
        }
        //          rt_kprintf("touch up: %s,%d\n",__FILE__,__LINE__);
    }
    //      rt_kprintf("touch up: %s,%d\n",__FILE__,__LINE__);
}

//rt_inline void EXTI_Enable(rt_uint32_t enable);

unsigned char touch_is_down()
{
    return touch_get_pin(touch_dev->chip->irq);
}

void touch_timeout(void* parameter)
{
    static unsigned int touched_down = 0;
    struct rtgui_event_mouse emouse;
    static struct _touch_previous
    {
        rt_uint32_t x;
        rt_uint32_t y;
    } touch_previous;

    /* touch time is too short and we lost the position already. */
    if((!touched_down) && touch_get_pin(touch_dev->chip->irq) != 0)
    {

        int tmer = HOLD_TIME ;
        rt_timer_stop(touch_dev->poll_timer);
        rt_timer_control(touch_dev->poll_timer , RT_TIMER_CTRL_SET_TIME , &tmer);
        return;
    }
    //      rt_kprintf("touch_dev up: (%d, %d)%s,%d\n", emouse.x, emouse.y,__FILE__,__LINE__);
    if(touch_get_pin(touch_dev->chip->irq) != 0)
    {
        //          rt_kprintf("touch_dev up: (%d, %d)%s,%d\n", emouse.x, emouse.y,__FILE__,__LINE__);
        int tmer = HOLD_TIME;
        //        EXTI_Enable(1);
        emouse.parent.sender = RT_NULL;
        emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
        emouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_UP);
        emouse.wid =  RT_NULL;

        /* use old value */
        emouse.x = touch_dev->x;
        emouse.y = touch_dev->y;

        sound(1);

        /* stop timer */
        rt_timer_stop(touch_dev->poll_timer);
        //          rt_kprintf("touch up: (%d, %d)\n", emouse.x, emouse.y);
        touched_down = 0;

        if((touch_dev->calibrating == RT_TRUE) && (touch_dev->calibration_func != RT_NULL))
        {
            /* callback function */
            touch_dev->calibration_func(emouse.x, emouse.y);
        }
        rt_timer_control(touch_dev->poll_timer , RT_TIMER_CTRL_SET_TIME , &tmer);
    }
    else
    {
        //          rt_kprintf("touch_dev up: (%d, %d)%s,%d\n", emouse.x, emouse.y,__FILE__,__LINE__);
        if(touched_down == 0)
        {
            //              rt_kprintf("touch_dev up: (%d, %d)%s,%d\n", emouse.x, emouse.y,__FILE__,__LINE__);
            int tmer = QUERY_TIME ;
            /* calculation */
            rtgui_touch_calculate();

            /* send mouse event */
            emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
            emouse.parent.sender = RT_NULL;
            emouse.wid =  RT_NULL;

            emouse.x = touch_dev->x;
            emouse.y = touch_dev->y;

            touch_previous.x = touch_dev->x;
            touch_previous.y = touch_dev->y;

            /* init mouse button */
            emouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_DOWN);

            //              rt_kprintf("touch_dev down: (%d, %d)\n", emouse.x, emouse.y);
            if(touch_get_pin(touch_dev->chip->irq) != 0)
            {
                return;
            }
            touched_down = 1;
            rt_timer_control(touch_dev->poll_timer , RT_TIMER_CTRL_SET_TIME , &tmer);
        }
        else
        {
            /* calculation */
            //              rt_kprintf("touch_dev up: (%d, %d)%s,%d\n", emouse.x, emouse.y,__FILE__,__LINE__);
            rtgui_touch_calculate();
            //              rt_kprintf("touch_dev up: (%d, %d)%s,%d\n", emouse.x, emouse.y,__FILE__,__LINE__);

#define previous_keep      8
            //判断移动距离是否小于previous_keep,减少误动作.
            if(touch_get_pin(touch_dev->chip->irq) != 0)
            {
                return;
            }
            if(
                (touch_previous.x < touch_dev->x + previous_keep)
                && (touch_previous.x > touch_dev->x - previous_keep)
                && (touch_previous.y < touch_dev->y + previous_keep)
                && (touch_previous.y > touch_dev->y - previous_keep))
            {
                return;
            }

            //              rt_kprintf("touch_dev up: (%d, %d)%s,%d\n", emouse.x, emouse.y,__FILE__,__LINE__);
            touch_previous.x = touch_dev->x;
            touch_previous.y = touch_dev->y;

            /* send mouse event */
            emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON ;
            emouse.parent.sender = RT_NULL;
            emouse.wid =  RT_NULL;

            emouse.x = touch_dev->x;
            emouse.y = touch_dev->y;

            /* init mouse button */
            emouse.button = (RTGUI_MOUSE_BUTTON_RIGHT | RTGUI_MOUSE_BUTTON_DOWN);
            //              rt_kprintf("touch_dev motion: (%d, %d)\n", emouse.x, emouse.y);
        }
    }

//      rt_kprintf("\n\r %d: (%d, %d)%s,%d",emouse.button, emouse.x, emouse.y,__FILE__,__LINE__);
#ifndef KEIL
    /* send event to server */
    if(touch_dev->calibrating != RT_TRUE)
    {
        if(!rtgui_wakeup())
            rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));
    }
#endif
}


void touch_irq()
{
    //      rt_kprintf("n\r touch_irq .\n\r");
    rt_timer_start(touch_dev->poll_timer);
}

//void NVIC_Configuration(void);
//  void touch_EXTI_Configuration(void);

/* RT-Thread Device Interface */
static rt_err_t rtgui_touch_init(rt_device_t dev)
{

    hw_gpio_exti_init(touch_dev->chip->irq, 0xc);

    touch_set_pin(touch_dev->chip->cs, 1);
    touch_set_pin(touch_dev->chip->cs, 0);

    ads7843_WriteByte(1 << 7);   /* 打开中断 */
    touch_set_pin(touch_dev->chip->cs, 1);

    return RT_EOK;
}


static rt_err_t rtgui_touch_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch(cmd)
    {
        case RT_TOUCH_CALIBRATION:
            touch_dev->calibrating = RT_TRUE;
            touch_dev->calibration_func = (rt_touch_calibration_func_t)args;
            break;

        case RT_TOUCH_NORMAL:
            touch_dev->calibrating = RT_FALSE;
            break;

        case RT_TOUCH_CALIBRATION_DATA:
        {
            struct calibration_data *data;

            data = (struct calibration_data *) args;

            //update
            touch_dev->min_x = data->min_x;
            touch_dev->max_x = data->max_x;
            touch_dev->min_y = data->min_y;
            touch_dev->max_y = data->max_y;
        }
        break;
    }

    return RT_EOK;
}

extern const spi_io_def touch;

void rt_hw_touch_init()
{
#ifdef STM32
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef   SPI_InitStructure;

    /* 开启GPIO时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOG, ENABLE);
#endif
    /* 配置 推挽输出，用于 TP_CS  */
    hw_gpio_out_init(touch.cs);

    /* 配置 PB5 引脚为上拉输入，用于 TP_BUSY */
    if(touch.busy.port)
    {
        hw_gpio_in_init(touch.busy, 1);
    }

    hw_gpio_out_init(touch.mosi);
    hw_gpio_out_init(touch.clk);
    hw_gpio_in_init(touch.miso, 1);
    touch_dev = (struct rtgui_touch_device *)rt_malloc(sizeof(struct rtgui_touch_device));
    if(touch_dev == RT_NULL)
    {
        rt_kprintf("\n\r ***error not rt_malloc %s ,L%d \n\r", __FILE__, __LINE__);
        return; /* no memory yet */
    }
    /* clear device structure */
    rt_memset(&(touch_dev->parent), 0, sizeof(struct rt_device));
    touch_dev->calibrating = false;

    /* init device structure */
    touch_dev->parent.type = RT_Device_Class_Unknown;
    touch_dev->parent.init = rtgui_touch_init;
    touch_dev->parent.control = rtgui_touch_control;
    touch_dev->parent.user_data = RT_NULL;
    touch_dev->chip = &touch;

    /* create 1/50 second timer */
    touch_dev->poll_timer = rt_timer_create("touch", touch_timeout, RT_NULL,
                                            HOLD_TIME, RT_TIMER_FLAG_PERIODIC);

    /* register touch device to RT-Thread */
    rt_device_register(&(touch_dev->parent), "touch", RT_DEVICE_FLAG_RDWR);
    rtgui_touch_init(&(touch_dev->parent));

    calibration_init();
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void touch_t(rt_uint16_t x , rt_uint16_t y)
{
    struct rtgui_event_mouse emouse ;
    emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
    emouse.parent.sender = RT_NULL;

    emouse.x = x ;
    emouse.y = y ;
    /* init mouse button */
    emouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_DOWN);
    rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));

    //      rt_thread_delay(2) ;
    emouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_UP);
    rtgui_server_post_event(&emouse.parent, sizeof(struct rtgui_event_mouse));
}

FINSH_FUNCTION_EXPORT(touch_t, x &y) ;
#endif
