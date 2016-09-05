/*
 * File      : touch.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2010 - 2012, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2010-01-01     Yi.Qiu      first version
 */

#include <rthw.h>
#include <rtthread.h>
#ifdef CONFIG_S3C2416
//  #include <S3C2416.h>
#include <s3c24x0.h>
#else
#include <s3c24x0.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui_system.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/event.h>
#endif

#include "lcd.h"
#include "touch.h"
#include<mcu_def.h>

/* ADCCON Register Bits */
#define S3C2410_ADCCON_ECFLG			(1<<15)
#define S3C2410_ADCCON_PRSCEN			(1<<14)
#define S3C2410_ADCCON_PRSCVL(x)		(((x)&0xFF)<<6)
#define S3C2410_ADCCON_PRSCVLMASK		(0xFF<<6)
#define S3C2410_ADCCON_SELMUX(x)		(((x)&0x7)<<3)
#define S3C2410_ADCCON_MUXMASK			(0x7<<3)
#define S3C2410_ADCCON_STDBM			(1<<2)
#define S3C2410_ADCCON_READ_START		(1<<1)
#define S3C2410_ADCCON_ENABLE_START		(1<<0)
#define S3C2410_ADCCON_STARTMASK		(0x3<<0)

/* ADCTSC Register Bits */
#define S3C2410_ADCTSC_UD_SEN			(1<<8) /* ghcstop add for s3c2440a */
#define S3C2410_ADCTSC_YM_SEN			(1<<7)
#define S3C2410_ADCTSC_YP_SEN			(1<<6)
#define S3C2410_ADCTSC_XM_SEN			(1<<5)
#define S3C2410_ADCTSC_XP_SEN			(1<<4)
#define S3C2410_ADCTSC_PULL_UP_DISABLE	(1<<3)
#define S3C2410_ADCTSC_AUTO_PST			(1<<2)
#define S3C2410_ADCTSC_XY_PST(x)		(((x)&0x3)<<0)

/* ADCDAT0 Bits */
#define S3C2410_ADCDAT0_UPDOWN			(1<<15)
#define S3C2410_ADCDAT0_AUTO_PST		(1<<14)
#define S3C2410_ADCDAT0_XY_PST			(0x3<<12)
#define S3C2410_ADCDAT0_XPDATA_MASK		(0x03FF)

/* ADCDAT1 Bits */
#define S3C2410_ADCDAT1_UPDOWN			(1<<15)
#define S3C2410_ADCDAT1_AUTO_PST		(1<<14)
#define S3C2410_ADCDAT1_XY_PST			(0x3<<12)
#define S3C2410_ADCDAT1_YPDATA_MASK		(0x03FF)

#define WAIT4INT(x)  (((x)<<8) | \
		     S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | S3C2410_ADCTSC_XP_SEN | \
		     S3C2410_ADCTSC_XY_PST(3))

#define AUTOPST	     (S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | S3C2410_ADCTSC_XP_SEN | \
		     S3C2410_ADCTSC_AUTO_PST | S3C2410_ADCTSC_XY_PST(0))

#define X_MIN		74
#define X_MAX		934
#define Y_MIN		920
#define Y_MAX		89

int adctsc;
struct s3c2410ts
{
	long xp;
	long yp;
	int count;
	int shift;

	int delay;
	int presc;

	char phys[32];
};
static struct s3c2410ts ts;

struct rtgui_touch_device
{
	struct rt_device parent;

	rt_timer_t poll_timer;
	rt_uint16_t x, y;

	rt_bool_t calibrating;
	rt_touch_calibration_func_t calibration_func;

	rt_touch_eventpost_func_t eventpost_func;
	void *eventpost_param;

	rt_uint16_t min_x, max_x;
	rt_uint16_t min_y, max_y;

	rt_uint16_t width;
	rt_uint16_t height;

	rt_bool_t first_down_report;
};
static struct rtgui_touch_device *touch_device = RT_NULL;
extern const int rtgui_touch_swap ;

#ifdef RT_USING_RTGUI
static void report_touch_input(int updown)
{
	struct rtgui_event_mouse emouse;

	RTGUI_EVENT_MOUSE_BUTTON_INIT(&emouse);
	emouse.wid = RT_NULL;

	/* set emouse button */
	emouse.button = RTGUI_MOUSE_BUTTON_LEFT;
	emouse.parent.sender = RT_NULL;

	if (updown)
	{
		ts.xp = ts.xp / ts.count;
		ts.yp = ts.yp / ts.count;;

        if((touch_device->calibrating == RT_TRUE) && (touch_device->calibration_func != RT_NULL))
        {
            touch_device->x = ts.xp;
            touch_device->y = ts.yp;
        }
        else
        {
            if(touch_device->max_x > touch_device->min_x)
            {
                touch_device->x = touch_device->width * (ts.xp - touch_device->min_x) / (touch_device->max_x - touch_device->min_x);
            }
            else
            {
                touch_device->x = touch_device->width * (touch_device->min_x - ts.xp) / (touch_device->min_x - touch_device->max_x);
            }

            if(touch_device->max_y > touch_device->min_y)
            {
                touch_device->y = touch_device->height * (ts.yp - touch_device->min_y) / (touch_device->max_y - touch_device->min_y);
            }
            else
            {
                touch_device->y = touch_device->height * (touch_device->min_y - ts.yp) / (touch_device->min_y - touch_device->max_y);
            }
        }
        emouse.x = touch_device->x;
        emouse.y = touch_device->y;
        if(touch_device->first_down_report == RT_TRUE)
        {
            sound(1);
            emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
            emouse.button |= RTGUI_MOUSE_BUTTON_DOWN;
        }
        else
        {
            emouse.parent.type = RTGUI_EVENT_MOUSE_MOTION;
            emouse.button = 0;
        }
    }
    else
    {
        emouse.x = touch_device->x;
        emouse.y = touch_device->y;
        emouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
        emouse.button |= RTGUI_MOUSE_BUTTON_UP;
        if((touch_device->calibrating == RT_TRUE) && (touch_device->calibration_func != RT_NULL))
        {
            /* callback function */
            touch_device->calibration_func(emouse.x, emouse.y);
        }
    }

    /* send event to server */
    if(touch_device->calibrating != RT_TRUE)
    {
//      rt_kprintf("touch_device %s: ts.x: %d, ts.y: %d\n", updown? "down" : "up",  touch_device->x, touch_device->y);
        if(!rtgui_wakeup())
            rtgui_server_post_event((&emouse.parent), sizeof(emouse));
//      rt_kprintf(" --touch_device %s: ts.x: %d, ts.y: %d\n", updown? "down" : "up",  touch_device->x, touch_device->y);
    }
}
#else
static void report_touch_input(int updown)
{
	struct rt_touch_event touch_event;

	if (updown)
	{
		ts.xp = ts.xp / ts.count;
		ts.yp = ts.yp / ts.count;

        if((touch_device->calibrating == RT_TRUE) && (touch_device->calibration_func != RT_NULL))
        {
            touch_device->x = ts.xp;
            touch_device->y = ts.yp;
        }
        else
        {
            if(touch_device->max_x > touch_device->min_x)
            {
                touch_device->x = touch_device->width * (ts.xp - touch_device->min_x) / (touch_device->max_x - touch_device->min_x);
            }
            else
            {
                touch_device->x = touch_device->width * (touch_device->min_x - ts.xp) / (touch_device->min_x - touch_device->max_x);
            }

            if(touch_device->max_y > touch_device->min_y)
            {
                touch_device->y = touch_device->height * (ts.yp - touch_device->min_y) / (touch_device->max_y - touch_device->min_y);
            }
            else
            {
                touch_device->y = touch_device->height * (touch_device->min_y - ts.yp) / (touch_device->min_y - touch_device->max_y);
            }
        }

        touch_event.x = touch_device->x;
        touch_event.y = touch_device->y;
        touch_event.pressed = 1;

        if(touch_device->first_down_report == RT_TRUE)
        {
            if(touch_device->calibrating != RT_TRUE && touch_device->eventpost_func)
            {
                touch_device->eventpost_func(touch_device->eventpost_param, &touch_event);
            }
        }
    }
    else
    {
        touch_event.x = touch_device->x;
        touch_event.y = touch_device->y;
        touch_event.pressed = 0;

        if((touch_device->calibrating == RT_TRUE) && (touch_device->calibration_func != RT_NULL))
        {
            /* callback function */
            touch_device->calibration_func(touch_event.x, touch_event.y);
        }

        if(touch_device->calibrating != RT_TRUE && touch_device->eventpost_func)
        {
            touch_device->eventpost_func(touch_device->eventpost_param, &touch_event);
        }
    }
}
#endif

static void touch_timer_fire(void *parameter)
{
	rt_uint32_t data0;
	rt_uint32_t data1;
	int updown;

	data0 = ADCDAT0;
	data1 = ADCDAT1;

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT0_UPDOWN));

	if (updown)
	{
		if (ts.count != 0)
		{
			report_touch_input(updown);
		}

		ts.xp = 0;
		ts.yp = 0;
		ts.count = 0;

        adctsc= ADCTSC = S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST;
        ADCCON |= S3C2410_ADCCON_ENABLE_START;
    }
}

static void s3c2410_adc_stylus_action(void)
{
	rt_uint32_t data0;
	rt_uint32_t data1;

    data0 = ADCDAT0;
    data1 = ADCDAT1;

    if(rtgui_touch_swap)
    {
        ts.yp += data0 & S3C2410_ADCDAT0_XPDATA_MASK;
        ts.xp += data1 & S3C2410_ADCDAT1_YPDATA_MASK;
    }
    else
    {
        ts.xp += data0 & S3C2410_ADCDAT0_XPDATA_MASK;
        ts.yp += data1 & S3C2410_ADCDAT1_YPDATA_MASK;
    }
    ts.count ++;

    if(ts.count < (1 << ts.shift))
    {
        adctsc= ADCTSC = S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST;
        ADCCON |= S3C2410_ADCCON_ENABLE_START;
    }
    else
    {
        if(touch_device->first_down_report)
        {
            report_touch_input(1);
            ts.xp = 0;
            ts.yp = 0;
            ts.count = 0;
            touch_device->first_down_report = 0;
        }
        /* start timer */
        rt_timer_start(touch_device->poll_timer);
        adctsc=  ADCTSC = WAIT4INT(1);
        RT_ASSERT(ADCTSC = WAIT4INT(1));
    }

    SUBSRCPND |= BIT_SUB_ADC;
}

unsigned char touch_is_down()
{
    rt_uint32_t data0;
    rt_uint32_t data1;

    data0 = ADCDAT0;
    data1 = ADCDAT1;

    if((!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT0_UPDOWN)))
    {
        return 0;
    }
    return 1;
}

static void s3c2410_intc_stylus_updown(void)
{
	rt_uint32_t data0;
	rt_uint32_t data1;
	int updown;

	data0 = ADCDAT0;
	data1 = ADCDAT1;

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT0_UPDOWN));

	/* rt_kprintf("stylus: %s\n", updown? "down" : "up"); */

    if(updown)
    {
        touch_timer_fire(0);
    }
    else
    {
        /* stop timer */
        rt_timer_stop(touch_device->poll_timer);
        touch_device->first_down_report = RT_TRUE;
        if(ts.xp >= 0 && ts.yp >= 0)
        {
            report_touch_input(updown);
        }
        ts.count = 0;
        adctsc=  ADCTSC = WAIT4INT(0);
    }

    SUBSRCPND |= BIT_SUB_TC;
}

static void rt_touch_handler(int vector, void *param)
{
//      rt_kprintf("rt_touch_handler: 0x%x,0x%x\n",SUBSRCPND,INTPND);
    if(SUBSRCPND & BIT_SUB_ADC)
    {
        /* INT_SUB_ADC */
        s3c2410_adc_stylus_action();
    }

    if(SUBSRCPND & BIT_SUB_TC)
    {
        /* INT_SUB_TC */
        s3c2410_intc_stylus_updown();
    }

    /* clear interrupt */
    INTPND |= (1ul << INTADC);
}

/* RT-Thread Device Interface */
static rt_err_t rtgui_touch_init(rt_device_t dev)
{
    /* init touch_device screen structure */
    rt_memset(&ts, 0, sizeof(struct s3c2410ts));

    ts.delay = 50000;
    ts.presc = 9;
    ts.shift = 5;
    ts.count = 0;
    ts.xp = ts.yp = 0;

	ADCCON = S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(ts.presc);
	ADCDLY = ts.delay;

    adctsc= ADCTSC = WAIT4INT(0);

    rt_hw_interrupt_install(INTADC, rt_touch_handler, RT_NULL, "touch_irq");
    rt_hw_interrupt_umask(INTADC);

    /* clear interrupt */
    INTPND |= (1ul << INTADC);

    SUBSRCPND |= BIT_SUB_TC;
    SUBSRCPND |= BIT_SUB_ADC;

    /* install interrupt handler */
    INTSUBMSK &= ~BIT_SUB_ADC;
    INTSUBMSK &= ~BIT_SUB_TC;

    touch_device->first_down_report = RT_TRUE;
    rt_kprintf("rtgui_touch_init\n");
    return RT_EOK;
}

static rt_err_t rtgui_touch_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch(cmd)
    {
        case RT_TOUCH_CALIBRATION:
            touch_device->calibrating = RT_TRUE;
            touch_device->calibration_func = (rt_touch_calibration_func_t)args;
            break;

        case RT_TOUCH_NORMAL:
            touch_device->calibrating = RT_FALSE;
            break;

	case RT_TOUCH_CALIBRATION_DATA:
	{
		struct calibration_data *data;

		data = (struct calibration_data *)args;

            /* update */
            touch_device->min_x = data->min_x;
            touch_device->max_x = data->max_x;
            touch_device->min_y = data->min_y;
            touch_device->max_y = data->max_y;

		/*
			rt_kprintf("min_x = %d, max_x = %d, min_y = %d, max_y = %d\n",
				touch->min_x, touch->max_x, touch->min_y, touch->max_y);
		*/
	}
		break;

        case RT_TOUCH_EVENTPOST:
            touch_device->eventpost_func = (rt_touch_eventpost_func_t)args;
            break;

        case RT_TOUCH_EVENTPOST_PARAM:
            touch_device->eventpost_param = args;
            break;
    }

	return RT_EOK;
}

void rt_hw_touch_init()
{
	rt_err_t result = RT_FALSE;
	rt_device_t device = RT_NULL;
	struct rt_device_graphic_info info;

    touch_device = (struct rtgui_touch_device *)rt_malloc(sizeof(struct rtgui_touch_device));
    if(touch_device == RT_NULL)
    {
        rt_kprintf("no mem %s,L%d\n", __FILE__, __LINE__);
        return; /* no memory yet */
    }

    /* clear device structure */
    rt_memset(&(touch_device->parent), 0, sizeof(struct rt_device));
    touch_device->calibrating = RT_FALSE;
    touch_device->min_x = X_MIN;
    touch_device->max_x = X_MAX;
    touch_device->min_y = Y_MIN;
    touch_device->max_y = Y_MAX;
    touch_device->eventpost_func  = RT_NULL;
    touch_device->eventpost_param = RT_NULL;

    /* init device structure */
    touch_device->parent.type = RT_Device_Class_Unknown;
    touch_device->parent.init = rtgui_touch_init;
    touch_device->parent.control = rtgui_touch_control;
    touch_device->parent.user_data = RT_NULL;

    device = rt_device_find("lcd");
    if(device == RT_NULL)
    {
        rt_kprintf("no lcd device %s,L%d\n", __FILE__, __LINE__);
        return; /* no this device */
    }
    /* get graphic device info */
    result = rt_device_control(device, RTGRAPHIC_CTRL_GET_INFO, &info);
    if(result != RT_EOK)
    {

        /* get device information failed */
        rt_kprintf("get device information failed  %s,L%d\n", __FILE__, __LINE__);

		return;
	}

    touch_device->width = info.width;
    touch_device->height = info.height;

    /* create 1/8 second timer */
    touch_device->poll_timer = rt_timer_create("touch", touch_timer_fire, RT_NULL,
                               RT_TICK_PER_SECOND / 8, RT_TIMER_FLAG_PERIODIC);

    /* register touch device to RT-Thread */
    rt_device_register(&(touch_device->parent), "touch", RT_DEVICE_FLAG_RDWR);
    rtgui_touch_init(&(touch_device->parent));
    calibration_init();
}
