/*
 * File      : trap.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-03-13     Bernard      first version
 */

#include <rtthread.h>
#include <rthw.h>
#ifdef STM32
#include "stm32f10x.h"
#endif

#define MAX_HANDLERS  60

extern rt_uint8_t rt_interrupt_nest;

/* exception and interrupt handler table */
struct rt_irq_desc isr_table[MAX_HANDLERS];
rt_uint32_t rt_interrupt_from_thread, rt_interrupt_to_thread;
rt_uint32_t rt_thread_switch_interrupt_flag;

/**
 * @addtogroup S3C24X0
 */
/*@{*/

static void rt_hw_interrupt_handle(int vector, void *param)
{
    rt_kprintf("Unhandled interrupt %d occured!!! %s,%d\n", vector, __FILE__, __LINE__);
}

/**
 * This function will initialize hardware interrupt
 */
void rt_hw_interrupt_init(void)
{
    register rt_uint32_t idx;

    /* init exceptions table */
    rt_memset(isr_table, 0x00, sizeof(isr_table));
    for(idx = 0; idx < MAX_HANDLERS; idx++)
    {
        isr_table[idx].handler = rt_hw_interrupt_handle;
    }

    /* init interrupt nest, and context in thread sp */
    rt_interrupt_nest = 0;
    rt_interrupt_from_thread = 0;
    rt_interrupt_to_thread = 0;
    rt_thread_switch_interrupt_flag = 0;
}

/**
 * This function will install a interrupt service routine to a interrupt.
 * @param vector the interrupt number
 * @param new_handler the interrupt service routine to be installed
 * @param old_handler the old interrupt service routine
 */
rt_isr_handler_t rt_hw_interrupt_install(int vector, rt_isr_handler_t handler,
        void *param, char *name)
{
    rt_isr_handler_t old_handler = RT_NULL;
    if(vector < MAX_HANDLERS)
    {
        old_handler = isr_table[vector].handler;
        if(handler != RT_NULL)
        {
#ifdef RT_USING_INTERRUPT_INFO
            rt_strncpy(isr_table[vector].name, name, RT_NAME_MAX);
#endif /* RT_USING_INTERRUPT_INFO */
            isr_table[vector].handler = handler;
            isr_table[vector].param = param;
            rt_kprintf("\n interrupt %d:%s installed. %s,%s,%d\n", vector, name, __FUNCTION__, __FILE__, __LINE__);
        }
        else
        {
            rt_kprintf("\n ***NULL INTERRUPT! %s %s %d\n", __FUNCTION__, __FILE__, __LINE__);
        }
    }

    else
    {
        rt_kprintf("\n ***vector %d > MAX_HANDLERS! %s %s %d\n", vector, __FUNCTION__, __FILE__, __LINE__);
    }
    return old_handler;
}

void rt_hw_irq(unsigned short intstat)
{
    rt_isr_handler_t isr_func;

    /* get interrupt service routine */
    isr_func = isr_table[intstat].handler;

    /* turn to interrupt service routine */
    isr_func(intstat,isr_table[intstat].param);

    /* clear pending register */
    /* note: must be the last, if not, may repeat*/
}

/*@}*/
