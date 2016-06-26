/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-02-26     Bernard      first implementation
 * 2006-05-05     Bernard      add two test thread
 * 2006-08-10     Bernard      use rt_show_version to display version information
 * 2008-07-14     Bernard      modify the heap memory init parameter
 */

#include <rthw.h>
#include <rtthread.h>
#ifdef CONFIG_S3C2416
#include <S3C2416.h>
#else
#include <s3c24x0.h>
#endif
#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

extern void rt_hw_interrupt_init(void);
extern void rt_hw_board_init(void);
extern void rt_hw_rtc_init(void);
extern void rt_serial_init(void);
extern void rt_system_timer_init(void);
extern void rt_system_scheduler_init(void);
extern void rt_thread_idle_init(void);
extern void rt_hw_cpu_icache_enable(void);
extern void rt_show_version(void);
extern void rt_system_heap_init(void *, void *);
extern void rt_hw_finsh_init(void);

/**
 * @addtogroup mini2440
 */

/*@{*/

#ifdef RT_USING_FINSH
extern void finsh_system_init(void);
#endif

/*@}*/
