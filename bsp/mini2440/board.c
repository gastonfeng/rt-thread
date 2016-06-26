/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-03-24     Bernard      first implementation
 * 2006-05-05     Bernard      add DATA_COUNT definition
 * 2006-10-05     Alsor.Z       for s3c2410x porting
 * 2007-11-20     Yi.Qiu         add lcd,touch,console
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#include<mcu_def.h>
#include "board.h"
//  #include "led.h"
//  #include "serial_2440.h"

/**
 * @addtogroup mini2440
 */
/*@{*/

//  extern rt_uint32_t PCLK, FCLK, HCLK, UCLK;

extern void rt_hw_clock_init(void);
extern void rt_hw_mmu_init(void);

extern void rt_hw_get_clock(void);
extern void rt_hw_set_dividor(rt_uint8_t hdivn, rt_uint8_t pdivn);
extern void rt_hw_set_clock(rt_uint8_t sdiv, rt_uint8_t pdiv, rt_uint8_t mdiv);


/**
 * This function will handle rtos timer
 */
static void rt_timer_handler(int vector, void *param)
{
    rt_tick_increase();
}


/**
 * This function will init timer4 for system ticks
 */
void rt_hw_timer_init()
{
    /* timer4, pre = 15+1 */
    TCFG0 &= 0xffff00ff;
    TCFG0 |= 15 << 8;
    /* all are interrupt mode,set Timer 4 MUX 1/4 */
    TCFG1  &= 0xfff0ffff;
    TCFG1  |= 0x00010000;

    TCNTB4 = (rt_int32_t)(get_PCLK()/ (4 * 16 * RT_TICK_PER_SECOND)) - 1;
    /* manual update */
    TCON = (TCON & (~(0x0f << 20))) | (0x02 << 20);
    /* install interrupt handler */
    rt_hw_interrupt_install(INTTIMER4, rt_timer_handler, RT_NULL, "tick");
    rt_hw_interrupt_umask(INTTIMER4);

    /* start timer4, reload */
    TCON = (TCON & (~(0x0f << 20))) | (0x05 << 20);
//      rt_kprintf("\nTCNTB4=%d,PCLK=%d\n",TCNTB4,PCLK);
}

/**
 * This function will init s3ceb2410 board
 */
void rt_hw_board_init()
{
    /* initialize led port */

//      GPACON = 0x007FFFFF;
//      GPBCON = 0x00055555;
//      GPBUP = 0x000007FF;
//      GPCCON = 0xAAAAAAAA;
//      GPCUP = 0x0000FFFF;
//      GPDCON = 0xAAAAAAAA;
//      GPDUP = 0x0000FFFF;
//      GPECON = 0xAAAAAAAA;
//      GPEUP = 0x0000FFFF;
//    GPFCON = 0x000055AA;
//    GPFUP = 0x000000FF;
//    GPBDAT = GPBDAT | (1 << 5);
//    GPGCON = 0x4094FFBA;
//    GPGUP = 0x00007FEF;
//    GPGDAT = (GPGDAT & (~(1 << 4))) | (1 << 4) ;
//    GPGDAT = GPGDAT | (1 << 15);
//    GPJCON = 0x01555555;
//    GPJUP = 0x00000000;
//    GPJDAT = 0xfff;
//
//
//      GPHCON = 0;
//      GPHUP = 0xff;

//    hw_beep_init();
//    beep(1);
    /* initialize the system clock */
    rt_hw_clock_init();

    /* Get the clock */
    rt_hw_get_clock();

    //      rt_hw_led_init();
    /* initialize uart */
//      rt_hw_uart_init();


    /* initialize mmu */
    rt_hw_mmu_init();
    //      rt_kprintf("START %s ,L %d\n",__FILE__,__LINE__);
    /* initialize timer4 */
}


/*@}*/
