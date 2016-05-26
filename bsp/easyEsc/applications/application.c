/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#include "led.h"
#include "bldc.h"
#include "timer.h"

extern vu16 ADC_VALUE[];

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 512 ];
static struct rt_thread led_thread;
static void led_thread_entry(void* parameter)
{
    unsigned int count=0;

    rt_hw_led_init();
    rt_hw_hall_init();
    rt_hw_timer_init();
    
    rt_hw_timer_update_pwmduty(1500);

    while (1)
    {
        rt_hw_commutate();
        /* led1 on */
        rt_kprintf("led on, count : %d, %d,%d,%d,%d,%d,%d\r\n",count, ADC_VALUE[0], ADC_VALUE[1], ADC_VALUE[2], ADC_VALUE[3], ADC_VALUE[4], ADC_VALUE[5]);
        //rt_kprintf("led on, count : %d, %d\r\n",count, rt_hw_get_hall_status());

        count++;
        //rt_hw_led_on(count % 3);
        rt_hw_led_set(7);
        rt_thread_delay( RT_TICK_PER_SECOND / 1 ); /* sleep 0.5 second and switch to other thread */

        rt_hw_commutate();
        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        //rt_hw_led_off(count % 3);
        rt_hw_led_set(0);
        rt_thread_delay( RT_TICK_PER_SECOND / 1 );
    }
}

void rt_init_thread_entry(void* parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif
}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&led_stack[0],
                            sizeof(led_stack),
                            20,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

/*@}*/
