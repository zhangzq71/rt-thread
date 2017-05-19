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
 * 2014-04-27     Bernard      make code cleanup. 
 */

#include <board.h>
#include <rtthread.h>
#include <drivers/mtd_nand.h>
#include "finsh.h"
#include "time.h"
#ifdef RT_USING_DFS
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#include <dfs_posix.h>
#endif

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32f4xx_eth.h"
#endif

#ifdef RT_USING_GDB
#include <gdb_stub.h>
#endif
//rt_module_t module_ptr;
#define DATA_PATH "/Data"
void rt_init_thread_entry(void* parameter)
{    
     /* initialization RT-Thread Components */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_init();
#endif
    
    /* GDB STUB */
#ifdef RT_USING_GDB
    gdb_set_device("uart6");
    gdb_start();
#endif
#ifdef RT_USING_DFS  
    #ifdef RT_USING_DFS_UFFS
        /* mount nand flash partition 0 as root directory */
    if (dfs_mount("nand0", "/", "uffs", 0, 0) == 0)
    {
        mkdir("/sdcard",0);
        rt_kprintf("uffs initialized!\n");
    }
    else        
    {
        rt_kprintf("uffs initialzation failed!\n");
    }
            
    #endif /* RT_USING_DFS_UFFS */    
    #ifdef RT_USING_DFS_ELMFAT
        /* mount sd card fat partition 0 as root directory */
        if (dfs_mount("sd0", "/sdcard", "elm", 0, 0) == 0)
        {
            rt_kprintf("File System initialized!\n");        
        }
        else
        {
            rt_kprintf("File System initialzation failed!\n");
        }
            
    #endif /* RT_USING_DFS_ELMFAT */
//    module_ptr = rt_module_open("/hello.mo");
    
        
#endif /* DFS */
    /* LwIP Initialization */
#ifdef RT_USING_LWIP
    {
        extern void lwip_sys_init(void);

        /* register ethernetif device */
        eth_system_device_init();

        rt_hw_stm32_eth_init();

        /* init lwip system */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");
    }
#endif
}
int rt_application_init()
{
    rt_thread_t tid;


    tid = rt_thread_create("init",
        rt_init_thread_entry, RT_NULL,
        2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (tid != RT_NULL)
        rt_thread_startup(tid);
    


    return 0;
}
