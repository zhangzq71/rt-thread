#include <rtthread.h>

#ifndef __BLDC_H__
#define __BLDC_H__

void rt_hw_hall_init(void);
uint8_t rt_hw_get_hall_status(void);

void rt_hw_commutate(void);

#endif //__BLDC_H__