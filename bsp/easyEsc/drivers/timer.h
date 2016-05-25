#include <rtthread.h>

#ifndef __TIMER_H__
#define __TIMER_H__

#define Enable_AH()   TIM_CCxCmd( TIM1,TIM_Channel_1,TIM_CCx_Enable)
#define Disable_AH()  TIM_CCxCmd( TIM1,TIM_Channel_1,TIM_CCx_Disable)
#define Enable_AL()   TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable)
#define Disable_AL()  TIM_CCxNCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable)
#define Enable_BH()   TIM_CCxCmd( TIM1,TIM_Channel_2,TIM_CCx_Enable)
#define Disable_BH()  TIM_CCxCmd( TIM1,TIM_Channel_2,TIM_CCx_Disable)
#define Enable_BL()   TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Enable)
#define Disable_BL()  TIM_CCxNCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable)
#define Enable_CH()   TIM_CCxCmd( TIM1,TIM_Channel_3,TIM_CCx_Enable)
#define Disable_CH()  TIM_CCxCmd( TIM1,TIM_Channel_3,TIM_CCx_Disable)
#define Enable_CL()   TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Enable)
#define Disable_CL()  TIM_CCxNCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable)

void rt_hw_timer_init(void);
void rt_hw_timer_update_pwmduty(u16 duty);

#endif //__TIMER_H__