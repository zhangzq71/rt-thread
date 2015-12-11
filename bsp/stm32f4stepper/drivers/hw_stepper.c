#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "hw_stepper.h"

struct stepper_data _stepperData;

/*! \Brief Frequency of TIM2 in [Hz].
 *
 * Modify this according to frequency used. Because of the prescaler setting,
 * the timer1 frequency is the clock frequency divided by 8.
 */
// 
#define T1_FREQ 1000000

//! Number of (full)steps per round on stepper motor in use.
#define SPR 200

// Maths constants. To simplify maths when calculating in speed_cntr_Move().
#define ALPHA (2 * 3.14159 / SPR)                       // 2*pi/spr
#define A_T_x100 ((long)(ALPHA * T1_FREQ * 100))        // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ * 0.676) / 100))    // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA * 2 * 10000000000)            // ALPHA*2*10000000000
#define A_x20000 (int)(ALPHA * 20000)                   // ALPHA*20000

// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3

static const struct stepper_data _motorConfig = {
    .pin_plus = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin = GPIO_Pin_2,
            .GPIO_Mode = GPIO_Mode_OUT,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_Speed = GPIO_Speed_100MHz,
        },
    },
    .pin_dir = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin = GPIO_Pin_3,
            .GPIO_Mode = GPIO_Mode_OUT,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_Speed = GPIO_Speed_100MHz,
        },
    },
    .pin_en = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin = GPIO_Pin_3,
            .GPIO_Mode = GPIO_Mode_OUT,
            .GPIO_OType = GPIO_OType_PP,
            .GPIO_Speed = GPIO_Speed_100MHz,
        },
    },
};

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM5 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void stepper_init(void)
{
    uint16_t PrescalerValue;
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_DeInit(TIM2);
    
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 0; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 1000000) - 1;
    TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);
    
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    NVIC_Configuration();
    
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    static uint32_t cnt = 0;
    
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        TIM2->CCR1 = 10;
        TIM2->ARR = 10;
        
        if ((cnt++ % 100000) == 0)
            rt_kprintf("%d\n", cnt);
    }
}

//INIT_BOARD_EXPORT(stepper_init);
