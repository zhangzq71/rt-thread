#include <rtthread.h>
#include <stm32f10x.h>

#include "timer.h"

static void tim1_init(void);
static void tim1_nvic_init(void);
static void tim1_gpio_init(void);
static void Disable_PwmOutput(void);

/* 定义系统时钟 */
#define TIM1_FREQ				72000000

/* 定义PWM输出频率 */
//#define PWM_FREQ				16000
#define PWM_FREQ				24000

/* 定义TIM1周期 */
#define TIM1_PERIOD				(TIM1_FREQ / PWM_FREQ - 1)

/* 定义100%占空比 */
#define FULL_DUTY				TIM1_PERIOD


void rt_hw_timer_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	tim1_nvic_init();
	tim1_gpio_init();
	tim1_init();	
}

static void tim1_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef			TIM_OCInitStructure;
	TIM_BDTRInitTypeDef			TIM_BDTRInitStructure;
	
	/* 不分频，ABP2时钟=72MHz */
	//向上计数
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM1_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* PWM模式1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	/* 正向通道有效 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	/* 反向通道有效 */
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	/* 占空比时间 */
	TIM_OCInitStructure.TIM_Pulse = (u32)FULL_DUTY*10/100;
	/* PWM输出极性 */
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	/* 空闲状态下的非工作状态 */
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

#if defined SENSORLESS_CONTROL
    /* 配置CH4，用于同步ADC采样 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = (u32)FULL_DUTY*10/100;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
#endif

    /* CCR1的预加载寄存器使能，每次更新事件发生后才更新占空比 */
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 2;		           //0-255
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

#if defined SENSORLESS_CONTROL
	TIM_ITConfig(TIM1, TIM_IT_CC4, ENABLE);
	Enable_TIM1_CC4_IT();
#endif

	// TIM1 counter enable
	TIM_Cmd(TIM1, ENABLE);

	/* 主输出使能 */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	Disable_PwmOutput();
}

static void tim1_nvic_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* TIM1定时器捕获比较中断 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void tim1_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* PWM端口 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void rt_hw_timer_update_pwmduty(u16 duty)
{
	TIM1->CCR1 = duty;
	TIM1->CCR2 = duty;
	TIM1->CCR3 = duty;
	/* 在PWM ON的中点检测过零点 */
	TIM1->CCR4 = duty >> 1;
}

static void Disable_PwmOutput(void)
{
	Disable_AH();
	Disable_AL();
	Disable_BH();
	Disable_BL();
	Disable_CH();
	Disable_CL();
}

/* 定时器捕获比较事件中断 */
// 从ADC缓冲取得结果
#if defined SENSORLESS_CONTROL
void TIM1_CC_IRQHandler(void)
{
	if (Flag_TIM1_CC4_IF())
	{
		Clear_TIM1_CC4_IF();
	}
}
#endif