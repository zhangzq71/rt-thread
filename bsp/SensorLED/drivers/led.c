/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-11-15     bright       the first version
 */

#include <string.h>
#include "led.h"
/* RT_USING_COMPONENTS_INIT */
#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif

static const uint16_t gamma_table[] = {
	/* Gamma 1.3 */
	1,	4,	9,	17,	26,	37,	51,	67,
	84,	104,	126,	149,	175,	203,	233,	265,
	299,	334,	372,	412,	454,	499,	545,	593,
	643,	695,	749,	805,	864,	924,	986,	1050,
	1116,	1185,	1255,	1327,	1402,	1478,	1556,	1637,
	1719,	1803,	1890,	1978,	2068,	2161,	2255,	2352,
	2450,	2550,	2653,	2757,	2864,	2972,	3082,	3195,
	3309,	3426,	3544,	3664,	3787,	3911,	4038,	4166,
	4296,	4429,	4563,	4700,	4838,	4978,	5121,	5265,
	5411,	5560,	5710,	5862,	6017,	6173,	6331,	6492,
	6654,	6818,	6985,	7153,	7323,	7495,	7670,	7846,
	8024,	8204,	8387,	8571,	8757,	8945,	9135,	9327,
	9521,	9718,	9916,	10116,	10318,	10522,	10728,	10936,
	11146,	11358,	11572,	11788,	12006,	12226,	12448,	12672,
	12897,	13125,	13355,	13587,	13821,	14057,	14294,	14534,
	14776,	15020,	15265,	15513,	15763,	16014,	16268,	16524,
	16781,	17041,	17302,	17566,	17831,	18099,	18368,	18640,
	18913,	19189,	19466,	19745,	20027,	20310,	20595,	20883,
	21172,	21463,	21756,	22051,	22349,	22648,	22949,	23252,
	23557,	23864,	24173,	24484,	24797,	25112,	25429,	25748,
	26069,	26391,	26716,	27043,	27372,	27702,	28035,	28370,
	28707,	29045,	29386,	29728,	30073,	30419,	30768,	31118,
	31471,	31825,	32182,	32540,	32900,	33263,	33627,	33993,
	34361,	34731,	35104,	35478,	35854,	36232,	36612,	36994,
	37378,	37764,	38152,	38542,	38933,	39327,	39723,	40121,
	40521,	40922,	41326,	41732,	42139,	42549,	42960,	43374,
	43789,	44207,	44626,	45048,	45471,	45896,	46324,	46753,
	47184,	47617,	48052,	48490,	48929,	49370,	49813,	50258,
	50705,	51154,	51604,	52057,	52512,	52969,	53428,	53888,
	54351,	54816,	55282,	55751,	56222,	56694,	57169,	57645,
	58123,	58604,	59086,	59570,	60057,	60545,	61035,	61527,
	62021,	62517,	63016,	63516,	64018,	64521,	65027,	65535,
};

#define DUTY_MIN    30
#define DUTY_MAX    950

static uint16_t duty_table[256];

static void PIR_sensor_init(void);
static void PWM_init(void);
static void led_timer(void *param);

static rt_timer_t ledTimer;

struct _tag_Led_State {
    unsigned short PIR_raised : 1;
    unsigned short PIR_downed : 1;
    unsigned short PWM_duty : 14;
} LED_State[3];

/* Initial led gpio pin  */
void rt_hw_led_init(void)
{
    int i = 0;
	
    PIR_sensor_init();
    PWM_init();

    for (i = 0; i < 256; i++)
    {
        uint16_t val = (uint16_t)((float)(DUTY_MAX - DUTY_MIN) * (float)gamma_table[i] / (float)65535);
        duty_table[i] = val + 30;
    }

		memset(&LED_State, 0, sizeof(LED_State));
    ledTimer = rt_timer_create("timer1", led_timer, RT_NULL, 1, RT_TIMER_FLAG_PERIODIC);
    if (ledTimer != RT_NULL)
        rt_timer_start(ledTimer);
}

static void PIR_sensor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
	
    // Enable 3 EXTI line for PIR modules
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /*Configure GPIO pins : PA1 PA2 PA3 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // setup PA.4 for controlling enable/disable of XL6001

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1 | EXTI_PinSource2 | EXTI_PinSource3);

    EXTI_InitStruct.EXTI_Line = EXTI_Line1 | EXTI_Line2 | EXTI_Line3;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&EXTI_InitStruct);

    /* EXTI interrupt init*/
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_1_IRQn | EXTI2_3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
		NVIC->ISER[0] = 0x10060;
}

static void PWM_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // TIM3 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* GPIOC and GPIOA and GPIOB clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);

    /* GPIOA Configuration: TIM3 CH1 (PA6) and TIM3 CH2 (PA7) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIOB Configuration: TIM3 CH4 (PB1) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Connect TIM3 pins to AF1 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_Period = 2000;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 10;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 10;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 10;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void led_pir_int(char which, char isRaised)
{

}

static void led_timer(void *param)
{
    static char count_down = 0;

    if (LED_State[0].PIR_raised == 1)
    {
        TIM_SetCompare1(TIM3, duty_table[LED_State[0].PWM_duty]);
        TIM_SetCompare2(TIM3, duty_table[LED_State[0].PWM_duty]);
        TIM_SetCompare4(TIM3, duty_table[LED_State[0].PWM_duty]);
			
        if (LED_State[0].PWM_duty < 255)
            LED_State[0].PWM_duty++;
    }
    else if (LED_State[0].PIR_downed == 1)
    {
        TIM_SetCompare1(TIM3, duty_table[LED_State[0].PWM_duty]);
        TIM_SetCompare2(TIM3, duty_table[LED_State[0].PWM_duty]);
        TIM_SetCompare4(TIM3, duty_table[LED_State[0].PWM_duty]);

        if (count_down++ == 1)
        {
            if (LED_State[0].PWM_duty > 0)
            {
                LED_State[0].PWM_duty--;
            }
            else
            {
                LED_State[0].PIR_downed = 0;
            }

            count_down = 0;
        }
    }
}


void EXTI0_1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_SET)
        {
            LED_State[0].PIR_raised = 1;
        }
        else
        {
            LED_State[0].PIR_raised = 0;
            LED_State[0].PIR_downed = 1;
        }

        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI2_3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == Bit_SET)
        {
            LED_State[1].PIR_raised = 1;
        }
        else
        {
            LED_State[1].PIR_raised = 0;
            LED_State[1].PIR_downed = 1;
        }

        EXTI_ClearITPendingBit(EXTI_Line2);
    }
    else if (EXTI_GetITStatus(EXTI_Line3) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET)
        {
            LED_State[2].PIR_raised = 1;
        }
        else
        {
            LED_State[2].PIR_raised = 0;
            LED_State[2].PIR_downed = 1;
        }
        
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
