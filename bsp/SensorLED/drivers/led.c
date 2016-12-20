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
	0,	0,	0,	0,	0,	0,	1,	1,
	2,	3,	4,	5,	7,	9,	11,	13,
	16,	19,	23,	27,	32,	37,	42,	48,
	55,	62,	69,	78,	87,	96,	107,	118,
	130,	142,	155,	169,	184,	200,	217,	234,
	253,	272,	293,	314,	337,	360,	385,	410,
	437,	465,	494,	524,	556,	588,	622,	658,
	694,	732,	771,	812,	854,	897,	942,	988,
	1036,	1085,	1136,	1189,	1243,	1298,	1356,	1415,
	1475,	1538,	1602,	1667,	1735,	1804,	1876,	1949,
	2024,	2100,	2179,	2260,	2343,	2427,	2514,	2603,
	2693,	2786,	2881,	2978,	3078,	3179,	3283,	3389,
	3497,	3607,	3720,	3835,	3952,	4072,	4194,	4319,
	4446,	4575,	4707,	4842,	4979,	5118,	5261,	5405,
	5553,	5703,	5856,	6011,	6169,	6330,	6494,	6660,
	6830,	7002,	7177,	7355,	7536,	7719,	7906,	8096,
	8289,	8484,	8683,	8885,	9090,	9298,	9510,	9724,
	9942,	10163,	10387,	10614,	10845,	11079,	11317,	11557,
	11802,	12049,	12300,	12555,	12813,	13074,	13339,	13608,
	13880,	14156,	14435,	14718,	15005,	15295,	15589,	15887,
	16189,	16494,	16803,	17117,	17433,	17754,	18079,	18408,
	18740,	19077,	19418,	19762,	20111,	20464,	20821,	21182,
	21547,	21917,	22290,	22668,	23050,	23436,	23827,	24222,
	24621,	25025,	25433,	25845,	26262,	26683,	27109,	27539,
	27974,	28413,	28857,	29306,	29759,	30217,	30680,	31147,
	31619,	32095,	32577,	33063,	33554,	34050,	34551,	35056,
	35567,	36082,	36602,	37128,	37658,	38194,	38734,	39280,
	39830,	40386,	40947,	41513,	42084,	42661,	43243,	43830,
	44422,	45019,	45622,	46231,	46844,	47463,	48088,	48718,
	49353,	49994,	50641,	51293,	51950,	52614,	53282,	53957,
	54637,	55323,	56014,	56712,	57415,	58123,	58838,	59558,
	60285,	61017,	61755,	62499,	63249,	64005,	64767,	65535,
};

#define DUTY_MIN    30
#define DUTY_MAX    955
#define DELAY_TIME  3

static uint16_t duty_table[256];

static void PIR_sensor_init(void);
static void PWM_init(void);
static void led_timer(void *param);

static rt_timer_t ledTimer;

struct _tag_Led_State {
    unsigned short PIR_raised : 1;
    unsigned short PIR_downed : 1;
    unsigned short delay_time : 6;
    unsigned short PWM_duty : 8;
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
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
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
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIOB Configuration: TIM3 CH4 (PB1) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
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
    static char count_time = 0;
    static char count_down = 0;

    if (LED_State[0].PIR_raised == 1)
    {
        TIM_SetCompare1(TIM3, duty_table[LED_State[0].PWM_duty]);
        TIM_SetCompare2(TIM3, duty_table[LED_State[0].PWM_duty]);
        TIM_SetCompare4(TIM3, duty_table[LED_State[0].PWM_duty]);
			
        if (LED_State[0].PWM_duty < 255)
            LED_State[0].PWM_duty++;

        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == Bit_RESET)
        {
            if (count_time++ >= 100)
            {
                count_time = 0;

                LED_State[0].delay_time++;
                if (LED_State[0].delay_time > DELAY_TIME)
                {
                    LED_State[0].PIR_downed = 1;
                    LED_State[0].PIR_raised = 0;

                    LED_State[0].delay_time = 0;
                }
            }
        }
        else
        {
            LED_State[0].delay_time = 0;
            count_time = 0;
        }
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
            LED_State[0].delay_time = 0;
        }
        else
        {
            // LED_State[0].PIR_raised = 0;
            // LED_State[0].PIR_downed = 1;
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

            LED_State[0].PIR_raised = 1;
            LED_State[0].delay_time = 0;

        }
        else
        {
            // LED_State[1].PIR_raised = 0;
            // LED_State[1].PIR_downed = 1;
        }

        EXTI_ClearITPendingBit(EXTI_Line2);
    }
    else if (EXTI_GetITStatus(EXTI_Line3) == SET)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == Bit_SET)
        {
            LED_State[2].PIR_raised = 1;

            LED_State[0].PIR_raised = 1;
            LED_State[0].delay_time = 0;
        }
        else
        {
            // LED_State[2].PIR_raised = 0;
            // LED_State[2].PIR_downed = 1;
        }
        
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}
