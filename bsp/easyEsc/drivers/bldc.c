#include <rtthread.h>
#include <stm32f10x.h>
#include "timer.h"

//#define SENSOR_LOGIC                 1

#define ADC1_DR_ADDR                    ((uint32_t)0x4001244C)

#define ADC_CHANNEL_VCC                 ADC_Channel_3
#define ADC_CHANNEL_CURRENT             ADC_Channel_8
#define ADC_CHANNEL_SPEED               ADC_Channel_9

#if defined SENSOR_LOGIC
 #define ADC_NUMBER                     3
#else
 #define ADC_NUMBER                     6
 
 // define the ADC input channel
 #define ADC_CHANNEL_HA                  ADC_Channel_7
 #define ADC_CHANNEL_HB                  ADC_Channel_6
 #define ADC_CHANNEL_HC                  ADC_Channel_5
#endif  

static uint8_t _hallStatus = 0;
vu16 ADC_VALUE[ADC_NUMBER] = { 0 };

static void adc_init(void);

void rt_hw_hall_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);

    // define ADC port    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#if defined SENSOR_LOGIC    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#else
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

    adc_init();
}

uint8_t rt_hw_get_hall_status(void)
{
    _hallStatus = GPIO_ReadInputData(GPIOA) & ((1 << 7) | (1 << 6) | (1 << 5));
    _hallStatus = (_hallStatus) >> 5;
    
    return _hallStatus;
}

static void adc_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_ADCCLKConfig(RCC_PCLK2_Div4);

    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_VALUE;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ADC_NUMBER;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    //DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // ADC内置温度传感器禁止
    ADC_TempSensorVrefintCmd(DISABLE);

    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = ADC_NUMBER;
    ADC_Init(ADC1, &ADC_InitStructure);

#if defined SENSOR_LOGIC
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_VCC,     1, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_CURRENT, 2, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_SPEED,   3, ADC_SampleTime_7Cycles5);
#else
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_VCC,     1, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_CURRENT, 2, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_SPEED,   3, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_HA, 4, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_HB, 5, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_HC, 6, ADC_SampleTime_7Cycles5);
#endif

    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 external trigger */
    ADC_ExternalTrigConvCmd(ADC1, DISABLE);

    ADC_Cmd(ADC1, ENABLE);
    /* 重置ADC校准寄存器 */
    ADC_ResetCalibration(ADC1);
    /* 检测ADC校准寄存器状态 */
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* 开始ADC校准程序 */
    ADC_StartCalibration(ADC1);
    /* 检测ADC校准状态 */
    while(ADC_GetCalibrationStatus(ADC1));

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void rt_hw_commutate(void)
{
    _hallStatus++;
    _hallStatus = _hallStatus % 6;
    
    u8 hall = _hallStatus + 1;
    
    Disable_AH();
    Disable_AL();
    Disable_BH();
    Disable_BL();
    Disable_CH();
    Disable_CL();
    
    switch (hall)
    {
    /* 顺时针方向旋转 */
        case 3:
        {
            Enable_CH();
            Enable_AL();
        }
        break;

        case 1:
        {
            Enable_CH();
            Enable_BL();
        }
        break;

        case 5:
        {
            Enable_AH();
            Enable_BL();
        }
        break;

        case 4:
        {
            Enable_AH();
            Enable_CL();;
        }
        break;

        case 6:
        {
            Enable_BH();
            Enable_CL();
        }
        break;

        case 2:
        {
            Enable_BH();
            Enable_AL();
        }
        break;
    }
}