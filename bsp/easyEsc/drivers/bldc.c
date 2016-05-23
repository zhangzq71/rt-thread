#include <rtthread.h>
#include <stm32f10x.h>

#define hall1_rcc                    RCC_APB2Periph_GPIOA
#define hall1_gpio                   GPIOA
#define hall1_pin                    (GPIO_Pin_5)

#define hall2_rcc                    RCC_APB2Periph_GPIOA
#define hall2_gpio                   GPIOA
#define hall2_pin                    (GPIO_Pin_6)

#define hall3_rcc                    RCC_APB2Periph_GPIOA
#define hall3_gpio                   GPIOA
#define hall3_pin                    (GPIO_Pin_7)

uint8_t _hallStatus = 0;

void rt_hw_hall_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t rt_hw_get_hall_status(void)
{
    _hallStatus = GPIO_ReadInputData(GPIOA) & (7 << 5));
    _hallStatus = (_hallStatus) >> 5;
    
    return _hallStatus;
}

