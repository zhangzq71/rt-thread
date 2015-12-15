#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <math.h>
#include "hw_stepper.h"

// Direction of stepper motor movement
#define CW  0
#define CCW 1


/*! \brief Holding data used by timer interrupt for speed ramp calculation.
 *
 *  Contains data used by timer interrupt to calculate speed profile.
 *  Data is written to it by move(), when stepper motor is moving (timer
 *  interrupt running) data is read/updated when calculating a new step_delay
 */
struct speed_ramp_data {
    //! What part of the speed ramp we are in.
    unsigned char run_state : 3;
    //! Direction stepper motor should move.
    unsigned char dir : 1;
    //! Peroid of next timer delay. At start this value set the accelration rate.
    unsigned int step_delay;
    //! What step_pos to start decelaration
    unsigned int decel_start;
    //! Sets deceleration rate.
    signed int decel_val;
    //! Minimum time delay (max speed)
    signed int min_delay;
    //! Counter used when accelerateing/decelerateing to calculate step_delay.
    signed int accel_count;
};

struct stm32_gpio {
    GPIO_TypeDef *gpio;
    GPIO_InitTypeDef init;
};

struct stm32_timer {
    TIM_TypeDef *timer;
};

struct stepper_data {
    struct speed_ramp_data speed_ramp;
    struct stm32_timer pwm_timer;
    struct stm32_gpio pin_plus;
    struct stm32_gpio pin_dir;
    struct stm32_gpio pin_en;
};


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

static struct stepper_data _motorConfig[] = {
    {
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
        .pwm_timer = TIM2,
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
    
    TIM_DeInit(_motorConfig[0].pwm_timer.timer);
    
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 0; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(_motorConfig[0].pwm_timer.timer, &TIM_TimeBaseStructure);
    
    PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 1000000) - 1;
    TIM_PrescalerConfig(_motorConfig[0].pwm_timer.timer, PrescalerValue, TIM_PSCReloadMode_Immediate);
    
    TIM_ClearFlag(_motorConfig[0].pwm_timer.timer, TIM_FLAG_Update);
    TIM_ITConfig(_motorConfig[0].pwm_timer.timer, TIM_IT_Update, ENABLE);
    
    NVIC_Configuration();
    
    TIM_Cmd(_motorConfig[0].pwm_timer.timer, ENABLE);
}

void stepper_move(motor m, uint32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
    //! Number of steps before we hit max speed.
    unsigned int max_s_lim;
    //! Number of steps before we must start deceleration (if accel does not hit max speed).
    unsigned int accel_lim;

    // Set direction from sign on step value.
    if (step < 0)
    {
        _motorConfig[m].speed_ramp.dir = CCW;
        step = -step;
    }
    else
    {
        _motorConfig[m].speed_ramp.dir = CW;
    }

    // If moving only 1 step.
    if (step == 1)
    {
        // Move one step...
        _motorConfig[m].speed_ramp.accel_count = -1;
        // ...in DECEL state.
        _motorConfig[m].speed_ramp.run_state = DECEL;
        // Just a short delay so main() can act on 'running'.
        _motorConfig[m].speed_ramp.step_delay = 1000;

        _motorConfig[m].pwm_timer.timer->CCR1 = 10;
        _motorConfig[m].pwm_timer.timer->ARR = 10;
    }
    // Only move if number of steps to move is not zero.
    else if (step != 0)
    {
        // Refer to documentation for detailed information about these calculations.

        // Set max speed limit, by calc min_delay to use in timer.
        // min_delay = (alpha / tt)/ w
        _motorConfig[m].speed_ramp.min_delay = A_T_x100 / speed;

        // Set accelration by calc the first (c0) step delay .
        // step_delay = 1/tt * sqrt(2*alpha/accel)
        // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
        _motorConfig[m].speed_ramp.step_delay = (unsigned int)((float)T1_FREQ_148 * (float)sqrt((float)A_SQ / (float)accel)) / (float)100;

        // Find out after how many steps does the speed hit the max speed limit.
        // max_s_lim = speed^2 / (2*alpha*accel)
        max_s_lim = (long)speed * speed / (long)(((long)A_x20000 * accel) / 100);
        // If we hit max speed limit before 0,5 step it will round to 0.
        // But in practice we need to move atleast 1 step to get any speed at all.
        if (max_s_lim == 0)
        {
            max_s_lim = 1;
        }

        // Find out after how many steps we must start deceleration.
        // n1 = (n1+n2)decel / (accel + decel)
        accel_lim = ((long)step * decel) / (accel + decel);
        // We must accelrate at least 1 step before we can start deceleration.
        if (accel_lim == 0)
        {
            accel_lim = 1;
        }

        // Use the limit we hit first to calc decel.
        if (accel_lim <= max_s_lim)
        {
            _motorConfig[m].speed_ramp.decel_val = accel_lim - step;
        }
        else
        {
            _motorConfig[m].speed_ramp.decel_val = -((long)max_s_lim*accel)/decel;
        }
        // We must decelrate at least 1 step to stop.
        if (_motorConfig[m].speed_ramp.decel_val == 0)
        {
            _motorConfig[m].speed_ramp.decel_val = -1;
        }

        // Find step to start decleration.
        _motorConfig[m].speed_ramp.decel_start = step + _motorConfig[m].speed_ramp.decel_val;

        // If the maximum speed is so low that we dont need to go via accelration state.
        if (_motorConfig[m].speed_ramp.step_delay <= _motorConfig[m].speed_ramp.min_delay)
        {
            _motorConfig[m].speed_ramp.step_delay = _motorConfig[m].speed_ramp.min_delay;
            _motorConfig[m].speed_ramp.run_state = RUN;
        }
        else
        {
            _motorConfig[m].speed_ramp.run_state = ACCEL;
        }

        // Reset counter.
        _motorConfig[m].speed_ramp.accel_count = 0;

        _motorConfig[m].pwm_timer.timer->CCR1 = 10;
        _motorConfig[m].pwm_timer.timer->ARR = 10;
        TIM_Cmd(_motorConfig[m].pwm_timer.timer, ENABLE);
    }
}

void timer_handler(motor m)
{
    // Holds next delay period.
    unsigned int new_step_delay;
    // Remember the last step delay used when accelrating.
    static int last_accel_delay;
    // Counting steps when moving.
    static unsigned int step_count = 0;
    // Keep track of remainder from new_step-delay calculation to incrase accurancy
    static unsigned int rest = 0;

    _motorConfig[m].pwm_timer.timer->CCR1 = _motorConfig[m].speed_ramp.step_delay;
    _motorConfig[m].pwm_timer.timer->ARR = _motorConfig[m].speed_ramp.step_delay;

    switch (_motorConfig[m].speed_ramp.run_state)
    {
        case STOP:
            break;

        case ACCEL:
            break;

        case RUN:
            break;

        case DECEL:
            break;
    }

    _motorConfig[m].speed_ramp.step_delay = new_step_delay;
}

void TIM2_IRQHandler(void)
{
    static uint32_t cnt = 0;

    if (TIM_GetITStatus(_motorConfig[0].pwm_timer.timer, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(_motorConfig[0].pwm_timer.timer, TIM_IT_Update);
        
        _motorConfig[0].pwm_timer.timer->CCR1 = 10;
        _motorConfig[0].pwm_timer.timer->ARR = 10;
        
        if ((cnt++ % 100000) == 0)
            rt_kprintf("%d\n", cnt);
    }
}

//INIT_BOARD_EXPORT(stepper_init);
