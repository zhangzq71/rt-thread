#ifndef __HW_STEPPER_H__
#define __HW_STEPPER_H__

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

struct stepper_data {
    struct speed_ramp_data speed_ramp;
    struct stm32_gpio pin_plus;
    struct stm32_gpio pin_dir;
    struct stm32_gpio pin_en;
};

#endif	// __HW_STEPPER_H__
