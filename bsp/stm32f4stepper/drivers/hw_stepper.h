#ifndef __HW_STEPPER_H__
#define __HW_STEPPER_H__

typedef enum _tagMotor { 
    MOTOR1 = 0, 
    MOTOR2 = 1,
    MOTOR3 = 2,
    MOTOR4 = 3,
    MOTOR5 = 4,
} motor;

void stepper_init(void);

/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step  Number of steps to move (pos - CW, neg - CCW).
 *  \param accel  Accelration to use, in 0.01*rad/sec^2.
 *  \param decel  Decelration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 */
 void stepper_move(motor m, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif	// __HW_STEPPER_H__
