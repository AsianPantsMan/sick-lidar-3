/*
 * motor_control.h
 *
 *  Created on: Jan 28, 2026
 *      Author: caseyear
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32g4xx_hal.h"
#include "drv8245-q1.h"
#include "tim.h"

//---------------------------------------------------------------------- Motor Parameters ----------------------------------------------------------------------

#define MOTOR_PWM_FREQUENCY 20000  // 20 kHz
#define MOTOR_MAX_DUTY 100         // 100% duty cycle
#define MOTOR_MIN_DUTY 0           // 0% duty cycle

//---------------------------------------------------------------------- Motor Direction ----------------------------------------------------------------------

typedef enum {
    MOTOR_DIRECTION_FORWARD = 0,
    MOTOR_DIRECTION_REVERSE = 1,
    MOTOR_DIRECTION_BRAKE = 2,
    MOTOR_DIRECTION_COAST = 3
} motor_direction_t;

//---------------------------------------------------------------------- Motor State ----------------------------------------------------------------------

typedef enum {
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_RUNNING = 1,
    MOTOR_STATE_FAULT = 2
} motor_state_t;

//---------------------------------------------------------------------- Motor Handle ----------------------------------------------------------------------

typedef struct {
    drv8245_handle_t *drv_handle;
    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;

    motor_state_t state;
    motor_direction_t direction;
    uint8_t duty_cycle;
    int16_t target_speed;
    int16_t current_speed;
} motor_handle_t;

//---------------------------------------------------------------------- Function Declarations ----------------------------------------------------------------------

void motor_init(motor_handle_t *motor);
void motor_setSpeed(motor_handle_t *motor, int16_t speed);
void motor_setDirection(motor_handle_t *motor, motor_direction_t direction);
void motor_enable(motor_handle_t *motor);
void motor_disable(motor_handle_t *motor);
void motor_brake(motor_handle_t *motor);
void motor_coast(motor_handle_t *motor);
void motor_emergencyStop(motor_handle_t *motor);
void motor_updatePWM(motor_handle_t *motor, uint8_t duty_cycle);
motor_state_t motor_getState(motor_handle_t *motor);
void motor_checkFaults(motor_handle_t *motor);

#endif /* INC_MOTOR_CONTROL_H_ */
