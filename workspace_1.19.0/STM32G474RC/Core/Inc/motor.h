/*
 * motor.h
 *
 *  Created on: Mar 7, 2026
 *      Author: Nguyen
 */
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include "tim.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

/* ============================================================================
 * MOTOR SPECIFICATIONS
 * ============================================================================ */
#define MOTOR_PPR               1996.3f     // Pulses Per Revolution (encoder resolution)
#define MOTOR_CPR               996.8f      // Counts Per Revolution (quadrature mode)
#define MAX_RPM                 280.0f      // Maximum motor speed in RPM
#define MAX_SPEED_TICKS_PER_SEC (MOTOR_PPR * (MAX_RPM / 60.0f))  // Max speed in ticks/sec


typedef struct {
    /* ------------------------------------------------------------------------
     * Hardware Interfaces
     * ------------------------------------------------------------------------ */
    GPIO_TypeDef *nsleep_port;
    uint16_t nsleep_pin;

    GPIO_TypeDef *nfault_port;
    uint16_t nfault_pin;

    GPIO_TypeDef *direction_port;
    uint16_t direction_pin;

    GPIO_TypeDef *led_port;
    uint16_t led_pin;

    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_channel;

    TIM_HandleTypeDef* encoder_timer;

    int16_t rpm;

    bool reverse;

    /* ------------------------------------------------------------------------
     * PID Tuning Parameters
     * ------------------------------------------------------------------------ */
    float Kp;                               // Proportional gain
    float Ki;                               // Integral gain
    float Kd;                               // Derivative gain
    float tau;                              // Derivative filter time constant (seconds)

    /* ------------------------------------------------------------------------
     * Setpoint (Target Speed)
     * ------------------------------------------------------------------------ */
    volatile int32_t target_speed_tics;     // Desired speed in ticks/sec (from RPi)

    /* ------------------------------------------------------------------------
     * Encoder State Variables
     * ------------------------------------------------------------------------ */
    volatile int16_t last_encoder_count;    // Previous encoder reading
    volatile int16_t current_encoder_count; // Current encoder reading

    /* ------------------------------------------------------------------------
     * Speed Measurement (Process Variable)
     * ------------------------------------------------------------------------ */
    float current_speed_tics;               // Filtered current speed (ticks/sec)
    float last_speed_tics;                  // Previous speed for derivative calculation

    /* ------------------------------------------------------------------------
     * PID Internal State
     * ------------------------------------------------------------------------ */
    float error;                            // Current error (setpoint - measurement)
    float last_error;                       // Previous error for trapezoidal integration
    float error_integral;                   // Accumulated integral (deprecated - use i_term)

    float p_term;                           // Proportional component
    float i_term;                           // Integral component (with anti-windup)
    float d_term;                           // Derivative component (filtered)

    float pid_output;                       // Combined PID output (ticks/sec)
    float duty_cycle;
    /* ------------------------------------------------------------------------
     * Output
     * ------------------------------------------------------------------------ */
    //int8_t output_pwm_duty;                 // Final PWM duty cycle (-100 to +100)

} Motor;

void Motor_Init(Motor* motor, bool reverse,
		GPIO_TypeDef *nsleep_port, uint16_t nsleep_pin,
		GPIO_TypeDef *nfault_port, uint16_t nfault_pin,
		GPIO_TypeDef *direction_port, uint16_t direction_pin,
		GPIO_TypeDef *led_port, uint16_t led_pin,
		TIM_HandleTypeDef *pwm_timer, uint32_t pwm_channel,
		TIM_HandleTypeDef *encoder_timer, int16_t rpm);

#endif /* INC_MOTOR_H_ */
