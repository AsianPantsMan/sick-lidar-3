/*
 * pid.h
 *
 *  Created on: Feb 25, 2026
 *      Author: caseyear
 */
#include "stm32g4xx_hal.h"
#include "tim.h"
#include <stdint.h>
#include <stdbool.h>
#include "drv8245-q1.h"
#include "butterworth_filter.h"

#ifndef INC_PID_H_
#define INC_PID_H_

#define MOTOR_PPR 					1996.3f
#define MOTOR_CPR 					996.8f

#define MAX_RPM 					260.0f
#define MAX_SPEED_TICKS_PER_SEC		(MOTOR_PPR*(MAX_RPM/60))

#define pid_loop_frequency			100.0f
#define period						(1.0f / pid_loop_frequency)

#define integral_limit 				5000

typedef struct {
    // Hardware handles
    TIM_HandleTypeDef* pwm_timer;
    uint32_t pwm_channel;
    TIM_HandleTypeDef* encoder_timer;

    // PID Parameters (tunable)
    float Kp;
    float Ki;
    float Kd;

    // Target setpoint from Raspberry Pi (rpm)
    volatile int32_t target_speed_rpm;
    volatile int32_t previous_speed_rpm;

    // Encoder state
    volatile int16_t last_encoder_count;
    volatile int16_t current_encoder_count;

    // Calculated current speed (Process Variable)
    float current_speed_rpm;
    float rpm_filt;          // filtered RPM state
	float rpm_tau;

    // PID internal state
    float error_integral;
    float last_error;
    float pid_output;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float raw_speed_rpm;

    //Filter
    Butterworth_Filter_t rpm_filter;

    // Output
    int8_t output_pwm_duty;


} MotorControl_t;

void MotorControl_Init(MotorControl_t* motor,
                       TIM_HandleTypeDef* pwm_timer,
                       uint32_t pwm_channel,
                       TIM_HandleTypeDef* encoder_timer,
                       float Kp, float Ki, float Kd);

void MotorControl_SetTargetSpeed(MotorControl_t* motor, int32_t target_rpm);
int32_t MotorControl_getTargetRpm(MotorControl_t* motor);

float MotorControl_getRpm(MotorControl_t* motor);

void MotorControl_RunPID(MotorControl_t* motor);

float MotorControl_getIntegral(MotorControl_t* motor);

float MotorControl_duty(MotorControl_t* motor);

float MotorControl_pidOutput(MotorControl_t* motor);

float MotorControl_getP(MotorControl_t* motor);

float MotorControl_getI(MotorControl_t* motor);

float MotorControl_getD(MotorControl_t* motor);

float MotorControl_getError(MotorControl_t* motor);

float MotorControl_getRawspeed(MotorControl_t* motor);

#endif /* INC_PID_H_ */


