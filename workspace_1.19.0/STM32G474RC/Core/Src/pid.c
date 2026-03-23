/*
 * pid.c
 *
 * PID Motor Control Implementation
 * Uses discrete-time PID with trapezoidal integration and filtered derivative
 *
 * Created on: Jan 22, 2025
 * Author: caseyear
 */

#include "pid.h"
#include <stdlib.h>
#include <stm32g4xx_hal_tim.h>
#include "drv8245-q1.h"
#include "stm32g4xx_hal.h"

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */
void MotorControl_Init(Motor* motor,
                       float Kp, float Ki, float Kd)
{

    /* Set PID gains */
    //Kp
    if (Kp < 0){
    	motor->Kp = 0;
    } else{
    	motor->Kp = Kp;
    }

    //Ki
    if (Ki < 0){
    	motor->Ki = 0;
    } else{
    	motor->Ki = Ki;
    }

    //Kd
    if (Kd < 0){
    	motor->Kd = 0;
    } else{
    	motor->Kd = Kd;
    }

    /* Initialize PID terms */
    motor->p_term = 0.0f;
    motor->i_term = 0.0f;
    motor->d_term = 0.0f;

    /* Initialize setpoint and state variables */
    motor->target_speed_tics = 0;
    motor->last_encoder_count = 0;
    motor->current_encoder_count = 0;
    motor->current_speed_tics = 0.0f;
    motor->last_speed_tics = 0.0f;
    motor->duty_cycle = 0.0f;

    /* Initialize error tracking */
    motor->error = 0.0f;
    motor->last_error = 0.0f;
    motor->error_integral = 0.0f;  // Deprecated, but kept for compatibility

    /* Initialize outputs */
    motor->pid_output = 0.0f;

    /* Derivative filter time constant (seconds)
     * tau = 0.02s provides good noise rejection at 100Hz update rate
     * Cutoff frequency ≈ 1/(2*pi*tau) ≈ 8 Hz */
    motor->tau = 0.02f;
}

/* ============================================================================
 * SETPOINT CONTROL
 * ============================================================================ */
void MotorControl_SetTargetSpeed(Motor* motor, int32_t target_tics)
{
    motor->target_speed_tics = target_tics;

    /* OPTION: Reset integral on setpoint change to reduce overshoot
     * Uncomment if quick direction changes are needed */
    // motor->i_term = 0.0f;
}

void MotorControl_SetTargetSpeed_RPM(Motor* motor, int32_t target_rpm)
{
	motor->target_speed_tics = (target_rpm * MOTOR_PPR) / 60.0f;
}

/* ============================================================================
 * GETTER FUNCTIONS
 * ============================================================================ */
int32_t MotorControl_getTargetTics(Motor* motor) {
    return motor->target_speed_tics;
}

float MotorControl_getTics(Motor* motor) {
    return motor->current_speed_tics;
}

float MotorControl_getIntegral(Motor* motor) {
    return motor->i_term;  // Return actual integral term, not deprecated variable
}

float MotorControl_pidOutput(Motor* motor) {
    return motor->pid_output;
}

float MotorControl_getP(Motor* motor) {
    return motor->p_term;
}

float MotorControl_getI(Motor* motor) {
    return motor->i_term;
}

float MotorControl_getD(Motor* motor) {
    return motor->d_term;
}

float MotorControl_getDuty(Motor* motor) {
    return motor->duty_cycle;
}

float MotorControl_getError(Motor* motor) {
    return motor->error;
}

uint16_t MotorControl_getEncoder(Motor* motor) {
    return motor->current_encoder_count;
}

//float MotorControl_getRawspeed(MotorControl_t* motor) {
//    return motor->raw_speed_tics;
//}

/* ============================================================================
 * PID CONTROL LOOP
 * ============================================================================ */
void MotorControl_RunPID(Motor* motor)
{
    /* ------------------------------------------------------------------------
     * 1. READ ENCODER
     * ------------------------------------------------------------------------ */
    if (motor->encoder_timer != NULL) {
        motor->current_encoder_count = (uint16_t)__HAL_TIM_GET_COUNTER(motor->encoder_timer);
    }

    /* Calculate encoder ticks moved since last sample
     * Note: Handles 16-bit overflow automatically due to two's complement arithmetic */
    int16_t ticks_moved = (int16_t)(motor->current_encoder_count - motor->last_encoder_count);

    //2. CALCULATE SPEED (Process Variable)
    //Convert ticks moved to ticks/second
    motor->current_speed_tics = (float)ticks_moved * PID_LOOP_FREQUENCY;

    //3. CALCULATE ERROR
    motor->error = (float)motor->target_speed_tics - motor->current_speed_tics;

    //4. PROPORTIONAL TERM
    motor->p_term = motor->Kp * motor->error;

    /*5. INTEGRAL TERM (Trapezoidal Integration)
     * I_out += Ki * dt/2 * (error[k] + error[k-1])
     * Trapezoidal rule reduces integration error compared to rectangular method*/
    motor->i_term += 0.5f * motor->Ki * PID_PERIOD * (motor->error + motor->last_error);

    //Dynamic antiwindup scheme
    float limMinInt, limMaxInt;

    if (MAX_SPEED_TICKS_PER_SEC > motor->p_term){
    	limMaxInt = MAX_SPEED_TICKS_PER_SEC - motor->p_term;
    } else{
    	limMaxInt = 0.0f;
    }

    if (-MAX_SPEED_TICKS_PER_SEC < motor->p_term){
    	limMinInt = -MAX_SPEED_TICKS_PER_SEC - motor->p_term;
    } else{
    	limMinInt = 0.0f;
    }
    /* Anti-windup: Clamp integral term to prevent saturation */
    if (motor->i_term > limMaxInt) {
        motor->i_term = limMaxInt;
    } else if (motor->i_term < limMinInt) {
        motor->i_term = limMinInt;
    }

    /* ------------------------------------------------------------------------
     * 6. DERIVATIVE TERM (Band-Limited Differentiator)
     *
     * Uses derivative-on-measurement (not derivative-on-error) to avoid
     * "derivative kick" when setpoint changes.
     *
     * Discrete implementation of first-order low-pass filter on derivative:
     * d_term = (2*Kd*(measurement[k-1] - measurement[k]) + (2*tau - dt)*d_term[k-1]) / (2*tau + dt)
     *
     * Note: Negative sign because derivative is on measurement (opposite of error rate)
     * ------------------------------------------------------------------------ */
    motor->d_term = -(2.0f * motor->Kd * (motor->current_speed_tics - motor->last_speed_tics)
                    + (2.0f * motor->tau - PID_PERIOD) * motor->d_term)
                    / (2.0f * motor->tau + PID_PERIOD);

    /* ------------------------------------------------------------------------
     * 7. UPDATE STATE VARIABLES FOR NEXT ITERATION
     * ------------------------------------------------------------------------ */
    motor->last_error = motor->error;
    motor->last_speed_tics = motor->current_speed_tics;
    motor->last_encoder_count = motor->current_encoder_count;

    /* ------------------------------------------------------------------------
     * 8. COMBINE PID TERMS
     * ------------------------------------------------------------------------ */
    motor->pid_output = motor->p_term + motor->i_term + motor->d_term;

    motor->duty_cycle = motor->pid_output/(MOTOR_PPR * (motor->rpm / 60.0f));

    //10. APPLY TO HARDWARE
     motor_driver_setSpeed(motor, motor->duty_cycle);

}

/* ============================================================================
 * REFERENCES
 * ============================================================================ */
/*
 * Helpful resources used in development:
 * - https://www.ctrlaltftc.com/practical-examples/ftc-motor-control
 * - https://www.youtube.com/watch?v=HJ-C4Incgpw&t=237s
 * - https://www.youtube.com/watch?v=NVLXCwc8HzM&t=201s
 * - https://github.com/curiores/ArduinoTutorials (filter design)
 * - https://github.com/LemLib/LemLib/blob/master/src/lemlib/PID.cpp
 */
