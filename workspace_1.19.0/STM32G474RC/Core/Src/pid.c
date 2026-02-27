/*
 * pid.c
 *
 *  Created on: Jan 22, 2026
 *      Author: caseyear
 */
#include "pid.h"
#include <stdlib.h>
#include <stm32g4xx_hal_tim.h>
#include "drv8245-q1.h"
#include "stm32g4xx_hal.h"
#include "butterworth_filter.h"

void MotorControl_Init(MotorControl_t* motor,
                       TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel,
                       TIM_HandleTypeDef* encoder_timer,
                       float Kp, float Ki, float Kd)
{
    motor->pwm_timer = pwm_timer;
    motor->pwm_channel = pwm_channel;
    motor->encoder_timer = encoder_timer;

    motor->Kp = Kp;
    motor->Ki = Ki;
    motor->Kd = Kd;

    motor->target_speed_rpm = 0;
    motor->last_encoder_count = 0;
    motor->current_encoder_count = 0;
    motor->current_speed_rpm = 0.0f;
    motor->rpm_filt = 0.0f;
    motor->rpm_tau  = 0.05f;
    motor->error_integral = 0.0f;
    motor->last_error = 0.0f;
    motor->output_pwm_duty = 0;

    Butterworth_Init(&motor->rpm_filter);

}


void MotorControl_SetTargetSpeed(MotorControl_t* motor, int32_t target_rpm)
{
    motor->target_speed_rpm = target_rpm;
}

int32_t MotorControl_getTargetRpm(MotorControl_t* motor){
	return motor->target_speed_rpm;
}

float MotorControl_getRpm(MotorControl_t* motor){
	return motor->current_speed_rpm;
}

void MotorControl_RunPID(MotorControl_t* motor)
{
    // Read current encoder count

	if (motor->encoder_timer != NULL) {
	    motor->current_encoder_count = (uint16_t)__HAL_TIM_GET_COUNTER(motor->encoder_timer);
	}

    // Calculate ticks moved
    int16_t ticks_moved = (int16_t)(motor->current_encoder_count - motor->last_encoder_count);
    motor->last_encoder_count = motor->current_encoder_count;

//    //USE FMAC hardware filter calculation core to implement 5th order Chevby LP filter
//    float scaled_error = raw_error / MAX_SPEED_TICKS_PER_SEC;
//    if (scaled_error > 1.0f)  scaled_error = 1.0f;
//    if (scaled_error < -1.0f) scaled_error = -1.0f;
//    //To convert from floating point to fixed point, all coefficients must be multiplied by
//    //32768 (0x8000) and cast to int16
//    int16_t q15_error = (int16_t)(scaled_error * 32767.0f);
//    int16_t q15_filtered_error = FMAC_Filter_Apply(q15_error);
//    float scaled_filtered_error = (float)q15_filtered_error / 32767.0f;
//    float error = scaled_filtered_error * MAX_SPEED_TICKS_PER_SEC; // Un-scale it

    // Calculate speed in rpm
    float raw_speed_rpm = (float)ticks_moved * (60.0f / (MOTOR_PPR * period));

    //Butterworth 4th order lowpass
     motor-> current_speed_rpm = Butterworth_Apply(&motor->rpm_filter, raw_speed_rpm);

    // Low-pass filter (EMA): rpm_filt += alpha*(rpm_raw - rpm_filt)
//    float dt = period;                       // if period is fixed and accurate
//    float tau = motor->rpm_tau;
//    float alpha = dt / (tau + dt);
//    motor->rpm_filt += alpha * (motor->current_speed_rpm - motor->rpm_filt);

//    // filter out hight frequency noise to increase derivative performance
//    currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
//    previousFilterEstimate = currentFilterEstimate;

    float error = (float)motor->target_speed_rpm - motor->current_speed_rpm;

    float p_term = motor->Kp * error;

    // reset the integral if the speed is changed allow drivetrain
    //to change direction faster than waiting for integral sum to change
    if (motor -> target_speed_rpm != motor -> previous_speed_rpm) {
        motor -> error_integral = 0;
    }
    motor -> previous_speed_rpm = motor -> target_speed_rpm;

    motor->error_integral += error * period;

    //Anti wind up
    if (motor -> error_integral > integral_limit){
    	motor -> error_integral = integral_limit;
    }
    if (motor -> error_integral < -integral_limit){
    	motor -> error_integral = -integral_limit;
    }

    float i_term = motor->Ki * motor->error_integral;

    float d_term = motor->Kd * (error - motor->last_error) / period;
    motor->last_error = error;

    //Calculate output
    float pid_output = p_term + i_term + d_term;

    //Convert rpm to duty cycle
    motor->output_pwm_duty = ((int32_t)pid_output / MAX_RPM)*100;

    //Case for duty cycle is higher then 100%
    if (motor->output_pwm_duty > 100) motor->output_pwm_duty = 100;
    if (motor->output_pwm_duty < -100) motor->output_pwm_duty = -100;

    // Apply to hardware
	if (motor->pwm_channel == TIM_CHANNEL_1) {
		MD2_setSpeed(&htim8, TIM_CHANNEL_1, motor->output_pwm_duty);
	} else if (motor->pwm_channel == TIM_CHANNEL_4) {
		MD1_setSpeed(&htim1, TIM_CHANNEL_4, motor->output_pwm_duty);
	}
}

//Help
/*
 * https://www.ctrlaltftc.com/practical-examples/ftc-motor-control
 * https://www.youtube.com/watch?v=HJ-C4Incgpw&t=237s
 * https://www.youtube.com/watch?v=NVLXCwc8HzM&t=201s
 * https://www.youtube.com/watch?v=HJ-C4Incgpw
 * https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/LowPass/ButterworthFilter.ipynb
 * https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl_NoAtomic/SpeedControl_NoAtomic.ino
 * https://github.com/LemLib/LemLib/blob/master/src/lemlib/PID.cpp
 * https://github.com/putrafurqan/stm32_dsp_fir_filter/blob/main/docs/matlab/main_filter_design.m
 */
