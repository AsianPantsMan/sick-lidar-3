#include "motor_control.h"
#include <stdlib.h>

void MotorControl_Init(MotorControl_t* motor,
                       TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel,
                       GPIO_TypeDef* dir_port, uint16_t dir_pin,
                       TIM_HandleTypeDef* encoder_timer,
                       float Kp, float Ki, float Kd)
{
    motor->pwm_timer = pwm_timer;
    motor->pwm_channel = pwm_channel;
    motor->dir_gpio_port = dir_port;
    motor->dir_gpio_pin = dir_pin;
    motor->encoder_timer = encoder_timer;

    motor->Kp = Kp;
    motor->Ki = Ki;
    motor->Kd = Kd;

    motor->target_speed_ticks_per_sec = 0;
    motor->last_encoder_count = 0;
    motor->current_encoder_count = 0;
    motor->current_speed_ticks_per_sec = 0.0f;
    motor->error_integral = 0.0f;
    motor->last_error = 0.0f;
    motor->output_pwm = 0;
    motor->pid_loop_counter = 0;

    // Start encoder and PWM
    HAL_TIM_Encoder_Start(motor->encoder_timer, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);

    HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);
}

void MotorControl_SetTargetSpeed(MotorControl_t* motor, int32_t target_ticks_per_sec)
{
    if (target_ticks_per_sec > MAX_SPEED_TICKS_PER_SEC) {
        target_ticks_per_sec = MAX_SPEED_TICKS_PER_SEC;
    } else if (target_ticks_per_sec < -MAX_SPEED_TICKS_PER_SEC) {
        target_ticks_per_sec = -MAX_SPEED_TICKS_PER_SEC;
    }
    motor->target_speed_ticks_per_sec = target_ticks_per_sec;
}

void MotorControl_RunPID(MotorControl_t* motor)
{
    motor->pid_loop_counter++;

    // Read encoder and calculate tick/sec
    motor->current_encoder_count = (int16_t)(motor->encoder_timer->Instance->CNT);
    int16_t ticks_moved = motor->current_encoder_count - motor->last_encoder_count;
    motor->last_encoder_count = motor->current_encoder_count;
    motor->current_speed_ticks_per_sec = (float)ticks_moved / PID_PERIOD_T;

    // Calculate error = target speed - current speed
    float error = (float)motor->target_speed_ticks_per_sec - motor->current_speed_ticks_per_sec;

    // PID terms

    //P term = error * p constant
    float p_term = motor->Kp * error;

    //I term calculation
    // calculate the integral (change in error * time passed)
    motor->error_integral += error * PID_PERIOD_T;

    //Handle sign flip reset
    if (((error>0) - (error<0)) != ((motor->last_error>0) - (motor->last_error<0))) motor->error_integral = 0;

    //Anti windup range
    if (motor->error_integral > MAX_INTEGRAL_VALUE) motor->error_integral = MAX_INTEGRAL_VALUE;
    if (motor->error_integral < -MAX_INTEGRAL_VALUE) motor->error_integral = -MAX_INTEGRAL_VALUE;
    float i_term = motor->Ki * motor->error_integral;

    //D term calculation
    float d_term = motor->Kd * (error - motor->last_error) / PID_PERIOD_T;
    motor->last_error = error;

    // Calculate output
    float pid_output = p_term + i_term + d_term;
    motor->output_pwm = (int32_t)pid_output;

    if (motor->output_pwm > MAX_PWM_VALUE) motor->output_pwm = MAX_PWM_VALUE;
    if (motor->output_pwm < -MAX_PWM_VALUE) motor->output_pwm = -MAX_PWM_VALUE;

    // Apply to hardware

    //Forward
    if (motor->output_pwm >= 0) {
        motor->dir_gpio_port->BSRR = motor->dir_gpio_pin;
        if (motor->pwm_channel == TIM_CHANNEL_1) {
            motor->pwm_timer->Instance->CCR1 = motor->output_pwm;
        } else if (motor->pwm_channel == TIM_CHANNEL_4) {
            motor->pwm_timer->Instance->CCR4 = motor->output_pwm;
        }
    //Reverse
    } else {
        motor->dir_gpio_port->BSRR = (uint32_t)motor->dir_gpio_pin << 16U;
        if (motor->pwm_channel == TIM_CHANNEL_1) {
            motor->pwm_timer->Instance->CCR1 = -motor->output_pwm;
        } else if (motor->pwm_channel == TIM_CHANNEL_4) {
            motor->pwm_timer->Instance->CCR4 = -motor->output_pwm;
        }
    }
}

void MotorControl_Stop(MotorControl_t* motor)
{
    motor->target_speed_ticks_per_sec = 0;
    motor->output_pwm = 0;
    motor->error_integral = 0.0f;
    motor->last_error = 0.0f;

    if (motor->pwm_channel == TIM_CHANNEL_1) {
        motor->pwm_timer->Instance->CCR1 = 0;
    } else if (motor->pwm_channel == TIM_CHANNEL_4) {
        motor->pwm_timer->Instance->CCR4 = 0;
    }
}

float MotorControl_GetCurrentSpeed(MotorControl_t* motor)
{
    return motor->current_speed_ticks_per_sec;
}
