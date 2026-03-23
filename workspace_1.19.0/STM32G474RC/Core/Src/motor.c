/*
 * motor.c
 *
 *  Created on: Mar 8, 2026
 *      Author: Nguyen
 */

#include <stdlib.h>
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include "tim.h"
#include "motor.h"

void Motor_Init(Motor* motor, bool reverse,
		GPIO_TypeDef *nsleep_port, uint16_t nsleep_pin,
		GPIO_TypeDef *nfault_port, uint16_t nfault_pin,
		GPIO_TypeDef *direction_port, uint16_t direction_pin,
		GPIO_TypeDef *led_port, uint16_t led_pin,
		TIM_HandleTypeDef *pwm_timer, uint32_t pwm_channel,
		TIM_HandleTypeDef* encoder_timer, int16_t rpm)
{
    motor->Kp = 0.0f;
    motor->Ki = 0.0f;
    motor->Kd = 0.0f;
    motor->tau = 0.0f;

	motor->reverse = reverse;
    motor->nsleep_port = nsleep_port;
    motor->nsleep_pin = nsleep_pin;

    motor->nfault_port = nfault_port;
    motor->nfault_pin = nfault_pin;

    motor->direction_port = direction_port;
    motor->direction_pin = direction_pin;

    motor->led_port = led_port;
    motor->led_pin = led_pin;

    motor->pwm_timer = pwm_timer;
    motor->pwm_channel = pwm_channel;

    motor->encoder_timer = encoder_timer;

    motor->rpm = rpm;

    motor->target_speed_tics = 0;
    motor->last_encoder_count = 0;
    motor->current_encoder_count = 0;
    motor->current_speed_tics = 0.0f;
    motor->last_speed_tics = 0.0f;
    motor->error = 0.0f;
    motor->last_error = 0.0f;
    motor->error_integral = 0.0f;
    motor->p_term = 0.0f;
    motor->i_term = 0.0f;
    motor->d_term = 0.0f;
    motor->pid_output = 0.0f;
    motor->duty_cycle = 0.0f;

    HAL_TIM_Encoder_Start((motor->encoder_timer), TIM_CHANNEL_ALL);
}
