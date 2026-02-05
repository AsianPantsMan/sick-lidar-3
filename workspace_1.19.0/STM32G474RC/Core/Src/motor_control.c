/*
 * motor_control.c
 *
 *  Created on: Jan 28, 2026
 *      Author: caseyear
 */

#include "motor_control.h"
#include <stdio.h>
#include <stdlib.h>

void motor_init(motor_handle_t *motor) {
    if (motor == NULL || motor->drv_handle == NULL) {
        printf("ERROR: Motor handle not initialized\r\n");
        return;
    }

    // Initialize motor state
    motor->state = MOTOR_STATE_STOPPED;
    motor->direction = MOTOR_DIRECTION_FORWARD;
    motor->duty_cycle = 0;
    motor->target_speed = 0;
    motor->current_speed = 0;

    // Initialize GPIO pins
    HAL_GPIO_WritePin(motor->enable_port, motor->enable_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);

    // Start PWM timer
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);

    printf("Motor initialized\r\n");
}

void motor_setSpeed(motor_handle_t *motor, int16_t speed) {
    // speed range: -100 to +100 (negative = reverse, positive = forward)

    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    motor->target_speed = speed;

    // Determine direction
    if (speed > 0) {
        motor_setDirection(motor, MOTOR_DIRECTION_FORWARD);
        motor->duty_cycle = (uint8_t)speed;
    } else if (speed < 0) {
        motor_setDirection(motor, MOTOR_DIRECTION_REVERSE);
        motor->duty_cycle = (uint8_t)abs(speed);
    } else {
        motor->duty_cycle = 0;
    }

    // Update PWM
    motor_updatePWM(motor, motor->duty_cycle);

    printf("Motor speed set: %d%% (%s)\r\n",
           motor->duty_cycle,
           motor->direction == MOTOR_DIRECTION_FORWARD ? "FWD" : "REV");
}

void motor_setDirection(motor_handle_t *motor, motor_direction_t direction) {
    motor->direction = direction;

    switch(direction) {
        case MOTOR_DIRECTION_FORWARD:
            HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
            // Update DRV8245 phase control
            drv8245_setDriveOutput(1, 1);  // Enable, Forward
            break;

        case MOTOR_DIRECTION_REVERSE:
            HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
            // Update DRV8245 phase control
            drv8245_setDriveOutput(1, 0);  // Enable, Reverse
            break;

        case MOTOR_DIRECTION_BRAKE:
            motor_brake(motor);
            break;

        case MOTOR_DIRECTION_COAST:
            motor_coast(motor);
            break;
    }
}

void motor_enable(motor_handle_t *motor) {
    HAL_GPIO_WritePin(motor->enable_port, motor->enable_pin, GPIO_PIN_SET);
    drv8245_setSpiControl(1);  // Enable via SPI
    motor->state = MOTOR_STATE_RUNNING;
    printf("Motor enabled\r\n");
}

void motor_disable(motor_handle_t *motor) {
    HAL_GPIO_WritePin(motor->enable_port, motor->enable_pin, GPIO_PIN_RESET);
    drv8245_setSpiControl(0);  // Disable via SPI
    motor->state = MOTOR_STATE_STOPPED;
    motor_updatePWM(motor, 0);
    printf("Motor disabled\r\n");
}

void motor_brake(motor_handle_t *motor) {
    // Active brake: short both motor terminals
    motor->duty_cycle = 0;
    motor_updatePWM(motor, 0);

    // Set both outputs low for braking
    uint8_t data[2];
    data[0] = SPI_IN;
    data[1] = SPI_IN_S_EN_IN1;  // Enable but no phase = brake
    drv8245_writeData(data);

    motor->state = MOTOR_STATE_STOPPED;
    printf("Motor braking\r\n");
}

void motor_coast(motor_handle_t *motor) {
    // Coast: high-impedance state
    motor->duty_cycle = 0;
    motor_updatePWM(motor, 0);
    motor_disable(motor);

    motor->state = MOTOR_STATE_STOPPED;
    printf("Motor coasting\r\n");
}

void motor_emergencyStop(motor_handle_t *motor) {
    // Immediate stop with brake
    motor_brake(motor);
    motor_disable(motor);

    printf("EMERGENCY STOP\r\n");
}

void motor_updatePWM(motor_handle_t *motor, uint8_t duty_cycle) {
    // duty_cycle: 0-100 (percentage)

    if (duty_cycle > 100) duty_cycle = 100;

    motor->duty_cycle = duty_cycle;

    // Calculate PWM compare value
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);
    uint32_t compare = (arr * duty_cycle) / 100;

    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, compare);
}

motor_state_t motor_getState(motor_handle_t *motor) {
    return motor->state;
}

void motor_checkFaults(motor_handle_t *motor) {
    // Check nFAULT pin
    GPIO_PinState fault_state = HAL_GPIO_ReadPin(motor->drv_handle->faultPort,
                                                   motor->drv_handle->faultPin);

    if (fault_state == GPIO_PIN_RESET) {
        // Fault detected
        motor->state = MOTOR_STATE_FAULT;
        printf("MOTOR FAULT DETECTED\r\n");

        // Get detailed fault information
        drv8245_getFaultSummary();

        // Stop motor
        motor_emergencyStop(motor);

        // Attempt to clear faults
        drv8245_clearFaults();
    }
}
