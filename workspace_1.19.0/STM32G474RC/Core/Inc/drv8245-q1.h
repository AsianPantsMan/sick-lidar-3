/*
 * drv8245-q1.h
 *
 *  Created on: Jan 28, 2026
 *      Author: caseyear
 */

#ifndef INC_DRV8245_Q1_H_
#define INC_DRV8245_Q1_H_

#include "main.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include "tim.h"
#include "motor.h"


void delay_us(uint32_t microseconds);
HAL_StatusTypeDef motor_driver_init(Motor *motor);
void motor_driver_setSpeed(Motor *motor, float duty);


#endif
