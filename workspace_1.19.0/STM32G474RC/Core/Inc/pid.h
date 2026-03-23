/*
 * pid.h
 *
 * PID Motor Control System for STM32G4
 * Implements velocity control using encoder feedback
 *
 * Created on: Feb 25, 2025
 * Author: caseyear
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32g4xx_hal.h"
#include "tim.h"
#include <stdint.h>
#include <stdbool.h>
#include "motor.h"
#include "drv8245-q1.h"

/* ============================================================================
 * PID LOOP CONFIGURATION
 * ============================================================================ */
#define PID_LOOP_FREQUENCY      10.0f      // PID update rate (Hz)
#define PID_PERIOD              (1.0f / PID_LOOP_FREQUENCY)  // Sample time (seconds)


void MotorControl_Init(Motor* motor,
                       float Kp, float Ki, float Kd);

void MotorControl_SetTargetSpeed(Motor* motor, int32_t target_tics);

void MotorControl_SetTargetSpeed_RPM(Motor* motor, int32_t target_rpm);


void MotorControl_RunPID(Motor* motor);

int32_t MotorControl_getTargetTics(Motor* motor);
float MotorControl_getTics(Motor* motor);
float MotorControl_getIntegral(Motor* motor);
float MotorControl_duty(Motor* motor);
float MotorControl_pidOutput(Motor* motor);
float MotorControl_getP(Motor* motor);
float MotorControl_getI(Motor* motor);
float MotorControl_getD(Motor* motor);
float MotorControl_getError(Motor* motor);
float MotorControl_getDuty(Motor* motor);
uint16_t MotorControl_getEncoder(Motor* motor);

#endif /* INC_PID_H_ */
