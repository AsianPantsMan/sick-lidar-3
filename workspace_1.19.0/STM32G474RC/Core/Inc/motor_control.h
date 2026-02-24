#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32g4xx_hal.h"
#include "tim.h"
#include <stdint.h>
#include <stdbool.h>

// Motor specifications
#define ENCODER_PPR_AT_OUTPUT         1993.6f
#define QUADRATURE_FACTOR             2f
#define TICKS_PER_REVOLUTION          (ENCODER_PPR_AT_OUTPUT / QUADRATURE_FACTOR)

// PID Loop timing (2kHz for best performance)
#define PID_LOOP_FREQUENCY_HZ         2000.0f
#define PID_PERIOD_T                   (1.0f / PID_LOOP_FREQUENCY_HZ)

// PWM limits (based on TIM1/TIM8 ARR = 999)
#define MAX_PWM_VALUE                 999
#define MAX_SPEED_TICKS_PER_SEC       35000
#define MAX_INTEGRAL_VALUE            10000.0f

typedef struct {
    // Hardware handles
    TIM_HandleTypeDef* pwm_timer;
    uint32_t pwm_channel;
    GPIO_TypeDef* dir_gpio_port;
    uint16_t dir_gpio_pin;
    TIM_HandleTypeDef* encoder_timer;

    // PID Parameters
    float Kp;
    float Ki;
    float Kd;

    // Target from Raspberry Pi
    volatile int32_t target_speed_ticks_per_sec;

    // Encoder state
    volatile int16_t last_encoder_count;
    volatile int16_t current_encoder_count;
    float current_speed_ticks_per_sec;

    // PID internal state
    float error_integral;
    float last_error;
    int32_t output_pwm;

    uint32_t pid_loop_counter;
} MotorControl_t;

// Function prototypes
void MotorControl_Init(MotorControl_t* motor,
                       TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel,
                       GPIO_TypeDef* dir_port, uint16_t dir_pin,
                       TIM_HandleTypeDef* encoder_timer,
                       float Kp, float Ki, float Kd);

void MotorControl_SetTargetSpeed(MotorControl_t* motor, int32_t target_ticks_per_sec);
void MotorControl_RunPID(MotorControl_t* motor);
void MotorControl_Stop(MotorControl_t* motor);
float MotorControl_GetCurrentSpeed(MotorControl_t* motor);

#endif // MOTOR_CONTROL_H
