/*
 * pid.c
 *
 *  Created on: Jan 22, 2026
 *      Author: caseyear
 */
#include "pid.h"

uint16_t getEncoderRpm(TIM_HandleTypeDef *timer) {
    static uint16_t prev_cnt = 0;

    uint16_t curr_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(timer) * 2;

    // signed delta with wrap handling for 16-bit counter
    int32_t ticks = (int32_t)(int16_t)(curr_cnt - prev_cnt);
    prev_cnt = curr_cnt;
    printf("prev_cnt: %d\r\n", prev_cnt);
    printf("curr_cnt: %d\r\n", curr_cnt);

    // dt is fixed if you're in a fixed-rate timer callback:
    const float dt_sec = 1.0f / 100;
    printf("dt_sec: %f\r\n", dt_sec);

    // RPM = ticks/dt * 60 / CPR
    return (float)ticks * (60.0f / (MOTOR_CPR * dt_sec));
    // equivalently: ticks * (60 * SAMPLE_HZ / MOTOR_CPR)
}

//uint8_t calcPid(){
//
//}
