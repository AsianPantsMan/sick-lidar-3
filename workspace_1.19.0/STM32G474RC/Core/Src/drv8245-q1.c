//Motor spec https://www.gobilda.com/5303-series-saturn-planetary-gear-motor-71-2-1-ratio-24mm-length-8mm-rex-shaft-260-rpm-3-3-5v-encoder/
//motor driver spec https://www.ti.com/product/DRV8245-Q1
/*
* drv8245-q1.c
*
*  Created on: Jan 28, 2026
*      Author: caseyear
*/

#include "drv8245-q1.h"
#include "main.h"
#include <stdio.h>
#include "tim.h"



void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < cycles);
}


HAL_StatusTypeDef motor_driver_init(Motor *motor){
	GPIO_PinState fault_status;
//	//debug block, check nSLEEP & nFAULT before init
//	GPIO_PinState sleep_status1 = HAL_GPIO_ReadPin(MD1_nSLEEP_GPIO_Port, MD1_nSLEEP_Pin);
//	printf("sleep pin is: %d\r\n", sleep_status1);
//	fault_status1 = HAL_GPIO_ReadPin(MD1_nFAULT_GPIO_Port, MD1_nFAULT_Pin);
//	printf("fault pin is: %d\r\n", fault_status1);

	//initiate wakeup sequence
	printf("nSLEEP asserted high on motor driver\r\n");
	HAL_GPIO_WritePin(motor->nsleep_port, motor->nsleep_pin, GPIO_PIN_SET);
	HAL_Delay(1);

	fault_status = HAL_GPIO_ReadPin(motor->nfault_port, motor->nfault_pin);
	if (fault_status == GPIO_PIN_RESET) {
		printf("nFAULT pin asserted low\r\n");
	}else{
		printf("error: nFAULT pin stuck high");
		return HAL_ERROR;
	}

	//pulse nSLEEP to acknowledge device wake up
	printf("send reset pulse on nSLEEP\r\n");

	//RESET Pulse
	HAL_GPIO_WritePin(motor->nsleep_port, motor->nsleep_pin, GPIO_PIN_RESET);
	delay_us(30);
	HAL_GPIO_WritePin(motor->nsleep_port, motor->nsleep_pin, GPIO_PIN_SET);

//  //BYPASSING FAULT CHECK BECAUSE NFAULT PIN ON STM32 IS LIKELY BURNT OUT FROM A PRIOR 5V SIGNAL SENT, UNCOMMENT FOR FUTURE CHECK, THE MD STILL WORKS
//	fault_status2 = HAL_GPIO_ReadPin(MD2_nFAULT_GPIO_Port, MD2_nFAULT_Pin);
//	if (fault_status2 == GPIO_PIN_SET) {
//		printf("nFAULT pin de-asserted high, in standby\r\n");
//	}else{
//		printf("initialization incomplete\r\n");
//
//		return HAL_ERROR;
//	}
	//turn on m12 led
	HAL_GPIO_WritePin(motor->led_port, motor->led_pin, GPIO_PIN_SET);
	printf("now in standby. motor driver12 ready!\r\n");

    // Start encoder and PWM
    HAL_TIM_Encoder_Start(motor->encoder_timer, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);

    HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, 0);

	return HAL_OK;
}

void motor_driver_setSpeed(Motor *motor, float duty) {

    /* Clamp duty cycle to valid range */
    if (duty > 1) {
        duty = 1;
    } else if (duty < -1) {
        duty = -1;
    }

	uint32_t timer_period = __HAL_TIM_GET_AUTORELOAD(motor->pwm_timer);

	if (motor->reverse) {
	    // If reversed, "forward" means setting the direction pin LOW
//	    forward_state = GPIO_PIN_RESET;
//	    reverse_state = GPIO_PIN_SET;
		//Set direction +duty is forward, -duty is backward
	    if (duty >= 0) {
	    	HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_RESET);
	    	uint32_t pwm_output = (uint32_t)(timer_period * duty);  // Round instead of truncate
	    	__HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, pwm_output);
	    	//HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
	    //Reverse
	    } else {
	    	HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_SET);
	    	uint32_t pwm_output = (uint32_t)(timer_period * (-duty));  // Round instead of truncate
	        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, pwm_output);
	    	//HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
	    }
	} else {
	    // Normal operation, "forward" means setting the direction pin HIGH
//	    forward_state = GPIO_PIN_SET;
//	    reverse_state = GPIO_PIN_RESET;

		//Set direction +duty is forward, -duty is backward
	    if (duty >= 0) {
	    	HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_SET);
	    	uint32_t pwm_output = (uint32_t)(timer_period * duty);  // Round instead of truncate
	    	__HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, pwm_output);
	    	//HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
	    //Reverse
	    } else {
	    	HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, GPIO_PIN_RESET);
	    	uint32_t pwm_output = (uint32_t)(timer_period * (-duty));  // Round instead of truncate
	        __HAL_TIM_SET_COMPARE(motor->pwm_timer, motor->pwm_channel, pwm_output);
	    	//HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_channel);
	    }
	}



}

