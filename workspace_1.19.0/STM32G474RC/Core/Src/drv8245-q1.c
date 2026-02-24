/*
* drv8245-q1.c
*
*  Created on: Jan 28, 2026
*      Author: caseyear
*/

#include "drv8245-q1.h"
#include "main.h"
#include <stdio.h>



void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < cycles);
}

void md1_reset_pulse(){
	HAL_GPIO_WritePin(MD1_nSLEEP_GPIO_Port, MD1_nSLEEP_Pin, GPIO_PIN_RESET);
	delay_us(30);
	HAL_GPIO_WritePin(MD1_nSLEEP_GPIO_Port, MD1_nSLEEP_Pin, GPIO_PIN_SET);
}

void md2_reset_pulse(){
	HAL_GPIO_WritePin(MD2_nSLEEP_GPIO_Port, MD2_nSLEEP_Pin, GPIO_PIN_RESET);
	delay_us(30);
	HAL_GPIO_WritePin(MD2_nSLEEP_GPIO_Port, MD2_nSLEEP_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef MD1_motor_init(){
	GPIO_PinState fault_status1;

//	//debug block, check nSLEEP & nFAULT before init
//	GPIO_PinState sleep_status1 = HAL_GPIO_ReadPin(MD1_nSLEEP_GPIO_Port, MD1_nSLEEP_Pin);
//	printf("sleep pin is: %d\r\n", sleep_status1);
//	fault_status = HAL_GPIO_ReadPin(MD1_nFAULT_GPIO_Port, MD1_nFAULT_Pin);
//	printf("fault pin is: %d\r\n", fault_status1);

	//initiate wakeup sequence
	printf("nSLEEP asserted high on motor driver 1\r\n");
	HAL_GPIO_WritePin(MD1_nSLEEP_GPIO_Port, MD1_nSLEEP_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	fault_status1 = HAL_GPIO_ReadPin(MD1_nFAULT_GPIO_Port, MD1_nFAULT_Pin);
	if (fault_status1 == GPIO_PIN_RESET) {
		printf("nFAULT pin asserted low\r\n");
	}else{
		printf("error: nFAULT pin stuck high");
		return HAL_ERROR;
	}

	//pulse nSLEEP to acknowledge device wake up
	printf("send reset pulse on nSLEEP\r\n");
	md1_reset_pulse();

	fault_status1 = HAL_GPIO_ReadPin(MD1_nFAULT_GPIO_Port, MD1_nFAULT_Pin);
	if (fault_status1 == GPIO_PIN_SET) {
		printf("nFAULT pin de-asserted high, in standby\r\n");
	}else{
		printf("initialization incomplete\r\n");

		return HAL_ERROR;
	}
	//turn on md1 led
	HAL_GPIO_WritePin(MD1_LED_GPIO_Port, MD1_LED_Pin, GPIO_PIN_SET);
	printf("now in standby. motor driver 1 ready!\r\n");

	return HAL_OK;

}

HAL_StatusTypeDef MD2_motor_init(){
	GPIO_PinState fault_status2;

//	//debug block, check nSLEEP & nFAULT before init
//	GPIO_PinState sleep_status2 = HAL_GPIO_ReadPin(MD2_nSLEEP_GPIO_Port, MD2_nSLEEP_Pin);
//	printf("sleep pin is: %d\r\n", sleep_status2);
//	fault_status2 = HAL_GPIO_ReadPin(MD2_nFAULT_GPIO_Port, MD2_nFAULT_Pin);
//	printf("fault pin is: %d\r\n", fault_status2);

	//initiate wakeup sequence
	printf("nSLEEP asserted high on motor driver 2\r\n");
	HAL_GPIO_WritePin(MD2_nSLEEP_GPIO_Port, MD2_nSLEEP_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	fault_status2 = HAL_GPIO_ReadPin(MD2_nFAULT_GPIO_Port, MD2_nFAULT_Pin);
	if (fault_status2 == GPIO_PIN_RESET) {
		printf("nFAULT pin asserted low\r\n");
	}else{
		printf("error: nFAULT pin stuck high");
		return HAL_ERROR;
	}

	//pulse nSLEEP to acknowledge device wake up
	printf("send reset pulse on nSLEEP\r\n");
	md2_reset_pulse();

//  //BYPASSING FAULT CHECK BECAUSE NFAULT PIN ON STM32 IS LIKELY BURNT OUT FROM A PRIOR 5V SIGNAL SENT, UNCOMMENT FOR FUTURE CHECK, THE MD STILL WORKS
//	fault_status2 = HAL_GPIO_ReadPin(MD2_nFAULT_GPIO_Port, MD2_nFAULT_Pin);
//	if (fault_status2 == GPIO_PIN_SET) {
//		printf("nFAULT pin de-asserted high, in standby\r\n");
//	}else{
//		printf("initialization incomplete\r\n");
//
//		return HAL_ERROR;
//	}
	//turn on md2 led
	HAL_GPIO_WritePin(MD2_LED_GPIO_Port, MD2_LED_Pin, GPIO_PIN_SET);
	printf("now in standby. motor driver 2 ready!\r\n");

	return HAL_OK;
}

uint8_t MD1_setSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty) {
	if (duty > 100) {
		printf("ERROR: Invalid duty cycle %d%%\r\n", duty);
		return 0;
	}

	uint32_t timer_period = __HAL_TIM_GET_AUTORELOAD(htim);

	//calculate compare value based on timer period
	uint32_t compare_value = (timer_period * duty) / 100;
	printf("[DEBUG] Calculated compare value: %lu\r\n", compare_value);

	__HAL_TIM_SET_COMPARE(htim, channel, compare_value);

	//start PWM output
	HAL_StatusTypeDef pwm_result = HAL_TIM_PWM_Start(htim, channel);
	printf("HAL_TIM_PWM_Start result: %d\r\n", pwm_result);
	if (pwm_result != HAL_OK) {
		printf("ERROR: Failed to start PWM\r\n");
		return 0;
	}

	printf("Motor speed set to %d%% (Compare: %lu/%lu)\r\n", duty, compare_value, timer_period);

	return duty;
}

uint8_t MD2_setSpeed(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty) {
	if (duty > 100) {
		printf("ERROR: Invalid duty cycle %d%%\r\n", duty);
		return 0;
	}

	uint32_t timer_period = __HAL_TIM_GET_AUTORELOAD(htim);

	//calculate compare value based on timer period
	uint32_t compare_value = (timer_period * duty) / 100;

	__HAL_TIM_SET_COMPARE(htim, channel, compare_value);

	//start PWM output
	HAL_StatusTypeDef pwm_result = HAL_TIM_PWM_Start(htim, channel);
	if (pwm_result != HAL_OK) {
		printf("ERROR: Failed to start PWM\r\n");
		return 0;
	}

	printf("Motor speed set to %d%% (Compare: %lu/%lu)\r\n", duty, compare_value, timer_period);


	return duty;
}
