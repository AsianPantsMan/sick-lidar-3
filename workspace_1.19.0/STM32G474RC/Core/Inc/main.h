/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IO3_Pin GPIO_PIN_13
#define IO3_GPIO_Port GPIOC
#define IO2_Pin GPIO_PIN_14
#define IO2_GPIO_Port GPIOC
#define IO1_Pin GPIO_PIN_15
#define IO1_GPIO_Port GPIOC
#define OSCILLATOR_IN_Pin GPIO_PIN_0
#define OSCILLATOR_IN_GPIO_Port GPIOF
#define OSCILLATOR_OUT_Pin GPIO_PIN_1
#define OSCILLATOR_OUT_GPIO_Port GPIOF
#define MCU_NRST_Pin GPIO_PIN_10
#define MCU_NRST_GPIO_Port GPIOG
#define ENCODER1_A_Pin GPIO_PIN_0
#define ENCODER1_A_GPIO_Port GPIOC
#define ENCODER1_B_Pin GPIO_PIN_1
#define ENCODER1_B_GPIO_Port GPIOC
#define MD1_PH_IN2_Pin GPIO_PIN_2
#define MD1_PH_IN2_GPIO_Port GPIOC
#define MD1_EN_IN1_Pin GPIO_PIN_3
#define MD1_EN_IN1_GPIO_Port GPIOC
#define MD1_IPROPI_Pin GPIO_PIN_0
#define MD1_IPROPI_GPIO_Port GPIOA
#define MD1_DRVOFF_Pin GPIO_PIN_1
#define MD1_DRVOFF_GPIO_Port GPIOA
#define MD1_nFAULT_Pin GPIO_PIN_2
#define MD1_nFAULT_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOA
#define MD_SPI_NSS_Pin GPIO_PIN_4
#define MD_SPI_NSS_GPIO_Port GPIOA
#define MD_SPI_SCK_Pin GPIO_PIN_5
#define MD_SPI_SCK_GPIO_Port GPIOA
#define MD_SPI_MISO_Pin GPIO_PIN_6
#define MD_SPI_MISO_GPIO_Port GPIOA
#define MD_SPI_MOSI_Pin GPIO_PIN_7
#define MD_SPI_MOSI_GPIO_Port GPIOA
#define IMU_LED_Pin GPIO_PIN_4
#define IMU_LED_GPIO_Port GPIOC
#define MCU_LED_Pin GPIO_PIN_5
#define MCU_LED_GPIO_Port GPIOC
#define MD2_LED_Pin GPIO_PIN_0
#define MD2_LED_GPIO_Port GPIOB
#define MD1_LED_Pin GPIO_PIN_1
#define MD1_LED_GPIO_Port GPIOB
#define PI_LED_Pin GPIO_PIN_2
#define PI_LED_GPIO_Port GPIOB
#define BACKUP_USART_TX_Pin GPIO_PIN_10
#define BACKUP_USART_TX_GPIO_Port GPIOB
#define BACKUP_USART_RX_Pin GPIO_PIN_11
#define BACKUP_USART_RX_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_12
#define IMU_INT_GPIO_Port GPIOB
#define MD2_nFAULT_Pin GPIO_PIN_13
#define MD2_nFAULT_GPIO_Port GPIOB
#define MD2_DRVOFF_Pin GPIO_PIN_14
#define MD2_DRVOFF_GPIO_Port GPIOB
#define MD2_IPROPI_Pin GPIO_PIN_15
#define MD2_IPROPI_GPIO_Port GPIOB
#define MD2_EN_IN1_Pin GPIO_PIN_6
#define MD2_EN_IN1_GPIO_Port GPIOC
#define MD2_PH_IN2_Pin GPIO_PIN_7
#define MD2_PH_IN2_GPIO_Port GPIOC
#define ENCODER2_B_Pin GPIO_PIN_8
#define ENCODER2_B_GPIO_Port GPIOC
#define ENCODER2_A_Pin GPIO_PIN_9
#define ENCODER2_A_GPIO_Port GPIOC
#define IMU_SDA_Pin GPIO_PIN_8
#define IMU_SDA_GPIO_Port GPIOA
#define IMU_SCL_Pin GPIO_PIN_9
#define IMU_SCL_GPIO_Port GPIOA
#define IMU_RST_Pin GPIO_PIN_10
#define IMU_RST_GPIO_Port GPIOA
#define JTAG_SWDIO_Pin GPIO_PIN_13
#define JTAG_SWDIO_GPIO_Port GPIOA
#define JTAG_SWCLK_Pin GPIO_PIN_14
#define JTAG_SWCLK_GPIO_Port GPIOA
#define BACKUP_I2C_SCL_Pin GPIO_PIN_15
#define BACKUP_I2C_SCL_GPIO_Port GPIOA
#define BACKUP_SPI_SCK_Pin GPIO_PIN_10
#define BACKUP_SPI_SCK_GPIO_Port GPIOC
#define BACKUP_SPI_MISO_Pin GPIO_PIN_11
#define BACKUP_SPI_MISO_GPIO_Port GPIOC
#define BACKUP_SPI_MOSI_Pin GPIO_PIN_12
#define BACKUP_SPI_MOSI_GPIO_Port GPIOC
#define BACKUP_SPI_NSS_Pin GPIO_PIN_2
#define BACKUP_SPI_NSS_GPIO_Port GPIOD
#define PI_UART_TX_Pin GPIO_PIN_3
#define PI_UART_TX_GPIO_Port GPIOB
#define PI_UART_RX_Pin GPIO_PIN_4
#define PI_UART_RX_GPIO_Port GPIOB
#define BACKUP_CAN_TX_Pin GPIO_PIN_5
#define BACKUP_CAN_TX_GPIO_Port GPIOB
#define BACKUP_CAN_RX_Pin GPIO_PIN_6
#define BACKUP_CAN_RX_GPIO_Port GPIOB
#define BACKUP_I2C_SDA_Pin GPIO_PIN_7
#define BACKUP_I2C_SDA_GPIO_Port GPIOB
#define BOOT_Pin GPIO_PIN_8
#define BOOT_GPIO_Port GPIOB
#define IO4_Pin GPIO_PIN_9
#define IO4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
