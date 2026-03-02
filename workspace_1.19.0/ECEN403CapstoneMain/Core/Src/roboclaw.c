/*
 * roboclaw.c
 *
 *  Created on: Mar 2, 2026
 *      Author: caseyear
 */

#include "roboclaw.h"
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

//Calculates CRC16 of nBytes of data in byte array message
unsigned int crc16(unsigned char *packet, int nBytes) {
	unsigned int crc = 0; // Initialize CRC to 0

	for (int byte = 0; byte < nBytes; byte++){
		crc = crc ^ ((unsigned int)packet[byte] << 8);
		for (unsigned char bit = 0; bit < 8; bit++){
			if (crc & 0x8000){
				crc = (crc << 1) ^ 0x1021;
			}else{
				crc = crc << 1;
			}
		}
	}
	return crc;
}

int setM1Speed(uint8_t address, int32_t speed){
	uint8_t packet[8];
	uint8_t response = 0;
	uint16_t crc = 0;
	HAL_StatusTypeDef tx_status, rx_status;

	// Build packet
	packet[0] = address;                    // Address
	packet[1] = 35;                         // Command 35: Drive M1 with signed speed
	packet[2] = (speed >> 24) & 0xFF;       // Speed byte 3 (MSB)
	packet[3] = (speed >> 16) & 0xFF;       // Speed byte 2
	packet[4] = (speed >> 8) & 0xFF;        // Speed byte 1
	packet[5] = speed & 0xFF;               // Speed byte 0 (LSB)

	crc = crc16(packet, 6);

	packet[6] = (crc >> 8) & 0xFF;          // CRC high byte first
	packet[7] = crc & 0xFF;                 // CRC low byte second

	printf("[M1] addr=0x%02X cmd=%d speed=%ld crc=0x%04X\r\n",
			address, packet[1], (long)speed, crc);
	printf("[M1] TX: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
			packet[0], packet[1], packet[2], packet[3],
			packet[4], packet[5], packet[6], packet[7]);

	// Send packet (10ms timeout for margin at 38400 baud)
	tx_status = HAL_UART_Transmit(&huart2, packet, 8, 10);
	if (tx_status != HAL_OK) {
		printf("[M1] TX FAILED: HAL status=%d, UART state=0x%02X, error=0x%02lX\r\n",
				tx_status, huart2.gState, huart2.ErrorCode);
		return 0;
	}

	// Wait for 0xFF ACK response (10ms timeout)
	rx_status = HAL_UART_Receive(&huart2, &response, 1, 10);
	if (rx_status != HAL_OK) {
		printf("[M1] RX FAILED: HAL status=%d, UART state=0x%02X, error=0x%02lX\r\n",
				rx_status, huart2.RxState, huart2.ErrorCode);
		return 0;
	}

	printf("[M1] RX response: 0x%02X\r\n", response);

	// Check for success response
	if (response == 0xFF) {
		return 1;  // Success
	}

	printf("[M1] Unexpected response: 0x%02X (expected 0xFF)\r\n", response);
	return 0;  // Invalid response
}

int setM2Speed(uint8_t address, int32_t speed) {  // Changed to int32_t
    uint8_t packet[8];
    uint8_t response;
    uint16_t crc_result;

    // Build packet
    packet[0] = address;                    // Address
    packet[1] = 36;                         // Command 36 for M2
    packet[2] = (speed >> 24) & 0xFF;       // Speed byte 3 (MSB)
    packet[3] = (speed >> 16) & 0xFF;       // Speed byte 2
    packet[4] = (speed >> 8) & 0xFF;        // Speed byte 1
    packet[5] = speed & 0xFF;               // Speed byte 0 (LSB)

    crc_result = crc16(packet, 6);

    packet[6] = (crc_result >> 8) & 0xFF;   // CRC high byte first
    packet[7] = crc_result & 0xFF;           // CRC low byte second

    // Send packet (10ms timeout)
    if (HAL_UART_Transmit(&huart2, packet, 8, 10) != HAL_OK) {
        return 0;  // Transmit failed
    }

    // Wait for 0xFF response (10ms timeout)
    if (HAL_UART_Receive(&huart2, &response, 1, 10) != HAL_OK) {
        return 0;  // Receive timeout or error
    }

    // Check for success response
    if (response == 0xFF) {
        return 1;  // Success
    }

    return 0;  // Invalid response
}
