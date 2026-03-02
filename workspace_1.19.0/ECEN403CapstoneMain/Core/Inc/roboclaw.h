/*
 * roboclaw.h
 *
 *  Created on: Mar 2, 2026
 *      Author: caseyear
 */

#ifndef INC_ROBOCLAW_H_
#define INC_ROBOCLAW_H_

#include <stdint.h>

#define GET_M1_ENCODER 16
#define GET_M2_ENCODER 17
#define GET_M1_SPEED 18
#define GET_M2_SPEED 19
#define GET_M1_ENCODER_COUNT 22
#define GET_M2_ENCODER_COUNT 23


unsigned int crc16(unsigned char *packet, int nBytes);
int setM1Speed(uint8_t address, int32_t speed);
int setM2Speed(uint8_t address, int32_t speed);

#endif /* INC_ROBOCLAW_H_ */
