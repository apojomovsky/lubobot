/*
 * create2.cpp
 *
 *  Created on: Sep 21, 2019
 *      Author: alexis
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "create2.h"


namespace irobot {

create2::create2(UART_HandleTypeDef* _uart, GPIO_TypeDef* _brcPort,
		uint16_t _brcPin) :
		uart(_uart), brcPort(_brcPort), brcPin(_brcPin) {
}

create2::~create2() {
}

void create2::start() {
	uint8_t txByte = 128;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::stop() {
	uint8_t txByte = 173;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::reset() {
	uint8_t txByte = 7;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::goSafeMode() {
	uint8_t txByte = 131;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::goFullMode() {
	uint8_t txByte = 132;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::clean() {
	uint8_t txByte = 135;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::drive(int16_t velocity, int16_t radius) {
	velocity = clamp(velocity, -500, 500);
	radius = clamp(radius, -2000, 2000);
	uint8_t txData[5];
	txData[0] = 137;
	txData[1] = velocity >> 8;
	txData[2] = velocity;
	txData[3] = radius >> 8;
	txData[4] = radius;
	HAL_UART_Transmit(uart, txData, 5, 500);
}

void create2::driveVelocity(int16_t rightVel, int16_t leftVel) {
//	rightVel = clamp(rightVel, -100, 100);
//	leftVel = clamp(leftVel, -100, 100);
	uint8_t txData[5];
	txData[0] = 145;
	txData[1] = rightVel >> 8;
	txData[2] = rightVel & 0xFF;
	txData[3] = leftVel >> 8;
	txData[4] = leftVel & 0xFF;
	HAL_UART_Transmit(uart, txData, 5, 100);
}

void create2::drivePWM(int16_t rightPWM, int16_t leftPWM) {
	rightPWM = clamp(rightPWM, -255, 255);
	leftPWM = clamp(leftPWM, -255, 255);
	uint8_t txData[5];
	txData[0] = 146;
	txData[1] = rightPWM >> 8;
	txData[2] = rightPWM;
	txData[3] = leftPWM >> 8;
	txData[4] = leftPWM;
	HAL_UART_Transmit(uart, txData, 5, 500);
}

void create2::pauseStream() {
    uint8_t txByte = 150;
    HAL_UART_Transmit(uart, &txByte, 1, 500);
    txByte = (uint8_t) 0x00;
    HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::resumeSteam() {
	uint8_t txByte = 150;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
	txByte = 1;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}


uint16_t create2::readLeftEncoder() {
  uint16_t leftEncoderCount = 0;
  uint8_t txByte;
  uint8_t* txBytePtr = &txByte;
  txByte = 142;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  txByte = 43;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  HAL_UART_Receive(uart, (uint8_t*)&leftEncoderCount, 2, 300);
  return leftEncoderCount;
}


uint16_t create2::readRightEncoder() {
  uint16_t rightEncoderCount = 0;
  uint8_t txByte;
  uint8_t* txBytePtr = &txByte;
  txByte = 142;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  txByte = 44;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  HAL_UART_Receive(uart, (uint8_t*)&rightEncoderCount, 2, 300);
  return rightEncoderCount;
}
} /* namespace irobot */

#ifdef __cplusplus
}
#endif
