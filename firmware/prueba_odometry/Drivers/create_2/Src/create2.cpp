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
	rightVel = clamp(rightVel, -500, 500);
	leftVel = clamp(leftVel, -500, 500);
	uint8_t txData[5];
	txData[0] = 145;
	txData[1] = rightVel >> 8;
	txData[2] = rightVel;
	txData[3] = leftVel >> 8;
	txData[4] = leftVel;
	HAL_UART_Transmit(uart, txData, 5, 500);
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

void create2::requestSensorData(uint8_t sensorID) {
	uint8_t packetID = 0;
	if (sensorID > 100) {
		switch (sensorID) {
		case 101:
		case 102:
		case 103:
		case 104:
			packetID = 7;
			break;
		case 105:
		case 106:
		case 107:
		case 108:
			packetID = 14;
			break;
		case 109:
		case 110:
		case 111:
		case 112:
		case 113:
		case 114:
		case 115:
		case 116:
			packetID = 18;
			break;
		case 117:
		case 118:
		case 119:
		case 120:
		case 121:
		case 122:
			packetID = 45;
			break;
		}

	} else {
		packetID = sensorID;
	}

	uint8_t txByte = 142;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
	txByte = (uint8_t) packetID;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
}

void create2::requestSensorDataList(uint8_t numOfRequests,
		uint8_t requestIDs[]) {
	uint8_t packetIDs[numOfRequests];
	for (uint8_t i = 0; i < numOfRequests; ++i) {
		if (requestIDs[i] > 100) {
			switch (requestIDs[i]) {
			case 101:
			case 102:
			case 103:
			case 104:
				packetIDs[i] = 7;
				break;
			case 105:
			case 106:
			case 107:
			case 108:
				packetIDs[i] = 14;
				break;
			case 109:
			case 110:
			case 111:
			case 112:
			case 113:
			case 114:
			case 115:
			case 116:
				packetIDs[i] = 18;
				break;
			case 117:
			case 118:
			case 119:
			case 120:
			case 121:
			case 122:
				packetIDs[i] = 45;
				break;
			}

		} else {
			packetIDs[i] = requestIDs[i];
		}
	}
	uint8_t txByte = 149;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
	HAL_UART_Transmit(uart, &numOfRequests, 1, 500);
	for (int i = 0; i < numOfRequests; ++i) {
		HAL_UART_Transmit(uart, &packetIDs[i], 1, 500);
	}
}

void create2::beginDataStream(uint8_t numOfRequests, uint8_t requestIDs[]) {
	uint8_t packetIDs[numOfRequests];
	for (int i = 0; i < numOfRequests; i++) {
		if (requestIDs[i] > 100) {
			switch (requestIDs[i]) {
			case 101:
			case 102:
			case 103:
			case 104:
				packetIDs[i] = 7;
				break;
			case 105:
			case 106:
			case 107:
			case 108:
				packetIDs[i] = 14;
				break;
			case 109:
			case 110:
			case 111:
			case 112:
			case 113:
			case 114:
			case 115:
			case 116:
				packetIDs[i] = 18;
				break;
			case 117:
			case 118:
			case 119:
			case 120:
			case 121:
			case 122:
				packetIDs[i] = 45;
				break;
			}

		} else {
			packetIDs[i] = requestIDs[i];
		}
	}
	uint8_t txByte = 148;
	HAL_UART_Transmit(uart, &txByte, 1, 500);
	HAL_UART_Transmit(uart, &numOfRequests, 1, 500);
	for (int i = 0; i < numOfRequests; ++i) {
		HAL_UART_Transmit(uart, &packetIDs[i], 1, 500);
	}
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

int create2::getSensorData(uint8_t sensorID) {
	int returnVal = -2;
	uint8_t packetID = 0;
	if (sensorID > 100) {
		switch (sensorID) {
		case 101:
		case 102:
		case 103:
		case 104:
			packetID = 7;
			break;
		case 105:
		case 106:
		case 107:
		case 108:
			packetID = 14;
			break;
		case 109:
		case 110:
		case 111:
		case 112:
		case 113:
		case 114:
		case 115:
		case 116:
			packetID = 18;
			break;
		case 117:
		case 118:
		case 119:
		case 120:
		case 121:
		case 122:
			packetID = 45;
			break;
		}

	} else {
		packetID = sensorID;
	}
	uint8_t txByte = 142;
	uint8_t MSB;
	uint8_t LSB;
	HAL_UART_Transmit(uart, &txByte, 1, 100);
	HAL_UART_Transmit(uart, &packetID, 1, 100);
	if (is_single_byte(packetID)) {
		if(HAL_UART_Receive(uart, &LSB, 1, 100) != HAL_OK)
		{
		  return -1;
		}
		returnVal = LSB && 0xFF;
	}
	else {
		if(HAL_UART_Receive(uart, &MSB, 1, 100) != HAL_OK)
		{
		  return -1;
		}
		if(HAL_UART_Receive(uart, &LSB, 1, 100) != HAL_OK)
		{
		  return -1;
		}
		returnVal = (int)((MSB << 7) | (LSB && 0xFF));
	}
	return returnVal;
}

void create2::getSensorData(int returnVal[], uint8_t numOfRequests,
		uint8_t requestIDs[]) {
	uint8_t packetIDs[numOfRequests];
	for (int i = 0; i < numOfRequests; i++) {
		if (requestIDs[i] > 100) {
			switch (requestIDs[i]) {
			case 101:
			case 102:
			case 103:
			case 104:
				packetIDs[i] = 7;
				break;
			case 105:
			case 106:
			case 107:
			case 108:
				packetIDs[i] = 14;
				break;
			case 109:
			case 110:
			case 111:
			case 112:
			case 113:
			case 114:
			case 115:
			case 116:
				packetIDs[i] = 18;
				break;
			case 117:
			case 118:
			case 119:
			case 120:
			case 121:
			case 122:
				packetIDs[i] = 45;
				break;
			}
		} else {
			packetIDs[i] = requestIDs[i];
		}
	}
	uint8_t txByte = 149;
	uint8_t rxBytes[2];
	HAL_UART_Transmit(uart, &txByte, 1, 500);
	HAL_UART_Transmit(uart, &numOfRequests, 1, 500);
	for (int i = 0; i < numOfRequests; ++i) {
		HAL_UART_Transmit(uart, &packetIDs[i], 1, 500);
	}
	uint8_t pos = 0;
	for (int i = 0; i < numOfRequests / 2; ++i) {
		HAL_UART_Receive(uart, rxBytes, 2, 500);
		returnVal[pos++] = (rxBytes[0] << 7) | rxBytes[1];
	}
}

/**
 Returns false if failed, due to a timeout
 returns true if successful and fills the buffer provided by the pointer
 **/
bool create2::getSensorData(uint8_t * buffer, uint8_t bufferLength) {
	uint8_t rxByte;
	while (bufferLength-- > 0) {
		if (HAL_UART_Receive(uart, &rxByte, 1, 200) != HAL_OK) {
			return false; // Timed out
		}
		*buffer++ = rxByte;
	}
	return true;
}

void create2::readEncoders(uint16_t* leftEncoderCount, uint16_t* rightEncoderCount) {
  uint16_t rxBuffer[2];
  for(uint8_t i=0; i<2; ++i) {
	  rxBuffer[i] = 0;
  }
  uint8_t txByte;
  uint8_t* txBytePtr = &txByte;
  txByte = 149;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  txByte = 2;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  txByte = 43;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  txByte = 44;
  HAL_UART_Transmit(uart, txBytePtr, 1, 100);
  HAL_UART_Receive(uart, (uint8_t*)rxBuffer, 4, 300);
  *leftEncoderCount = (rxBuffer[0]);
  *rightEncoderCount = (rxBuffer[1]);
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

bool create2::is_single_byte(uint8_t val) {
	for (int i = 0; i < 22; ++i) {
		if (val == single_byte_packets[i]) {
			return true;
		}
	}
	return false;
}

} /* namespace irobot */

#ifdef __cplusplus
}
#endif
