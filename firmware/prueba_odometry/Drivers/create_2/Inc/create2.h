/*
 * create2.h
 *
 *  Created on: Sep 21, 2019
 *      Author: alexis
 */

#ifndef create2_CREATE2_H_
#define create2_CREATE2_H_

#define clamp(value, min, max) (value < min ? min : value > max ? max : value)

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f7xx_hal.h>
#include <float.h>
#include <stdlib.h>


#define ARM_MATH_CM7
#include <arm_math.h>

namespace irobot {

namespace util {

  static const uint8_t STREAM_HEADER = 19;
  static const float V_3_TICKS_PER_REV = 508.8;
  static const uint32_t V_3_MAX_ENCODER_TICKS = 65535;
  static const float MAX_RADIUS = 2.0;
  static const float STRAIGHT_RADIUS = 32.768;
  static const float IN_PLACE_RADIUS = 0.001;
//  static const float PI = 3.14159;
  static const float TWO_PI = 6.28318;
  static const float EPS = 0.0001;
  static const float WHEEL_DIAMETER = 0.072;
  static const float AXLE_LENGTH = 0.235;

  inline float normalizeAngle(const float& angle) {
    float a = angle;
    while (a < -PI) a += TWO_PI;
    while (a > PI) a -= TWO_PI;
    return a;
  }
}  // namespace util

const uint8_t single_byte_packets[22] = { 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18, 21,
		24, 34, 35, 36, 37, 38, 45, 52, 53, 58 };

enum packetIds {
	//all sent within packet 7, need to unpack byte upon retrieval
	rightBumper = 101,
	leftBumper = 102,
	rightWheelDrop = 103,
	leftWheelDrop = 104,
	//all sent within packet 14, need to unpack byte upon retrieval
	sideBrushOvercurrent = 105,
	mainBrushOvercurrent = 106,
	rightWheelOvercurrent = 107,
	leftWheelOvercurrent = 108,
	//all sent within packet 18, need to unpack byte upon retrieval
	cleanButton = 109,
	spotButton = 110,
	dockButton = 111,
	minuteButton = 112,
	hourButton = 113,
	dayButton = 114,
	scheduleButton = 115,
	clockButton = 116,
	//all sent within packet 45, need to unpack byte upon retrieval
	lightBumperLeft = 117,
	lightBumperFrontLeft = 118,
	lightBumperCenterLeft = 119,
	lightBumperCenterRight = 120,
	lightBumperFrontRight = 121,
	lightBumperRight = 122,
	//boolean values
	wallSensor = 8,
	leftCliff = 9,
	frontLeftCliff = 10,
	frontRightCliff = 11,
	rightCliff = 12,
	virtualWall = 13,
	songPlaying = 37,
	stasis = 58,
	//byte values
	dirtDetector = 15,
	irOmnidirectional = 17,
	irLeft = 52,
	irRight = 53,
	chargingState = 21, //0-5
	chargingSources = 34, //0-3
	operationalMode = 35, //0-3
	songNumber = 36, //0-15
	streamPackets = 38, //0-108
	//char values
	batteryTempInC = 24,
	//int values
	distanceTraveled = 19,
	degreesTurned = 20,
	currentFlow = 23,
	wallSignal = 27, //0-1027
	leftCliffSignal = 28, //0-4095
	frontLeftCliffSignal = 29, //0-4095
	frontRightCliffSignal = 30, //0-4095
	rightCliffSignal = 31, //0-4095
	requestedVelocity = 39, //-500 - 500
	requestedRadius = 40,
	requestedRightVelocity = 41, //-500 - 500
	requestedLeftVelocity = 42, //-500 - 500
	lightBumperLeftSignal = 46, //0-4095
	lightBumperFrontLeftSignal = 47, //0-4095
	lightBumperCenterLeftSignal = 48, //0-4095
	lightBumperCenterRightSignal = 49, //0-4095
	lightBumperFrontRightSignal = 50, //0-4095
	lightBumperRightSignal = 51, //0-4095
	leftMotorCurrent = 54,
	rightMotorCurrent = 55,
	mainBrushCurrent = 56,
	sideBrushCurrent = 57,
	//unsigned int values
	batteryVoltage = 22,
	batterCharge = 25,
	batteryCapacity = 26,
	leftEncoderCount = 43,
	rightEncoderCount = 44
};

class create2 {
public:
	create2(UART_HandleTypeDef* _uart, GPIO_TypeDef* _brcPort,
			uint16_t _brcPin);
	virtual ~create2();
	void start();
	void reset();
	void stop();
//		void turnOff();
//		void resetBaudRate();
//		void setBaud(long rate);
	void goSafeMode();
	void goFullMode();
	void goPassiveMode();
	void clean();
//		void maxClean();
//		void spotClean();
//		void seekDock();

	// Available is safe or full modes
	// Wheel motors
	void drive(int16_t velocity, int16_t radius);
//		void driveLeft(int left);
//		void driveRight(int right);
	void driveVelocity(int16_t rightVel, int16_t leftVel);
	void drivePWM(int16_t rightPWM, int16_t leftPWM);
//		void turnCW(unsigned short velocity,unsigned short degrees);
//		void turnCCW(unsigned short velocity, unsigned short degrees);

	// Cleaning motors
//		void enableBrushes(bool mainBrushDirection, bool sideBrushDirection, bool mainBrush, bool vacuum, bool sideBrush);
//		void setMainBrush(bool direction, bool enable);
//		void setSideBrush(bool direction, bool enable);
//		void enableVacuum(bool enable);
//		void setMainBrushSpeed(char speed);
//		void setSideBrushSpeed(char speed);
//		void setVaccumSpeed(char speed);

	// Leds
//		void setPowerLEDs(uint8_t color, uint8_t intensity);
//		void setDebrisLED(bool enable);
//		void setSpotLED(bool enable);
//		void setDockLED(bool enable);
//		void setWarningLED(bool enable);
//		void setScheduleLEDs(uint8_t days, bool schedule, bool clock, bool am, bool pm, bool colon);
//		void setDigitLEDs(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4);
//		void setDigitLEDFromASCII(uint8_t digit, char letter);

	// Non blocking sensor functions
	void requestSensorData(uint8_t sensorID);
	void requestSensorDataList(uint8_t numOfRequests, uint8_t requestIDs[]);
	void beginDataStream(uint8_t numOfRequests, uint8_t requestIDs[]);
	void pauseStream();
	void resumeSteam();

	// Blocking sensor functions - these will request data and wait until a response is received,
	// then return the response.
	int getSensorData(uint8_t sensorID);
	void getSensorData(int returnVal[], uint8_t numOfRequests, uint8_t requestIDs[]);
	bool getSensorData(uint8_t * buffer, uint8_t bufferLength);

    void readEncoders(uint16_t* leftEncoderCount, uint16_t* rightEncoderCount);

    uint16_t readLeftEncoder();
    uint16_t readRightEncoder();

private:
	bool is_single_byte(uint8_t val);

	UART_HandleTypeDef* uart;

	GPIO_TypeDef* brcPort;

	uint16_t brcPin;
};

}

#ifdef __cplusplus
}
#endif

#endif /* create2_CREATE2_H_ */
