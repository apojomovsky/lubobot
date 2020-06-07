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
#include <arm_math.h>

namespace util {

static const uint8_t STREAM_HEADER = 19;
static const float V_3_TICKS_PER_REV = 508.8;
static const uint32_t V_3_MAX_ENCODER_TICKS = 65535;
static const float MAX_RADIUS = 2.0;
static const float STRAIGHT_RADIUS = 32.768;
static const float IN_PLACE_RADIUS = 0.001;
//static const float PI = 3.14159;
static const float TWO_PI = 6.28318;
static const float EPS = 0.0001;
static const float WHEEL_DIAMETER = 0.072;
static const float AXLE_LENGTH = 0.235;

inline float normalizeAngle(const float &angle) {
	float a = angle;
	while (a < -PI)
		a += TWO_PI;
	while (a > PI)
		a -= TWO_PI;
	return a;
}
}  // namespace util

namespace irobot {

class create2 {
public:
	create2(UART_HandleTypeDef *_uart, GPIO_TypeDef *_brcPort,
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
	int* getSensorData(uint8_t numOfRequests, uint8_t requestIDs[]);
	void decodeIR(uint8_t value);
	bool getSensorData(uint8_t *buffer, uint8_t bufferLength);

	uint16_t readLeftEncoder();
	uint16_t readRightEncoder();

private:
	UART_HandleTypeDef *uart;

	GPIO_TypeDef *brcPort;

	uint16_t brcPin;
};

}

#ifdef __cplusplus
}
#endif

#endif /* create2_CREATE2_H_ */
