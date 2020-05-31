/*
 * maincpp.cpp
 *
 *  Created on: Sep 22, 2019
 *      Author: alexis
 */

#include "main.h"
#include <algorithm>    // std::max


#ifdef __cplusplus
extern "C" {
#endif

#include "create2.h"

extern UART_HandleTypeDef huart5;

irobot::create2 robot(&huart5, BRC_GPIO_Port, BRC_Pin);

void maincpp(void) {
  HAL_Delay(2000);
  robot.start();
  HAL_Delay(2000);
  robot.goFullMode();
  HAL_Delay(2000);
  robot.drivePWM(64, -64);
  HAL_Delay(2000);
  robot.drivePWM(0, 0);
  HAL_Delay(1000);
  robot.drivePWM(-64, 64);
  HAL_Delay(2000)	;
  robot.drivePWM(0, 0);
  HAL_Delay(1000);
  robot.stop();
  std::max(100, 50);
}

#ifdef __cplusplus
}
#endif
