///*
// * maincpp.cpp
// *
// *  Created on: Sep 22, 2019
// *      Author: alexis
// */
//

#include <cmsis_os.h>
#include <main.h>
#include <ros.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <create2.h>
//#include <std_msgs/String.h>
//
extern UART_HandleTypeDef huart5;
//
irobot::create2 robot(&huart5, BRC_GPIO_Port, BRC_Pin);
//ros::NodeHandle nh;
//
//void maincpp(void) {
//  HAL_Delay(1000);
//  robot.start();
//  HAL_Delay(1000);
//  robot.goSafeMode();
//  HAL_Delay(1000);
//  robot.drivePWM(64, -64);
//  HAL_Delay(1000);
//  robot.drivePWM(0, 0);
//  HAL_Delay(500);
//  robot.drivePWM(-64, 64);
//  HAL_Delay(1000)	;
//  robot.drivePWM(0, 0);
//  HAL_Delay(1000);
//  robot.stop();
//  std::max(100, 50);
//}

#include <maincpp.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.spinOnce();
  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  osDelay(1000);
  robot.start();
  osDelay(1000);
  robot.goSafeMode();
  osDelay(1000);
  robot.drivePWM(64, -64);
  osDelay(1000);
  robot.drivePWM(0, 0);
  osDelay(1000);
  robot.drivePWM(-64, 64);
  osDelay(1000);
  robot.drivePWM(0, 0);
  osDelay(1000);
  robot.stop();
}

void loop(void)
{
  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();
  osDelay(1000);
}

#ifdef __cplusplus
}
#endif
