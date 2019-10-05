/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <main.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <MPU6050.h>

ros::NodeHandle nh;

//TIM_HandleTypeDef htim2;
//UART_HandleTypeDef huart2;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

MPU6050 accelgyro(0x69);

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
}

void loop(void)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();

  HAL_Delay(1000);
}
