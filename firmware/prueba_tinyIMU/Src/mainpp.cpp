/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <main.h>
#include <mainpp.h>
#include <ros.h>
#include <tiny_msgs/tinyVector.h>
#include <tiny_msgs/tinyIMU.h>
#include <stdio.h>
#include <string.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include "cmsis_os.h"

ros::NodeHandle nh;

extern I2C_HandleTypeDef hi2c1;

int16_t ax, ay, az;
int16_t gx, gy, gz;

char buff[50];

tiny_msgs::tinyIMU imu_msg;
ros::Publisher imu_pub("tinyImu", &imu_msg);

uint32_t seq;

#ifdef __cplusplus
 extern "C" {
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(imu_pub);
  I2Cdev_init(&hi2c1);
  MPU6050_setAddress(0x68);
  MPU6050_initialize();
}

void loop(void)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "/my_frame";
  imu_msg.header.seq = seq;

  imu_msg.accel.x = ax;
  imu_msg.accel.y = ay;
  imu_msg.accel.z = az;
  imu_msg.gyro.x = gx;
  imu_msg.gyro.y = gy;
  imu_msg.gyro.z = gz;

  imu_pub.publish(&imu_msg);
  osDelay(10);
  nh.spinOnce();
}

#ifdef __cplusplus
}
#endif
