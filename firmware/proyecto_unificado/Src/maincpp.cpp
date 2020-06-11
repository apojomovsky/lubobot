#include <stdio.h>
#include <string.h>

#include <cmsis_os.h>

#include <create2.h>
#include <main.h>
#include <maincpp.h>

#include <std_msgs/String.h>
#include <ros.h>

const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

extern UART_HandleTypeDef huart5;
irobot::create2 robot(&huart5, BRC_GPIO_Port, BRC_Pin);

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

void setup(void) {
	nh.initNode();
    nh.advertise(chatter);
	robot.start();
	vTaskDelay(500);
	robot.pauseStream();
	vTaskDelay(500);
	robot.goSafeMode();
	vTaskDelay(500);
	robot.driveVelocity(-64, 64);
	vTaskDelay(1000);
	robot.driveVelocity(0, 0);
	vTaskDelay(500);
	robot.driveVelocity(64, -64);
	vTaskDelay(1000);
	robot.driveVelocity(0, 0);
}

void loop(void) {
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();
	vTaskDelay(xDelay);
}
