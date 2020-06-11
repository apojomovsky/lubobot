#include <cmsis_os.h>
#include <main.h>
#include <ros.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <maincpp.h>
#include <std_msgs/String.h>

const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

extern UART_HandleTypeDef huart5;

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
}

void loop(void) {
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();
	vTaskDelay(xDelay);
}

#ifdef __cplusplus
}
#endif
