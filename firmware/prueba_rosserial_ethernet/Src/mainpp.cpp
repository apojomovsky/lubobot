#include <cmsis_os.h>
#include <main.h>
#include <ros.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#ifdef __cplusplus
extern "C" {
#endif

#include <mainpp.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>
#include <I2Cdev.h>
#include <create2.h>
#include <MPU6050.h>

using namespace irobot;

ros::NodeHandle nh;

//extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart5;

irobot::create2 robot(&huart5, BRC_GPIO_Port, BRC_Pin);

//int16_t ax, ay, az;
//int16_t gx, gy, gz;

//char buff[50];

//sensor_msgs::Imu imu_msg;
//std_msgs::String string_msg;
geometry_msgs::Twist cmdvel_msg;
std_msgs::UInt16 left_encoder_msg;
std_msgs::UInt16 right_encoder_msg;

ros::Publisher left_encoder_pub("left_encoder", &left_encoder_msg);
ros::Publisher right_encoder_pub("right_encoder", &right_encoder_msg);
//ros::Publisher imu_pub("alexis", &imu_msg);
//ros::Publisher chatter("chatter", &string_msg);

float vel_right, vel_left;
const float wheels_dist = 235.0;

uint16_t leftEncoderRead;
uint16_t rightEncoderRead;

void cmdvel_cb(const geometry_msgs::Twist& msg) {
	vel_right = msg.linear.x + msg.angular.z * wheels_dist / 2;
	vel_left = msg.linear.x - msg.angular.z * wheels_dist / 2;
	robot.driveVelocity(vel_right, vel_left);
}

ros::Subscriber<geometry_msgs::Twist> cmdvel_sub("cmd_vel", &cmdvel_cb);

//void ToQuaternion(geometry_msgs::Quaternion* quat, int16_t yaw, int16_t pitch,
//		int16_t roll) // yaw (Z), pitch (Y), roll (X)
//		{
//	// Abbreviations for the various angular functions
//	double cy = cos(yaw * 0.5);
//	double sy = sin(yaw * 0.5);
//	double cp = cos(pitch * 0.5);
//	double sp = sin(pitch * 0.5);
//	double cr = cos(roll * 0.5);
//	double sr = sin(roll * 0.5);
//
//	quat->w = cy * cp * cr + sy * sp * sr;
//	quat->x = cy * cp * sr - sy * sp * cr;
//	quat->y = sy * cp * sr + cy * sp * cr;
//	quat->z = sy * cp * cr - cy * sp * sr;
//}

// Only set once
//imu_msg.linear_acceleration_covariance[0] = -1;
//imu_msg.orientation_covariance[0] = -1;
//imu_msg.angular_velocity_covariance[0] = -1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}


void setup(void) {
//  nh.advertise(chatter);
//  nh.advertise(imu_pub);
  nh.advertise(left_encoder_pub);
  nh.advertise(right_encoder_pub);
  nh.subscribe(cmdvel_sub);
  nh.initNode();
//  I2Cdev_init(&hi2c1);
//  MPU6050_setAddress(0x68);
//  MPU6050_initialize();
//  HAL_Delay(500);
  robot.start();
  robot.pauseStream();
//  HAL_Delay(500);
  robot.goSafeMode();
//  HAL_Delay(500);
//  robot.drivePWM(64, -64);
//  HAL_Delay(500);
//  robot.drivePWM(0, 0);
//  HAL_Delay(500);
//  robot.drivePWM(-64, 64);
//  HAL_Delay(500);
//  robot.stop();
//  HAL_Delay(500);

//  for(int i=0; i<100; ++i) {
//    sprintf(uartBuffer, "%d %d\n",leftEncoderRead, rightEncoderRead);
//	HAL_UART_Transmit(&huart3, (uint8_t*)uartBuffer, strlen(uartBuffer), 500);
//    HAL_Delay(500);
//  value = robot.getSensorData(7);
//    vTaskDelay(portTICK_PERIOD_MS * 100);
//  }
//  robot.beginDataStream(2, requestIDs);
//  HAL_Delay(2000);
//  robot.pauseStream();
//  HAL_Delay(2000);
//  robot.resumeSteam();
//  HAL_Delay(2000);
//  robot.drivePWM(0, 0);
//  HAL_Delay(1000);
//  robot.stop();
//  robot.goSafeMode();
}



char uartBuffer[100];
int8_t rotationDirection = 1;
uint8_t i =0;
void loop(void) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//	if(i == 10) {
//		i=0;
//		rotationDirection = -rotationDirection;
//	}
//    robot.drivePWM(rotationDirection * 32, - rotationDirection * 32);
//    rightEncoderRead = 0;
//    sprintf(uartBuffer, "%d\n",leftEncoderRead);
//	HAL_UART_Transmit(&huart3, (uint8_t*)uartBuffer, strlen(uartBuffer), 500);

    // copy in the data
    left_encoder_msg.data = robot.readLeftEncoder();
    left_encoder_pub.publish(&left_encoder_msg);

    right_encoder_msg.data = robot.readRightEncoder();
    right_encoder_pub.publish(&right_encoder_msg);
//    right_encoder_pub.publish(&right_encoder_msg);

////  MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//  // Updated on every step
//  imu_msg.header.frame_id = "map";
//  imu_msg.linear_acceleration.x = 0;
//  imu_msg.linear_acceleration.y = 0;
//  imu_msg.linear_acceleration.z = 0;
//  imu_msg.orientation.w = 0;
//  imu_msg.orientation.x = 0;
//  imu_msg.orientation.y = 0;
//  imu_msg.orientation.z = 0;
//
////  ToQuaternion(&imu_msg.orientation, gz, gy, gx);
//  imu_pub.publish(&imu_msg);
//
////  string_msg.data = "Hola!";
////  chatter.publish(&string_msg);
	nh.spinOnce();
//
	i++;

//	robot.requestSensorData(batteryVoltage);
//	vTaskDelay(1 / portTICK_PERIOD_MS);
//	int sensorData = robot.getSensorData(batteryVoltage);
	vTaskDelay(200 / portTICK_PERIOD_MS);
}

#ifdef __cplusplus
}
#endif
