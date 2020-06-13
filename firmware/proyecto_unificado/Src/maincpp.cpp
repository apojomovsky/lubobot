#include <stdio.h>
#include <string.h>

#include <cmsis_os.h>

//#include <create2.h>
#include <main.h>
#include <maincpp.h>

//#include <geometry_msgs/Twist.h>
//#include <std_msgs/UInt16.h>
#include <ros.h>
#include <tiny_msgs/tinyIMU.h>

#include <MPU6050.h>



extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart5;

int16_t ax, ay, az, gx, gy, gz;

tiny_msgs::tinyIMU imu_msg;

uint32_t seq;

//irobot::create2 robot(&huart5, BRC_GPIO_Port, BRC_Pin);

const TickType_t xDelay = 20 / portTICK_PERIOD_MS;

ros::NodeHandle nh;
//std_msgs::UInt16 left_ticks_msg;
//std_msgs::UInt16 right_ticks_msg;
//ros::Publisher left_ticks_pub("left_ticks", &left_ticks_msg);
//ros::Publisher right_ticks_pub("right_ticks", &right_ticks_msg);
ros::Publisher imu_pub("tinyImu", &imu_msg);

//geometry_msgs::Twist cmdvel_msg;

//float vel_right, vel_left;
//const float wheels_dist = 235.0;
//
//void cmdvel_cb(const geometry_msgs::Twist& msg) {
//	vel_right = msg.linear.x + msg.angular.z * wheels_dist / 2;
//	vel_left = msg.linear.x - msg.angular.z * wheels_dist / 2;
//	robot.driveVelocity(vel_right, vel_left);
//}

//ros::Subscriber<geometry_msgs::Twist> cmdvel_sub("cmd_vel", &cmdvel_cb);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

//uint16_t prevTicksLeft = 0;
//uint16_t prevTicksRight = 0;
//bool firstLoop = true;
//uint32_t prevOnDataTime = 0;

void setup(void) {
	nh.initNode();
//    nh.advertise(left_ticks_pub);
//    nh.advertise(right_ticks_pub);
//    nh.subscribe(cmdvel_sub);
    nh.advertise(imu_pub);

	I2Cdev_init(&hi2c1);
	MPU6050_setAddress(0x68);
	MPU6050_initialize();
	seq = 0;

//	robot.start();
//	vTaskDelay(500);
//	robot.pauseStream();
//	vTaskDelay(500);
//	robot.goSafeMode();
//	vTaskDelay(500);
//	robot.driveVelocity(-64, 64);
//	vTaskDelay(1000);
//	robot.driveVelocity(0, 0);
//	vTaskDelay(500);
//	robot.driveVelocity(64, -64);
	vTaskDelay(1000);
//	robot.driveVelocity(0, 0);
}

void loop(void) {
//	if (firstLoop) {
//		prevTicksLeft = robot.readLeftEncoder();
//		prevTicksRight = robot.readRightEncoder();
//		prevOnDataTime = HAL_GetTick();
//		vTaskDelay(100 / portTICK_PERIOD_MS);
//		firstLoop = false;
//		return;
//	}
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//	// Get cumulative ticks (wraps around at 65535)
//	uint16_t totalTicksLeft = robot.readLeftEncoder();
//	uint16_t totalTicksRight = robot.readRightEncoder();
//    left_ticks_msg.data = totalTicksLeft;
//    right_ticks_msg.data = totalTicksRight;
//    left_ticks_pub.publish(&left_ticks_msg);
//    right_ticks_pub.publish(&right_ticks_msg);

    seq++;
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

    imu_pub.publish( &imu_msg );

    nh.spinOnce();
	vTaskDelay(xDelay);
}
