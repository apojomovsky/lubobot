#include <cmsis_os.h>
#include <main.h>
#include <ros.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif

#include <mainpp.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <I2Cdev.h>
#include <create2.h>
#include <MPU6050.h>

ros::NodeHandle nh;


extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2;

irobot::create2 robot(&huart2, BRC_GPIO_Port, BRC_Pin);

int16_t ax, ay, az;
int16_t gx, gy, gz;

char buff[50];

sensor_msgs::Imu imu_msg;
std_msgs::String string_msg;
ros::Publisher imu_pub("alexis", &imu_msg);
ros::Publisher chatter("chatter", &string_msg);



void ToQuaternion(geometry_msgs::Quaternion* quat, int16_t yaw, int16_t pitch, int16_t roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quat->w = cy * cp * cr + sy * sp * sr;
    quat->x = cy * cp * sr - sy * sp * cr;
    quat->y = sy * cp * sr + cy * sp * cr;
    quat->z = sy * cp * cr - cy * sp * sr;
}

// Only set once
//imu_msg.linear_acceleration_covariance[0] = -1;
//imu_msg.orientation_covariance[0] = -1;
//imu_msg.angular_velocity_covariance[0] = -1;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
//  nh.initNode();
//  nh.advertise(chatter);
//  nh.advertise(imu_pub);
//  I2Cdev_init(&hi2c1);
//  MPU6050_setAddress(0x68);
//  MPU6050_initialize();
  HAL_Delay(5000);
  robot.start();
  HAL_Delay(5000);
  robot.goFullMode();
  HAL_Delay(2000);
  robot.drivePWM(128, -128);
  HAL_Delay(2000);
  robot.drivePWM(0, 0);
  HAL_Delay(1000);
  robot.drivePWM(-128, 128);
  HAL_Delay(2000);
  robot.drivePWM(0, 0);
  HAL_Delay(1000);
  robot.stop();
}

void loop(void)
{
//  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
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
//  nh.spinOnce();
//
//  vTaskDelay(100 / portTICK_PERIOD_MS);
}

#ifdef __cplusplus
}
#endif
