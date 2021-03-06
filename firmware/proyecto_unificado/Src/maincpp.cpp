#include <stdio.h>
#include <string.h>

#include <cmsis_os.h>

#include <create2.h>
#include <main.h>
#include <maincpp.h>

#include <geometry_msgs/Twist.h>
#include <lubobot_msgs/LuboEncoders.h>
#include <lubobot_msgs/LuboIMU.h>
#include <ros.h>

#include <MPU6050.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart5;

irobot::create2 robot(&huart5, BRC_GPIO_Port, BRC_Pin);

ros::NodeHandle nh;

geometry_msgs::Twist cmdvel_msg;
lubobot_msgs::LuboEncoders encoders_msg;
lubobot_msgs::LuboIMU imu_msg;

ros::Publisher imu_pub("lubo_imu", &imu_msg);
ros::Publisher encoders_pub("lubo_encoders", &encoders_msg);

// MPU6050
MPU6050 accelgyro;

// tinyIMU
int16_t ax, ay, az, gx, gy, gz;
uint32_t seq;

static const float axleLength = 0.235;  // m
static const float maxVelocity = 0.1;   // m/s

#define BOUND_CONST(val, min, max) (val < min ? min : (val > max ? max : val))

void cmdvel_cb(const geometry_msgs::Twist& msg) {
  const float xVel = msg.linear.x;
  const float angularVel = msg.angular.z;
  const float leftVel = xVel - ((axleLength / 2.0) * angularVel);
  const float rightVel = xVel + ((axleLength / 2.0) * angularVel);
  const float boundedLeftVel = BOUND_CONST(leftVel, -maxVelocity, maxVelocity);
  const float boundedRightVel =
      BOUND_CONST(rightVel, -maxVelocity, maxVelocity);
  const int16_t leftCmd = roundf(boundedLeftVel * 1000);
  const int16_t rightCmd = roundf(boundedRightVel * 1000);
  robot.driveVelocity(rightCmd, leftCmd);
}

ros::Subscriber<geometry_msgs::Twist> cmdvel_sub("cmd_vel", &cmdvel_cb);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  nh.getHardware()->reset_rbuf();
}

void setup(void) {
  nh.initNode();
  nh.advertise(encoders_pub);
  nh.advertise(imu_pub);
  nh.subscribe(cmdvel_sub);

//  I2Cdev_init(&hi2c1);
  accelgyro = MPU6050();
  accelgyro.initialize();
  vTaskDelay(500);

  // TODO: apply calibration values dynamically via rosparam or rosservice
  accelgyro.setXAccelOffset(-490);
  accelgyro.setYAccelOffset(-2274);
  accelgyro.setZAccelOffset(1865);
  accelgyro.setXGyroOffset(32);
  accelgyro.setYGyroOffset(-56);
  accelgyro.setZGyroOffset(79);

  vTaskDelay(500);

  seq = 0;

  robot.start();
  vTaskDelay(500);
  robot.pauseStream();
  vTaskDelay(500);
  robot.goSafeMode();
  vTaskDelay(500);
}

const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

void loop(void) {
  seq++;
  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  uint16_t totalTicksLeft = robot.readLeftEncoder();
  uint16_t totalTicksRight = robot.readRightEncoder();
  encoders_msg.left = totalTicksLeft;
  encoders_msg.right = totalTicksRight;

  imu_msg.header.stamp = nh.now();
  imu_msg.header.seq = seq;
  imu_msg.accel.x = accelgyro.getAccelerationX();
  imu_msg.accel.y = accelgyro.getAccelerationY();
  imu_msg.accel.z = accelgyro.getAccelerationZ();
  imu_msg.gyro.x = accelgyro.getRotationX();
  imu_msg.gyro.y = accelgyro.getRotationY();
  imu_msg.gyro.z = accelgyro.getRotationZ();

  encoders_pub.publish(&encoders_msg);
  imu_pub.publish(&imu_msg);

  nh.spinOnce();
  vTaskDelay(xDelay);
}
