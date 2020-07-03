#include "lubo_imu_relay.h"

LuboIMURelay::LuboIMURelay() {
  nh_ = ros::NodeHandle();

  nh_private_ = ros::NodeHandle("~");

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_msgs", 10);

  lubo_imu_sub_ =
      nh_.subscribe("lubo_imu", 1, &LuboIMURelay::LuboIMUCallback, this);

  nh_private_.param<bool>("scale", scale_, false);
}

void LuboIMURelay::LuboIMUCallback(const lubobot_msgs::LuboIMU& lubo_imu_msg_) {
  static const float g_to_m_s2 = 9.81;  // conversion from g to m/s^2
  static const float deg_to_rad =
      3.14159265359 / 180.;  // conversion from degree to radiants
  static const float scale_accel =
      1 / 16384.;  // conversion from MPU6050 accel readings to g
  static const float scale_gyro =
      1 / 131.;  // conversion from MPU6050 gyro readings to degree/second

  sensor_msgs::Imu big_imu_msg;

  big_imu_msg.header = lubo_imu_msg_.header;

  if (scale_) {
    big_imu_msg.linear_acceleration.x =
        lubo_imu_msg_.accel.x * scale_accel * g_to_m_s2;
    big_imu_msg.linear_acceleration.y =
        lubo_imu_msg_.accel.y * scale_accel * g_to_m_s2;
    big_imu_msg.linear_acceleration.z =
        lubo_imu_msg_.accel.z * scale_accel * g_to_m_s2;

    big_imu_msg.angular_velocity.x =
        lubo_imu_msg_.gyro.x * scale_gyro * deg_to_rad;
    big_imu_msg.angular_velocity.y =
        lubo_imu_msg_.gyro.y * scale_gyro * deg_to_rad;
    big_imu_msg.angular_velocity.z =
        lubo_imu_msg_.gyro.z * scale_gyro * deg_to_rad;
  } else {
    big_imu_msg.linear_acceleration.x = lubo_imu_msg_.accel.x;
    big_imu_msg.linear_acceleration.y = lubo_imu_msg_.accel.y;
    big_imu_msg.linear_acceleration.z = lubo_imu_msg_.accel.z;

    big_imu_msg.angular_velocity.x = lubo_imu_msg_.gyro.x;
    big_imu_msg.angular_velocity.y = lubo_imu_msg_.gyro.y;
    big_imu_msg.angular_velocity.z = lubo_imu_msg_.gyro.z;
  }

  // FIll Quaternion
  big_imu_msg.orientation.w = 0.0;
  big_imu_msg.orientation.x = 0.0;
  big_imu_msg.orientation.y = 0.0;
  big_imu_msg.orientation.z = 0.0;

  for (int i = 0; i < 9; ++i) {
    big_imu_msg.angular_velocity_covariance[i] = 0;
    big_imu_msg.linear_acceleration_covariance[i] = 0;
    big_imu_msg.orientation_covariance[i] = 0;
  }

  imu_pub_.publish(big_imu_msg);
}
