#ifndef LUBOIMURELAY_H
#define LUBOIMURELAY_H

#include <lubobot_msgs/LuboIMU.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class LuboIMURelay {
 public:
  LuboIMURelay();

 private:
  ros::Publisher imu_pub_;

  ros::Subscriber lubo_imu_sub_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool scale_;  // if true, the raw values from MPU6050 are scaled down to
                // m/s^2 and rad/s

  void LuboIMUCallback(const lubobot_msgs::LuboIMU&);
};

#endif  // LUBOIMURELAY_H