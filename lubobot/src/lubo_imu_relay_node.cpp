#include <ros/ros.h>

#include "lubo_imu_relay.h"

int main(int argc, char** argv) {
  bool scale;

  ros::init(argc, argv, "LuboIMURelay");
  ros::NodeHandle n;

  LuboIMURelay lubo_relay;
  ros::spin();

  return 1;
}
