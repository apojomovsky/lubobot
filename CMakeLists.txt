cmake_minimum_required(VERSION 3.0.2)

project(lubobot)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
)

catkin_install_python(PROGRAMS
  scripts/make_urdf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)