cmake_minimum_required(VERSION 3.0.2)
project(lubobot)

find_package(catkin REQUIRED COMPONENTS
  lubobot_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS lubobot_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lubo_odom_node src/lubo_odom_node.cpp)
target_link_libraries(lubo_odom_node ${catkin_LIBRARIES})
add_dependencies(lubo_odom_node ${catkin_EXPORTED_TARGETS})

add_executable(lubo_imu_relay
  src/lubo_imu_relay.cpp
  src/lubo_imu_relay_node.cpp)
target_link_libraries(lubo_imu_relay ${catkin_LIBRARIES})
add_dependencies(lubo_imu_relay ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/make_urdf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)