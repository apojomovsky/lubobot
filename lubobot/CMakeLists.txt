cmake_minimum_required(VERSION 3.0.2)
project(lubobot)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

add_message_files(
  FILES
  luboEncoders.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_node src/odom_node.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES})
add_dependencies(odom_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/diff_tf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)