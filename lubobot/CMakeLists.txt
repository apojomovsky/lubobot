cmake_minimum_required(VERSION 3.0.2)
project(lubobot)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(diff_tf_cpp src/diff_tf.cpp)
# target_link_libraries(diff_tf_cpp ${catkin_LIBRARIES})
# add_dependencies(diff_tf_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/diff_tf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)