cmake_minimum_required(VERSION 3.8)
project(robolubo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# --- C++ executables ---
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp std_msgs)
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

# --- Python module ---
ament_python_install_package(${PROJECT_NAME})

# --- Python scripts / nodes ---
# install(PROGRAMS
#   robolubo/my_py_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# --- Launch files ---
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# --- Config files ---
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
