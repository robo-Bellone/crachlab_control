cmake_minimum_required(VERSION 3.5)
project(motor_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# add_executable(motor_node src/motor_node.cpp)
# target_link_libraries(motor_node pigpiod_if2)
# ament_target_dependencies(motor_node rclcpp)


add_executable(motor_drive_node src/motor_drive_node.cpp)
target_link_libraries(motor_drive_node pigpiod_if2)
ament_target_dependencies(motor_drive_node rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

# install(TARGETS
#   motor_node
#   DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  motor_drive_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
