cmake_minimum_required(VERSION 3.8)
project(mobile_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(cmd_vel_mobile_controller examples/cmd_vel_mobile.cpp)
ament_target_dependencies(cmd_vel_mobile_controller rclcpp nav_msgs geometry_msgs)

add_executable(cmd_joint_vel examples/cmd_joint_vel_mobile.cpp)
ament_target_dependencies(cmd_joint_vel rclcpp std_msgs sensor_msgs)

add_executable(cmd_joint_torque examples/cmd_joint_torque_mobile.cpp)
ament_target_dependencies(cmd_joint_torque rclcpp std_msgs sensor_msgs)

install(
  DIRECTORY config examples launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  cmd_vel_mobile_controller
  cmd_joint_vel
  cmd_joint_torque
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
