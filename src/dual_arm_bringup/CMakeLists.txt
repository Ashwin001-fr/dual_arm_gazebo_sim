cmake_minimum_required(VERSION 3.5)
project(dual_arm_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install Python launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDFs
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

