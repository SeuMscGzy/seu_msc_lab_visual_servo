cmake_minimum_required(VERSION 3.5)
project(ur5_servo)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(joint_vel_sender src/joint_vel_sender.cpp)
#Include directories
ament_target_dependencies(joint_vel_sender
    rclcpp
    std_msgs
)

# Install executable
install(TARGETS joint_vel_sender 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
