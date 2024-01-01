cmake_minimum_required(VERSION 3.5)
project(conveyor_belt_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
)

link_directories(${SERIAL_LIBRARY_DIRS})

# Create executable
add_executable(belt_control src/belt_control_node.cpp)
target_link_libraries(belt_control ${SERIAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(belt_control rclcpp std_msgs)

# Install the executable
install(TARGETS
  belt_control
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_package()