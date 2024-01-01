cmake_minimum_required(VERSION 3.5)
project(object_detector)

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
find_package(VISP REQUIRED)

include_directories(
  include
  ${VISP_INCLUDE_DIRS}
)


# Create executable
add_executable(object_detector src/object_detector.cpp)
target_link_libraries(object_detector ${VISP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(object_detector rclcpp std_msgs)
# Install the executable
install(TARGETS
  object_detector
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_package()