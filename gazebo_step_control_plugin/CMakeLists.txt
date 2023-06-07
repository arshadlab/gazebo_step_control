cmake_minimum_required(VERSION 3.5)
project(gazebo_step_control_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_step_control_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)


link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

# gazebo_ros_init
add_library(gazebo_step_control SHARED
  src/gazebo_step_control.cpp
)
ament_target_dependencies(gazebo_step_control
  "gazebo_step_control_interface"
  "gazebo_dev"
  "gazebo_ros"
  "std_srvs"
)

ament_export_libraries(gazebo_step_control)
ament_export_dependencies(gazebo_dev)

ament_package()

install(
  PROGRAMS
    DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS
  gazebo_step_control
  LIBRARY DESTINATION lib
  )
