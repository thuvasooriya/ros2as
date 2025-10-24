cmake_minimum_required(VERSION 3.10)
project(zumo_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(Python_EXECUTABLE "$ENV{CONDA_PREFIX}/bin/python")
set(Python_INCLUDE_DIRS "$ENV{CONDA_PREFIX}/include/python3.11")
set(Python_LIBRARIES "$ENV{CONDA_PREFIX}/lib/libpython3.11.dylib")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZumoSensors.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
