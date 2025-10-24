cmake_minimum_required(VERSION 3.10)
project(zumo_serial)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(zumo_msgs REQUIRED)
find_package(zumo_defs REQUIRED)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(include ${Boost_INCLUDE_DIRS})

# Add executable
add_executable(zumo_serial_node   src/zumo_serial_node.cpp
                                  src/main.cpp)

target_link_libraries(zumo_serial_node ${Boost_LIBRARIES})

ament_target_dependencies(zumo_serial_node  rclcpp std_msgs zumo_msgs zumo_defs)


install(TARGETS
  zumo_serial_node  
  DESTINATION lib/${PROJECT_NAME})

ament_package()
