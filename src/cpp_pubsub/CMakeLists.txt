cmake_minimum_required(VERSION 3.10)
project(cpp_pubsub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(talker   src/my_publisher.cpp
                        src/pub_main.cpp)

ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/my_subscriber.cpp
                        src/sub_main.cpp)

ament_target_dependencies(listener rclcpp std_msgs)

add_executable(sine_talker src/sine_publisher.cpp
                           src/sine_pub_main.cpp)

ament_target_dependencies(sine_talker rclcpp std_msgs)

add_executable(sine_listener src/sine_subscriber.cpp
                             src/sine_sub_main.cpp)

ament_target_dependencies(sine_listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  sine_talker
  sine_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
