cmake_minimum_required(VERSION 3.10)
project(zumo_calibration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(zumo_msgs REQUIRED)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

include_directories(include ${Boost_INCLUDE_DIRS}
                            ${Eigen3_INCLUDE_DIRS})

# Add executable
add_executable(zumo_acc_calib_node    src/zumo_acc_calib.cpp
                                      src/acc_main.cpp)

target_include_directories(zumo_acc_calib_node PUBLIC ${Eigen3_INCLUDE_DIRS})

# Specify libraries to link a library or executable target against
target_link_libraries(zumo_acc_calib_node   ${Boost_LIBRARIES} 
                                            ${Eigen_LIBRARIES})

ament_target_dependencies(zumo_acc_calib_node   rclcpp 
                                                zumo_msgs
                                                Eigen3)


# Add executable
add_executable(zumo_mag_calib_node    src/zumo_mag_calib.cpp
                                      src/mag_main.cpp)

ament_target_dependencies(zumo_mag_calib_node   rclcpp
                                                zumo_msgs)

install(TARGETS
  zumo_acc_calib_node
  zumo_mag_calib_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
