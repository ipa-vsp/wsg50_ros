cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_tcp_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


set(DRIVER_SOURCES
  src/checksum.cpp include/wsg_50_tcp_driver/checksum.h
  src/cmd.cpp include/wsg_50_tcp_driver/cmd.h
  src/common.cpp include/wsg_50_tcp_driver/common.h
  src/function.cpp include/wsg_50_tcp_driver/function.h
  src/msg.cpp include/wsg_50_tcp_driver/msg.h
  src/tcp.cpp include/wsg_50_tcp_driver/tcp.h
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

add_executable(test_node src/test.cpp ${DRIVER_SOURCES})
target_link_libraries(test_node ${catkin_LIBRARIES})