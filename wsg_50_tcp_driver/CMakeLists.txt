cmake_minimum_required(VERSION 2.8.3)
project(wsg_50_tcp_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  controller_manager
  geometry_msgs
  hardware_interface
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  wsg_50_common
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES wsg50Move.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wsg_50_tcp_driver
  CATKIN_DEPENDS actionlib_msgs actionlib control_msgs wsg_50_common controller_manager geometry_msgs hardware_interface roscpp rospy sensor_msgs std_msgs std_srvs tf tf2
#  DEPENDS system_lib
)

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
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_node src/test.cpp ${DRIVER_SOURCES})
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(wsg50_action_node src/wsg50.cpp ${DRIVER_SOURCES})
target_link_libraries(wsg50_action_node ${catkin_LIBRARIES})
add_dependencies(wsg50_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
