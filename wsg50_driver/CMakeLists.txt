cmake_minimum_required(VERSION 3.8)
project(wsg50_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

set(TCP_SRC
  src/checksum.cpp
  src/cmd.cpp
  src/common.cpp
  src/function.cpp
  src/msg.cpp
  src/tcp.cpp
)

add_library(${PROJECT_NAME}
  SHARED
  ${TCP_SRC}
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)

add_library(gripper_server SHARED src/main.cpp)
target_link_libraries(gripper_server ${PROJECT_NAME})
target_include_directories(gripper_server PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gripper_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "sensor_msgs"
  "control_msgs"
)

rclcpp_components_register_node(gripper_server PLUGIN "wsg50::GripperActionServer" EXECUTABLE gripper_server_node)

install(TARGETS gripper_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(action_client SHARED
  src/test_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "sensor_msgs"
  "control_msgs")
rclcpp_components_register_node(action_client PLUGIN "wsg50::GripperActionClient" EXECUTABLE gripper_client_node)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
