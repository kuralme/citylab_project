cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_patrol_srv REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES geometry_msgs
)

# Define executables
add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp std_msgs geometry_msgs sensor_msgs robot_patrol_srv)
add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp std_msgs geometry_msgs sensor_msgs robot_patrol_srv)
add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp std_msgs geometry_msgs sensor_msgs robot_patrol_srv)

add_executable(robot_patrol_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(robot_patrol_action_node rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs sensor_msgs tf2)
rosidl_target_interfaces(robot_patrol_action_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  robot_patrol_node
  direction_service_node
  test_service_node
  patrol_with_service_node
  robot_patrol_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  action
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
