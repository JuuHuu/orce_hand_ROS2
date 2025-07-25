cmake_minimum_required(VERSION 3.5)
project(orca_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TactileFrame.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
