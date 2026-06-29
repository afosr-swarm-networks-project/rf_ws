cmake_minimum_required(VERSION 3.8)
project(rf_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IqFrame.msg"
  "msg/StftFrame.msg"
  "msg/RfDetection.msg"
  "msg/RfDetectionArray.msg"
  "msg/RfScore.msg"
  "msg/RfScoreArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
