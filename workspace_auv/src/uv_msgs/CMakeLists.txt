cmake_minimum_required(VERSION 3.8)
project(uv_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommand.msg"
  "msg/AuvState.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/ObjectPosition.msg"
  "msg/ObjectPositionArray.msg"
  "msg/PidGains.msg"
  "msg/PidState.msg"
  "msg/DeviceCommand.msg"
  "msg/Waypoint.msg"
  "msg/WaypointPath.msg"
  "msg/TaskStatus.msg"
  "msg/PoseInfo.msg"
  "srv/RunTask.srv"
  "srv/ExecTask.srv"
  "srv/GetState.srv"
  "action/BasicMotion.action"
  DEPENDENCIES std_msgs sensor_msgs action_msgs builtin_interfaces
)

ament_package()
