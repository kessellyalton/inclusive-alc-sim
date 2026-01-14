cmake_minimum_required(VERSION 3.8)
project(alc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Placeholder.msg"
  "msg/LearnerState.msg"
  "msg/TutorAction.msg"
  "srv/Placeholder.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

