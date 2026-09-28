cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/NavigationGoal.msg"
  "msg/SystemHealth.msg"
  "msg/SafetyAlert.msg"
  "msg/WaypointList.msg"
  "msg/SemanticDetection.msg"
  "srv/QuerySemanticMemory.srv"
  "srv/TriggerFailsafe.srv"
  "srv/SetFlightMode.srv"
  "action/ExecuteSemanticNavigation.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
