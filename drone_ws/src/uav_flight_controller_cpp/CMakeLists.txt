cmake_minimum_required(VERSION 3.8)
project(uav_flight_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)

include_directories(include)

# Shared Component Library
add_library(${PROJECT_NAME}_core SHARED
  src/flight_controller_node.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  uav_interfaces
)
rclcpp_components_register_node(${PROJECT_NAME}_core
  PLUGIN "uav_flight_controller::FlightControllerNode"
  EXECUTABLE flight_controller_node
)

# Standalone Executable
add_executable(flight_controller_node src/flight_controller_node.cpp)
target_include_directories(flight_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(flight_controller_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  uav_interfaces
)

install(TARGETS ${PROJECT_NAME}_core flight_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs uav_interfaces)

ament_package()
