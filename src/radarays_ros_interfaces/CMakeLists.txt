cmake_minimum_required(VERSION 3.10)
project(radarays_ros_interfaces)

# Required core packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add missing dependencies BEFORE rosidl_generate_interfaces
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Define message, service, and action files
set(msg_files
  "msg/RadarParams.msg"
  "msg/RadarMaterial.msg"
  "msg/RadarMaterials.msg"
  "msg/RadarModel.msg"
)

set(srv_files
  "srv/GetRadarParams.srv"
)

set(action_files
  "action/GenRadarImage.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()


