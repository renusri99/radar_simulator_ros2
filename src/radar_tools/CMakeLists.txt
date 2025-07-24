cmake_minimum_required(VERSION 3.5)
project(radar_tools)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# For message generation
find_package(rosidl_default_generators REQUIRED)

# --- Declare messages ---
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/RadarInfo.msg
  msg/ImageCompMetrics.msg
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs
)

# Make generated headers available
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Include dirs (optional if you don't have your own headers)
include_directories(include)

# --- Executables ---
add_executable(radar_img_to_pcl src/radar_img_to_pcl.cpp)
ament_target_dependencies(radar_img_to_pcl
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(radar_img_to_pcl ${cpp_typesupport_target}) # <-- this links the generated headers

add_executable(image_slicer src/image_slicer.cpp)
ament_target_dependencies(image_slicer
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(image_slicer ${cpp_typesupport_target})

# --- Install targets ---
install(TARGETS
  radar_img_to_pcl
  image_slicer
  DESTINATION lib/${PROJECT_NAME}
)

# Install msg headers if needed
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
