cmake_minimum_required(VERSION 3.10)
project(radarays_ros VERSION 1.0.1)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rmagine REQUIRED COMPONENTS core OPTIONAL_COMPONENTS embree cuda optix)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarMaterial.msg"
  "msg/RadarMaterials.msg"
  "msg/RadarModel.msg"
  "msg/RadarParams.msg"
  "srv/GetRadarParams.srv"
  "action/GenRadarImage.action"
  DEPENDENCIES builtin_interfaces sensor_msgs action_msgs
)

# Include directories
include_directories(include)

# Definitions
set(RADARAYS_ROS_DEFINITIONS "")
set(RADARAYS_ROS_LIBRARIES radarays)

if(TARGET rmagine::optix)
  list(APPEND RADARAYS_ROS_LIBRARIES radarays_gpu)
  add_definitions(-DRADARAYS_WITH_GPU)
endif()

if(TARGET rmagine::embree)
  list(APPEND RADARAYS_ROS_LIBRARIES radarays_cpu)
  add_definitions(-DRADARAYS_WITH_CPU)
endif()

# Core library
add_library(radarays
  src/radarays_ros/radar_algorithms.cpp
  src/radarays_ros/ros_helper.cpp
  src/radarays_ros/Radar.cpp
)
target_include_directories(radarays PUBLIC include)
ament_target_dependencies(radarays
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  OpenCV
  rmagine::core
)

# CPU plugin
if(TARGET rmagine::embree)
  add_library(radarays_cpu
    src/radarays_ros/RadarCPU.cpp
  )
  target_link_libraries(radarays_cpu radarays rmagine::embree)
endif()

# GPU plugin
if(TARGET rmagine::optix)
  find_package(CUDA REQUIRED)
  enable_language(CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
  
  add_library(radarays_gpu SHARED
    src/radarays_ros/image_algorithms.cu
    src/radarays_ros/radar_algorithms.cu
    src/radarays_ros/RadarGPU.cpp
  )
  target_link_libraries(radarays_gpu
    ${CUDA_LIBRARIES}
    radarays
    rmagine::optix
  )
endif()

# Radar simulator executable
add_executable(radar_simulator src/radar_simulator.cpp)
target_link_libraries(radar_simulator
  radarays
  ${OpenCV_LIBS}
  rmagine::core
)

if(TARGET radarays_cpu)
  target_compile_definitions(radar_simulator PUBLIC RADARAYS_WITH_CPU)
  target_link_libraries(radar_simulator radarays_cpu)
endif()

if(TARGET radarays_gpu)
  target_compile_definitions(radar_simulator PUBLIC RADARAYS_WITH_GPU)
  target_link_libraries(radar_simulator radarays_gpu)
endif()

# Example test
if(TARGET radarays_cpu)
  add_executable(ray_reflection_test src/ray_reflection_test.cpp)
  target_link_libraries(ray_reflection_test
    radarays
    rmagine::core
    rmagine::embree
    ${OpenCV_LIBS}
  )
endif()

# Install
install(TARGETS
  radarays
  radar_simulator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET radarays_cpu)
  install(TARGETS radarays_cpu
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

if(TARGET radarays_gpu)
  install(TARGETS radarays_gpu
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY msg srv action
  DESTINATION share/${PROJECT_NAME}/
)

# Export
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  OpenCV
  rmagine
)
ament_export_include_directories(include)
ament_export_libraries(${RADARAYS_ROS_LIBRARIES})

ament_package()
