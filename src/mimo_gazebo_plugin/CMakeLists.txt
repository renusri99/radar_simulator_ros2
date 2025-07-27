cmake_minimum_required(VERSION 3.10)
project(radarays_gazebo_plugins VERSION 2.0.0)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# DEFAULT RELEASE
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

find_package(rmagine 2.2.1 REQUIRED COMPONENTS core)
find_package(rmagine 2.2.1 OPTIONAL_COMPONENTS embree cuda optix mimo)

# Set dependencies
set(RM_GZ_DEPS
  rclcpp
  sensor_msgs
  radarays_ros
  cv_bridge
  image_transport
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${ignition-gazebo6_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

### BUILD EMBREE SENSOR PLUGIN ###
if(TARGET rmagine::embree)
  message(WARNING "BUILDING RadaRays Embree Ignition Plugin")

  add_library(radarays_embree_gzplugin SHARED
    src/radarays_embree_gzplugin.cpp
  )

  ament_target_dependencies(radarays_embree_gzplugin
    ${RM_GZ_DEPS}
    ignition-gazebo6
    rmagine::embree
  )

  add_library(radarays_embree_ros_gzplugin SHARED 
    src/radarays_embree_ros_gzplugin.cpp
  )

  ament_target_dependencies(radarays_embree_ros_gzplugin
    ${RM_GZ_DEPS}
    ignition-gazebo6
    rmagine::embree
  )
endif()

### BUILD OPTIX SENSOR PLUGIN ###
if(TARGET rmagine::optix)
  message(WARNING "BUILDING RadaRays Optix Ignition Plugin")

  add_library(radarays_optix_gzplugin SHARED
    src/radarays_optix_gzplugin.cpp
  )

  ament_target_dependencies(radarays_optix_gzplugin
    ${RM_GZ_DEPS}
    ignition-gazebo6
    rmagine::optix
  )

  add_library(radarays_optix_ros_gzplugin SHARED 
    src/radarays_optix_ros_gzplugin.cpp
  )

  ament_target_dependencies(radarays_optix_ros_gzplugin
    ${RM_GZ_DEPS}
    ignition-gazebo6
    rmagine::optix
  )
endif()

### BUILD MIMO RADAR SUPPORT ###
if(TARGET rmagine::mimo)
  message(WARNING "BUILDING RadaRays MIMO Radar Support")

  add_library(radarays_mimo_gzplugin SHARED
    src/radarays_mimo_gzplugin.cpp
  )

  ament_target_dependencies(radarays_mimo_gzplugin
    ${RM_GZ_DEPS}
    ignition-gazebo6
    rmagine::mimo
  )
endif()

ament_export_dependencies(${RM_GZ_DEPS})
ament_package()
