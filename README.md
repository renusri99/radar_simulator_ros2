\# Radar Simulation Framework (ROS 2 + Ignition Gazebo)



This project simulates radar sensors for Automated Guided Vehicles (AGVs) using ROS 2 Humble and Ignition Gazebo Fortress. It supports both CPU and GPU-based FMCW radar and MIMO radar with point cloud output.



\# Features



\- CPU and GPU-based radar simulation (Embree / OptiX)

\- MIMO radar support with angle-of-arrival processing

\- Radar image to point cloud conversion

\- RViz2 visualization support

\- Multi-bounce radar wave simulation



\#Requirements



\- ROS 2 Humble

\- Ignition Gazebo Fortress

\- OpenCV

\- RViz2

\- rmagine (with embree and/or optix)

\- CMake + colcon





\# How to Build



```bash

cd ~/ros\_ws

colcon build --packages-select "package\_name"

source install/setup.bash



\#  Launch ignition gazebo with radar sensor



```bash

ros2 launch radarays\_gazebo\_plugins example\_robot.launch.py





\# Parameters can be passed using the launch file or during run time

&nbsp; ```bash

ros2 run radarays\_ros radar\_simulator \\

&nbsp; --ros-args \\

&nbsp; --params-file ~/ros\_ws/src/radarays\_ros/config/oru4\_test\_ros2.yaml \\

&nbsp; --params-file ~/ros\_ws/src/radarays\_ros/config/RadarModel.yaml





\# Radar Simulation Environment Variables

export IGN\_GAZEBO\_RESOURCE\_PATH=$IGN\_GAZEBO\_RESOURCE\_PATH:~/ros\_ws/src/radarays\_gazebo\_plugins

export IGN\_PLUGIN\_PATH=$IGN\_PLUGIN\_PATH:~/ros\_ws/install/radarays\_gazebo\_plugins/lib

export LD\_LIBRARY\_PATH=$LD\_LIBRARY\_PATH:~/ros\_ws/install/radarays\_gazebo\_plugins/lib:~/ros\_ws/install/rmagine/lib



