from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('radar_tools'),
        'config',
        'image_slicer.yaml'
    )

    with open(config_file, 'r') as f:
        all_params = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package='radar_tools',
            executable='image_slicer',
            name='image_slicer_navtech',
            output='screen',
            parameters=[all_params['image_slicer_navtech']],
            remappings=[
                ('image_in', '/Navtech/Polar'),
                ('image_out', '/Navtech/polar_zoomed'),
            ]
        ),
        Node(
            package='radar_tools',
            executable='image_slicer',
            name='image_slicer_sim',
            output='screen',
            parameters=[all_params['image_slicer_sim']],
            remappings=[
                ('image_in', '/radar/image'),
                ('image_out', '/radar/image_zoomed'),
            ]
        )
    ])
