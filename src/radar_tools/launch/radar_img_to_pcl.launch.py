from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    radar_info_file = os.path.join(
        get_package_share_directory('radar_tools'),
        'config',
        'navtech.yaml'
    )

    pcl_config_file = os.path.join(
        get_package_share_directory('radar_tools'),
        'config',
        'img_to_pcl.yaml'
    )

    return LaunchDescription([
        Node(
            package='radar_tools',
            executable='radar_img_to_pcl',
            name='radar_img_to_pcl',
            output='screen',
            parameters=[
                {'frame': 'navtech'},
                radar_info_file,
                pcl_config_file
            ],
            remappings=[
                ('cloud', '/radar/pcl_real'),
                ('radar_image', '/radar/image')
                
            ]
        )
    ])
