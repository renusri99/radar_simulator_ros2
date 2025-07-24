from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('radar_tools'),
        'config',
        'plot_img_column.yaml'
    )

    return LaunchDescription([
        Node(
            package='radar_tools',
            executable='plot_img_column',
            name='plot_img_column_real',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('topic', '/Navtech/Polar')
            ]
        ),
        Node(
            package='radar_tools',
            executable='plot_img_column',
            name='plot_img_column_sim',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('topic', '/radar/image')
            ]
        )
    ])
