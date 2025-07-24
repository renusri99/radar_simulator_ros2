from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radar_tools',
            executable='compare_radar_images',
            name='compare_radar_images',
            output='screen',
            parameters=[{
                'topic_in_1': '/Navtech/Polar',
                'topic_in_2': '/radar/image',
                'topic_out': '/real_to_sim_gap',
                'time_sync': 'virtual'
            }]
        ),
        Node(
            package='radar_tools',
            executable='compare_radar_images_writer',
            name='compare_radar_images_writer',
            output='screen',
            parameters=[{
                'output_csv': 'image_compare.csv'  # optional, defaults to this anyway
            }]
        )
    ])
