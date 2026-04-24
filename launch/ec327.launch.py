from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('nav_pkg'),
        'rviz',
        'ec327.rviz'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_house.launch.py'
                )
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                )
            ),
            launch_arguments={'use_sim_time': 'True'}.items()
        ),

        Node(
            package='nav_pkg',
            executable='nav_node',
            name='nav',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='nav_pkg',
            executable='ctrl_node',
            name='controller',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])