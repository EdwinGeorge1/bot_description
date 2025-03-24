from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    display_launch_dir = os.path.join(get_package_share_directory('bot_description'), 'launch')
    custom_world_path = os.path.join(get_package_share_directory('bot_bringup'), 'world', 'bot.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
        launch_arguments={'world': custom_world_path}.items()
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(display_launch_dir, 'display.launch.py'))
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bot', 
            '-topic', '/robot_description',
            '-x', '-2.0',  
            '-y', '-0.5', 
            ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        display_launch,
        spawn_entity_node  # Spawn the robot into Gazebo
    ])