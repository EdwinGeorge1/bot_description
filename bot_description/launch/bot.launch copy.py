from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('bot_description')

    # Define the default world file path
    # Instead of using get_package_share_directory, directly set the path
    world_file_path = '/home/edwin/ros2_ws/src/bot_description/world/bot.world'


    # Declare the world file as a launch argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Path to the Gazebo world file'
    )

    # Print world file path for debugging
    print(f"Loading world file: {world_file_path}")

    # Load robot description from Xacro
    xacro_file = os.path.join(share_dir, 'urdf', 'bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Ensure Gazebo loads the correct world file
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,  # Add world file argument
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])


