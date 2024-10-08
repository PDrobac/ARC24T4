import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch_ros.actions import SetRemap

from launch.launch_description_sources import PythonLaunchDescriptionSource

ws_path = os.environ["WS_PATH"]

def generate_launch_description():
    print(">>> Using Workspace path:", ws_path)
    print("To export the current directory as WS_PATH, enter: > export WS_PATH=$(pwd) <")
    
    ld = LaunchDescription()

    slam = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ws_path + "/src/your_code/src/slam_toolbox/launch/online_sync_launch.py")
        )
    ])
    
    reactive = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ws_path + "/src/your_code/ARC24T4/reactive/launch/reactive_node_launch.py")
        )
    ])

    # SLAM Toolbox node configuration
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # Or async_slam_toolbox_node based on your requirements
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # Set to True if you are using simulation time e.g., with Gazebo
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }]
    )
    
    ld.add_action(slam)
    ld.add_action(slam_toolbox)
    ld.add_action(reactive)
    
    return ld