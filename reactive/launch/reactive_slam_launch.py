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
    
    ld.add_action(slam)
    ld.add_action(reactive)
    
    return ld