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
    
    import_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ws_path + "/src/your_code/ARC24T4/safety_node2/launch/safety_node_launch.py")
        )
    ])
    
    reactive = Node(
        package="reactive",
        executable="reactive_front",
        name="reactive",
        parameters=[{'max_velocity': 5.0}]
    )
    
    reactive2 = Node(
        package="reactive",
        executable="reactive_front",
        name="reactive",
        parameters=[{'max_velocity': 4.0}],
        remappings=[
            ("/drive", "/opp_drive"),
            ("/scan", "/opp_scan"),
            ("/viz_scan", "/opp_viz_scan"),
            ("/viz_marker", "/opp_viz_marker")
        ]
    )

    ld.add_action(import_launch)
    ld.add_action(reactive)
    ld.add_action(reactive2)
    
    return ld