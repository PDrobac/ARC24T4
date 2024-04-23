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

def generate_launch_description():
    ld = LaunchDescription()
    
    import_f1tenth_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("/home/pero/sim_ws/src/your_code/ARC24T4/safety_node2/launch/safety_node_launch.py")
        )
    ])
    
    reactive = Node(
        package="reactive",
        executable="reactive",
        name="reactive",
        parameters=[],
        remappings=[
            ("/drive", "/drive_in")
        ]
    )
    
    ld.add_action(import_f1tenth_launch)
    ld.add_action(reactive)
    
    return ld