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

workspace_prefix = os.environ['WS_PREFIX']

def generate_launch_description():
    ld = LaunchDescription()
    
    import_f1tenth_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(workspace_prefix + "/src/your_code/ARC24T4/safety_node2/launch/safety_node_launch.py")
        )
    ])
    
    #wall_follow = Node(
    #    package="wall_follow",
    #    executable="wall_follow",
    #    name="wall_follow",
    #    parameters=[
    #        {'kp': 0.006},
    #        {'ki': 0.},
    #        {'kd': 0.03},
    #        {'theta': 54.},
    #        {'L': 3.},
    #        {'v_max': 1.5},
    #        {'v_max_angle': 10.}
    #        ],
    #    remappings=[
    #        ("/drive", "/drive_in"),
    #        ("/ego_racecar/odom", "/odom"),
    #        ("/cmd_vel", "/teleop")
    #    ]
    #)
    
    ld.add_action(import_f1tenth_launch)
    ld.add_action(wall_follow)
    return ld
