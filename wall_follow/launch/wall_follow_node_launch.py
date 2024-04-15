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

analyzer_params_filepath = '/arc2024/ws/src/your_code/ARC24T4/wall_follow/launch/diagnostics_aggregator_config.yaml'

def generate_launch_description():
    ld = LaunchDescription()
    
    import_f1tenth_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("/arc2024/ws/src/your_code/ARC24T4/safety_node2/launch/safety_node_launch.py")
        )
    ])
    
    wall_follow = Node(
        package="wall_follow",
        executable="wall_follow",
        name="wall_follow",
        parameters=[
            {'kp': 0.008},
            {'ki': 0.},
            {'kd': 0.01},
            {'theta': 40.},
            {'L': 1.5},
            {'v_max': 3.5},
            {'v_max_angle': 10.}
            ],
        remappings=[
            ("/drive", "/drive_in")
        ]
    )
    
    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[analyzer_params_filepath]
    )
    
    ld.add_action(import_f1tenth_launch)
    ld.add_action(wall_follow)
    return ld