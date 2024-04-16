# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

workspace_prefix = os.environ['WS_PREFIX']

def generate_launch_description():
    ld = LaunchDescription()
    
    import_f1tenth_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(workspace_prefix + "/src/f1tenth_system/f1tenth_stack/launch/bringup_launch.py")
        )
    ])
    
    safety_node = Node(
        package="safety_node2",
        executable="safety_node",
        name="safety_node",
        parameters=[{'ttc_cutoff': 1.5}],
        remappings=[
            ("/drive_out", "/drive"),
            ("/odom", "/ego_racecar/odom"),
            ("/base_link", "/ego_racecar/base_link")
            #("/drive_in", "/drive"),
            #("/teleop_twist_keyboard_out", "/teleop_twist_keyboard"),
            #("/teleop_twist_keyboard_in", "/teleop_twist_keyboard")
        ]
    )

    # finalize
    ld.add_action(safety_node)

    return ld
