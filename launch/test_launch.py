import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='safety_node2',
            executable='safety_node.py',
            name='safety_node'),
  ])