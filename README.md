# safety_node2

Setup guide:

Clone this repository to your ros2 workspace's src subfolder (for me it was ~/sim_ws/src)

Run:
colcon build
source install/local_setup.bash

Now the program is ready, launch it using:
ros2 launch safety_node2 safety_node_launch.py

After rviz starts, go to new terminal and execute following line to make the car go forward:
ros2 topic pub /drive_in ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, drive: {steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 10.0, acceleration: 0.0, jerk: 0.0}}"


To install the slam toolbox:

# Navigate to your ROS 2 workspace
cd /path/to/your/ros2_workspace (for me it was ~/sim_ws/src/your_code, it should be the root folder of ARC24T4)

# Clone the slam_toolbox repository
git clone https://github.com/SteveMacenski/slam_toolbox.git src/slam_toolbox

# Install dependencies (if any) using ROS 2 package manager (rosdep)
cd src/slam_toolbox
rosdep install -i --from-paths . --rosdistro <your_ros_distro>

# Navigate back to your workspace root
cd ../.. + however many to return to sim_ws

# Build your workspace
colcon build
