#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Float32  # For publishing velocity, TTC, and distance for PlotJuggler
from visualization_msgs.msg import Marker  # For creating and updating the spherical marker

class WallFollow(Node):
    """
    This class implements the wall following mechanism
    """
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.
        
        self.get_logger().debug('Wall follow Inited')
        # Publishers
        # self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # # Subscribers
        # qos = QoSProfile(depth=10)
        # self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        # # Service for resetting the safety mechanism
        # self.create_service(Trigger, 'reset_safety_node', self.reset_safety_callback)

        # Declare parameter for dynamic reconfiguration
        # self.declare_parameter('ttc_cutoff', 1.8)

    def odom_callback(self, odom_msg: Odometry):
        pass
    
    def scan_callback(self, scan_msg: LaserScan):
       pass
   
    def drive_callback_ackerman(self, drive_msg: AckermannDriveStamped):
       pass

def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    wall_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()