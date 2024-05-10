#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger
from std_msgs.msg import Float32  # For publishing velocity, TTC, and distance for PlotJuggler
from visualization_msgs.msg import Marker  # For creating and updating the spherical marker

class Pursuit(Node):
    """
    The class that handles emergency braking with enhancements for data visualization and dynamic parameter adjustment.
    """
    def __init__(self):

        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)

    def odom_callback(self, odom_msg: Odometry):
        pass

    def scan_callback(self, scan_msg: LaserScan):
        pass

def main(args=None):
    rclpy.init(args=args)
    pursuit = Pursuit()
    rclpy.spin(pursuit)
    pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()