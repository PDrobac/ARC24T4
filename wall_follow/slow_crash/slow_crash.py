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

class SlowCrash(Node):
    """
    This class implements the wall following mechanism
    """
    def __init__(self):
        super().__init__('slow_crash')
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        # # Service for resetting the safety mechanism
        # self.create_service(Trigger, 'reset_safety_node', self.reset_safety_callback)

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)
    
    def scan_callback(self, scan_msg: LaserScan):
        self.publish_drive(float(0), float(0.5))

def main(args=None):
    rclpy.init(args=args)
    slow_crash = SlowCrash()
    rclpy.spin(slow_crash)
    slow_crash.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
