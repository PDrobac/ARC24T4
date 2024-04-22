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

class Reactive(Node):
    """
    This node implements the "Follow the gap" algo
    """
    def __init__(self):
        super().__init__('reactive')
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(Marker, '/viz_marker', 10)
        self.marker_pub0 = self.create_publisher(Marker, '/viz_marker0', 10)
        self.marker_pub1 = self.create_publisher(Marker, '/viz_marker1', 10)
        self.marker_pub2 = self.create_publisher(Marker, '/viz_marker2', 10)
        self.marker_pub3 = self.create_publisher(Marker, '/viz_marker3', 10)
        self.marker_pub4 = self.create_publisher(Marker, '/viz_marker4', 10)
        self.marker_scan = self.create_publisher(LaserScan, '/viz_scan', 10)
        
        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
    
    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)
    
    def scan_callback(self, scan_msg: LaserScan):
        pass
        speed = 0.5
        steering_angle = 0
        # implement follow the gap algo
        # obtain laser scans, process
        rng_proc = scan_msg.ranges
        # find closes point
        min_rng = min(rng_proc)
        idx = rng_proc.index(min_rng)
        rng_proc[idx] = 0
        
        # draw a "safety bubble" around it
        for i in range(1, 20, 1):
            left_idx = idx-i
            right_idx = idx+i
            rng_proc[left_idx] = 0
            rng_proc[right_idx] = 0
        
        # find the max length gap
        gap_len_max = 0
        gap_len_cur = 0
        gap_idx_start = -1
        for i in range(0, rng_proc.len(), 1):
            if rng_proc[i] == 0:
                continue
            i += 1
            while ((rng_proc[i] != 0) and (i < rng_proc.len())):
                gap_len_cur += 1
                i += 1
            if gap_len_cur > gap_len_max:
                gap_idx_start = i-gap_len_cur
                gap_len_max = gap_len_cur
        
        gap = rng_proc[gap_idx_start:gap_idx_start+gap_len_max]
            
            
                
            
        
        
        self.publish_drive(steering_angle, speed)
    
def main(args=None):
    rclpy.init(args=args)
    reactive = Reactive()
    rclpy.spin(reactive)
    reactive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()