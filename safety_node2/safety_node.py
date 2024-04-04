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

class SafetyNode(Node):
    """
    The class that handles emergency braking with enhancements for data visualization and dynamic parameter adjustment.
    """
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.
        self.ttc_cutoff = 1.8  # Default TTC threshold, can be adjusted via rqt_reconfigure
        self.brake = False
        self.ttc = 10000.

        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # Additional publishers for data visualization
        self.velocity_publisher = self.create_publisher(Float32, '/velocity', 10)
        self.ttc_publisher = self.create_publisher(Float32, '/ttc', 10)
        self.distance_publisher = self.create_publisher(Float32, '/min_distance', 10)
        # Publisher for spherical marker
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)

        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.create_subscription(AckermannDriveStamped, 'drive_in', self.drive_callback_ackerman, qos)
        self.create_subscription(Twist, 'cmd_vel', self.drive_callback_twist, qos)

        # Service for resetting the safety mechanism
        self.create_service(Trigger, 'reset_safety_node', self.reset_safety_callback)

        # Declare parameter for dynamic reconfiguration
        self.declare_parameter('ttc_cutoff', 1.8)

        # Timer to periodically update the marker
        self.create_timer(0.1, self.update_marker) # not sure if 0.1 is the right value

    def odom_callback(self, odom_msg: Odometry):
        # Update current speed and publish for visualization
        self.speed = odom_msg.twist.twist.linear.x
        self.velocity_publisher.publish(Float32(data=self.speed))

    def scan_callback(self, scan_msg: LaserScan):
        # Calculate TTC and determine whether to brake, also publish distance and TTC for visualization
        min_distance = min(scan_msg.ranges)
        self.distance_publisher.publish(Float32(data=min_distance))

        for i, distance in enumerate(scan_msg.ranges):
            if distance == 0:  # Ignore invalid readings
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            relative_speed = self.speed * math.cos(angle)
            ttc = distance / max(abs(relative_speed), 1e-5)  # Avoid /0
            if ttc < self.ttc_cutoff:
                self.brake = True
            if ttc < self.ttc:
                self.ttc = ttc

        self.ttc_publisher.publish(Float32(data=self.ttc))

    def drive_callback_ackerman(self, drive_msg: AckermannDriveStamped):
        # Apply brake if necessary and publish drive command
        if self.brake:
            drive_msg.drive.speed = 0.
        self.drive_publisher.publish(drive_msg)

    def drive_callback_twist(self, drive_msg: Twist):
        if self.brake:
            drive_msg.linear.x = 0.
        response = AckermannDriveStamped()
        response.drive.speed = drive_msg.linear.x
        self.drive_publisher.publish(response)

    def reset_safety_callback(self, request, response):
        # Reset safety mechanism based on TTC
        self.ttc_cutoff = self.get_parameter('ttc_cutoff').get_parameter_value().double_value
        if self.ttc < self.ttc_cutoff:
            response.success = False
            response.message = "Safety reset unsuccessful, TTC still too low"
        else:
            self.brake = False
            response.success = True
            response.message = "Safety reset successful"
        return response

    def update_marker(self):
        # Update and publish spherical marker based on brake status, has to be tested, might not work as expected
        marker = Marker()
        marker.header.frame_id = "base_link" 
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Marker position should be in front of the car
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        # Marker color should be red if braking, green if not
        marker.color.a = 1.0
        marker.color.r = 1.0 if self.brake else 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0 if self.brake else 1.0
        marker.pose.orientation.w = 1.0
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()