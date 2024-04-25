#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker  # For creating and updating the spherical marker
from copy import deepcopy

class ReactiveSpeed(Node):
    
    def __init__(self):
        super().__init__('reactive')
        self.declare_parameter('max_velocity', 3.)
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(Marker, '/viz_marker', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/viz_scan', 10)
        
        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)

    def calculate_speed(self, gap_size, distance):
        """Calculate dynamic speed based on gap size and distance."""
        if distance > 10:
            return min(5, 0.5 * gap_size)  # Increase speed in larger gaps
        else:
            return max(1, 0.3 * gap_size)  # Controlled speed in smaller gaps

    def publish_drive(self, steering_angle, speed):
        """Publish the drive command."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

    def publish_marker(self, steering_angle, speed):
        """Publish visualization markers."""
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(steering_angle / 2.0)
        quaternion.w = math.cos(steering_angle / 2.0)

        marker = Marker()
        marker.header.frame_id = "ego_racecar/base_link"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale = Vector3(x=speed, y=0.05, z=0.05)  # Adjust marker scale based on speed
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color
        marker.pose.position = Point(x=0.5, y=0.0, z=0.2)  # Position relative to the car
        marker.pose.orientation = quaternion
        self.marker_pub.publish(marker)

    def scan_callback(self, scan_msg):
        """Process each incoming LIDAR scan message."""
        ranges = deepcopy(scan_msg.ranges)
        gap_size, gap_distance = self.find_gaps(ranges, scan_msg.angle_increment)
        
        speed = self.calculate_speed(gap_size, gap_distance)
        steering_angle = self.calculate_steering_angle(ranges, gap_distance)
        
        # Logging and publishing
        msg_print = f"Chosen speed: {speed} m/s, angle: {steering_angle} radians, gap size: {gap_size}, distance: {gap_distance}"
        self.get_logger().info(msg_print)
        
        self.publish_drive(steering_angle, speed)
        self.publish_marker(steering_angle, speed)
        self.publish_scan(scan_msg, ranges)  # Additional functionality to highlight the selected gap

    def find_gaps(self, ranges, angle_increment):
        """Identify gaps from LIDAR data and calculate the optimal gap parameters."""
        min_gap_distance = 1.0  # Minimum distance to consider a valid gap
        max_gap_size = 0
        gap_distance = 0
        gap_start = None

        for i, distance in enumerate(ranges):
            if distance >= min_gap_distance:
                if gap_start is None:
                    gap_start = i
            else:
                if gap_start is not None:
                    gap_end = i - 1
                    current_gap_size = (gap_end - gap_start + 1) * angle_increment * sum(ranges[gap_start:gap_end + 1]) / (gap_end - gap_start + 1)
                    if current_gap_size > max_gap_size:
                        max_gap_size = current_gap_size
                        gap_distance = sum(ranges[gap_start:gap_end + 1]) / (gap_end - gap_start + 1)
                    gap_start = None

        # Check for a gap at the end of the range list
        if gap_start is not None:
            gap_end = len(ranges) - 1
            current_gap_size = (gap_end - gap_start + 1) * angle_increment * sum(ranges[gap_start:gap_end + 1]) / (gap_end - gap_start + 1)
            if current_gap_size > max_gap_size:
                max_gap_size = current_gap_size
                gap_distance = sum(ranges[gap_start:gap_end + 1]) / (gap_end - gap_start + 1)

        return max_gap_size, gap_distance

    # More precisely calculates the steering angle by pinpointing the exact middle of the largest gap
    def calculate_steering_angle(self, ranges, target_distance):
        """Calculate the steering angle based on the target distance and gap."""
        target_index = ranges.index(target_distance)
        steering_angle = (target_index - len(ranges) / 2) * (math.pi / len(ranges))
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    reactiveSpeed = ReactiveSpeed()
    rclpy.spin(reactiveSpeed)
    reactiveSpeed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
