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
    """
    This node implements the "Follow the gap" algo
    """
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

    # Dynamic Speed Adjustment: Incorporates a method (calculate_speed) that adjusts 
    # speed more intricately based on the size of the gap and its distance, allowing 
    # for faster speeds in larger, more distant gaps and controlled speeds in closer, 
    # smaller gaps
    def calculate_speed(self, gap_size, distance):
        if distance > 10:
            return min(5, 0.5 * gap_size)  # Increase speed in larger gaps
        else:
            return max(1, 0.3 * gap_size)  # Controlled speed in smaller gaps
    
    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

    def publish_marker(self, steering_angle=0.0, speed=1.0):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(steering_angle / 2.0)
        quaternion.w = math.cos(steering_angle / 2.0)

        marker = Marker()
        marker.header.frame_id = "ego_racecar/base_link"
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale = Vector3(x=speed, y=0.05, z=0.05)  # Width, height, depth
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color
        marker.pose.position = Point(x=0.0, y=0.0, z=0.2)  # Position of the marker
        marker.pose.orientation = quaternion  # No rotation
        self.marker_pub.publish(marker)
    
    def publish_scan(self, scan_msg: LaserScan, masked_out_ranges: list):
        new_scan_msg = deepcopy(scan_msg)
        new_scan_msg.ranges = masked_out_ranges
        for i, distance in enumerate(scan_msg.ranges):
            if distance != masked_out_ranges[i]:
                new_scan_msg.intensities.append(0.0)
            else:
                new_scan_msg.intensities.append(1.0)

        self.scan_pub.publish(new_scan_msg)
    
    def scan_callback(self, scan_msg: LaserScan):
        ranges = deepcopy(scan_msg.ranges)
        gap_start = None
        gap_end = None
        max_gap_size = 0
        selected_gap_distance = 0

        # Parameters
        min_distance_for_gap = 1.0  # Minimum distance to consider a gap (in meters)
        
        # Iterate over the ranges to find gaps
        for i in range(len(ranges)):
            distance = ranges[i]
            if distance >= min_distance_for_gap:
                if gap_start is None:
                    gap_start = i
            else:
                if gap_start is not None:
                    gap_end = i - 1
                    # Calculate gap size
                    gap_size = (gap_end - gap_start + 1) * scan_msg.angle_increment * (sum(ranges[gap_start:gap_end+1]) / (gap_end - gap_start + 1))
                    if gap_size > max_gap_size:
                        max_gap_size = gap_size
                        selected_gap_distance = sum(ranges[gap_start:gap_end+1]) / (gap_end - gap_start + 1)
                    gap_start = None
                    gap_end = None

        # Final check in case the last gap reaches the end of the range array
        if gap_start is not None:
            gap_end = len(ranges) - 1
            gap_size = (gap_end - gap_start + 1) * scan_msg.angle_increment * (sum(ranges[gap_start:gap_end+1]) / (gap_end - gap_start + 1))
            if gap_size > max_gap_size:
                max_gap_size = gap_size
                selected_gap_distance = sum(ranges[gap_start:gap_end+1]) / (gap_end - gap_start + 1)

        # Determine driving parameters based on the largest gap found
        gap_size = max_gap_size
        distance = selected_gap_distance
        speed = self.calculate_speed(gap_size, distance)

        # Logic to determine steering angle based on the center of the selected gap
        if gap_start is not None and gap_end is not None:
            center_idx = (gap_start + gap_end) // 2
            steering_angle = (center_idx - len(ranges) / 2) * scan_msg.angle_increment
        else:
            steering_angle = 0.0

        # Logging and publishing
        msg_print = f"Chosen speed: {speed} m/s, angle: {steering_angle} radians, gap size: {gap_size}, distance: {distance}"
        self.get_logger().info(msg_print)
        
        self.publish_drive(steering_angle, speed)
        self.publish_marker(steering_angle, speed)
        self.publish_scan(scan_msg, ranges)  # This probably needs modification to highlight the selected gap

    
def main(args=None):
    rclpy.init(args=args)
    reactiveSpeed = ReactiveSpeed()
    rclpy.spin(reactiveSpeed)
    reactiveSpeed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()