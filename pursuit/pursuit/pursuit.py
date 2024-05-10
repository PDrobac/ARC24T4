#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose
from ackermann_msgs.msg import AckermannDriveStamped

class Pursuit(Node):
    """
    The class that handles emergency braking with enhancements for data visualization and dynamic parameter adjustment.
    """
    def __init__(self):
        # Parameters
        self.lookahead_distance = 1.0
        self.waypoint_index = 0

        # TODO define type and fill with values
        self.path = [] # array of points?

        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

    def odom_callback(self, odom_msg: Odometry):
        # Helper variables
        position = odom_msg.pose.pose.position
        furthest_distance = 0
        
        # TODO define type, Point placeholder
        furthest_waypoint = Point()
        visible_waypoints = []

        # Find furthest waypoint
        for waypoint in self.path[self.waypoint_index:]:
            distance = math.sqrt((position.x - waypoint.x)**2 + (position.y - waypoint.y)**2)
            if distance < self.lookahead_distance:
                visible_waypoints.append(waypoint)
                if distance > furthest_distance:
                    furthest_waypoint = waypoint
                    self.waypoint_index = self.path.index(waypoint)

        # Calculate angle to furthest waypoint
        angle_desired = math.atan2(furthest_waypoint.y - position.y, furthest_waypoint.x - position.x)
        angle_current = odom_msg.pose.pose.orientation.z
        steering_angle = angle_desired - angle_current

        # Publish drive command
        self.publish_drive(steering_angle, 1.0)

def main(args=None):
    rclpy.init(args=args)
    pursuit = Pursuit()
    rclpy.spin(pursuit)
    pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()