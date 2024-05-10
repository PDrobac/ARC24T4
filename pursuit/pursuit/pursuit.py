#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped

class Pursuit(Node):
    #########################
    # all lines and sections marked with TODO might need adjustments, the rest should be good to go
    #########################
    def __init__(self):
        # Parameters
        self.lookahead_distance = 1.0 # TODO maybe make reconfigurable via rqt_reconfigure
        self.velocity = 1.0 # TODO maybe make reconfigurable via rqt_reconfigure
        self.waypoint_index = 0 
        self.nlog = 1000 # TODO maybe find better timestep value for logging current position
        self.cnt = 0
        self.logger = self.get_logger()

        # TODO define type and fill with values
        self.path = [] # define type and fill with values

        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/nav', 10)
        self.goalpoint_marker_publisher = self.create_publisher(Marker, '/goalpoint', 10)

        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        # TODO will need a '/path' subscriber

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

    def publish_goalpoint(self, marker_x, marker_y):
        # TODO might have issues
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color
        marker.pose.position = Point(x=marker_x, y=marker_y, z=0.0)  # Position of the marker
        self.goalpoint_marker_publisher.publish(marker)

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

        # Log
        # TODO this task is unclear: do we need to log it in a logger or publish the values?
        self.cnt += 1
        if self.cnt >= self.nlog:
            self.logger.info("X: {:.2f} Y: {:.2f}".format(furthest_waypoint.x, furthest_waypoint.y))
            self.cnt = 0

        # Publish goalpoint marker
        self.publish_goalpoint(furthest_waypoint.x, furthest_waypoint.y)

        # Publish drive command
        self.publish_drive(steering_angle, self.velocity)

def main(args=None):
    rclpy.init(args=args)
    pursuit = Pursuit()
    rclpy.spin(pursuit)
    pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()