#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped

class Pursuit(Node):
    #########################
    # all lines and sections marked with TODO might need adjustments, the rest should be good to go
    #########################
    def __init__(self):
        super().__init__('pursuit')
        # Parameters

        self.lookahead_distance = 0.7 # TODO maybe make reconfigurable via rqt_reconfigure
        self.velocity = 1.0 # TODO maybe make reconfigurable via rqt_reconfigure
        self.waypoint_index = 0 
        self.nlog = 1000 # TODO maybe find better timestep value for logging current position
        self.cnt = 0

        self.path = []

        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.goalpoint_marker_publisher = self.create_publisher(Marker, '/goalpoint', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/ground_truth_pose', 10)

        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, qos)
        self.create_subscription(Path, 'path', self.path_callback, qos)

    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to Euler angles
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        euler = r.as_euler('xyz', degrees=False)
        return euler[2]  # yaw is the rotation around z-axis

    def orthogonal_vector(self, yaw):
        # Calculate the vector orthogonal to the given yaw angle
        ortho_yaw = yaw + np.pi / 2
        return np.array([np.cos(ortho_yaw), np.sin(ortho_yaw), 0])

    def project_point_onto_vector(self, point, vector):
        return np.dot(point, vector) / np.linalg.norm(vector)

    def distance_orthogonal_to_orientation(self, point1, point2, orientation):
        # Convert quaternion to yaw (orientation around z-axis)
        yaw = self.quaternion_to_yaw(orientation)
        
        # Calculate orthogonal vector
        ortho_vector = self.orthogonal_vector(yaw)
        
        # Project points onto orthogonal vector
        proj_point1 = self.project_point_onto_vector(point1, ortho_vector)
        proj_point2 = self.project_point_onto_vector(point2, ortho_vector)
        
        # Calculate distance between projected points
        distance = proj_point1 - proj_point2
        
        return distance

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

    def publish_goalpoint(self, marker_x, marker_y):
        # TODO might have issues
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "my_sphere"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the pose of the marker
        marker.pose.position.x = marker_x
        marker.pose.position.y = marker_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker (1x1x1 here means 1m on each side)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Publish the marker
        self.goalpoint_marker_publisher.publish(marker)

    def path_callback(self, path_msg: Path):
        self.path = path_msg.poses

    def odom_callback(self, odom_msg: Odometry):
        # Helper variables
        position = odom_msg.pose.pose.position
        furthest_distance = 0.01
        
        # TODO define type, Pose placeholder
        furthest_waypoint = PoseStamped()
        visible_waypoints = []

        # Find furthest waypoint
        for waypoint in self.path[self.waypoint_index:]:
            distance = math.sqrt((position.x - waypoint.pose.position.x)**2 + (position.y - waypoint.pose.position.y)**2)
            if distance < self.lookahead_distance:
                visible_waypoints.append(waypoint)
                if distance > furthest_distance:
                    furthest_waypoint = waypoint
                    furthest_distance = distance
                    self.waypoint_index = self.path.index(waypoint)

        orientation = odom_msg.pose.pose.orientation
        point1 = np.array([furthest_waypoint.pose.position.x, furthest_waypoint.pose.position.y, furthest_waypoint.pose.position.z])

        if point1[0] != 0 or point1[1] != 0:
            point2 = np.array([position.x, position.y, position.z])
            distance = self.distance_orthogonal_to_orientation(point1, point2, orientation)
            steering_angle = 2 * distance / (furthest_distance**2)

            # Log
            self.cnt += 1
            if self.cnt >= self.nlog:
                self.odom_publisher.publish(odom_msg)
                self.cnt = 0

            # Publish goalpoint marker
            self.publish_goalpoint(furthest_waypoint.pose.position.x, furthest_waypoint.pose.position.y)

            # Publish drive command
            self.publish_drive(steering_angle, self.velocity)
        else:
            self.publish_drive(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    pursuit = Pursuit()
    rclpy.spin(pursuit)
    pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()