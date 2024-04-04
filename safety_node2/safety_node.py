#!/usr/bin/env python3
import math
import string
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_srvs.srv import Trigger


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive_out topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle,
        and to /drive_in to get the drive commands.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.ttc_cutoff = 1.8
        # TODO: create ROS subscribers and publishers.

        self.brake = False
        self.ttc = 10000.
        self.drive_publisher_ackerman = self.create_publisher(AckermannDriveStamped, '/drive_out', 10)
        #self.drive_publisher_twist = self.create_publisher(Twist, '/teleop_twist_keyboard_out', 10)
        
        self.create_subscription(
            LaserScan,                      # Message type
            'scan',                	    # Topic name
            self.scan_callback,           # Callback function
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        self.create_subscription(
            Odometry,                       # Message type
            'ego_racecar/odom',            # Topic name
            self.odom_callback,           # Callback function
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        self.create_subscription(
            AckermannDriveStamped,          # Message type
            'drive_in',                    # Topic name
            self.drive_callback_ackerman,          # Callback function
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )
    
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.drive_callback_twist,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        self.create_service(Trigger, 'reset_safety_node', self.reset_safety_callback)

    def odom_callback(self, odom_msg: Odometry):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg: LaserScan):
        # TODO: calculate TTC
        self.ttc = 10000.
        
        # TODO: publish command to brake
        for i, range_measurement in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            relative_speed = self.speed * math.cos(angle)**2

            ttc = 10000.
            if relative_speed > 0:
                ttc = range_measurement / abs(relative_speed)

            if not self.brake and ttc < self.ttc_cutoff:
                self.brake = True

            if ttc < self.ttc:
                self.ttc = ttc

    def drive_callback_ackerman(self, drive_msg: AckermannDriveStamped):
        if self.brake:
            drive_msg.drive.speed = 0.
        self.drive_publisher_ackerman.publish(drive_msg)

    def drive_callback_twist(self, drive_msg: Twist):
        if self.brake:
            drive_msg.linear.x = 0.
        response = AckermannDriveStamped()
        response.drive.speed = drive_msg.linear.x
        self.drive_publisher_ackerman.publish(response)

    def reset_safety_callback(self, request: Trigger.Request, response: Trigger.Response):
        if self.ttc < self.ttc_cutoff:
            response.success = False
            response.message = "Safety reset unsuccessful, TTC still too low"
        else:
            self.brake = False
            response.success = True
            response.message = "Safety reset successful"
        return response

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
