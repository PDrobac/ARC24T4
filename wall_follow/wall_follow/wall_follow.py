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

class WallFollow(Node):
    """
    This class implements the wall following mechanism
    """
    def __init__(self):
        super().__init__('wall_follow')
        self.speed = 0.
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.curr_angle = 0 # current steering angle
        self.des_angle = 0 # desired steering angle
        self.acc_error = 0 # accumulated steering error
        self.des_speed = 0 # desired drive speed
        
        timer_period = 0.5  # seconds - for a periodic callback
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, qos)
        # # Service for resetting the safety mechanism
        # self.create_service(Trigger, 'reset_safety_node', self.reset_safety_callback)

        # Declare parameter for dynamic reconfiguration
        self.declare_parameter('kp', 1)
        self.declare_parameter('ki', 0)
        self.declare_parameter('kd', 0)
        self.declare_parameter('timer_period', 0.5)
        self.get_logger().debug('Wall follow Inited')
        
    def pid(self, des_angle, dt):
        err = self.curr_angle - des_angle
        prop = self.kp * err
        self.acc_error += self.ki * err * dt
        integ = self.acc_error
        deriv = self.kd * err / dt
        return prop + integ + deriv

    def timer_callback(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = ((self.i % 3) - 1) * 0.2
        drive_msg.drive.speed = 0.5
        self.publisher_.publish(drive_msg)
        self.get_logger().info('Publishing AckermannDriveStamped')
        self.i += 1

    def odom_callback(self, odom_msg: Odometry):
        # get current steering angle
        pass
    
    def scan_callback(self, scan_msg: LaserScan):
        # calculate optimal steering angle AND speed
       pass
   
    def drive_callback_ackerman(self, drive_msg: AckermannDriveStamped):
        # implement autonomous driving
       pass

def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    wall_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()