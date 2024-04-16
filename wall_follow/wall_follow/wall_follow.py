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
        self.kp = 1
        self.ki = 0
        self.kd = 0

        self.theta = 60. # angle between scans a and b
        self.L = 3. # lookahead distance in front of car
        self.D = 1. # desired distance from wall
        self.v_max = 3.
        self.v_max_angle = 1.

        self.prev_error = 0. # error in previous D step
        self.integral = 0. # accumulated I error
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        # # Service for resetting the safety mechanism
        # self.create_service(Trigger, 'reset_safety_node', self.reset_safety_callback)

        # Declare parameter for dynamic reconfiguration
        self.i = True
        self.declare_parameter('kp', 0.006)
        self.declare_parameter('ki', 0.)
        self.declare_parameter('kd', 0.03)
        self.declare_parameter('theta', 54.)
        self.declare_parameter('L', 3.)
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('v_max_angle', 1.)
        #self.declare_parameter('D', 1.)
        self.get_logger().debug('Wall follow Inited')

    def pid(self, error):
        # pre-PID calculations
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        # re-check parameters
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        #print("Kp: " + str(self.kp))

        # result is steering angle as determined by PID
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)
    
    def scan_callback(self, scan_msg: LaserScan):
        # calculate optimal steering angle AND speed
        self.L = self.get_parameter('L').get_parameter_value().double_value
        self.theta = self.get_parameter('theta').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.v_max_angle = self.get_parameter('v_max_angle').get_parameter_value().double_value

        # laser measurement at 0 degrees
        angle_b = math.radians(-90) - scan_msg.angle_min
        index_b = int(angle_b / scan_msg.angle_increment)
        b = scan_msg.ranges[index_b]
        
        # measurement at <theta> degrees
        angle_a = angle_b + math.radians(self.theta)
        index_a = int(angle_a / scan_msg.angle_increment)
        a = scan_msg.ranges[index_a]

        # offset
        alpha = math.atan2(a * math.cos(math.radians(self.theta)) - b, a * math.sin(math.radians(self.theta))) # current angle
        dist_curr = b * math.cos(alpha)
        dist_fut = dist_curr + self.L * math.sin(alpha)
        error = dist_fut - self.D # offset from desired distance
        
        # steering angle PID expressed in degrees
        steering_angle = - math.degrees(self.pid(error))

        if self.i:
            print("Measurement at 0: " + str(b))
            print("Measurement at 60: " + str(a))
            print("Car angle: " + str(math.degrees(alpha)))
            print("Offset: " + str(error))
            print("Steering angle: " + str(steering_angle))
        #    self.i = False

        # determine desired speed
        speed = 0.5
        if abs(steering_angle) < self.v_max_angle:
            speed = self.v_max
        elif abs(steering_angle) < 10:
            speed = 1.5
        elif abs(steering_angle) < 20:
            speed = 1.

        self.publish_drive(steering_angle, speed)

def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    wall_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
