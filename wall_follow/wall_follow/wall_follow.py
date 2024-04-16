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
from std_msgs.msg import Float32, String  # For publishing velocity, TTC, and distance for PlotJuggler
from visualization_msgs.msg import Marker  # For creating and updating the spherical marker

class WallFollow(Node):
    """
    This class implements the wall following mechanism
    """
    def __init__(self):
        super().__init__('wall_follow')
        self.kp = 0.006
        self.ki = 0.
        self.kd = 0.01

        self.theta = 52. # angle between scans a and b
        self.L = 2.5 # lookahead distance in front of car
        self.D = 1. # desired distance from wall
        self.v_max = 3.5
        self.v_max_angle = 0.1

        self.prev_error = 0. # error in previous D step
        self.integral = 0. # accumulated I error
        self.prev_b = 100. # wall distance in previous step
        self.wall_lost = False
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.d_publisher = self.create_publisher(Float32, '/data/d', 10)
        self.angle_publisher = self.create_publisher(Float32, '/data/angle', 10)
        self.speed_publisher = self.create_publisher(Float32, '/data/speed', 10)
        self.error_publisher = self.create_publisher(Float32, '/data/error', 10)
        self.w_error_publisher = self.create_publisher(String, '/wall_lost_error', 10)
        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        # # Service for resetting the safety mechanism
        # self.create_service(Trigger, 'reset_safety_node', self.reset_safety_callback)

        # Declare parameter for dynamic reconfiguration
        #self.i = True
        self.declare_parameter('kp', 0.006)
        self.declare_parameter('ki', 0.)
        self.declare_parameter('kd', 0.03)
        self.declare_parameter('theta', 54.)
        self.declare_parameter('L', 3.)
        self.declare_parameter('v_max', 4.)
        self.declare_parameter('v_max_angle', 0.11)
        #self.declare_parameter('D', 1.)
        self.get_logger().debug('Wall follow Inited')
        self.timer = self.create_timer(1, self.timer_callback)

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

    def publish_data(self, dist_fut, steering_angle, speed):
        self.d_publisher.publish(Float32(data=dist_fut))
        self.angle_publisher.publish(Float32(data=steering_angle))
        self.speed_publisher.publish(Float32(data=speed))

    def timer_callback(self):
        self.error_publisher.publish(Float32(data=self.prev_error))
    
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

        # wall loss check
        if not self.wall_lost and b > 2 * self.prev_b:
            self.w_error_publisher.publish(String(data="LOST"))
            self.wall_lost = True
        elif self.wall_lost and b < 2 * self.D:
            self.w_error_publisher.publish(String(data="OKAY"))
            self.wall_lost = False
        self.prev_b = b
        
        # steering angle PID expressed in degrees
        steering_angle = - math.degrees(self.pid(error))

        #if self.i:
        #    print("Measurement at 0: " + str(b))
        #    print("Measurement at 60: " + str(a))
        #    print("Car angle: " + str(math.degrees(alpha)))
        #    print("Offset: " + str(error))
        #    print("Steering angle: " + str(steering_angle))
        #    self.i = False

        # determine desired speed
        speed = 0.5
        if abs(steering_angle) < self.v_max_angle:
            speed = self.v_max
        elif abs(steering_angle) < 1:
            speed = 1.5
        elif abs(steering_angle) < 10:
            speed = 1.

        self.publish_drive(steering_angle, speed)
        self.publish_data(dist_fut, steering_angle, speed)

def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    wall_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()