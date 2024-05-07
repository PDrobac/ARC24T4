#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker  # For creating and updating the spherical marker
from std_srvs.srv import Empty
from copy import deepcopy

class Reactive(Node):
    """
    This node implements the "Follow the gap" algo
    """
    def __init__(self):
        super().__init__('reactive')
        self.declare_parameter('max_velocity', 3.)
        self.save_map_service = self.create_client(Empty, 'save_map')
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(Marker, '/viz_marker', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/viz_scan', 10)
        #self.max_dist_pub = self.create_publisher(Float32, '/data/max_dist', 10)
        #self.front_dist_pub = self.create_publisher(Float32, '/data/front_dist', 10)
        #self.angle_pub = self.create_publisher(Float32, '/data/angle', 10)
        #self.speed_pub = self.create_publisher(Float32, '/data/speed', 10)
        #self.speed_pub_adjusted = self.create_publisher(Float32, '/data/speed_adjusted', 10)
        #self.speed_pub_front = self.create_publisher(Float32, '/data/speed_front', 10)
        
        # Subscribers
        # Subscriber for laser scan with high-quality QoS to ensure reliability with SLAM processing
        qos = QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)

    
        async def save_map(self):
            await self.save_map_service.call_async(Empty.Request())
        self.get_logger().info('Map saved!')
    
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
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red colord
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
        self.original_scan_pub.publish(scan_msg)
        ## DISPARITY EXTENDER
        speed = 5.0
        steering_angle = 0.0
        #get ranges
        ranges = deepcopy(scan_msg.ranges)
        masked_out_ranges = deepcopy(ranges)
        #find disparities
        disparity_TH = 0.35     # I hope its in meters
        car_safety_bbl = 0.3    # I really hope its meters
        
        for idx, rng in enumerate(ranges):
            
            # to the right
            if idx < len(ranges)-1: #gives error for the last element
                if (ranges[idx+1] - ranges[idx])>disparity_TH :  #checkes for disparities
                    angle = 2 * math.asin((car_safety_bbl/2)/max(rng, car_safety_bbl/2))   #calculates the angle for the safety bubble based on distance
                    
                    mask_N = math.ceil(angle / scan_msg.angle_increment)    #calculates number of laser samples
                    
                    for i in range(idx, min(len(ranges)-1, idx + mask_N)):  #decreases the range to rng
                        if masked_out_ranges[i] > rng:
                            masked_out_ranges[i] = rng
                        
            # to the left
            if idx > 0: #gives error for first element
                if (ranges[idx-1] - ranges[idx])>disparity_TH :
                    angle = 2 * math.asin((car_safety_bbl/2)/max(rng, car_safety_bbl/2))
                    
                    mask_N = math.ceil(angle / scan_msg.angle_increment)
                    
                    for i in range(max(0, idx - mask_N), idx):
                        if masked_out_ranges[i] > rng:
                            masked_out_ranges[i] = rng
                        
                        
        max_masked_out = max(masked_out_ranges[200:-200]) #find max value
        idx_direction = masked_out_ranges.index(max_masked_out)
                        
        steering_angle = 0.8*(scan_msg.angle_min + (idx_direction/(len(masked_out_ranges)-1))*(scan_msg.angle_max - scan_msg.angle_min)) #go in direction of max value
        #front_distance = ranges[len(ranges) // 2]

        #steering_impact = 1 - abs(steering_angle) / math.pi  # assuming steering_angle is bounded by [-pi, pi]

        #print(steering_angle)

        speed = self.get_parameter('max_velocity').get_parameter_value().double_value/(1+math.pow(2, -((max_masked_out)-5)/2))
        msg_print = "Chosen speed:" + str(speed) + " angle:" + str(steering_angle)
        self.get_logger().info(msg_print)

        #adjusted_speed = speed * steering_impact * 5/3
        #front_speed = front_distance * 0.5
        
        self.publish_drive(steering_angle, speed)
        self.publish_marker(steering_angle, speed)
        self.publish_scan(scan_msg, masked_out_ranges)

        #self.max_dist_pub.publish(Float32(data=max_masked_out))
        #self.front_dist_pub.publish(Float32(data=front_distance))
        #self.angle_pub.publish(Float32(data=steering_angle))
        #self.speed_pub.publish(Float32(data=speed))
        #self.speed_pub_adjusted.publish(Float32(data=adjusted_speed))
        #self.speed_pub_front.publish(Float32(data=front_speed))
    
def main(args=None):
    rclpy.init(args=args)
    reactive = Reactive()
    rclpy.spin(reactive)
    reactive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()