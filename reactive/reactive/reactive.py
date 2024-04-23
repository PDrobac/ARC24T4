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

class Reactive(Node):
    """
    This node implements the "Follow the gap" algo
    """
    def __init__(self):
        super().__init__('reactive')
        
        # Publishers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(Marker, '/viz_marker', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/viz_scan', 10)
        
        # Subscribers
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
    
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
        ## DISPARITY EXTENDER
        speed = 5.0
        steering_angle = 0.0
        #get ranges
        ranges = deepcopy(scan_msg.ranges)
        masked_out_ranges = deepcopy(ranges)
        #find disparities
        disparity_TH = 0.35     # I hope its in meters
        car_safety_bbl = 0.5    # I really hope its meters
        
        for idx, rng in enumerate(ranges):
            
            # to the right
            if idx < len(ranges)-1: #gives error for the last element
                if (ranges[idx+1] - ranges[idx])>disparity_TH :  #checkes for disparities
                    angle = 2 * math.asin((car_safety_bbl/2)/rng)   #calculates the angle for the safety bubble based on distance
                    
                    mask_N = math.ceil(angle / scan_msg.angle_increment)    #calculates number of laser samples
                    
                    for i in range(idx, min(len(ranges)-1, idx + mask_N)):  #decreases the range to rng
                        if masked_out_ranges[i] > rng:
                            masked_out_ranges[i] = rng
                        
            # to the left
            if idx > 0: #gives error for first element
                if (ranges[idx-1] - ranges[idx])>disparity_TH :
                    angle = 2 * math.asin((car_safety_bbl/2)/rng)
                    
                    mask_N = math.ceil(angle / scan_msg.angle_increment)
                    
                    for i in range(max(0, idx - mask_N), idx):
                        if masked_out_ranges[i] > rng:
                            masked_out_ranges[i] = rng
                        
                        
        max_masked_out = max(masked_out_ranges) #find max value
        idx_direction = masked_out_ranges.index(max_masked_out)
                        
        steering_angle = scan_msg.angle_min + (idx_direction/(len(masked_out_ranges)-1))*(scan_msg.angle_max - scan_msg.angle_min) #go in direction of max value
        
        #print(steering_angle)

        speed = 10/(1+math.pow(2, -(max_masked_out-5)/2))

        self.publish_drive(steering_angle, speed)
        self.publish_marker(steering_angle, speed)
        self.publish_scan(scan_msg, masked_out_ranges)
        
        ## FIND THE GAP
        
        # speed = 0.5
        # steering_angle = 0
        # # implement follow the gap algo
        # # obtain laser scans, process
        # rng_proc = scan_msg.ranges
        # # find closes point
        # min_rng = min(rng_proc)
        # idx = rng_proc.index(min_rng)
        # rng_proc[idx] = 0
        
        # safety_bbl_r = 20   #idx
        # # draw a "safety bubble" around it
        # for i in range(1, safety_bbl_r, 1):
        #     left_idx = idx-i
        #     right_idx = idx+i
        #     rng_proc[left_idx] = 0
        #     rng_proc[right_idx] = 0
        
        # # find the max length gap
        # gap_len_max = 0
        # gap_len_cur = 0
        # gap_idx_start = -1
        # for i in range(0, rng_proc.len(), 1):
        #     if rng_proc[i] == 0:
        #         continue
        #     i += 1
        #     while ((rng_proc[i] != 0) and (i < rng_proc.len())):
        #         gap_len_cur += 1
        #         i += 1
        #     if gap_len_cur > gap_len_max:
        #         gap_idx_start = i-gap_len_cur
        #         gap_len_max = gap_len_cur
        
        # gap = rng_proc[gap_idx_start:gap_idx_start+gap_len_max]

        # self.publish_drive(steering_angle, speed)
    
def main(args=None):
    rclpy.init(args=args)
    reactive = Reactive()
    rclpy.spin(reactive)
    reactive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()