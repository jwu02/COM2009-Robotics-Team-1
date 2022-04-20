#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees, sqrt
import numpy as np

class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()

class Tb3Odometry(object):
    def odom_cb(self, odom_data: Odometry):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

        # calculate distance travelled for Levy flights step size
        if self.startup:
            self.previous_x = self.posx
            self.previous_y = self.posy
        self.startup = False

        d_increment = sqrt((self.posx - self.previous_x) ** 2 + (self.posy - self.previous_y) ** 2)
        self.total_distance = self.total_distance + d_increment

        self.previous_x = self.posx
        self.previous_y = self.posy
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        
        self.total_distance = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.startup = True


    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data: LaserScan):

        # front_arc = np.array(scan_data.ranges[0:21][::-1] + scan_data.ranges[-20:][::-1])
        # self.min_distance = front_arc.min()
        # arc_angles = np.arange(-20, 21)
        # self.closest_object_position = arc_angles[np.argmin(front_arc)]

        #============================
        front_region = np.array((scan_data.ranges[-20:] + scan_data.ranges[0:21])[::-1])
        left_region = np.array(scan_data.ranges[21:62][::-1])
        right_region = np.array(scan_data.ranges[-40:340][::-1])
        
        front_arc = np.concatenate([left_region, front_region, right_region])
        self.min_distance = front_arc.min()
        arc_angles = np.arange(-60, 61)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]

        self.front_min_distance = front_region.min()
        self.front_max_distance = front_region.max()

        self.left_max_distance = left_region.max()
        self.right_max_distance = right_region.max()
        self.full_max_distance = np.array(scan_data.ranges).max()
        
    def __init__(self):
        self.min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

        #=============================
        self.front_min_distance = 0.0
        self.front_closest_object_angle = 0
        self.front_max_distance = 0.0

        self.full_max_distance = 0.0

        self.left_max_distance = 0.0
        
        self.right_max_distance = 0.0
        self.right_whisker_distance = 0.0
