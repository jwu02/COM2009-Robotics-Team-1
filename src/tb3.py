#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
# import the function to convert orientation from quaternions to angles
from tf.transformations import euler_from_quaternion
from math import degrees, sqrt
import numpy as np

class Tb3Move(object):
    MAX_LIN_VEL = 0.26 # m/s
    MAX_ANG_VEL = 1.82 # rad/s

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
        
        # current position and orientations
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

        # relative orientation
        if self.yaw >= -90:
            self.relative_yaw = self.yaw + 90
        else: # -91 to -179 deg
            self.relative_yaw = 270 + (180 + self.yaw)

        # relative position
        self.relative_posx = -self.posx
        self.relative_posy = -self.posy

        if self.startup:
            # initial startup position
            self.yaw0 = self.yaw
            self.posx0 = self.posx
            self.posy0 = self.posx

            # position of last odom callback
            self.previous_x = self.posx
            self.previous_y = self.posy
        
            self.startup = False

        # # calculate robot displacement from original start point
        # self.displacement = sqrt((self.posx-self.posx0)**2 + (self.posy-self.posy0)**2)

        # calculate total distance travelled for Levy flights step size
        # by adding small increments to the total everytime the robot moves
        distance_increment = sqrt((self.posx-self.previous_x)**2 + (self.posy-self.previous_y)**2)
        self.total_distance = self.total_distance + distance_increment

        self.previous_x = self.posx
        self.previous_y = self.posy
    
    def __init__(self):
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        self.yaw0 = 0.0
        self.posx0 = 0.0
        self.posy0 = 0.0

        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0

        self.relative_posx = 0.0
        self.relative_posy = 0.0
        self.relative_yaw = 0.0

        # self.displacement = 0.0
        
        # for calculating total distance travelled
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
