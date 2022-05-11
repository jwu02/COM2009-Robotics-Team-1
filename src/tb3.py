#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from team1.msg import TargetColour

# import the function to convert orientation from quaternions to angles
from tf.transformations import euler_from_quaternion
from math import degrees, sqrt, sin, cos, radians
import numpy as np

import os
import yaml

import cv2
from cv_bridge import CvBridge, CvBridgeError


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
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()

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

        if self.startup:
            # initial startup position
            self.yaw0 = self.yaw
            self.posx0 = self.posx
            self.posy0 = self.posy

            # position of last odom callback
            self.previous_x = self.posx
            self.previous_y = self.posy

            # angle is anti-clockwise rotation for +y-axis to point in same direction as starting yaw
            self.plane_rotation_angle = radians(((self.yaw0 % 360) - 90) % 360)
        
            self.startup = False
        
        # position and orientation relative to start
        self.relative_yaw = (self.yaw - self.yaw0) % 360

        # IMPROVEMENT: relative positions only works for current set up
        # we want +ve y-axis pointing in same direction as starting yaw
        self.relative_posx = ((self.posx-self.posx0)*cos(self.plane_rotation_angle) - \
                                (self.posy-self.posy0)*sin(self.plane_rotation_angle)) + self.posx0
        self.relative_posy = ((self.posx-self.posx0)*sin(self.plane_rotation_angle) + \
                                (self.posy-self.posy0)*cos(self.plane_rotation_angle)) + self.posy0

        # calculate robot displacement from original start point
        self.displacement = sqrt((self.posx-self.posx0)**2 + (self.posy-self.posy0)**2)

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

        self.displacement = 0.0

        self.plane_rotation_angle = 0.0
        
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

        front_region = np.array((scan_data.ranges[-20:] + scan_data.ranges[0:21])[::-1])
        left_region = np.array(scan_data.ranges[20:70][::-1])
        right_region = np.array(scan_data.ranges[290:340][::-1])
        rear_region = np.array(scan_data.ranges[120:240])

        self.front_min_distance = front_region.min()
        
        self.left_min_distance = left_region.min()
        self.left_max_distance = left_region.max()

        self.right_min_distance = right_region.min()
        self.right_max_distance = right_region.max()

        self.rear_min_distance = rear_region.min()


    def __init__(self):
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

        self.front_min_distance = 0
        self.front_max_distance = 0

        self.left_min_distance = 0
        self.left_max_distance = 0
        
        self.right_min_distance = 0
        self.right_max_distance = 0

        self.rear_min_distance = 0


class Tb3Camera(object):
    def camera_cb(self, img_data: Image):
        try:
            # specifying conversion (or encoding) to 8-bit BGR (blue-green-red) image format
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        
        # cropping original image to a more manageable file size
        crop_width = 1600
        crop_height = 100
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_y0:crop_y0+crop_width]

        # masking - apply filter to pixels to discard any pixel data that isn't 
        # related to colour of interest
        # convert cropped image into HSV colour space, makes process easier
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
        
        if self.target_colour:
            if not self.target_colour_captured:
                for c in self.TASK5_COLOURS:
                
                    
                    lower_threshold = (self.target_colour.h.min, self.target_colour.s.min, 100)
                    upper_threshold = (self.target_colour.h.max, self.target_colour.s.max, 255)

                    img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
                    self.target_pixel_count = np.count_nonzero((np.array(img_mask).flatten() == 255))
                    filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask=img_mask)

                    # displaying camera feedback for testing
                    cv2.imshow("filtered image", filtered_img)
                    cv2.waitKey(1)

            lower_threshold = (self.target_colour.h.min, self.target_colour.s.min, 100)
            upper_threshold = (self.target_colour.h.max, self.target_colour.s.max, 255)

            img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
            self.target_pixel_count = np.count_nonzero((np.array(img_mask).flatten() == 255))
            filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask=img_mask)

            # displaying camera feedback for testing
            cv2.imshow("filtered image", filtered_img)
            cv2.waitKey(1)
            

    def __init__(self):
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)
        self.cvbridge_interface = CvBridge()

        self.get_target_colour = False
        self.target_colour = TargetColour()

        self.target_pixel_count = 0

        self.TASK5_COLOURS = ["yellow", "red", "green", "blue"]
        self.target_colour_captured = False

        current_dir = os.path.dirname(os.path.abspath(__file__))
        hsv_data_file = os.path.join(current_dir, 'hsv_data.yaml')
        hsv_data = dict()
        with open(hsv_data_file, "r") as stream:
            try:
                hsv_data = yaml.safe_load(stream)
            except yaml.YAMLError as e:
                    print(e)