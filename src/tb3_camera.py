#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from team1.msg import TargetColour
import numpy as np

import os
import yaml

import cv2
from cv_bridge import CvBridge, CvBridgeError

class Tb3Camera(object):
    def camera_cb(self, img_data: Image):
        try:
            # specifying conversion (or encoding) to 8-bit BGR (blue-green-red) image format
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        
        # cropping original image to a more manageable file size
        crop_width = 1000
        crop_height = 100
        crop_y0 = int((width / 2) - (crop_width / 2))
        crop_z0 = int((height / 2) - (crop_height / 2))
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_y0:crop_y0+crop_width]

        # masking - apply filter to pixels to discard any pixel data that isn't 
        # related to colour of interest
        # convert cropped image into HSV colour space, makes process easier
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        if self.get_target_colour:
            # determine target colour
            current_dir = os.path.dirname(os.path.abspath(__file__))
            hsv_data_file = os.path.join(current_dir, 'hsv_data.yaml')
            with open(hsv_data_file, "r") as stream:
                try:
                    hsv_data = yaml.safe_load(stream)

                    # used to store number of "True" pixels for when each mask is applied
                    masks = {}

                    for colour, hsv in hsv_data.items():
                        # HSV Hue-Saturation-Value
                        # value = brightness generally 100-255 works quite well
                        lower_threshold = (hsv['h']['min'], hsv['s']['min'], 100)
                        upper_threshold = (hsv['h']['max'], hsv['s']['max'], 255)

                        # output Boolean Mask, a matrix same size and shape as number of pixels in image,
                        # containing only 1 or 0 values, illustrating which pixels do have a value 
                        # within specified range
                        img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)

                        # mask applied to image using Bitwise AND operation
                        # to get rid of image pixels with mask value False
                        filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask=img_mask)

                        # store number of "True" pixels for each mask with colour we are interested in
                        # print(np.array(img_mask))
                        true_pixel_count = np.count_nonzero((np.array(img_mask).flatten() == 255))
                        if true_pixel_count > 0:
                            masks[colour] = true_pixel_count

                    # identify target colour by finding colour mask with greatest number of 1s
                    if len(masks.keys()) == 0:
                        rospy.loginfo("No target beacon colour identified.")
                    else:
                        self.target_colour.colour = max(masks, key=masks.get)
                        self.target_hsv = hsv_data[self.target_colour.colour]
                        self.target_colour.h.min = self.target_hsv['h']['min']
                        self.target_colour.h.max = self.target_hsv['h']['max']
                        self.target_colour.s.min = self.target_hsv['s']['min']
                        self.target_colour.s.max = self.target_hsv['s']['max']

                except yaml.YAMLError as e:
                    print(e)
            
            self.get_target_colour = False
        
        if self.target_colour:
            lower_threshold = (self.target_colour.h.min, self.target_colour.s.min, 100)
            upper_threshold = (self.target_colour.h.max, self.target_colour.s.max, 255)

            img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
            self.target_pixel_count = np.count_nonzero((np.array(img_mask).flatten() == 255))
            filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask=img_mask)

            # calcualte centroid (central coordinate) of a blob of colour 
            # using principle of image moments

            # provide Boolean mask to obtain moments of colour blob
            m = cv2.moments(img_mask)
            # determining central point
            cy = m['m10'] / (m['m00'] + 1e-5)
            cz = m['m01'] / (m['m00'] + 1e-5)

            # displaying camera feedback for testing
            # cv2.circle(filtered_img, (int(cy), int(cz)), 10, (255, 0, 0), 2)
            # cv2.imshow("filtered image", filtered_img)
            # cv2.waitKey(1)

            # using cetroid component cy to determine how far robot needs to turn
            # to keep target beacon in sight / centre of vision
            self.y_error = cy - (crop_width / 2) # difference bewteen actual and target position
            

    def __init__(self):
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)
        self.cvbridge_interface = CvBridge()

        self.get_target_colour = False
        self.target_colour = TargetColour()
        # self.target_hsv = {
        #     'h': {'min': 0, 'max': 0},
        #     's': {'min': 0, 'max': 0}
        # }

        self.y_error = 0
        self.target_pixel_count = 0
