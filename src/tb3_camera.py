#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
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
        crop_width = width - 400
        crop_height = 400
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
                        filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask = img_mask)

                        # # display image in pop-up window
                        # cv2.imshow("filtered image", filtered_img)
                        # # 0 = wait indefinitely before executing rest of code
                        # # value x = wait x miliseconds before closing pop-up window down
                        # cv2.waitKey(0)

                        # store number of "True" pixels for each mask with colour we are interested in
                        # print(np.array(img_mask))
                        true_pixel_count = np.count_nonzero((np.array(img_mask).flatten() == 255))
                        if true_pixel_count > 0:
                            masks[colour] = true_pixel_count

                    # identify target colour by finding colour mask with greatest number of 1s
                    if len(masks.keys()) == 0:
                        print("No target beacon colour identified.")
                    else:
                        self.TARGET_COLOUR = max(masks, key=masks.get)
                        print(f"SEARCH INITIATED: The target beacon colour is {self.TARGET_COLOUR.capitalize()}.")

                except yaml.YAMLError as e:
                    print(e)
            
            self.get_target_colour = False


    def __init__(self):
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)
        self.cvbridge_interface = CvBridge()

        self.get_target_colour = False
        self.TARGET_COLOUR = None
