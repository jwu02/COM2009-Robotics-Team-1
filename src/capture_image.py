#!/usr/bin/env python3

# for capturing picture to analyse HSV values
# rosrun team1 capture_image.py

import rospy
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

node_name = "object_detection_node"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node. Currently waiting for an image...")
rate = rospy.Rate(5) # specify a rate at which we want the node to run

base_image_path = Path("/home/student/catkin_ws/src/team1/snaps")
base_image_path.mkdir(parents=True, exist_ok=True) # create directory, if it doesn't exist already

cvbridge_interface = CvBridge() # use to convert ROS image data into a format OpenCV can use

waiting_for_image = True # flag to indicate whether node has obtained an image yet or not

def show_and_save_image(img, img_name):
    full_image_path = base_image_path.joinpath(f"{img_name}.jpg")

    # # display image in pop-up window
    cv2.imshow(img_name, img)
    # 0 = wait indefinitely before executing rest of code
    # value x = wait x miliseconds before closing pop-up window
    cv2.waitKey(0)

    # save image to a .jpg file, supplying the fullpath and img data
    cv2.imwrite(str(full_image_path), img)
    print(f"Saved an image to '{full_image_path}'\n"
        f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        f"file size = {full_image_path.stat().st_size} bytes")

def camera_cb(img_data): # defining a function for a rospy.Subscriber()
    
    # change scope of variable to global so changes can be observed outside the function
    global waiting_for_image

    try:
        # specifying conversion (or encoding) to an 8-bit BGR (blue-green-red) image format
        # contain within try-except block, recommended procedure for conversion
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    height, width, channels = cv_img.shape

    print(f"Obtained an image of height {height}px and width {width}px.")

    show_and_save_image(cv_img, img_name = "yellow_beacon")

# /camera/colour/image_raw (on real robots)
# /camera/rgb/image_raw (in simulation)
rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)
# camera_cb defines the processes that should be performed
# every time a message is obtained on the subscribed topic

while waiting_for_image:
    rate.sleep() # maintain loop at speed of 5Hz, as defined earlier

# destroy any OpenCV image pop-up windows that may be still active
# or in memory before node shuts down
cv2.destroyAllWindows()