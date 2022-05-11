#! /usr/bin/env python3

import rospy
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from team1.srv import CaptureImage, CaptureImageRequest, CaptureImageResponse


class CaptureImageServer():

    SERVICE_NAME = "capture_image_service"

    def __init__(self):
        rospy.init_node(f"{self.SERVICE_NAME}_server")

        self.rate = rospy.Rate(10)

        my_service = rospy.Service(self.SERVICE_NAME, CaptureImage, self.service_cb)
        self.service_response = CaptureImageResponse()

        rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)

        self.get_image = False
        self.cvbridge_interface = CvBridge() # use to convert ROS image data into a format OpenCV can use


    def service_cb(self, service_request: CaptureImageRequest) -> CaptureImageResponse:
        self.service_response.response_signal = False

        self.get_image = True
        
        while not self.service_response.response_signal:
            self.rate.sleep()
            
        return self.service_response

    
    def save_image(self, img):
        base_image_path = Path.home().joinpath("catkin_ws/src/team1/snaps")
        base_image_path.mkdir(parents=True, exist_ok=True) # create directory, if it doesn't exist already
        full_image_path = base_image_path.joinpath(f"the_beacon.jpg")

        # cv2.imshow(img_name, img) # # display image in pop-up window
        # # value x = wait x miliseconds before closing pop-up window
        # cv2.waitKey(0) # 0 = wait indefinitely before executing rest of code

        # save image to a .jpg file, supplying the fullpath and img data
        cv2.imwrite(str(full_image_path), img)


    def camera_cb(self, img_data):
        try:
            # specifying conversion (or encoding) to an 8-bit BGR (blue-green-red) image format
            # contain within try-except block, recommended procedure for conversion
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        if self.get_image:
            self.save_image(cv_img)
            self.get_image = False
            self.service_response.response_signal = True


    def main(self):
        rospy.spin()


if __name__ == '__main__':
    get_target_colour_instance = CaptureImageServer()
    try:
        get_target_colour_instance.main()
    except rospy.ROSInterruptException:
        pass