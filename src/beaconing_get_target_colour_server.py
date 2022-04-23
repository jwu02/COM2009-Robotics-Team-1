#! /usr/bin/env python3

import rospy
from tb3 import Tb3Move, Tb3Odometry
from tb3_camera import Tb3Camera
from team1.srv import GetTargetColour, GetTargetColourRequest, GetTargetColourResponse


class GetTargetColourServer():

    def __init__(self):
        self.SERVICE_NAME = "get_target_colour_service"
        rospy.init_node(f"{self.SERVICE_NAME}_server")

        self.rate = rospy.Rate(10)

        my_service = rospy.Service(self.SERVICE_NAME, GetTargetColour, self.service_cb)
        rospy.loginfo(f"Starting /{self.SERVICE_NAME}")

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_camera = Tb3Camera()


    def service_cb(self, service_request: GetTargetColourRequest) -> GetTargetColourResponse:
        service_response = GetTargetColourResponse()

        # determine target colour by analysing start zone robot placed in
        self.turn(angle=180, angular=0.5)
        self.robot_camera.get_target_colour = True
        self.turn(angle=180, angular=0.5)

        if self.robot_camera.target_colour and self.robot_camera.target_hsv:
            service_response.target_colour.colour = self.robot_camera.target_colour
            service_response.target_colour.h.min = self.robot_camera.target_hsv['h']['min']
            service_response.target_colour.h.max = self.robot_camera.target_hsv['h']['max']
            service_response.target_colour.s.min = self.robot_camera.target_hsv['s']['min']
            service_response.target_colour.s.max = self.robot_camera.target_hsv['s']['max']
        
            return service_response


    def turn(self, angle=0.0, linear=0.0, angular=0.0):
        stop_direction = (self.robot_odom.relative_yaw + angle) % 360

        if stop_direction <= 20 or stop_direction >= 340:
            while not (self.robot_odom.relative_yaw <= 10 or self.robot_odom.relative_yaw >= 350):
                self.robot_controller.set_move_cmd(linear, angular)
        else:
            while self.robot_odom.relative_yaw < stop_direction:
                self.robot_controller.set_move_cmd(linear, angular)

        self.robot_controller.stop()


    def main(self):
        rospy.spin()


if __name__ == '__main__':
    get_target_colour_instance = GetTargetColourServer()
    try:
        get_target_colour_instance.main()
    except rospy.ROSInterruptException:
        pass