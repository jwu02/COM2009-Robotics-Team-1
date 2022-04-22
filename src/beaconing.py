#!/usr/bin/python3

import rospy
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from tb3_camera import Tb3Camera

import cv2


class Beaconing():

    def __init__(self):
        self.node_name = "beaconing"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
        self.robot_camera = Tb3Camera()

        self.TARGET_COLOUR = None
        self.lin_vel = 0.2
        self.ang_vel = 0.0

        self.stopped_at_target = False

        self.startup = True

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()

        # destroy any OpenCV image pop-up windows that may be still active
        # or in memory before node shuts down
        cv2.destroyAllWindows()
    

    def turn(self, angle=0.0, linear=0.0, angular=0.0):
        stop_direction = (self.robot_odom.relative_yaw + angle) % 360

        if stop_direction <= 10 or stop_direction >= 350:
            while not (self.robot_odom.relative_yaw <= 10 or self.robot_odom.relative_yaw >= 350):
                self.robot_controller.set_move_cmd(linear, angular)
        else:
            while self.robot_odom.relative_yaw < stop_direction:
                self.robot_controller.set_move_cmd(linear, angular)

        self.robot_controller.stop()


    def main(self):
        if self.robot_camera.TARGET_COLOUR == None:
            # determine target colour by analysing start zone robot placed in
            # turn around 180 deg
            self.turn(angle=180, angular=0.5)
        
            self.robot_camera.get_target_colour = True

            self.turn(angle=180, angular=0.5)
        
        




if __name__ == '__main__':
    beaconing_instance = Beaconing()
    try:
         beaconing_instance.main()
    except rospy.ROSInterruptException:
        pass