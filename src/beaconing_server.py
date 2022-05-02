#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import BeaconingAction, BeaconingGoal, BeaconingFeedback, BeaconingResult

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from tb3_camera import Tb3Camera


class BeaconingServer():

    ACTION_SERVER_NAME = "beaconing_action_server"

    def __init__(self):
        rospy.init_node(self.ACTION_SERVER_NAME)

        self.rate = rospy.Rate(10)

        self.actionserver = actionlib.SimpleActionServer(self.ACTION_SERVER_NAME, 
            BeaconingAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.result = BeaconingResult()

        self.robot_controller = Tb3Move()
        # self.robot_odom = robot_odom
        # self.robot_scan = robot_scan
        # self.robot_camera = robot_camera
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
        self.robot_camera = Tb3Camera()

        self.lin_vel = 0.2
        self.ang_vel = 0

    def beaconing_success() -> bool:
        pass

    
    def action_server_launcher(self, goal: BeaconingGoal):

        if not goal.target_colour:
            self.actionserver.set_aborted()
            return

        AVOIDANCE_DISTANCE = 0.4

        self.robot_camera.target_colour = goal.target_colour

        KP = -0.0005
        KP2 = 5 # for wall distance control loop

        while self.robot_scan.front_min_distance > AVOIDANCE_DISTANCE:
            self.ang_vel = KP * self.robot_camera.y_error

            if self.robot_scan.left_min_distance < AVOIDANCE_DISTANCE:
                distance_error = self.robot_scan.left_min_distance - AVOIDANCE_DISTANCE
                self.ang_vel = KP2 * distance_error
            elif self.robot_scan.right_min_distance < AVOIDANCE_DISTANCE:
                distance_error = self.robot_scan.right_min_distance - AVOIDANCE_DISTANCE
                self.ang_vel = KP2 * -distance_error
            
            if self.ang_vel > 1.82:
                self.ang_vel = 1.82
            if self.ang_vel < -1.82:
                self.ang_vel = -1.82
            
            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)

            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling beaconing.")
                self.actionserver.set_preempted()
                self.robot_controller.stop()
                break
        
            self.rate.sleep()
        
        if self.robot_odom.displacement > 1:
            self.result.beaconing_success = True
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
        

if __name__ == '__main__':
    BeaconingServer()
    rospy.spin()
