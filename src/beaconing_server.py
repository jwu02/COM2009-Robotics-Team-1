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
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
        self.robot_camera = Tb3Camera()

        self.lin_vel = 0.2
        self.ang_vel = 0

    def beaconing_success() -> bool:
        pass

    
    def action_server_launcher(self, goal: BeaconingGoal):

        self.result.beaconing_success = False

        if not goal.target_colour:
            self.actionserver.set_aborted()
            return

        AVOIDANCE_DISTANCE = 0.4

        self.robot_camera.target_colour = goal.target_colour

        KP = -0.0005 # for camera centre control loop - point towards target beacon colour

        def turn_till_path(ang_vel=0.0):
            """
            Turn till there is a path in front of the robot
            """

            self.robot_controller.stop()
            
            while self.robot_scan.front_min_distance < AVOIDANCE_DISTANCE*2:
                if self.robot_scan.rear_min_distance <= AVOIDANCE_DISTANCE-0.1:
                    # rear end potentially stuck against wall
                    # increase ang_vel in direction robot is turning to repel its back away from wall
                    if ang_vel > 0:
                        self.robot_controller.set_move_cmd(self.lin_vel, ang_vel+0.5)
                    else:
                        self.robot_controller.set_move_cmd(self.lin_vel, ang_vel-0.5)
                else:
                    self.robot_controller.set_move_cmd(0.0, ang_vel)

        while True:
            # closed loop control to adjust robot ang_vel so center of vision
            # is adjusted to point in direction of beacon detected
            self.ang_vel = KP * self.robot_camera.y_error

            if self.robot_scan.front_min_distance < AVOIDANCE_DISTANCE-0.1:
                turn_till_path(1)
            if self.robot_scan.left_min_distance < AVOIDANCE_DISTANCE:
                turn_till_path(-1)
            elif self.robot_scan.right_min_distance < AVOIDANCE_DISTANCE:
                turn_till_path(1)
            
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

            # when displacement is not near starting point to not confuse starting colour with target beacon
            # and close enough to target beacon that most of robot's vision filled with target colour pixels
            if self.robot_odom.displacement > 2 and self.robot_camera.target_pixel_count >= 100000:
                # break out of loop for success
                while self.robot_scan.front_min_distance > 0.4:
                    self.robot_controller.set_move_cmd(self.lin_vel, 0)

                self.result.beaconing_success = True
                break
        
        if self.result.beaconing_success:
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
        

if __name__ == '__main__':
    BeaconingServer()
    rospy.spin()
