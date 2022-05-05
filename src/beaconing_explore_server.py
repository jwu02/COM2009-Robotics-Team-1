#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import ExploreAction, ExploreGoal, ExploreFeedback, ExploreResult
from team1.msg import BeaconingAction, BeaconingGoal, BeaconingFeedback, BeaconingResult

from beaconing_server import BeaconingServer

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from tb3_camera import Tb3Camera
from random import randint, random, uniform, choice


class ExploreServer():

    ACTION_SERVER_NAME = "explore_action_server"

    def __init__(self):
        rospy.init_node(self.ACTION_SERVER_NAME)

        self.rate = rospy.Rate(10)

        self.actionserver = actionlib.SimpleActionServer(self.ACTION_SERVER_NAME, 
            ExploreAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.feedback = ExploreFeedback()
        self.result = ExploreResult()

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
        self.robot_camera = Tb3Camera()

        self.assignment_time_limit = rospy.Duration(90) # seconds

        self.lin_vel = 0.2
        self.ang_vel = 0
        # to calculate if robot has travelled the step_size required
        self.previous_distance_travelled = self.robot_odom.total_distance

        self.target_detected = False

        self.beaconing_goal = BeaconingGoal()
        self.beaconing_client = actionlib.SimpleActionClient(BeaconingServer.ACTION_SERVER_NAME, BeaconingAction)
        self.beaconing_client.wait_for_server()
    

    def beaconing_feedback_cb(self, feedback_data: BeaconingFeedback):
        """/beaconing_action_server feedback"""
        self.feedback.target_pixel_count = self.robot_camera.target_pixel_count


    def action_server_launcher(self, goal: ExploreGoal):
        self.request_time = rospy.get_rostime()

        if not goal.target_colour.colour:
            self.actionserver.set_aborted()
            return
        
        self.robot_camera.target_colour = goal.target_colour

        AVOIDANCE_DISTANCE = 0.4

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

        def left_path() -> bool:
            """
            Determine if there is a path to the left
            """
            return self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE*3

        def right_path() -> bool:
            """
            Determine if there is a path to the right
            """
            return self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE*3
        

        while rospy.get_rostime() < (self.request_time + self.assignment_time_limit + rospy.Duration(20)):

            # if obstacle detected ahead
            if self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE:
                self.robot_controller.stop()

                if left_path and right_path:
                    # if path in both directions, turn towards side with greatest distance reading
                    if self.robot_scan.left_max_distance > self.robot_scan.right_max_distance:
                        turn_till_path(1)
                    else:
                        turn_till_path(-1)
                elif not (left_path and right_path):
                    turn_till_path(choice([-1, 1])) # turn in random direction until path in front
                elif left_path:
                    turn_till_path(1) # turn left
                elif right_path:
                    turn_till_path(-1) # turn right
            
            else: # just move forward if no obstacle ahead
                # repel away from objects if coming too close from side
                repel_ang_vel = 0.0
                if self.robot_scan.left_min_distance < AVOIDANCE_DISTANCE*1.5:
                    repel_ang_vel = -0.3
                elif self.robot_scan.right_min_distance < AVOIDANCE_DISTANCE*1.5:
                    repel_ang_vel = 0.3

                # keep moving while step size not reached
                self.robot_controller.set_move_cmd(self.lin_vel, repel_ang_vel)

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                self.actionserver.set_preempted()
                self.robot_controller.stop()
                break
            
            # feedback data
            self.feedback.target_pixel_count = self.robot_camera.target_pixel_count
            self.feedback.displacement = self.robot_odom.displacement
            self.actionserver.publish_feedback(self.feedback)

            self.rate.sleep()


if __name__ == '__main__':
    ExploreServer()
    rospy.spin()
