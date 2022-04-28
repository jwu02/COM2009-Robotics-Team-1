#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import ExploreAction, ExploreGoal, ExploreFeedback, ExploreResult

from tb3 import Tb3Move, Tb3Odometry
from random import randint, random, uniform, choice

from sensor_msgs.msg import LaserScan
import numpy as np


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

        self.assignment_time_limit = rospy.Duration(90) # seconds

        self.lin_vel = 0.15
        # to calculate if robot has travelled the step_size required
        self.previous_distance_travelled = self.robot_odom.total_distance
    

    def action_server_launcher(self, goal: ExploreGoal):
        self.request_time = rospy.get_rostime()

        AVOIDANCE_DISTANCE = 0.4

        def generate_step_size() -> float:
            """
            Generate random step size for Levy flights exploration behaviour
            """

            # simulate a long tailed distribution
            LEVY_DISTRIBUTION = [0,0,0,0,1]

            # generate random step size
            if choice(LEVY_DISTRIBUTION) == 0:
                # short distance movements are more often
                return 1 + random()*0.5 # return 1-1.5 m
            else:
                # long distance movements are less often
                return 2 + random()*1 # return 2-3 m
        
        step_size = generate_step_size()

        def turn_till_path(ang_vel=0.0):
            self.robot_controller.stop()
            
            # turn till there is a path in front of robot
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
            return self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE*3

        def right_path() -> bool:
            return self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE*3


        while rospy.get_rostime() < (self.request_time + self.assignment_time_limit + rospy.Duration(20)):
            
            # if obstacle detected ahead
            if self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE:
                self.robot_controller.stop()
                rospy.logwarn("TOOOO CLOSE")

                if (left_path and right_path) or not (left_path and right_path):
                    turn_till_path(choice([-1, 1])) # turn in random direction until path in front
                elif left_path:
                    turn_till_path(1.0) # turn left until path in front
                elif right_path:
                    turn_till_path(-1.0) # turn right until path in front
            
            else: # just move forward if no obstacle ahead
                step_distance_travelled = self.robot_odom.total_distance - self.previous_distance_travelled

                # move step size generated from Levy flights
                if step_distance_travelled < step_size:
                    # repel away from objects if coming too close from side
                    repel_ang_vel = 0.0
                    if self.robot_scan.left_min_distance < AVOIDANCE_DISTANCE*1.5:
                        repel_ang_vel = -0.2
                    elif self.robot_scan.right_min_distance < AVOIDANCE_DISTANCE*1.5:
                        repel_ang_vel = 0.2

                    # keep moving while step size not reached
                    self.robot_controller.set_move_cmd(self.lin_vel, repel_ang_vel)

                    print(f"STEP SIZE: {step_size}[m]. TRAVELLED: {step_distance_travelled}[m].")
                else:
                    # if step size reached
                    self.robot_controller.stop()

                    # turn toward random direction for short random period of time
                    last_time = rospy.get_rostime()
                    random_ang_vel = uniform(1, 1.5) * choice([-1, 1])
                    random_turn_time = rospy.Duration(randint(1, 2))
                    while rospy.get_rostime() < last_time + random_turn_time:
                        self.robot_controller.set_move_cmd(0.0, random_ang_vel)

                    # generate a new step size
                    step_size = generate_step_size()
                    self.previous_distance_travelled = self.robot_odom.total_distance

            self.rate.sleep()
            

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data: LaserScan):

        front_region = np.array((scan_data.ranges[-20:] + scan_data.ranges[0:21])[::-1])
        self.front_min_distance = front_region.min()
        
        left_region = np.array(scan_data.ranges[20:70][::-1])
        self.left_min_distance = left_region.min()
        self.left_max_distance = left_region.max()

        right_region = np.array(scan_data.ranges[290:340][::-1])
        self.right_min_distance = right_region.min()
        self.right_max_distance = right_region.max()

        rear_region = np.array(scan_data.ranges[120:240])
        self.rear_min_distance = rear_region.min()


        
    def __init__(self):
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

        self.front_min_distance = 0.0
        self.front_max_distance = 0.0

        self.left_min_distance = 0.0
        self.left_max_distance = 0.0
        
        self.right_min_distance = 0.0
        self.right_max_distance = 0.0

        self.rear_min_distance = 0.0


if __name__ == '__main__':
    ExploreServer()
    rospy.spin()
