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

        self.lin_vel = 0.22
        self.ang_vel = 0.0
        # to calculate if robot has travelled the step_size required
        self.previous_distance_travelled = self.robot_odom.total_distance

        self.wall_to_follow = "left"
        self.follow_wall = False
    

    def action_server_launcher(self, goal: ExploreGoal):
        self.request_time = rospy.get_rostime()

        AVOIDANCE_DISTANCE = 0.35
        self.lim_ang_vel = (self.lin_vel / AVOIDANCE_DISTANCE) * 1

        def generate_step_size() -> float:
            """
            Generate random step size for Levy flights exploration behaviour
            """

            # simulate a long tailed distribution
            LEVY_DISTRIBUTION = [0,0,1,1,1]

            # generate random step size
            if choice(LEVY_DISTRIBUTION) == 0:
                # short distance movements are more often
                return 0.5 + random()*0.5 # return 1-1.5 m
            else:
                # long distance movements are less often
                return 1.0 + random()*0.5 # return 2-3 m
        
        step_size = generate_step_size()

        def turn_till_path(ang_vel=0.0):
            self.robot_controller.stop()
            
            # turn till there is a path in front of robot
            while self.robot_scan.front_min_distance < AVOIDANCE_DISTANCE*2:
                if self.robot_scan.rear_min_distance <= AVOIDANCE_DISTANCE-0.1:
                    # rear end potentially stuck against wall
                    # increase ang_vel in direction robot is turning to repel its back away from wall
                    if ang_vel > 0:
                        self.robot_controller.set_move_cmd(0.1, ang_vel+0.5)
                    else:
                        self.robot_controller.set_move_cmd(0.1, ang_vel-0.5)
                else:
                    self.robot_controller.set_move_cmd(0.0, ang_vel)

        def left_path() -> bool:
            return self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE*3

        def right_path() -> bool:
            return self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE*3

        def choose_wall() -> None:
            """
            Follow closet wall
            """
            if self.robot_scan.left_min_distance < self.robot_scan.right_min_distance:
                self.wall_to_follow = "left"
            else:
                self.wall_to_follow = "right"

        def follow_wall(direction):
            """
            Closed-loop control of distance of robot from wall
            """

            error = 0
            if direction == "left":
                error = AVOIDANCE_DISTANCE - self.robot_scan.left_whisker_distance
            elif direction == "right":
                error = self.robot_scan.right_whisker_distance - AVOIDANCE_DISTANCE
            else:
                print("Invalid direction please supply either 'left' or 'right'.")
                
            KP = -5.7 # -5 to -5.7?
            self.ang_vel = KP * error

            # sanitising data published to topic
            if self.ang_vel > self.lim_ang_vel:
                self.ang_vel = self.lim_ang_vel
            if self.ang_vel < -self.lim_ang_vel:
                self.ang_vel = -self.lim_ang_vel
            
            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)


        while rospy.get_rostime() < (self.request_time + self.assignment_time_limit + rospy.Duration(180)):
            
            # if obstacle detected ahead
            if self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE*1.2:
                self.robot_controller.stop()
                self.follow_wall = True
                self.previous_distance_travelled = self.robot_odom.total_distance

                if left_path():
                    turn_till_path(1) # turn left
                    choose_wall()
                elif right_path():
                    turn_till_path(-1) # turn right
                    choose_wall()
                elif (left_path() and right_path()) or not (left_path() and right_path()):
                    turn_direction = choice([-1, 1])
                    turn_till_path(turn_direction) # turn in random direction until path in front
                    choose_wall()
            
            else: # if no obstacle detected
                
                if self.follow_wall:
                    step_distance_travelled = self.robot_odom.total_distance - self.previous_distance_travelled

                    # move step size generated from Levy flights
                    if step_distance_travelled < step_size:
                    
                        follow_wall(self.wall_to_follow)
                    else: # if step size reached
                        # generate a new step size
                        step_size = generate_step_size()

                        self.follow_wall = False # stop following wall

                    print(f"{step_size=}[m]. {step_distance_travelled=}[m].")
                else: # if not following wall just go straight forward
                    self.robot_controller.set_move_cmd(self.lin_vel, 0.0)


            self.rate.sleep()
            

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data: LaserScan):

        front_region = np.array((scan_data.ranges[-30:] + scan_data.ranges[0:31])[::-1])
        left_region = np.array(scan_data.ranges[90-20:90+20][::-1])
        right_region = np.array(scan_data.ranges[270-20:270+20][::-1])
        rear_region = np.array(scan_data.ranges[120:240])

        self.front_distance = scan_data.ranges[0]
        self.front_min_distance = front_region.min()
        self.front_max_distance = front_region.max()
        
        self.left_min_distance = left_region.min()
        self.left_max_distance = left_region.max()
        self.left_upper_distance = scan_data.ranges[50]
        self.left_perp_distance = scan_data.ranges[90]
        wisker_angle = 60
        self.left_whisker_distance = scan_data.ranges[wisker_angle]

        self.right_min_distance = right_region.min()
        self.right_max_distance = right_region.max()
        self.right_upper_distance = scan_data.ranges[-50]
        self.right_perp_distance = scan_data.ranges[-90]
        self.right_whisker_distance = scan_data.ranges[-wisker_angle]

        self.rear_min_distance = rear_region.min()

        
    def __init__(self):
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

        self.front_distance = 0.0
        self.front_min_distance = 1000000
        self.front_max_distance = 0.0

        self.left_min_distance = 0.0
        self.left_max_distance = 0.0
        self.left_upper_distance = 0.0 # for determining upcoming path to left
        self.left_perp_distance = 0.0 # for determinig wall to left appear again
        self.left_whisker_distance = 0.0 # for close loop control
        
        self.right_min_distance = 0.0
        self.right_max_distance = 0.0
        self.right_upper_distance = 0.0
        self.left_perp_distance = 0.0
        self.right_whisker_distance = 0.0

        self.rear_min_distance = 0.0


if __name__ == '__main__':
    ExploreServer()
    rospy.spin()
