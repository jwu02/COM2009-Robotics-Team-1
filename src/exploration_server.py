#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import Task5ExplorationAction, Task5ExplorationGoal, Task5ExplorationFeedback, Task5ExplorationResult

from tb3 import Tb3Move, Tb3Odometry

from sensor_msgs.msg import LaserScan
import numpy as np
from random import choice


class ExploreServer():

    ACTION_SERVER_NAME = "exploration_action_server"

    def __init__(self):
        rospy.init_node(self.ACTION_SERVER_NAME)

        self.rate = rospy.Rate(10)

        self.actionserver = actionlib.SimpleActionServer(self.ACTION_SERVER_NAME, 
            Task5ExplorationAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # self.feedback = Task5ExplorationFeedback()
        self.result = Task5ExplorationResult()

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()

        self.assignment_time_limit = rospy.Duration(90) # seconds

        self.lin_vel = 0.15
        self.ang_vel = 0.0
        self.wall_to_follow = "left"
    

    def action_server_launcher(self, goal: Task5ExplorationGoal):
        self.request_time = rospy.get_rostime()

        AVOIDANCE_DISTANCE = 0.35
        # limiting ang_vel velocity to avoid excessive angle changes when error is too big
        self.lim_ang_vel = (self.lin_vel / AVOIDANCE_DISTANCE) * 1.2

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
                
            KP = -5.5 # -5 to -5.7?
            self.ang_vel = KP * error

            # sanitising data published to topic
            if self.ang_vel > self.lim_ang_vel:
                self.ang_vel = self.lim_ang_vel
            if self.ang_vel < -self.lim_ang_vel:
                self.ang_vel = -self.lim_ang_vel
            
            print(f"{direction=}")
            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)

        def turn_till_path(ang_vel=0.0):
            self.robot_controller.stop()
            
            # turn till there is a path in front of robot
            while not (self.robot_scan.front_min_distance > AVOIDANCE_DISTANCE*2):
                if self.robot_scan.rear_min_distance <= AVOIDANCE_DISTANCE-0.1:
                    # rear end potentially stuck against wall
                    self.robot_controller.set_move_cmd(self.lin_vel, ang_vel)
                else:
                    self.robot_controller.set_move_cmd(0.0, ang_vel)

        def left_path() -> bool:
            return self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE*2

        def right_path() -> bool:
            return self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE*2

        while rospy.get_rostime() < (self.request_time + self.assignment_time_limit + rospy.Duration(20)):
            
            # if obstacle detected ahead
            if self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE:
                self.robot_controller.stop()
                # rospy.logwarn("TOOOO CLOSE")

                # if (left_path() and right_path()) or not (left_path() and right_path()):
                #     turn_till_path(choice([-1, 1])) # turn in random direction until path in front
                if left_path():
                    turn_till_path(1) # turn left
                    self.wall_to_follow = "right"
                elif right_path():
                    turn_till_path(-1) # turn right
                    self.wall_to_follow = "left"
            
            else: # if no obstacle detected
                if self.ang_vel < 0.5 and self.ang_vel > -0.5 and self.robot_scan.front_min_distance < AVOIDANCE_DISTANCE:
                    # to let robot reach end of path without following wall entirely
                    self.robot_controller.set_move_cmd(self.lin_vel, 0.0)
                else:
                    if self.wall_to_follow == "left":
                        if self.robot_scan.left_perp_distance > AVOIDANCE_DISTANCE*2:
                            # go straight ahead if upcoming path to left, instead of wall following
                            self.robot_controller.set_move_cmd(self.lin_vel, 0.0)
                        elif self.robot_scan.left_upper_distance < AVOIDANCE_DISTANCE*2:
                            follow_wall(self.wall_to_follow)

                    elif self.wall_to_follow == "right":
                        if self.robot_scan.right_perp_distance > AVOIDANCE_DISTANCE*2:
                            self.robot_controller.set_move_cmd(self.lin_vel, 0.0)
                        elif self.robot_scan.right_upper_distance < AVOIDANCE_DISTANCE*2:
                            follow_wall(self.wall_to_follow)

            self.rate.sleep()
            

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data: LaserScan):

        front_region = np.array((scan_data.ranges[-20:] + scan_data.ranges[0:21])[::-1])
        left_region = np.array(scan_data.ranges[80:100][::-1])
        right_region = np.array(scan_data.ranges[260:280][::-1])
        rear_region = np.array(scan_data.ranges[120:240])

        self.front_distance = scan_data.ranges[0]
        self.front_min_distance = front_region.min()
        self.front_max_distance = front_region.max()
        
        self.left_min_distance = left_region.min()
        self.left_max_distance = left_region.max()
        self.left_upper_distance = scan_data.ranges[50]
        self.left_perp_distance = scan_data.ranges[90]
        self.left_whisker_distance = scan_data.ranges[60]

        self.right_min_distance = right_region.min()
        self.right_max_distance = right_region.max()
        self.right_upper_distance = scan_data.ranges[-50]
        self.right_perp_distance = scan_data.ranges[-90]
        self.right_whisker_distance = scan_data.ranges[-60]

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
