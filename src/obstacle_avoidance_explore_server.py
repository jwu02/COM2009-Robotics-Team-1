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

        AVOIDANCE_DISTANCE = 0.3

        def generate_step_size():
            """
            Generate random step size for Levy flights exploration behaviour
            """

            # simulate a long tailed distribution
            LEVY_DISTRIBUTION = [0,0,0,0,0,1]

            # generate random step size
            if choice(LEVY_DISTRIBUTION) == 0:
                # short distance movements are more often
                return 0.2 + random() * 0.3 # return 0.2-0.5 m
            else:
                # long distance movements are less often
                return 2 + random() * 1 # return 2-3 m
        
        step_size = generate_step_size()

        def turn_till_path(ang_vel=0.0):
            while self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE * 1.5:
                if self.robot_scan.rear_min_distance <= AVOIDANCE_DISTANCE: # rear end potentially stuck against wall
                    self.robot_controller.set_move_cmd(self.lin_vel, ang_vel)
                else:
                    self.robot_controller.set_move_cmd(0.0, ang_vel)

        while rospy.get_rostime() < (self.request_time + self.assignment_time_limit + rospy.Duration(20)):
            
            # Obstacle avoidance algorithm
            # if full_min_distance <= AVOIDANCE_DISTANCE
                # stop
                # get full_min_distance_angle

                # if angle in front region >340 and <20
                    # if left and right path or no where to turn
                        # turn random direction
                    # elif path to left
                        # turn left
                    # elif path to right
                        # turn right
                
                # elif angle in front left region
                    # turn right (until there is a path, take into account dead-ends)
                
                # elif angle in front right region
                    # turn left
                
                # elif angle in rear region
                    # increase lin_vel

                # while turning
                    # if angle in rear left region
                        # move forward towards right (decrease ang_vel and increase lin_vel)
                    # elif angle in rear right region
                        # move forward towards left  (decrease ang_vel and increase lin_vel)
            # else move forward

            
            if self.robot_scan.full_min_distance <= AVOIDANCE_DISTANCE:
                rospy.logwarn("TOOOO CLOSE")
                
                # if closest angle detected in front
                if self.robot_scan.closest_object_position > -20 and self.robot_scan.closest_object_position < 20:
                    # if path to both left and right or no path to left and right
                    if self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE * 2 and self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE * 2 or \
                        self.robot_scan.left_max_distance < AVOIDANCE_DISTANCE * 2 and self.robot_scan.right_max_distance < AVOIDANCE_DISTANCE * 2:
                        turn_till_path(choice([-1, 1])) # turn in random direction until path in front
                    # if path to left
                    elif self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE * 2:
                        turn_till_path(1.0) # turn left until path in front
                    # if path to right
                    elif self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE * 2:
                        turn_till_path(-1.0) # turn right until path in front

                # if closest angle detected in right region and path to left
                elif self.robot_scan.closest_object_position >= 20 and self.robot_scan.closest_object_position <= 90 and \
                    self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE * 2:
                    turn_till_path(1.0) # turn left until path in front

                # if closest angle detected in left region and path to right
                elif self.robot_scan.closest_object_position <= -20 and self.robot_scan.closest_object_position >= -90 and \
                    self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE * 2:
                    turn_till_path(-1.0) # turn right until path in front
                
                else: # rear region
                    # increase lin_vel
                    self.robot_controller.set_move_cmd(0.2, 0.0)

            else: # just move forward if not near obstacle
                step_distance_travelled = self.robot_odom.total_distance - self.previous_distance_travelled

                if step_distance_travelled < step_size:
                    # keep moving while step size not reached
                    self.robot_controller.set_move_cmd(self.lin_vel, 0.0)
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

        full_arc = np.array(scan_data.ranges[0:180][::-1] + scan_data.ranges[180:])
        self.full_min_distance = full_arc.min()
        arc_angles = np.arange(-180, 181)
        self.closest_object_position = arc_angles[np.argmin(full_arc)]

        front_region = np.array((scan_data.ranges[-20:] + scan_data.ranges[0:21])[::-1])
        self.front_min_distance = front_region.min()
        
        left_region = np.array(scan_data.ranges[20:90][::-1])
        self.left_max_distance = left_region.max()

        right_region = np.array(scan_data.ranges[270:340][::-1])
        self.right_max_distance = right_region.max()

        rear_region = np.array(scan_data.ranges[120:240])
        self.rear_min_distance = rear_region.min()
        
        # front_arc = np.concatenate([left_region, front_region, right_region])
        # self.min_distance = front_arc.min()
        # arc_angles = np.arange(-60, 61)
        # self.closest_object_position = arc_angles[np.argmin(front_arc)]

        
        # self.front_max_distance = front_region.max()
        # self.straight_ahead_distance = scan_data.ranges[0]

        # self.full_max_distance = np.array(scan_data.ranges).max()


        
    def __init__(self):
        self.full_min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

        #=============================
        self.full_min_distance = 0.0
        self.full_max_distance = 0.0

        self.front_min_distance = 0.0
        self.front_closest_object_angle = 0
        self.front_max_distance = 0.0
        self.straight_ahead_distance = 0.0

        self.left_max_distance = 0.0
        
        self.right_max_distance = 0.0
        self.right_whisker_distance = 0.0

        self.rear_min_distance = 0.0


if __name__ == '__main__':
    ExploreServer()
    rospy.spin()
