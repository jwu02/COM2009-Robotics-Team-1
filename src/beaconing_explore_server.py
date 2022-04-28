#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import ExploreAction, ExploreGoal, ExploreFeedback, ExploreResult

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from tb3_camera import Tb3Camera
from random import randint, random, uniform


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
        # to calculate if robot has travelled the step_size required
        self.previous_distance_travelled = self.robot_odom.total_distance
    

    def action_server_launcher(self, goal: ExploreGoal):
        success = True
        if not goal.target_colour.colour: success = False
        if not success:
            self.actionserver.set_aborted()
            return
        
        rospy.loginfo(f"SEARCH INITIATED: The target beacon colour is {goal.target_colour.colour}.")

        self.request_time = rospy.get_rostime()

        AVOIDANCE_DISTANCE = 0.4


        def generate_step_size():
            """
            Generate random step size for Levy flights exploration behaviour
            """

            # simulate a long tailed distribution
            LEVY_DISTRIBUTION = [0,0,0,0,0,1]

            # generate random step size
            if LEVY_DISTRIBUTION[randint(0, len(LEVY_DISTRIBUTION))-1] == 0:
                # short distance movements are more often
                return 0.2 + random() * 0.3 # return 0.2-0.5 m
            else:
                # long distance movements are less often
                return 2 + random() * 1 # return 2-3 m
        

        step_size = generate_step_size()

        while rospy.get_rostime() < (self.request_time + self.assignment_time_limit + rospy.Duration(20)):

            if self.robot_scan.min_distance <= AVOIDANCE_DISTANCE:
                self.robot_controller.stop()

                if self.robot_scan.closest_object_position >= 0 and \
                    self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE * 1.5:
                    # turn left
                    self.robot_controller.set_move_cmd(0.0, 1.0)
                elif self.robot_scan.closest_object_position < 0 and \
                    self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE * 1.5:
                    # turn right
                    self.robot_controller.set_move_cmd(0.0, -1.0)
                else:
                    # no where to turn, turn around
                    while self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE * 1.5:
                        self.robot_controller.set_move_cmd(0.0, -1.0)
            else:
                
                step_distance_travelled = self.robot_odom.total_distance - self.previous_distance_travelled

                print(f"DISPLACEMENT: {self.robot_odom.displacement}")

                if step_distance_travelled < step_size:
                    # keep moving while step size not reached
                    self.robot_controller.set_move_cmd(self.lin_vel, 0.0)
                else:
                    # if step size reached
                    self.robot_controller.stop()

                    # turn toward random direction for short random period of time
                    last_time = rospy.get_rostime()
                    TURN_DIRECTION = [-1, 1]
                    random_direction = TURN_DIRECTION[randint(0, len(TURN_DIRECTION))-1]
                    random_ang_vel = uniform(1, 1.5) * random_direction
                    random_turn_time = rospy.Duration(randint(1, 2))
                    while rospy.get_rostime() < last_time + random_turn_time:
                        self.robot_controller.set_move_cmd(0.0, random_ang_vel)

                    # generate a new step size
                    step_size = generate_step_size()
                    self.previous_distance_travelled = self.robot_odom.total_distance

            # ===================================================

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling exploration.")
                self.actionserver.set_preempted()
                self.robot_controller.stop()
                success = False
                break

            # # feedback data to client
            # self.feedback
            # self.actionserver.publish_feedback(self.feedback)

            self.rate.sleep()
        
        if success:
            rospy.loginfo("BEACONING COMPLETE: The robot has now stopped.")
            
            # self.result.

            # self.actionserver.set_succeeded(self.result)
            # self.robot_controller.stop()
            

if __name__ == '__main__':
    ExploreServer()
    rospy.spin()
