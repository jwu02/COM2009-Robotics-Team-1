#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import ExploreAction, ExploreGoal, ExploreFeedback, ExploreResult

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from random import randint, random, uniform


class ExploreServer():
    feedback = ExploreFeedback()
    result = ExploreResult()

    def __init__(self):
        self.ACTION_SERVER_NAME = "explore_action_server"
        rospy.init_node(self.ACTION_SERVER_NAME)

        self.rate = rospy.Rate(10)

        self.actionserver = actionlib.SimpleActionServer(self.ACTION_SERVER_NAME, 
            ExploreAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()
        rospy.loginfo(f"Starting /{self.ACTION_SERVER_NAME}.")

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()

        self.assignment_time_limit = rospy.Duration(90) # seconds

        self.lin_vel = 0.2
        # to calculate if robot has travelled the step_size required
        self.previous_distance_travelled = self.robot_odom.total_distance
    

    def action_server_launcher(self, goal: ExploreGoal):

        self.request_time = rospy.get_rostime()

        # success = True
        # if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
        #     print("Invalid fwd_velocity. Select a value between (excluding) 0 and 0.26 m/s.")
        #     success = False
        # if goal.approach_distance <= 0: # what's max distance LaserScan sensor can provide?
        #     print("Invalid approach_distance. Select a positive value!")
        #     success = False

        # if not success:
        #     self.actionserver.set_aborted()
        #     return
        
        #=================================================== 
        AVOIDANCE_DISTANCE = 0.4


        def generate_step_size():
            """
            Generate random step size for Levy flights exploration behaviour
            """

            # simulate a long tailed distribution
            LEVY_DISTRIBUTION = [0,0,0,0,1]

            # generate random step size
            if LEVY_DISTRIBUTION[randint(0, len(LEVY_DISTRIBUTION))-1] == 0:
                return 0.3 + random() * 0.5 # return 0.3-0.8 m
            else:
                return 1.5 + random() * 2.5 # return 1.5-3 m
        

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

                print(f"STEP SIZE: {step_size}, TRAVELLED: {step_distance_travelled}")

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

            #===================================================

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling exploration.")
                self.actionserver.set_preempted()
                self.robot_controller.stop()
                success = False
                break

            feedback = ExploreFeedback()
            result = ExploreResult()

            # # feedback data to client
            # self.feedback
            # self.actionserver.publish_feedback(self.feedback)
        
            # if success:
            #     rospy.loginfo("Object detection completed sucessfully.")
                
            #     self.result.total_distance_travelled = self.feedback.current_distance_travelled
            #     self.result.closest_object_distance = self.robot_scan.min_distance
            #     self.result.closest_object_angle = self.robot_scan.closest_object_position

            #     self.actionserver.set_succeeded(self.result)
            #     self.robot_controller.stop()

            self.rate.sleep()
            

if __name__ == '__main__':
    ExploreServer()
    rospy.spin()
