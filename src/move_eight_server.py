#!/usr/bin/python3

import rospy
import actionlib

from std_msgs.msg import Empty
from team1.msg import MoveEightAction, MoveEightGoal, MoveEightFeedback, MoveEightResult

from tb3 import Tb3Move, Tb3Odometry


class MoveEightServer(object):

    ACTION_SERVER_NAME = "move_eight_action_server"

    PATH_RADIUS = 0.5 # m

    def __init__(self):
        rospy.init_node(self.ACTION_SERVER_NAME)
        self.rate = rospy.Rate(100)

        self.actionserver = actionlib.SimpleActionServer(self.ACTION_SERVER_NAME, 
            MoveEightAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.feedback = MoveEightFeedback()
        self.result = MoveEightResult()

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()

        self.lin_vel = 0.15
        # angular velocity = linear velocity / radius
        self.ang_vel = self.lin_vel / self.PATH_RADIUS

        self.assignment_time_limit = rospy.Duration(60) # +- 5 seconds

        # self.empty_publisher = rospy.Publisher('/reset', Empty, queue_size=10)
        # # real robot subscribes to a /reset topic can be used reset its odometry to zero at its current position
        # self.empty_publisher.publish(Empty())

        rospy.on_shutdown(self.shutdown_ops)

    
    def shutdown_ops(self):
        self.robot_controller.stop()
    

    def action_server_launcher(self, goal: MoveEightGoal):
        self.request_time = rospy.get_rostime()

        self.result.response_signal = True
        if not goal.request_signal:
            self.result.response_signal = False

        if not self.result.response_signal:
            self.actionserver.set_aborted()
            return

        def circle_poles():
            # # check if there has been a request to cancel the action mid-way through
            # if self.actionserver.is_preempt_requested():
            #     rospy.loginfo("Cancelling exploration.")
            #     self.actionserver.set_preempted()
            #     self.robot_controller.stop()
            #     success = False
            #     break

            # feedback data to client
            self.feedback.x = self.robot_odom.relative_posx
            self.feedback.y = self.robot_odom.relative_posy
            self.feedback.yaw = self.robot_odom.relative_yaw
            self.actionserver.publish_feedback(self.feedback)

            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)
            self.rate.sleep()

        # incase relative_yaw >355 to start with, and while loop below ends before robot gets to move
        while not (self.robot_odom.relative_yaw < 10):
            circle_poles()

        # first loop (red)
        while not (self.robot_odom.relative_yaw > 355):
            circle_poles()
            
        self.ang_vel *= -1
        # self.empty_publisher.publish(Empty())

        # # incase relative_yaw <5 after loop 1, and while loop below ends before robot gets to move
        # while not (self.robot_odom.relative_yaw > 355):
        #     circle_poles()
        
        # second loop (blue)
        while not (self.robot_odom.relative_yaw < 5):
            circle_poles()
        
        self.robot_controller.stop()

        if self.result.response_signal:
            rospy.loginfo("Move eight action server completed.")
            self.actionserver.set_succeeded(self.result)


if __name__ == '__main__':
    MoveEightServer()
    rospy.spin()
