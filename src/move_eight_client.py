#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import MoveEightAction, MoveEightGoal, MoveEightFeedback

from tb3 import Tb3Move
from move_eight_server import MoveEightServer


class MoveEightClient():

    CLIENT_NAME = "move_eight_client"

    def __init__(self):
        rospy.init_node(self.CLIENT_NAME)
        self.rate = rospy.Rate(1)

        self.m8_client = actionlib.SimpleActionClient(MoveEightServer.ACTION_SERVER_NAME, MoveEightAction)
        self.m8_client.wait_for_server()
        self.feedback_data = MoveEightFeedback()

        self.robot_controller = Tb3Move()

        self.m8_complete = False

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()

        if not self.m8_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.m8_client.cancel_goal()
    

    def move8_feedback_cb(self, feedback_data: MoveEightFeedback):
        """/move_eight_action_server feedback"""
        self.feedback_data = feedback_data


    def main(self):
        # move eight action
        # send goal to action server
        self.m8_client.send_goal(MoveEightGoal(), feedback_cb=self.move8_feedback_cb)
    
        while self.m8_client.get_state() < 2:
            print(f"x={self.feedback_data.x:.2f} [m], y={self.feedback_data.y:.2f} [m], \
yaw={self.feedback_data.yaw:.1f} [degrees].")
            self.rate.sleep()


if __name__ == '__main__':
    m8_client_instance = MoveEightClient()
    try:
        m8_client_instance.main()
    except rospy.ROSInterruptException:
        pass