#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import ExploreAction, ExploreGoal, ExploreFeedback, ExploreResult, TargetColour

from beaconing_explore_server import ExploreServer

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan


class ObstacleAvoidanceClient():

    CLIENT_NAME = "obstacle_avoidance_client"

    def __init__(self):
        rospy.init_node(self.CLIENT_NAME)

        self.rate = rospy.Rate(10)

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()

        self.exploration_complete = False

        self.explore_client = actionlib.SimpleActionClient(ExploreServer.ACTION_SERVER_NAME, ExploreAction)
        self.explore_client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()

        if not self.exploration_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.explore_client.cancel_goal()

    
    def explore_feedback_cb(self, feedback_data: ExploreFeedback):
        """/explore_action_server feedback"""

        pass


    def main(self):
        # explore action
        # send empty goal to action server
        self.explore_client.send_goal(ExploreGoal(), feedback_cb=self.explore_feedback_cb)

        while self.explore_client.get_state() < 2:
            self.rate.sleep()



if __name__ == '__main__':
    beaconing_client_instance = ObstacleAvoidanceClient()
    try:
        beaconing_client_instance.main()
    except rospy.ROSInterruptException:
        pass