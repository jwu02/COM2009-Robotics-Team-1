#!/usr/bin/env python3

import rospy
import actionlib

from com2009_msgs.msg import SearchAction, SearchGoal, SearchFeedback, SearchResult

class SearchActionClient():

    def feedback_callback(self, feedback_data: SearchFeedback):
        pass

    def __init__(self):
        node_name = "search_action_client"
        action_server_name = "/search_action_server"
        
        rospy.init_node(node_name)

        self.rate = rospy.Rate(10)

        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient(action_server_name, SearchAction)
        self.client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        pass

    def send_goal(self, fwd_velocity, approach_distance):
        self.goal.fwd_velocity = fwd_velocity
        self.goal.approach_distance = approach_distance
        
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

    def main(self):
        self.send_goal(fwd_velocity=0.2, approach_distance=0.4)

        # rosmsg info actionlib_msgs/GoalStatus to see all states
        while self.client.get_state() < 2:
            self.rate.sleep()


if __name__ == '__main__':
    search_client_instance = SearchActionClient()
    try:
        search_client_instance.main()
    except rospy.ROSInterruptException:
        pass