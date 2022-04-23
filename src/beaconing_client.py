#!/usr/bin/python3

import rospy
import actionlib

from team1.srv import GetTargetColour, GetTargetColourRequest, GetTargetColourResponse
from team1.msg import ExploreAction, ExploreGoal, ExploreFeedback, ExploreResult

from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

import cv2


class BeaconingClient():

    def __init__(self):
        self.CLIENT_NAME = "beaconing_client"
        rospy.init_node(self.CLIENT_NAME)

        self.rate = rospy.Rate(10)

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()

        self.exploration_complete = False

        self.explore_client = actionlib.SimpleActionClient("explore_action_server", ExploreAction)
        self.explore_client.wait_for_server()
        self.explore_goal = ExploreGoal() # represents the target colour label and HSV thresholds

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()

        # destroy any OpenCV image pop-up windows that may be still active
        # or in memory before node shuts down
        cv2.destroyAllWindows()

        if not self.exploration_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.explore_client.cancel_goal()

    
    def explore_feedback_cb(self, feedback_data: ExploreFeedback):
        """/explore_action_server feedback"""

        pass


    def main(self):
        rospy.wait_for_service("get_target_colour_service")
        # create connection to service once it's running and specify service message type
        # so rospy.ServiceProxy() can process messages appropriately
        get_target_colour_service = rospy.ServiceProxy("get_target_colour_service", GetTargetColour)

        # use rospy.ServiceProxy() instance to send service request message to service 
        # and obtain response back from server
        get_target_colour_service_response = get_target_colour_service(GetTargetColourRequest())
        print(get_target_colour_service_response)

        # explore action
        self.explore_goal = get_target_colour_service_response
        # send goal to action server
        self.explore_client.send_goal(self.explore_goal, feedback_cb=self.explore_feedback_cb)

        while self.explore_client.get_state() < 2:
            self.rate.sleep()



if __name__ == '__main__':
    beaconing_client_instance = BeaconingClient()
    try:
        beaconing_client_instance.main()
    except rospy.ROSInterruptException:
        pass