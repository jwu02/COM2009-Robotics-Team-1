#!/usr/bin/python3

import rospy
import actionlib

from team1.srv import GetTargetColour, GetTargetColourRequest, GetTargetColourResponse
from team1.msg import ExploreAction, ExploreGoal, ExploreFeedback, ExploreResult
from team1.msg import BeaconingAction, BeaconingGoal, BeaconingFeedback, BeaconingResult

from beaconing_get_target_colour_server import GetTargetColourServer
from beaconing_explore_server import ExploreServer
from beaconing_server import BeaconingServer

from tb3 import Tb3Move

import cv2


class BeaconingClient():

    CLIENT_NAME = "beaconing_client"

    def __init__(self):
        rospy.init_node(self.CLIENT_NAME)

        self.rate = rospy.Rate(10)

        self.robot_controller = Tb3Move()

        self.exploration_complete = False

        self.explore_client = actionlib.SimpleActionClient(ExploreServer.ACTION_SERVER_NAME, ExploreAction)
        self.explore_client.wait_for_server()
        self.explore_goal = ExploreGoal() # represents the target colour label and HSV thresholds

        self.beaconing_client = actionlib.SimpleActionClient(BeaconingServer.ACTION_SERVER_NAME, BeaconingAction)
        self.beaconing_client.wait_for_server()
        self.beaconing_goal = BeaconingGoal()

        self.target_pixel_count = 0
        self.displacement = 0
        self.beaconing_success = False

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
        self.target_pixel_count = feedback_data.target_pixel_count
        self.displacement = feedback_data.displacement


    def beaconing_feedback_cb(self, feedback_data: BeaconingFeedback):
        """/beaconing_action_server feedback"""
        pass
        # self.feedback.target_pixel_count = self.robot_camera.target_pixel_count
    

    def main(self):
        rospy.wait_for_service(GetTargetColourServer.SERVICE_NAME)
        # create connection to service once it's running and specify service message type
        # so rospy.ServiceProxy() can process messages appropriately
        get_target_colour_service = rospy.ServiceProxy(GetTargetColourServer.SERVICE_NAME, GetTargetColour)

        # use rospy.ServiceProxy() instance to send service request message to service 
        # and obtain response back from server
        get_target_colour_service_response = get_target_colour_service(GetTargetColourRequest())

        # explore action
        self.explore_goal = get_target_colour_service_response
        # send goal to exploration action server
        rospy.loginfo(f"SEARCH INITIATED: The target beacon colour is {self.explore_goal.target_colour.colour}.")
        
        self.explore_client.send_goal(self.explore_goal, feedback_cb=self.explore_feedback_cb)
        # object detected valid only if robot 1m outside of starting point
        # to not confuse starting zone colour with beacon colour
        while not (self.displacement > 1 and self.target_pixel_count > 5000):
            self.rate.sleep()
        self.explore_client.cancel_goal()

        # send beaconing goal to another action server with different behaviour to beacon towards detected target
        rospy.loginfo("TARGET DETECTED: Beaconing initiated.")
        self.beaconing_goal = self.explore_goal
        self.beaconing_client.send_goal(self.beaconing_goal, feedback_cb=self.beaconing_feedback_cb)
        while self.beaconing_client.get_state() < 2:
            self.rate.sleep()
        
        # success message
        if self.beaconing_client.get_result().beaconing_success:
            rospy.loginfo("BEACONING COMPLETE: The robot has now stopped.")


if __name__ == '__main__':
    beaconing_client_instance = BeaconingClient()
    try:
        beaconing_client_instance.main()
    except rospy.ROSInterruptException:
        pass