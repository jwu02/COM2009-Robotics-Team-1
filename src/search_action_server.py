#!/usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_msgs.msg import SearchFeedback, SearchResult, SearchAction, SearchGoal

# Import some helper functions from the tb3.py module within this package
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan


class SearchActionServer(object):
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # importing external classes
        # simplify process of obtaining odometry data and controlling the robot
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
    
    def action_server_launcher(self, goal: SearchGoal):

        rate = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid fwd_velocity. Select a value between (excluding) 0 and 0.26 m/s.")
            success = False
        # need to also check max distance LaserScan sensors can provide
        if goal.approach_distance <= 0:
            print("Invalid approach_distance. Select a positive value!")
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        while True:
            # if robot close to something stop and turn rapidly
            while self.robot_scan.min_distance <= goal.approach_distance:
                self.robot_controller.set_move_cmd(0.0, 1.82)
                self.robot_controller.publish()

                # IMPROVEMENTS: 
                # - instead of just turning rapidly randomly, 
                #       turn towards direction with max LiDAR sensor reading
            
            # set the robot velocity:
            self.robot_controller.set_move_cmd(goal.fwd_velocity, 0.0)
            self.robot_controller.publish()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling object detection.")
                self.actionserver.set_preempted()
                self.robot_controller.stop() # stop the robot
                success = False
                break # exit the loop

            # feedback data to client - currently nothing
            self.actionserver.publish_feedback(self.feedback)

            rate.sleep()

        self.robot_controller.stop()
        
        if success:
            rospy.loginfo("Object detection completed sucessfully.")

            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
