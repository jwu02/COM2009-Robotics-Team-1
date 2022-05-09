#!/usr/bin/python3

import argparse # import the "argparse" CLI library
from pathlib import Path
import os
import yaml

import rospy
import actionlib

from std_msgs.msg import String
from team1.msg import TargetColour
from team1.msg import Task5ExplorationAction, Task5ExplorationGoal, Task5ExplorationFeedback, Task5ExplorationResult

from exploration_server import ExploreServer

from tb3 import Tb3Move

import cv2


class ExplorationClient():

    CLIENT_NAME = "exploration_client"

    def __init__(self):
        rospy.init_node(self.CLIENT_NAME)

        self.rate = rospy.Rate(10)

        self.robot_controller = Tb3Move()

        self.exploration_complete = False

        self.explore_client = actionlib.SimpleActionClient(ExploreServer.ACTION_SERVER_NAME, Task5ExplorationAction)
        self.explore_client.wait_for_server()
        self.explore_goal = Task5ExplorationGoal()
        self.target_colour = TargetColour()

        # Command-Line Interface:
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.CLIENT_NAME}' node.")
        # arguments that we define with a - will be optional
        # but we need to define default values in case non provided
        cli.add_argument("-target_colour", metavar="TARGET_COLOUR", type=String, default="yellow", help="Name of a colour: yellow, red, green or blue")
       
        # obtain the arguments passed to this node from the command-line
        # rospy.myargv() make it work regardless of whether we call the node using rosrun or roslaunch
        self.args = cli.parse_args(rospy.myargv()[1:])

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()

        # destroy any OpenCV image pop-up windows that may be still active
        # or in memory before node shuts down
        cv2.destroyAllWindows()

        if not self.exploration_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.explore_client.cancel_goal()
    

    def main(self):

        # construct target colour goal message to send to action server
        current_dir = os.path.dirname(os.path.abspath(__file__))
        hsv_data_file = os.path.join(current_dir, 'hsv_data.yaml')
        with open(hsv_data_file, "r") as stream:
            try:
                hsv_data = yaml.safe_load(stream)

                if not (self.args.target_colour.data in hsv_data.keys()):
                    self.args.target_colour.data = "yellow"
                    # raise Exception("Not a valid colour argument, valid arguments are yellow, red, green or blue")

                target_hsv = hsv_data[self.args.target_colour.data]
                self.target_colour.colour = self.args.target_colour.data
                self.target_colour.h.min = target_hsv['h']['min']
                self.target_colour.h.max = target_hsv['h']['max']
                self.target_colour.s.min = target_hsv['s']['min']
                self.target_colour.s.max = target_hsv['s']['max']
            except yaml.YAMLError as e:
                print(e)
        
        # explore action
        self.explore_goal.target_colour = self.target_colour
        self.explore_client.send_goal(self.explore_goal)
        while self.explore_client.get_state() < 2:
            self.rate.sleep()


if __name__ == '__main__':
    beaconing_client_instance = ExplorationClient()
    try:
        beaconing_client_instance.main()
    except rospy.ROSInterruptException:
        pass