#!/usr/bin/python3

import rospy
import actionlib

from team1.msg import Task5ExplorationAction, Task5ExplorationGoal, Task5ExplorationFeedback, Task5ExplorationResult
from team1.srv import CaptureImage, CaptureImageRequest, CaptureImageResponse

from exploration_capture_image_server import CaptureImageServer

from tb3 import Tb3Move, Tb3Odometry, Tb3Camera

from sensor_msgs.msg import LaserScan
import numpy as np
from random import choice, randint, random, uniform


class ExploreServer():

    ACTION_SERVER_NAME = "exploration_action_server"

    def __init__(self):
        rospy.init_node(self.ACTION_SERVER_NAME)

        self.rate = rospy.Rate(20)

        self.actionserver = actionlib.SimpleActionServer(self.ACTION_SERVER_NAME, 
            Task5ExplorationAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # self.feedback = Task5ExplorationFeedback()
        self.result = Task5ExplorationResult()

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()
        self.robot_camera = Tb3Camera()

        self.assignment_time_limit = rospy.Duration(180) # seconds

        self.lin_vel = 0.17
        self.ang_vel = 0.0
        self.follow_wall = False
        self.wall_to_follow = "left"
        # to calculate if robot has travelled the step_size required
        self.previous_distance_travelled = self.robot_odom.total_distance

        self.beacon_image_captured = False
    

    def action_server_launcher(self, goal: Task5ExplorationGoal):
        self.request_time = rospy.get_rostime()
        self.robot_camera.target_colour = goal.target_colour

        AVOIDANCE_DISTANCE = 0.3
        # limiting ang_vel velocity to avoid excessive angle changes when error is too big
        self.lim_ang_vel = (self.lin_vel / AVOIDANCE_DISTANCE) * 1.5

        def follow_wall(direction):
            """
            Closed-loop control of distance of robot from wall
            """

            error = 0
            if direction == "left":
                error = AVOIDANCE_DISTANCE - self.robot_scan.left_whisker_distance
            elif direction == "right":
                error = self.robot_scan.right_whisker_distance - AVOIDANCE_DISTANCE
            else:
                print("Invalid direction please supply either 'left' or 'right'.")
                
            KP = -5.5 # -5 to -5.7?
            self.ang_vel = KP * error

            # sanitising data published to topic
            if self.ang_vel > self.lim_ang_vel:
                self.ang_vel = self.lim_ang_vel
            if self.ang_vel < -self.lim_ang_vel:
                self.ang_vel = -self.lim_ang_vel
            
            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)

        def generate_step_size() -> float:
            """
            Generate random step size for Levy flights exploration behaviour
            """

            # simulate a long tailed distribution
            LEVY_DISTRIBUTION = [0,0,0,1,1,1]

            # generate random step size
            x = choice(LEVY_DISTRIBUTION)
            if x==0: return 1 + random()*1
            elif x==1: return 7 + random()*2

        
        step_size = generate_step_size()

        def turn_till_path(ang_vel=0.0) -> None:
            self.robot_controller.stop()

            # turn till there is a path in front of robot
            while not (self.robot_scan.front_min_distance > AVOIDANCE_DISTANCE*1.5):
                if self.robot_scan.rear_min_distance <= AVOIDANCE_DISTANCE*0.7:
                    # rear end potentially stuck against wall
                    self.robot_controller.set_move_cmd(0.05, ang_vel)
                else:
                    self.robot_controller.set_move_cmd(0.0, ang_vel)

                if not self.beacon_image_captured and (self.robot_camera.target_pixel_count > 10000):
                    capture_image()

        def left_path() -> bool:
            return self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE*2

        def right_path() -> bool:
            return self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE*2

        def choose_wall() -> None:
            """
            Follow closet wall
            """
            if self.robot_scan.left_min_distance < self.robot_scan.right_min_distance:
                self.wall_to_follow = "left"
            else:
                self.wall_to_follow = "right"

        def capture_image():
            rospy.wait_for_service(CaptureImageServer.SERVICE_NAME)
            # create connection to service once it's running and specify service message type
            # so rospy.ServiceProxy() can process messages appropriately
            get_target_colour_service = rospy.ServiceProxy(CaptureImageServer.SERVICE_NAME, CaptureImage)

            # use rospy.ServiceProxy() instance to send service request message to service 
            # and obtain response back from server
            service_request = CaptureImageRequest()
            service_request.request_signal = True
            get_target_colour_service_response = get_target_colour_service(service_request)

            if get_target_colour_service_response: # .get_response().response_signal
                self.beacon_image_captured = True
        

        while rospy.get_rostime() < (self.request_time + self.assignment_time_limit + rospy.Duration(180)):
            
            # if obstacle detected ahead
            if self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE*1:
                self.robot_controller.stop()
                self.follow_wall = True

                if left_path():
                    turn_till_path(1) # turn left
                    choose_wall()
                elif right_path():
                    turn_till_path(-1) # turn right
                    choose_wall()
                elif (left_path() and right_path()) or not (left_path() and right_path()):
                    turn_direction = choice([-1, 1])
                    turn_till_path(turn_direction) # turn in random direction until path in front
                    choose_wall()
            
            else: # if no obstacle detected
                if not self.beacon_image_captured and (self.robot_camera.target_pixel_count > 10000):
                    capture_image()

                step_distance_travelled = self.robot_odom.total_distance - self.previous_distance_travelled

                # move step size generated from Levy flights
                if step_distance_travelled < step_size:
                    if self.follow_wall: # follow_wall = True if obstacle detected
                        follow_wall(self.wall_to_follow)
                    else:
                        # keep moving while step size not reached
                        self.robot_controller.set_move_cmd(self.lin_vel, 0)

                    print(f"{step_size=}[m]. {step_distance_travelled=}[m].")
                else:
                    # if step size reached
                    self.follow_wall = False
                    self.robot_controller.stop()

                    # turn toward random direction for short random period of time
                    last_time = rospy.get_rostime()
                    random_ang_vel = uniform(1, 1.5) * choice([-1, 1])
                    random_turn_time = rospy.Duration(randint(1, 2))
                    while rospy.get_rostime() < last_time + random_turn_time:
                        self.robot_controller.set_move_cmd(0.0, random_ang_vel)

                    # generate a new step size
                    step_size = generate_step_size()
                    self.previous_distance_travelled = self.robot_odom.total_distance

            self.rate.sleep()
            

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data: LaserScan):

        front_region = np.array((scan_data.ranges[-20:] + scan_data.ranges[0:21])[::-1])
        left_region = np.array(scan_data.ranges[90-20:90+20][::-1])
        right_region = np.array(scan_data.ranges[270-20:270+20][::-1])
        rear_region = np.array(scan_data.ranges[120:240])

        self.front_distance = scan_data.ranges[0]
        self.front_min_distance = front_region.min()
        self.front_max_distance = front_region.max()
        
        self.left_min_distance = left_region.min()
        self.left_max_distance = left_region.max()
        self.left_upper_distance = scan_data.ranges[50]
        self.left_perp_distance = scan_data.ranges[90]
        wisker_angle = 60
        self.left_whisker_distance = scan_data.ranges[wisker_angle]

        self.right_min_distance = right_region.min()
        self.right_max_distance = right_region.max()
        self.right_upper_distance = scan_data.ranges[-50]
        self.right_perp_distance = scan_data.ranges[-90]
        self.right_whisker_distance = scan_data.ranges[-wisker_angle]

        self.rear_min_distance = rear_region.min()

        
    def __init__(self):
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

        self.front_distance = 0.0
        self.front_min_distance = 1000000
        self.front_max_distance = 0.0

        self.left_min_distance = 0.0
        self.left_max_distance = 0.0
        self.left_upper_distance = 0.0 # for determining upcoming path to left
        self.left_perp_distance = 0.0 # for determinig wall to left appear again
        self.left_whisker_distance = 0.0 # for close loop control
        
        self.right_min_distance = 0.0
        self.right_max_distance = 0.0
        self.right_upper_distance = 0.0
        self.left_perp_distance = 0.0
        self.right_whisker_distance = 0.0

        self.rear_min_distance = 0.0


if __name__ == '__main__':
    ExploreServer()
    rospy.spin()
