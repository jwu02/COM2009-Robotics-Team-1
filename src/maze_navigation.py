#!/usr/bin/env python3

import rospy
# Import some helper functions from the tb3.py module within this package
from tb3 import Tb3Move, Tb3Odometry

from sensor_msgs.msg import LaserScan
import numpy as np

class MazeNavigation():

    def __init__(self):
        self.node_name = "maze_navigation"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(100)

        # importing external classes
        # simplify process of obtaining odometry data and controlling the robot
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()

        self.lin_vel = 0.15
        self.ang_vel = 0.0
        self.MAX_ANG_VEL = 1.82

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()


    def main(self):
        TARGET_DIST_FROM_WALL = 0.3
        FRONT_STOPPING_DISTANCE = 0.4

        # limiting angular velocity to avoid excessive angle changes when error is too big
        self.MAX_ANG_VEL = self.lin_vel / TARGET_DIST_FROM_WALL * 1.75


        def follow_right_wall():
            """Closed-loop control of distance value at scan_data.ranges[-70]
                so robot follow wall to its right at particular distance away
            """

            KP = -5.5 # -4 to -6? -4.4, 5.7
            error = self.robot_scan.right_whisker_distance - TARGET_DIST_FROM_WALL
            self.ang_vel = KP * error

            # sanitising data published to topic
            if self.ang_vel > self.MAX_ANG_VEL : self.ang_vel = self.MAX_ANG_VEL
            if self.ang_vel < -self.MAX_ANG_VEL : self.ang_vel = -self.MAX_ANG_VEL
            
            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)


        while True:
            # accelerate if open area ahead and robot not turning too much
            if self.robot_scan.front_min_distance > 1 and \
                self.ang_vel >= -0.2 and self.ang_vel <= 0.2:
                self.lin_vel = 0.25
            else:
                self.lin_vel = 0.25
            
            if self.robot_scan.front_min_distance <= FRONT_STOPPING_DISTANCE and \
                self.robot_scan.front_max_distance <= 1:
                # adding the second condition makes sure the robot doesn't spin around at some tight junctions
                
                # IMPROVEMENT: alternative way to determine which turn to take?
                #               and how much to turn?

                # might cause robot to spin around tight areas sometimes
                if self.robot_scan.right_max_distance <= 1 and \
                    self.robot_scan.left_max_distance <= 1:
                    # if deadend turn around (by turning left)
                    while self.robot_scan.front_min_distance <= 0.8:
                        # IMPROVEMENT: add condition for when turning, 
                        #   if rear of robot close to wall, increase linear velocity
                        self.lin_vel = 0
                        if self.robot_scan.rear_min_distance <= 0.25: # rear end potentially stuck against wall
                            self.lin_vel = 0.05
                        
                        self.robot_controller.set_move_cmd(self.lin_vel, self.MAX_ANG_VEL)

                elif self.robot_scan.right_max_distance <= 1.5:
                    # if no path to right, turn left
                    while self.robot_scan.front_min_distance <= 0.8:
                        self.lin_vel = 0
                        if self.robot_scan.rear_min_distance <= 0.25: # rear end potentially stuck against wall
                            self.lin_vel = 0.05
                        
                        self.robot_controller.set_move_cmd(self.lin_vel, self.MAX_ANG_VEL)

                elif self.robot_scan.left_max_distance <= 1.5:
                    # if no path to left, turn right
                    while self.robot_scan.front_min_distance <= 0.8:
                        self.lin_vel = 0
                        if self.robot_scan.rear_min_distance <= 0.25: # rear end potentially stuck against wall
                            self.lin_vel = 0.05
                        
                        self.robot_controller.set_move_cmd(self.lin_vel, -self.MAX_ANG_VEL)
                    
            else:
                # if possible move forward
                follow_right_wall()

            self.rate.sleep()


class Tb3LaserScan(object):

    def laserscan_cb(self, scan_data: LaserScan):

        front_region = np.array(scan_data.ranges[-20:] + scan_data.ranges[0:20])
        left_region = np.array(scan_data.ranges[90-60:90+60])
        right_region = np.array(scan_data.ranges[270-60:270+60])
        rear_region = np.array(scan_data.ranges[120:240])

        self.front_min_distance = front_region.min()
        self.front_max_distance = front_region.max()

        self.left_max_distance = left_region.max()

        self.right_max_distance = right_region.max()
        self.right_whisker_distance = scan_data.ranges[-60]

        self.rear_min_distance = rear_region.min()
    

    def __init__(self):

        self.front_min_distance = 0.0
        self.front_closest_object_angle = 0
        self.front_max_distance = 0.0

        self.left_max_distance = 0.0
        
        self.right_max_distance = 0.0
        self.right_whisker_distance = 0.0

        self.rear_min_distance = 0.0
        
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb) 


if __name__ == '__main__':
    maze_nav_instance = MazeNavigation()
    try:
        maze_nav_instance.main()
    except rospy.ROSInterruptException:
        pass