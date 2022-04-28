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

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()

        self.lin_vel = 0.26
        self.ang_vel = 0.0
        self.lim_ang_vel = 0.0

        self.ASSIGNMENT_TIME_LIMIT = rospy.Duration(150) # seconds

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()


    def main(self):

        self.request_time = rospy.get_rostime()

        AVOIDANCE_DISTANCE = 0.35

        # limiting angular velocity to avoid excessive angle changes when error is too big
        self.lim_ang_vel = (self.lin_vel / AVOIDANCE_DISTANCE) * 1.5


        def follow_right_wall():
            """Closed-loop control of distance value at scan_data.ranges[-70]
                so robot follow wall to its right at particular distance away
            """

            KP = -5.5 # -5 to -5.7?
            error = self.robot_scan.right_whisker_distance - AVOIDANCE_DISTANCE
            self.ang_vel = KP * error

            # sanitising data published to topic
            if self.ang_vel > self.lim_ang_vel:
                self.ang_vel = self.lim_ang_vel
            if self.ang_vel < -self.lim_ang_vel:
                self.ang_vel = -self.lim_ang_vel
            
            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)

        def no_path_right() -> bool:
            return self.robot_scan.right_max_distance < AVOIDANCE_DISTANCE*3
        
        def no_path_left() -> bool:
            return self.robot_scan.left_max_distance < AVOIDANCE_DISTANCE*3
        
        def turn_till_path(ang_vel=0.0):
            self.robot_controller.stop()
            
            # turn till there is a path in front of robot
            while self.robot_scan.front_min_distance < AVOIDANCE_DISTANCE*2:
                lin_vel = 0

                if self.robot_scan.rear_min_distance < AVOIDANCE_DISTANCE-0.1:
                    # rear end potentially stuck against wall
                    # increase ang_vel in direction robot is turning to repel its back away from wall
                    lin_vel = 0.05 # too high robot will accelerate into wall

                    if ang_vel > 0:
                        self.robot_controller.set_move_cmd(lin_vel, ang_vel)
                    else:
                        self.robot_controller.set_move_cmd(lin_vel, ang_vel)
                else:
                    self.robot_controller.set_move_cmd(0.0, ang_vel)
            

        while rospy.get_rostime() < (self.request_time + self.ASSIGNMENT_TIME_LIMIT + rospy.Duration(20)):

            if self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE+0.1 and \
                self.robot_scan.front_max_distance <= 1:
                # adding the second condition makes sure the robot doesn't spin around at some tight junctions
                # IMPROVEMENT: alternative way to determine which turn to take?
                #               and how much to turn?

                # might cause robot to spin around tight areas sometimes
                if no_path_right and no_path_left:
                    turn_till_path(self.lim_ang_vel) # turn left

                elif no_path_right:
                    turn_till_path(self.lim_ang_vel) # turn left

                elif no_path_left:
                    turn_till_path(-self.lim_ang_vel) # turn right
                    
            else:
                # if possible move forward and follow right wall
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