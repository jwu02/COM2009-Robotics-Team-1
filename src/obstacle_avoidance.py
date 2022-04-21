#!/usr/bin/python3

import rospy
# Import some helper functions from the tb3.py module within this package
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from random import randint, random


class ObstacleAvoidance():

    def __init__(self):
        self.node_name = "obstacle_avoidance"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(10)

        # importing external classes
        # simplify process of obtaining odometry data and controlling the robot
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
        self.robot_scan = Tb3LaserScan()

        self.lin_vel = 0.2
        self.ang_vel = 0.0
        self.MAX_ANG_VEL = 1.82

        # to calculate if robot has travelled the step_size required
        self.previous_distance_travelled = self.robot_odom.total_distance

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()
    

    def main(self):
        AVOIDANCE_DISTANCE = 0.4

        # simple method of simulating levy frequency distribution
        LEVY_FREQUENCY = [0,0,0,0,1]

        def generate_step_size():
            # generate random step size
            if LEVY_FREQUENCY[randint(0, len(LEVY_FREQUENCY))-1] == 0:
                return 0.3 + random() * 0.5 # return 0.3-0.8 m
            else:
                return 1.5 + random() * 1.5 # return 1.5-3 m
        
        step_size = generate_step_size()

        while True:
            
            if self.robot_scan.min_distance <= AVOIDANCE_DISTANCE:
                self.robot_controller.stop()

                if self.robot_scan.closest_object_position >= 0 and \
                    self.robot_scan.left_max_distance > AVOIDANCE_DISTANCE * 1.5:
                    # turn left
                    self.robot_controller.set_move_cmd(0.0, 1.0)
                    self.robot_controller.publish()
                elif self.robot_scan.closest_object_position < 0 and \
                    self.robot_scan.right_max_distance > AVOIDANCE_DISTANCE * 1.5:
                    # turn right
                    self.robot_controller.set_move_cmd(0.0, -1.0)
                    self.robot_controller.publish()
                else:
                    # no where to turn, turn around
                    while self.robot_scan.front_min_distance <= AVOIDANCE_DISTANCE * 1.5:
                        self.robot_controller.set_move_cmd(0.0, -1.0)
                        self.robot_controller.publish()
            else:
                
                step_distance_travelled = self.robot_odom.total_distance - self.previous_distance_travelled

                print(f"STEP SIZE: #{step_size}, TRAVELLED: #{step_distance_travelled}")

                if step_distance_travelled < step_size:
                    # keep moving while step size not reached
                    self.robot_controller.set_move_cmd(self.lin_vel, 0.0)
                    self.robot_controller.publish()
                else:
                    # if step size reached
                    self.robot_controller.stop()

                    # turn toward random direction for short random period of time
                    last_time = rospy.get_rostime()
                    ANG_VELS = [-1.2,1.2]
                    random_turn_angle = ANG_VELS[randint(0, len(ANG_VELS))-1]
                    random_turn_time = rospy.Duration(randint(1, 2))
                    while rospy.get_rostime() < last_time + random_turn_time:
                        self.robot_controller.set_move_cmd(0.0, random_turn_angle)
                        self.robot_controller.publish()

                    # generate a new step size
                    step_size = generate_step_size()
                    self.previous_distance_travelled = self.robot_odom.total_distance

            self.rate.sleep()



if __name__ == '__main__':
    obstacle_avoidance_instance = ObstacleAvoidance()
    try:
         obstacle_avoidance_instance.main()
    except rospy.ROSInterruptException:
        pass