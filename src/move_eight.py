#!/usr/bin/env python3

import rospy
from tb3 import Tb3Move, Tb3Odometry

class MoveEight():
    PATH_RADIUS = 0.5 # m
    
    def __init__(self):
        self.node_name = "move_eight"

        rospy.init_node(self.node_name)
        self.rate = rospy.Rate(1) # Hz

        # importing external classes
        # simplify process of obtaining odometry data and controlling the robot
        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()

        self.lin_vel = 0.15
        # angular velocity = linear velocity / radius
        self.ang_vel = self.lin_vel / self.PATH_RADIUS

        self.simulation_start_time = rospy.get_rostime()
        self.assignment_time_limit = rospy.Duration(60) # seconds

        rospy.on_shutdown(self.shutdown_ops)


    def shutdown_ops(self):
        self.robot_controller.stop()


    def main(self):

        def circle_poles():
            print(f"x={(self.robot_odom.relative_posx):.2f} [m], y={(self.robot_odom.relative_posy):.2f} [m], \
yaw={(self.robot_odom.relative_yaw):.1f} [degrees].")
            self.robot_controller.set_move_cmd(self.lin_vel, self.ang_vel)

            self.rate.sleep()

        # first loop (red)
        while not (self.robot_odom.relative_yaw > 350):
            circle_poles()
            
        self.ang_vel *= -1
        
        # second loop (blue)
        while not (self.robot_odom.relative_yaw < 20):
            circle_poles()

        # correcting position at end so robot finish near starting position
        while self.robot_odom.relative_posy <= -0.1:
            circle_poles()
        
        self.robot_controller.stop()

        # IMPROVEMENTS (simulation):
        # - sometimes circle blue pole more than once before stopping

        # IMPROVEMENTS (real robots):
        # - code works completely differently on real robots
        # - robot doesn't turn correct moment, have a look at /reset topic
        # - robot speed different in simulation vs real, 
        #       -> cus rate same as rate message published at


if __name__ == '__main__':
    move_eight_instance = MoveEight()
    try:
        move_eight_instance.main()
    except rospy.ROSInterruptException:
        pass