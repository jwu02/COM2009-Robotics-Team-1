#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class MoveEight():

    def odom_callback(self, odom_data:Odometry):
        # obtain the orientation co-ords:
        orientation_x = odom_data.pose.pose.orientation.x
        orientation_y = odom_data.pose.pose.orientation.y
        orientation_z = odom_data.pose.pose.orientation.z
        orientation_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, orientation_y, 
                                                    orientation_z, orientation_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw + 2*pi if yaw < 0 else yaw # shift yaw range from (-pi to pi) to (0 to 2pi)
        self.theta_z -= 3/2*pi if self.theta_z > 3/2*pi else -pi/2 # make yaw start counting from 0

        # If this is the first time that this odom_callback has run, then 
        # obtain a "reference position" (used to determine how far the robot has moved
        # during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z
    

    def __init__(self):
        self.node_name = "move_eight"

        self.startup = True # flag if this node has just been launched

        """
        this is where the topic actually gets created on the ROS network
        if it doesn't exist already
        """
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        rospy.init_node(self.node_name, anonymous=True)
        # publisher will publish messages at this frequency
        self.rate = rospy.Rate(1) # Hz

        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        # define the robot pose variables and set them all to zero to start with:
        # variables to use for the "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0

        # angular velocity = linear velocity / radius
        MAX_LINEAR_VEL = 0.26 # m/s
        MAX_ANGULAR_VEL = 1.82 # rad/s
        PATH_RADIUS = 0.5 # m

        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.15
        self.vel_cmd.angular.z = self.vel_cmd.linear.x / PATH_RADIUS
        
        # message to indicate that our node is active
        rospy.loginfo(f"The '{self.node_name}' node is active...")


    def main(self):
        while self.theta_z > pi:
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()
        
        # first (red) loop
        while self.theta_z < 345/360*2*pi: # stop loop after turning approx. just below 2*pi radians
            print(f"x={(self.x):.2f} [m], y={(self.y):.2f} [m], yaw={(self.theta_z):.1f} [degrees].")
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

        self.vel_cmd.angular.z *= -1 # turn other direction,  clockwise

        # second (blue) loop
        while self.theta_z > 15/360*2*pi:
            print(f"x={(self.x):.2f} [m], y={(self.y):.2f} [m], yaw={(self.theta_z):.1f} [degrees].")
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()
        
        # stop robot after completing second loop
        self.pub.publish(Twist())

        # IMPROVEMENTS:
        # - sometimes robot doesnt stop exactly where it started
        # - sometimes, because of the low rospy.rate() robot may 
        #       miss the if statement check and doesnt stop at finish point
        

if __name__ == '__main__':
    move_eight_instance = MoveEight()
    try:
        move_eight_instance.main()
    except rospy.ROSInterruptException:
        pass