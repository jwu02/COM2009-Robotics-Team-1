#!/usr/bin/python3

import roslaunch
import rospy
from pathlib import Path
import os

map_dir = Path.home().joinpath("catkin_ws/src/team1/maps")
if not os.path.exists(map_dir):
    os.makedirs(map_dir)
map_path = f"{map_dir}/task5_map"

rospy.init_node("exploration_map_getter", anonymous=True)
rate = rospy.Rate(1/5) # hz

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

# rospy.logwarn(f"Saving map at time: {rospy.get_time()}...")

node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=f"-f {map_path}")
process = launch.launch(node)

try:
    rate.sleep()
except rospy.ROSInterruptException:
    pass

rospy.spin()
