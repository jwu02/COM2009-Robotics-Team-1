#!/usr/bin/python3

import roslaunch
import rospy
from pathlib import Path
import os

class ExplorationMapGetter:

    def __init__(self) -> None:
        
        map_dir = Path.home().joinpath("catkin_ws/src/team1/maps")
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
        self.map_path = f"{map_dir}/task5_map"


    def main(self):
        rospy.init_node("exploration_map_getter", anonymous=True)

        self.rate = rospy.Rate(1/2) # hz

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        # rospy.logwarn(f"Saving map at time: {rospy.get_time()}...")

        node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=f"-f {self.map_path}")
        process = self.launch.launch(node)
        print("Tests ajsndsiabdibasdjbnas;jkfbqas;kfdhnas;jkdna;ksjndol'jsanmd;j")

        self.rate.sleep()

if __name__ == '__main__':
    map_getter_instance = ExplorationMapGetter()
    map_getter_instance.main()
    rospy.spin()
