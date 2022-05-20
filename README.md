# Commands
- `chmod -R +x src launch` for entire folders
- `cd maps` directory, then `eog task5_map.pgm` to view map file stored

### Bash aliases
- `./bash_aliases` contains handy shortcuts for launching assignment related nodes which you can copy into `/home/student/.bash_aliases`

### After creating a new message/service/action
1. `cd ~/catkin_ws`
2. `catkin build`
- `catkin_make` instead of `catkin build` on local environment (on real robots)

### Exporting ROS package for submission
1. `cd ~/catkin_ws/src/`
2. `tar -cvf /mnt/u/wsl-ros/team1.tar team1`
3. creates `team1.tar` file in `U:\wsl-ros` to submit

### Obtain image hsv values
- `rosrun team1 capture_image.py` to capture image
- `rosrun com2009_examples image_colours.py /home/student/catkin_ws/src/team1/snaps/blue_beacon.jpg` to analyse image

# Resources
- built in message data types http://wiki.ros.org/msg