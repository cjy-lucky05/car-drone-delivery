# robot_simulation
A stage ROS robot simulation package for simgle robot and multi robot

set environment variable 'BASE_TYPE' in ~/.bashrc,optional value is NanoRobot\NanoCar\4WD\4WD_OMNI\NanoRobot_Pro\NanoCar_Pro
e.g export BASE_TYPE=NanoRobot

roslaunch robot_simulation simulation_one_robot.launch
launch a single Robot simulation in stage

roslaunch robot_simulation simulation_one_robot_with_map.launch
launch a Robot simulation with map and amcl node

roslaunch robot_simulation simulation_two_robot.launch
launch a two Robot simulation in stage

roslaunch robot_simulation simulation_two_robot_with_map.launch
launch a two Robot simulation with share map and independ amcl node


