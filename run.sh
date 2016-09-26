#!/bin/bash

cd ~/catkin_ws
catkin_make install
roslaunch octomap_to_gazebo generate_world_and_contours.launch
roslaunch octomap_to_gazebo launch_gazebo_world.launch world:=office3.world

