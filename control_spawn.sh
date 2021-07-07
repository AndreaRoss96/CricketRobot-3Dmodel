#!/bin/bash

echo "cleaning work space ..."

rm -r ~/catkin_ws/*
mkdir ~/catkin_ws/src/
mkdir ~/catkin_ws/src/cricket_robot/

echo "copying ..."

cp -a * ~/catkin_ws/src/cricket_robot/

echo "setting up workspace"

mv ~/catkin_ws/src/cricket_robot/cricket_control ~/catkin_ws/src/cricket_control
cd ~/catkin_ws
catkin_create_pkg controller_manager joint_state_controller robot_state_publisher

echo "I'm in the ws"

catkin_make
. ~/catkin_ws/devel/setup.bash

echo "ws set up, launch:"

echo "Starting the simulation"
roslaunch --wait cricket_robot gazebo.launch &

sleep(10000)
echo "**********************************************************************"
echo "Load the controllers for two joints by running the other launch file"
roslaunch cricket_control cricket_control.launch

cd ../Desktop/my_cricket/CricketRobot-3Dmodel-main/