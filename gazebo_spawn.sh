#!/bin/bash

echo "removing ..."

rm -r ~/catkin_ws/*
mkdir ~/catkin_ws/src/
mkdir ~/catkin_ws/src/cricket_robot/

echo "copying ..."

cp -a * ~/catkin_ws/src/cricket_robot/
#`cp -a cricket_gazebo/ ~/catkin_ws/src/cricket_robot/`
#`cp -a package.xml ~/catkin_ws/src/cricket_robot/`

echo "setting up workspace"

cd ~/catkin_ws

echo "I'm in the ws"

catkin_make
. ~/catkin_ws/devel/setup.bash

echo "ws set up, launch:"

roslaunch cricket_robot joints.launch
