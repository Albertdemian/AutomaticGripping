#!/bin/bash

echo "============================================="
echo "DO you Have ROS Installed? (Y)es or (N)o ..."
read ros_inst

if [ $ros_inst == "Y" ]
then 
    echo "ROS will be installed automatically ..."
else 
    echo "SKIPPING ROS INSTALL!"
fi

#to install ROS
if [ $ros_inst == "Y" ]
then 
    echo "Installing ROS  MELODIC ..."
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
    chmod +x install_ros_melodic.sh
    ./install_ros_melodic.sh
fi

echo "============================================="
echo "creating ROS workspace ..."
source /opt/ros/melodic/setup.bash
mkdir -p ~/Automatic_gripping/src
cd ~/Automatic_gripping/
catkin_make

source devel/setup.bash

echo "============================================="
echo "Cloning Git repository ..."
cd ~/Automatic_gripping/src/

git clone --branch pick_place https://tfs.university.innopolis.ru/tfs/IndustrialRoboticsLab/HoloRobotics/_git/AutomationGripping .
cd ..

echo "============================================="
echo "installing dependencies ..."

sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

echo "============================================="
echo "installing REALSESE ROS package ..."

sudo apt-get install ros-melodic-realsense2-camera

echo "============================================="
echo "installing OCTOMAP ..."
sudo apt-get install ros-$ROS_DISTRO-octomap*

echo "Package ready to use. ENJOY! \(^.^)/ "