# Introduction 
## This repository is for automatic gripping planning using Robot's own perception. 

<center><img src="images/interface.png" alt="point cloud map with ur10e" style="width: 60%;"/></center>

In this project, The serial manipulator is equipped with parallel two-finger gripper and depth camera on the end-effector link. The task is to scan the surrounding building a point-cloud map to define collision-free workspace. 

Using MoveIt, gripping planning pipeline is implemented to grasp and place detected objects. 

**TO BE IMPLEMENTED:** Using vision module from camera, object detection and recognition and optimal grasping posture will be implemented.

**Robot Hardware :**  UR10e \
**Depth camera module :** Intel Realsense D435 \
**Depth map:** Octomap

---
# Getting Started

This pipeline is built and tested on **ROS Melodic** other versions are not tested but theoritically, it should work properly\.

### Make sure you have dependencies satisfied: 

* **ROS**:  
Follow this [link](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS

* **Catkin**:  
Install [here](http://wiki.ros.org/catkin/)

* **MoveIt:**  
```bash
sudo apt install ros-$ROS_DISTRO-moveit
```
* **Intel Realsense ROS Package :**
```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

* **Octomap :**
```bash
sudo apt-get install ros-$ROS_DISTRO-octomap
```
or follow instructions [here](https://wiki.ros.org/octomap)

* **Octomap Server :** 
```bash
sudo apt-get install ros-$ROS_DISTRO-octomap-server
``` 
### To use simulation make sure to install Gazebo: 
Instructions to install gazebo [here](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1)

> **Note :** Depth camera module is not yet implemented for simulation on gazebo. Only moveit pipeline for motion planning. 

---

# Build and Test
TODO: Describe and show how to build your code and run the tests. 

# Contribute
TODO: Explain how other users and developers can contribute to make your code better. 

If you want to learn more about creating good readme files then refer the following [guidelines](https://docs.microsoft.com/en-us/azure/devops/repos/git/create-a-readme?view=azure-devops). You can also seek inspiration from the below readme files:
- [ASP.NET Core](https://github.com/aspnet/Home)
- [Visual Studio Code](https://github.com/Microsoft/vscode)
- [Chakra Core](https://github.com/Microsoft/ChakraCore)