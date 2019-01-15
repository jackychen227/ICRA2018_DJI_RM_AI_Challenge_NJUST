# ICRA2018_DJI_RM_AI_Challenge_NJUST
`ICRA2018_DJI_RM_AI_CHALLENGE_NJUST` is a stack for the research of [RoboMaster AI challenge](https://www.robomaster.com/zh-CN/resource/pages/729?type=announcementSub), developed by team Alliance at NJUST. This repository mainly focuses on the simulation part.

At the beginning of the project, we planned to create a stack of simulation environment to test our algorithm and train our deep reinforcement learning based decision making model. Then we would put it into practice. After weeks of brokenly developing, we basically got the simulation part done with some flaw. Considering our limited time and effort, we decide to make the simulation part an open-source software stack. It will be our great pleasure if the stack do something useful during your development.

In this project, we make integrated application of ROS(Kinetic) and GAZEBO to build the simulated model training platform.  

# Get  started
Robot simulation is an essential tool in every roboticist's toolbox. A well-designed simulator makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. 

## Description of Packages
	• robots_description: main package of simulation environments(85% finished)
	• infantry_teleop_tools: infantry teleoperation with joystick and keyboard(OK)
	• infantry_2dnav: infantry navigation with the navigation stack(OK)
	• Vision: armor detecting package(refer to RoboMaster open-source code)(developing)
## Software Requirements
	• ROS kinetic (Ubuntu 16.04)
## Hardware
	• Processor: Intel® Core™ i5-6500 CPU @ 3.20GHz × 4 
	• NVIDIA GeForce GT 750M, 2 GB memory size
	• OS type: 64-bit
Tip: Recommend not to deploy this project with virtual machine.
### Install dependencies for Ubuntu 16.04 kinetic
#### Install ROS
Please follow [the installing and configuring ROS environment tutorial](http://wiki.ros.org/kinetic/Installation/Ubuntu) on ROS Wiki.
```Bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

mkdir -p ~/catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace
cd ..
catkin_make
echo "source /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
#### Install Third-party Library
```Bash
sudo apt-get install ros-kinetic-joy ros-kinetic-map-server ros-kinetic-amcl ros-kinetic-move-base ros-kinetic-controller-manager ros-kinetic-cv-bridge ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-control ros-kinetic-ros-controllers
```
# Build and Install
```Bash
cd catkin_ws/src/
git clone 
catkin_make
```

# Run
```Bash
roscore
roslaunch infantryb display.launch
```
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/infantryb%20display.png)
## Simulation:
Tip: We set the "gui" parameter false as default in order to ease the computing load of GPU, if you want to get the gazebo view, you can change the false to true in the launch file.
```Bash
roslaunch infantryb simulation_environments.launch
```
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/simulation_environments_1.png)
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/simulation_environments_2.png)
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/simulation_environments_3.png)

## Armor Detection:
```Bash
rosrun infantry_vision image_prod_cons 
```
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/image_prod_cons_1.png)
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/image_prod_cons_2.png)
## Joystick Teleoperation:
```Bash
roslaunch joystick_teleop joystick_teleop.launch
```
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/joystick_teleop.png)
## Navigation:
```Bash
roslaunch infantryb gazebo_color.launch
roslaunch infantry_2dnav move_base.launch
```
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/infantry_2dnav_1.png)
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/infantry_2dnav_2.png)
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/infantry_2dnav_3.png)
![Image text](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/blob/master/docs/images/infantry_2dnav_4.png)
# Documents
Please refer to the [docs](https://github.com/jackychen227/ICRA2018_DJI_RM_AI_Challenge_NJUST/tree/master/docs) folder.
# Copyright and License
ICRA2018_DJI_RM_AI_CHALLENGE_NJUST is provided under the BSD license.
