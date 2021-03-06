# A-star Algorithm on ROS Turtlebot-2

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)
---


### Authors
Arpit Aggarwal Shantam Bajpai


### Introduction to the Project
In this project, the A-star motion planning algorithm was used on ROS Turtlebot 2 to navigate in a configuration space consisting of static obstacles.


### Results
This is the video of the application of A-Star algorithm on Turtlebot-2.

![](https://j.gifs.com/NLygjm.gif)


### Software Required
For this project you will need to install the rospy, numpy, heapq(Priority queue for A*), matplotlib and gazebo to run the simulations.


### Simulation platforms used
For the simulation we used the gazebo and turtlebot2 package. The world file is located in the world folder and defines the setup of the gazebo environment.


### Instructions for running the code
For running the code please follow the detailed instructions given below.
First we create a catkin workspace for our project

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

After creating your catkin workspace change your directory to src and clone this repository

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/arp95/turtlebot_astar.git
cd ../
catkin_make
```

After cloning the repository lets create an executable for our .py file that contains the code to run our program.

```
cd ~/catkin_ws/src/turtlebot_astar/scripts
chmod +x turtlebot_astar.py
cd ../../../
catkin_make
```

Once all the above steps have been performed lets source our catkin workspace and then run the program

```
source ./devel/setup.bash
roslaunch turtlebot_astar demo.launch x:=0 y:=3 yaw:=0
```

Above as you can see in the end I have given the x,y and yaw arguments in the command line. This indicates the initial spawn position of the turtlebot in the gazebo environment. The coordinates are represented by (x,y) and the orientation is given by the yaw argument.
Once you run the environment a second terminal will pop up in which you need to enter the following information:

```
x coordinate for the start node(in meters, same as the one given in the roslaunch command):
y coordinate for the start node(in meters, same as the one given in the roslaunch command):
orientation for the start node(in radians, same as the yaw value given in the roslaunch command):
x-coordinate of the goal node(in meters):
y-coordinate of the goal node(in meters):
Enter the first value of RPM:
Enter the second value of RPM:
Enter the clearance(Basically maximum distance of the robot from the obstacle given in meters):
```

After entering all these values in the terminal, the A-star algorithm finds the optimum path between the entered start node and goal node. Then the "dvx, dvy, dw" values, which are the velocities in x-direction and y-direction and angular velocity along z-axis are published on the ROS Topic of the Turtlebot to move it from one point to another point.


### Credits
The following links were helpful for this project:
1. https://github.com/AtsushiSakai/PythonRobotics
