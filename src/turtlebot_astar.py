"""
 *  MIT License
 *
 *  Copyright (c) 2019 Arpit Aggarwal Shantam Bajpai
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
"""

# header files
from utils import *
import sys
import rospy
from geometry_msgs.msg import Point, Twist
from math import pow, atan2, sqrt
from nav_msgs.msg import Odometry

# global variables
x = 0.0
y = 0.0
theta = 0.0

# callback for subscriber
def callback(msg):
    global x 
    global y
    global theta
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation

# ROS node initialisation
rospy.init_node('turtlebot_astar')
sub = rospy.Subscriber('/odom', Odometry, callback)
pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
r = rospy.Rate(4)
speed = Twist()

# input from user
startX = float(input("Enter the x-coordinate for start node : "))
startY = float(input("Enter the y-coordinate for start node : "))
startOrientation = float(input("Enter the orientation for start node : "))
goalX = float(input("Enter the x-coordinate for goal node : "))
goalY = float(input("Enter the y-coordinate for goal node : "))
firstRPM = float(input("Enter the first value of RPM : "))
secondRPM = float(input("Enter the second value of RPM : "))
clearance = float(input("Enter the clearance of the rigid robot : "))

# take start and goal node as input
start = (startX, startY, startOrientation)
goal = (goalX, goalY)
wheelRPM = (firstRPM, secondRPM)
astar = AStar(start, goal, wheelRPM, clearance)

if(astar.IsValid(start[0], start[1])):
    if(astar.IsValid(goal[0], goal[1])):
        if(astar.IsObstacle(start[0],start[1]) == False):
            if(astar.IsObstacle(goal[0], goal[1]) == False):
                (explored_states, backtrack_states, actions, distance_from_start_to_goal) = astar.search()

                
                # move turtlebot
                x, y, theta = backtrack_states
                while not rospy.is_shutdown():
                    speed.linear.x = 0.1
                    speed.angular.z = 0.0
                    pub.publish(move_cmd)
                    r.sleep()
                    break


                # print optimal path found or not
                if(distance_from_start_to_goal == float('inf')):
                    print("\nNo optimal path found.")
                else:
                    print("\nOptimal path found. Distance is " + str(distance_from_start_to_goal))
            else:
                print("The entered goal node is an obstacle ")
                print("Please check README.md file for running Astar_rigid.py file.")
        else:
            print("The entered start node is an obstacle ")
            print("Please check README.md file for running Astar_rigid.py file.")
    else:
        print("The entered goal node outside the map ")
        print("Please check README.md file for running Astar_rigid.py file.")
else:
    print("The entered start node is outside the map ")
    print("Please check README.md file for running Astar_rigid.py file.")
