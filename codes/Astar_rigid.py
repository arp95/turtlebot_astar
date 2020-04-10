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


# map size is 1000 cm x 1000 cm
startX = float(input("Enter the x-coordinate for start node(in m) : "))
startY = float(input("Enter the y-coordinate for start node(in m) : "))
startOrientation = float(input("Enter the orientation for start node : "))
goalX = float(input("Enter the x-coordinate for goal node(in m) : "))
goalY = float(input("Enter the y-coordinate for goal node(in m) : "))
firstRPM = float(input("Enter the first value of RPM : "))
secondRPM = float(input("Enter the second value of RPM : "))
clearance = float(input("Enter the clearance of the rigid robot(in m) : "))

# take start and goal node as input
start = (startX * 100.0, startY * 100.0, startOrientation) # (4, 3)
goal = (goalX * 100.0, goalY * 100.0) # (-4, -3)
wheelRPM = (firstRPM, secondRPM) # (100, 50)
astar = AStar(start, goal, wheelRPM, clearance * 100.0)

if(astar.IsValid(start[0], start[1])):
    if(astar.IsValid(goal[0], goal[1])):
        if(astar.IsObstacle(start[0],start[1]) == False):
            if(astar.IsObstacle(goal[0], goal[1]) == False):
                states = astar.search()
                explored_states = states[0]
                backtrack_states = states[1]
                astar.animate(explored_states, backtrack_states)

                # print optimal path found or not
                if(len(backtrack_states) == 0):
                    print("\nNo optimal path found.")
                else:
                    print("\nOptimal path found.")
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
