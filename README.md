# Using A-star Algorithm on ROS Turtlebot

[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)
---


### Authors
Arpit Aggarwal Shantam Bajpai


### Instructions for running the code
To run the code for finding the path, follow the following commands:

```
cd codes
python Astar_rigid.py
```


### Results
The following is an example of A-star algorithm applied on a rigid robot (Start Node(in m)- (4, 3, 0), Goal Node(in m)- (-4, -3), Wheel RPM- (100, 50), Clearance(in m)- 0.2):
![Screenshot](output.png)


### Software Required
To run the .py files, use Python 3. ROS Turtlebot package and Standard Python 3 libraries like rospy, numpy, heapq and matplotlib are used.
