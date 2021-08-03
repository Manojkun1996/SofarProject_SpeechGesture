#!/usr/bin/env python

## @package robot_control_server
# Implementation of a server/client pattern.
# Given a location, the module checks its consistency and then moves the robot accordingly.

from __future__ import print_function

from sofarproject_speechgesture.srv import MoveRobot, MoveRobotResponse
import rospy
import time
import random

## The time scale of the simulation
timeScale = rospy.get_param("time_scale")

##
# Checks if the requested position is inside the map boundaries
# @param x The x position of the location
# @param y The y position of the location
# @return  The consistency of the location with respect to the map
def checkConsistency(x, y):

    # Get the map's xmax and ymax
    xmax = rospy.get_param("map/xmax")
    ymax = rospy.get_param("map/ymax")

    if x >= 0 and x <= xmax and y >= 0 and y <= ymax:
        return True
    else:
        print('Robot control: Requested location is invalid.\n')
        return False

##
# Callback function for the service
# @param req The client's requested location
# @return    Whether the robot was able to reach the destination or not
def moveToDestination(req):

    # Check consistency of the requested location
    isConsistent = checkConsistency(req.x, req.y)

    if isConsistent:

        # Sleep for a random amount of time and then notify that the destination has been reached
        time.sleep(random.randint(2, 5) / timeScale)
        
        print('Robot control: The robot reached destination (%d, %d).\n'%(req.x, req.y))
        return MoveRobotResponse(True)

    else:
        return MoveRobotResponse(False)

##
# Client initialization
def robotControlServer():

    rospy.init_node('robot_control')
    s = rospy.Service('robot_control', MoveRobot, moveToDestination)
    print('Robot control: The robot control service is ready.\n')
    rospy.spin()

if __name__ == "__main__":
    robotControlServer()