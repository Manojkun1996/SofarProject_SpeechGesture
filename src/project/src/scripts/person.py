#!/usr/bin/env python

## @package person
# Implements two publishers.
# Mimics the behaviour of a person controlling the robot using voice commands or pointing gestures.

import rospy
import time
import random
import math
from sofarproject_speechgesture.msg import Location, VoiceCommand

##
# Publishes either a voice command or a location depending on the robot state.
def person():

    # Voice command publisher 
    pub1 = rospy.Publisher('voice_command', VoiceCommand, queue_size=1)

    # Pointing gesture publisher
    pub2 = rospy.Publisher('pointing_gesture', Location, queue_size=1)

    # Initialize the node
    rospy.init_node('person', anonymous=True)

    # Retrieve the map's dimensions
    mapx = rospy.get_param("map/xmax")
    mapy = rospy.get_param("map/ymax")

    # Retrieve the time scale
    timeScale = rospy.get_param("time_scale")

    while not rospy.is_shutdown():

        time.sleep(random.randint(5, 30) / timeScale)

        # Retrieve the robot state
        robotState = rospy.get_param("robot/state")

        # Logic for sending the commands
        if robotState == 'normal':

            # Send the play voice command
            com = VoiceCommand()
            com.command = 'play'
            print('Person: Sending voice command: play.\n')
            pub1.publish(com)

        elif robotState == 'play':

            # Point a location
            loc = Location()
            loc.x = random.randint(0, mapx + math.floor(mapx * 0.3))
            loc.y = random.randint(0, mapy + math.floor(mapy * 0.3))
            print('Person: Pointing location (%d, %d).\n'%(loc.x, loc.y))
            pub2.publish(loc)
        
        else:
            print("Person: I'm letting the robot sleep peacefully.\n")

if __name__ == "__main__":
    try:
        person()
    except rospy.ROSInterruptException:
        pass