#!/usr/bin/env python
#Play audio feedback

#Standard imports
import rospy
import os
import sys
import roslib
#Sepcific import
from std_msgs.msg import String
from msgpack import loads
#Load rospackage
roslib.load_manifest('pupil_ros')

#Init ROS
rospy.init_node('audio_player', anonymous=True)
#Play audio on feedback
def callback(color):	
    if color.data == 'Car':
        os.system("play ~/catkin_ws/src/pupil_ros/audio/Car.flac -q")
    elif color.data == "Person":
        os.system("play ~/catkin_ws/src/pupil_ros/audio/Person.flac -q")
    else:
        print 'Unknown object'
        print color.data

#Listen for audio data
def listener():
    data = rospy.Subscriber('matcher/object', String, callback)
    rospy.spin()

#Main spin
if __name__ == '__main__':
    while True:
        listener()
        #Break on rospy exit ctrl+c
        if rospy.is_shutdown():
                break
