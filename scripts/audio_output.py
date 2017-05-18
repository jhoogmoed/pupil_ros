#!/usr/bin/env python
#Play audio

import rospy
import os
from std_msgs.msg import String
import sys
from msgpack import loads
import roslib
roslib.load_manifest('pupil_ros')


def callback(color):	
    # print color.data
    if color.data == 'Car':
        os.system("play ~/catkin_ws/src/pupil_ros/audio/Car.flac -q")
    elif color.data == "Person":
        os.system("play ~/catkin_ws/src/pupil_ros/audio/Person.flac -q")
    else:
        print 'Unknown'

def listener():
    rospy.init_node('audio_player', anonymous=True)
    data = rospy.Subscriber('matcher/object', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
