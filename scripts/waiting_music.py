#!/usr/bin/env python

#Play audio waiting music

#Imports
import rospy
import os
import sys
import roslib
from std_msgs.msg import Bool
from msgpack import loads

roslib.load_manifest('pupil_ros')

class waiter:
    wait = None
    #Cancel when calibrated
    def callback(self, waiting):	
        self.wait = waiting.data
        if self.wait is False: 
            os.system("SIGINT")
            os.system("SIGINT")

 
        

    #Start when calibrating
    def listener(self):
        data = rospy.Subscriber('waiting', Bool, self.callback, queue_size=1)
        rospy.init_node('waiting_music', anonymous=True)
        if self.wait is True:
            os.system("play ~/catkin_ws/src/pupil_ros/audio/Elevator_Music.flac -q")
        
        

if __name__ == '__main__':
    waiter = waiter()
    while True:
        waiter.listener()
        #Break on rospy exit ctrl+c
        if rospy.is_shutdown():
            break
    
    
