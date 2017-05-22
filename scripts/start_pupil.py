#!/usr/bin/env python

#Start pupil capture
import os
import roslib
roslib.load_manifest('pupil_ros')

os.system("nohup pupil_capture > /dev/null &")
