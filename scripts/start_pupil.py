#!/usr/bin/env python
import os
import time
import roslib
roslib.load_manifest('pupil_ros')

os.system("nohup pupil_capture > /dev/null &")
