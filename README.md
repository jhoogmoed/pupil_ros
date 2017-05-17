# Pupil ROS
This ROS package maps user's gaze data from the [Pupil Labs eye tracker](https://pupil-labs.com/), onto a different video feed, using markers and feature matching.

## Requirements
1. Ubuntu 16.04
2. ROS Kinetic installation. Older versions may work but are not tested.

## Installation
1. **Install Pupil Remove**
    Download Pupil Remote [here](https://docs.pupil-labs.com/)
2. **Install pip**
    >sudo apt-get install python-pip
    >sudo pip install --upgrade pip
3. **Install zmq**
    >sudo pip install pyzmq
4. **Install sox**
    >sudo apt-get install sox
5. **Place the folder pupil_ros to your catkin workspace, and build the ROS package.**
    >cd Path/To/catkin_ws
    >catkin_make
    >source devel/setup.bash

    This will build six custom ROS messages: pupil, pupil_positions, gaze, gaze_positions, surface_position, accu
6. **Print markers **
    Print the QR-code like trackers. They can be found inside the **markers/** folder

## Using the package
1. **Make pupil_zmq_ros_pub.py executable**
    cd to the installed pupil_ros package in your catkin workspace:
    >cd ~/catkin_ws/src/pupil_ros/scripts
    >chmod +x pupil_zmq_ros_pub.py main_run_v4.py audio_output.py start_pupil.py
2. **Set correct settings in Pupil Remote:**
* Load plugin **Pupil_Remote** and **Frame_Publisher**
* Set localhost to 50010
* Set Frame Publisher to bgr
3. **Calibrate Pupil Remote**
    Calibrate the eye tracker using a calibration method of your choice. 
4. **Launch ROS package:**
    >roslaunch pupil_ros pupil_ros.launch

## How to grab the accuracy data
1. Setup rosbag record 
    >rosbag record matcher/gaze_accu
2. Quit recording when ready
3. Make csv file with custom BAGFILE_NAME and LOGFILE_NAME (or logfile.txt)
    >rostopic echo -b BAGFILE_NAME.bag -p /matcher/gaze_accu > LOGFILE_NAME.csv