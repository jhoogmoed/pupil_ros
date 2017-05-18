#!/usr/bin/env python
#Use the roslaunch file
#1 - setup al subscriptions
#2 - setup al publishers
#3 - calibrate first image
#4 - save first image and h_car matrix
#5 - feature matching person and find h_person
#6 - gaze data trough h matrices
#7 - mapping gaze on semantic scene labeling
#8 - returning color+ object name

#Import main
import sys
import os
import select
import rospy
import cv2
import roslib
import time
import numpy as np
from calibrate import calibrate
from confidence import detection

#Import specific ros
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Point

#Import specifik other
from cv_bridge import CvBridge, CvBridgeError
from pupil_ros.msg import gaze_positions
from pupil_ros.msg import accu
from feature_matching_orb_flann import warper

#Load ros package
roslib.load_manifest('pupil_ros')

#Main script
class main_matcher:
	global x
	global y
	#Class init
	def __init__(self):
		os.system("clear")
		self.st_old = time.time()
		self.t =0
		self.color_old = ''
		self.start = False
		self.p = Point()
		self.p2 = Point()
		#Initiation of vars
		self.image_world = None
		self.image_car = None
		self.image_segnet = None
		self.gaze_point = None
		#Initiation of functions
		self.cv_bridge = CvBridge()
		#Initiation of subs
		self.image_car_s = None
		self.image_segnet_s = None
		self.image_world_s = None
		self.gaze_info_s = None
		#Initiation of pubs
		self.matcher_gaze = None
		self.matcher_object = None
		self.init_ros()
		self.init_subscribers()
		self.init_publishers()
		print 'Main: Initiation complete'

	#Image callbacks
	def callback_world(self, img):
		self.image_world = self.cv_bridge.imgmsg_to_cv2(img, 'bgr8')
	
	def callback_car(self, img):
		self.image_car = self.cv_bridge.imgmsg_to_cv2(img, 'bgr8')
	
	def callback_segnet(self, img):
		self.image_segnet = self.cv_bridge.imgmsg_to_cv2(img, 'bgr8')

	#Other callbacks
	def callback_gaze(self, info):
		self.gaze_point = info.gazes[0].norm_pos


	#Ros subscribers
	def init_subscribers(self):
		#Vehicle subscriptions
		self.image_car_s = rospy.Subscriber('ueye/left/image_rect_color', Image, self.callback_car, queue_size=2, buff_size=2**24)
		self.image_segnet_s = rospy.Subscriber('segnet/image', Image, self.callback_segnet, queue_size=2, buff_size=2**24)
		
		#Pupil subscriptions
		self.image_world_s = rospy.Subscriber('pupil_capture/world', Image, self.callback_world, queue_size=10)
		self.gaze_info_s = rospy.Subscriber('pupil_capture/gaze', gaze_positions, self.callback_gaze, queue_size=10)
		print 'Main: Subscribers defined'

	#Ros publishers
	def init_publishers(self):
		self.matcher_object = rospy.Publisher('matcher/object', String, queue_size=2)
		self.matcher_warp = rospy.Publisher('matcher/warp', Image, queue_size=2)
		self.matcher_pupil = rospy.Publisher('matcher/pupil', Image, queue_size=2)
		self.matcher_gaze_pupil = rospy.Publisher('matcher/gaze_pupil', Point, queue_size=2)
		self.matcher_gaze_accu = rospy.Publisher('matcher/gaze_accu', accu, queue_size=2)
		print 'Main: Publishers defined'

	#Ros nod start
	def init_ros(self):
		rospy.init_node('eyetracker_label_matcher', anonymous=True)
		print 'Main: Ros started'

	#Get keyboard inpu
	def GetChar(self,Block):
	  if Block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
		return sys.stdin.read(1)



	#On keypress 'enter' publish accuracy data
	def keypress(self):
		ac = accu()
		ac.mx = self.point_mark[0]
		ac.my = self.point_mark[1]
		ac.eyex0 = self.point_eye0[0]
		ac.eyey0 = self.point_eye0[1]
		ac.eyex1 = self.point_eye1[0]
		ac.eyey1 = self.point_eye1[1]
		ac.eyex2 = self.point_eye2[0]
		ac.eyey2 = self.point_eye2[1]
		self.matcher_gaze_accu.publish(ac)

	#Spin
	def spin(self):
		#Only update when images are found
		if self.image_world is None or self.image_car is None:
			return

		#Only check h calibration if first time
		if self.start is False:
			self.h_car,self.success_car,c1,self.c2, m_world, m_car ,self.marker_id = calibrate.find_homo(self.image_world,self.image_car)
			(self.wheight,self.wwidth,cw) = self.image_world.shape
			(self.cheight,self.cwidth,cc) = self.image_car.shape
			# print 'Trying to find markers'


			# Highlight found markers in world camera
			img_w = self.image_world
			for marker in m_world:
				marker.highlite_marker(img_w)

			img_out_pupil = self.cv_bridge.cv2_to_imgmsg(img_w, 'bgr8')
			self.matcher_pupil.publish(img_out_pupil)
			print "World markers: %s" % len(m_world)

			# Highlight found markers in car camera
			img_c = self.image_car
			for marker in m_car:
				marker.highlite_marker(img_c)

			img_out_ros = self.cv_bridge.cv2_to_imgmsg(img_c, 'bgr8')
			self.matcher_warp.publish(img_out_ros)
			print "Car markers: %s" % len(m_car)

			#Make first h matrix
			if self.success_car is True:
				self.image_start = self.image_world
				
				for marker in self.c2:
					cv2.circle(self.image_segnet, (int(marker[0]), int(marker[1])), 10, (0,255,255),-1)


				self.my = np.matrix(self.h_car)
				self.start = True
				self.t = self.t +1

				#Print feedback
				print 'Main: Calibrated'
				os.system("play ~/catkin_ws/src/pupil_ros/audio/calibrated.flac -q")

		#After first h calculate new h
		if self.start is True:

			#Find gaze point
			m_eye = np.matrix([self.gaze_point.x*self.wwidth, (1-self.gaze_point.y)*self.wheight,1])
			mx_tr = m_eye.transpose((1, 0))

			#Calculate warped gaze point
			mh_car = self.my*mx_tr
			self.x = int(mh_car[0]/mh_car[2])
			self.y = int(mh_car[1]/mh_car[2])

			#Print warped gaze point
			cv2.circle(self.image_car,(self.x,self.y),10,(0,0,255),-1)

			#Find Second h matrix
			mh_world = warper.warp(self.image_start, self.image_world)

			#Calculate double warped gaze point
			m_world = mh_world*mh_car
			self.x2 = int(m_world[0]/m_world[2])
			self.y2 = int(m_world[1]/m_world[2])

			#Print gaze points
			cv2.circle(self.image_car,(self.x2,self.y2),10,(0,255,0),-1)
			cv2.circle(self.image_world,(int(self.gaze_point.x*self.wwidth), int((1-self.gaze_point.y)*self.wheight)),10,(255,0,255),-1)
			img_out_ros = self.cv_bridge.cv2_to_imgmsg(self.image_car, 'bgr8')
			img_out_pupil = self.cv_bridge.cv2_to_imgmsg(self.image_world, 'bgr8')
			self.matcher_warp.publish(img_out_ros)
			self.matcher_pupil.publish(img_out_pupil)

			#If marker 1 present, update coordinates  
			for marker in self.marker_id:
				if marker[2] is 1:
					self.point_eye0 = [self.gaze_point.x*self.wwidth, (1-self.gaze_point.y)*self.wheight]
					self.point_eye1 = [self.x, self.y]
					self.point_eye2 = [self.x2, self.y2]
					self.point_mark = [marker[0],marker[1]]

			#Find color match
			ret2, segnet_thresh = cv2.threshold(self.image_segnet,50,255,cv2.THRESH_BINARY)
			try:
				if 0 < int(self.x2) < int(self.cwidth-30) and 0 < int(self.y2) < int(self.cheight-30):
					r,g,b,color, conf = detection.color(segnet_thresh, self.x2, self.y2)
					if self.color_old != color and (color is 'Car' or color is 'Person') :
						self.matcher_object.publish(color)  
					self.color_old = color
			except:
				print 'Out of range'
			



#Main spin
if __name__ == "__main__":
	main_matcher = main_matcher()
	while True:
		main_matcher.spin()
		#Read character
		gc = main_matcher.GetChar(False)
		if gc == '\n':
			main_matcher.keypress()
			print 'Recorded sample'
		
		#Break on rospy exit ctrl+c
		if rospy.is_shutdown():
			break


