import sys
import roslib
import rospy
import numpy as np
import cv2
roslib.load_manifest('pupil_ros')
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from mpl_toolkits.mplot3d import Axes3D


class warper:
	@staticmethod
	def warp(frame1, frame_world):
		# Variables
		n = 1000	# number of keypoints
		a = 0 #90 	# horizontal crop
		b = 0 #150	# vertical crop
		c = 0 #40  # vertical shift
		ratio = 0.80# treshhold
		h1 = np.zeros((3,3))
		h2 = np.zeros((3,3))
		h3 = np.zeros((3,3))
		h4 = np.zeros((3,3))

		# clear matches for which NN ratio > threshold 
		# def filter_distance(matches):
		# 	dist = [m.distance for m in matches]
		# 	thres_dist = (sum(dist) / len(dist)) * ratio
		# 	print thr
		# 	# keep only the reasonable matches
		# 	sel_matches = [m for m in matches if m.distance < thres_dist]
		# 	print sel_matches
		# 	return sel_matches

		# keep only symmetric matches
		def filter_asymmetric(matches, matches2):
			sel_matches = []
			for match1 in matches:
				for match2 in matches2:
					if kp1[match1.queryIdx] == kp1[match2.trainIdx] and kp2[match1.trainIdx] == kp2[match2.queryIdx]:
						sel_matches.append(match1)
						break
			return sel_matches

		# keep only symmetric matches
		def filter_matches(matches, matches2):
			# matches = filter_distance(matches)
			# matches2 = filter_distance(matches2)

			return filter_asymmetric(matches, matches2)


		#initialize ORB detector and flann matcher
		orb = None
		try:
			orb = cv2.ORB()
		except:
			orb = cv2.ORB_create()
		
		FLANN_INDEX_LSH = 6
		index_params= dict(algorithm = FLANN_INDEX_LSH,
						   table_number = 6, # 12
						   key_size = 12,     # 20
						   multi_probe_level = 1) #2
		search_params=dict(checks=50)
		flann = cv2.FlannBasedMatcher(index_params,search_params)

		#detecting and drawing keypoints
		kp1, des1 = orb.detectAndCompute(frame1,None)	# detect
		kp2, des2 = orb.detectAndCompute(frame_world,None)
		if des1 is not None and des2 is not None:    	#find matches betweed videos
			matches = flann.match(des1,des2)
			matches2 = flann.match(des2,des1)
			matches = sorted(matches, key = lambda x:x.distance)
			matches2 = sorted(matches2, key = lambda x:x.distance)
			sel_matches = filter_matches(matches,matches2)
			list_kp1 = [kp1[match.queryIdx].pt for match in sel_matches]
			list_kp2 = [kp2[match.trainIdx].pt for match in sel_matches]
			# list_kp1 = [kp1[match.queryIdx].pt for match in matches]
			# list_kp2 = [kp2[match.trainIdx].pt for match in matches]
			# print len(sel_matches)
		else:
			matches = []
			matches2 = []
			pass

		# make point arrays
		if len(sel_matches) > 3:
			pts_src = np.array([list_kp2[:n]])
			pts_dst = np.array([list_kp1[:n]])
			# Calculate Homography
			h5, mask = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC, 30)
			h = (h1+h2+h3+h4+h5)/5
			h1=h2
			h2=h3
			h3=h4
			h4=h5
			# Warp source image to destination based on homography
			# im_out = cv2.warpPerspective(frame_world, h, (frame1.shape[1],frame1.shape[0]))
			return h
		
	
			

