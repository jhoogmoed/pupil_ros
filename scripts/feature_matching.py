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

class warper:
    @staticmethod
    def warp(frame1, frame_world):
    # Initial zero homography matrices for smoothing in time
        h1 = np.zeros((3,3))
        h2 = np.zeros((3,3))

    # Variables
        y = frame1.shape[0] # vertical nr of pixels of calibration frame
        x = frame1.shape[1] # horizontal nr of pixels of calibration frame
        n = 500             # max number of keypoints to use
        a = y/2             # horizontal crop on both sides
        b = x/3             # vertical crop on both sides
        c = 0               # vertical shift in upwards direction
        ratio = 0.65        # treshhold for distance filtering
    
    # Crop the initial calibration frame
        frame1 = frame1[a:y,b:x-b]
        frame1 = cv2.copyMakeBorder(frame1,a,0,b,b,cv2.BORDER_CONSTANT,value=[0,0,0])

    # Clear matches for which NN ratio > threshold 
        def filter_distance(matches):
            dist = [m.distance for m in matches]
            thres_dist = (sum(dist) / len(dist)) * ratio
            sel_matches = [m for m in matches if m.distance < thres_dist]
            return sel_matches

    # Keep only symmetric matches
        def filter_asymmetric(matches, matches2):
            sel_matches = []
            for match1 in matches:
                for match2 in matches2:
                    if kp1[match1.queryIdx] == kp1[match2.trainIdx] and kp2[match1.trainIdx] == kp2[match2.queryIdx]:
                        sel_matches.append(match1)
                        break
            return sel_matches

    # Apply distance and symmetric filtering
        def filter_matches(matches, matches2):
            matches = filter_distance(matches)
            matches2 = filter_distance(matches2)

            return filter_asymmetric(matches, matches2)


    # Initialize ORB detector and FLANN matcher
        detector = None
        try:
            detector = cv2.ORB()
        except:
            detector = cv2.ORB_create()
        
        index_params= dict(algorithm = 6,
                           table_number = 6,        # 12
                           key_size = 20,           # 20
                           multi_probe_level = 1)   #2
        search_params=dict(checks=50)
        matcher = cv2.FlannBasedMatcher(index_params,search_params)

    # Detect and filter keypoints
        kp1, des1 = detector.detectAndCompute(frame1,None)
        kp2, des2 = detector.detectAndCompute(frame_world,None)

        if des1 is not None and des2 is not None:
            matches = sorted(matcher.match(des1,des2), key = lambda x:x.distance)
            matches2 = sorted(matcher.match(des2,des1), key = lambda x:x.distance)
            sel_matches = filter_matches(matches,matches2)
            list_kp1 = [kp1[match.queryIdx].pt for match in sel_matches]
            list_kp2 = [kp2[match.trainIdx].pt for match in sel_matches]
        else:
            matches = []
            matches2 = []
            pass

    # Calculate Homography and smooth over time
        if len(sel_matches) > 3:
            pts_src = np.array([list_kp2[:n]])
            pts_dst = np.array([list_kp1[:n]])
            h3, mask = cv2.findHomography(pts_src, pts_dst, cv2.RANSAC, 5)
            h = (h1+h2+h3)/3
            h1=h2
            h2=h3
    
    # Warp source image to destination based on homography
            img_warp = cv2.warpPerspective(frame_world, h, (frame1.shape[1],frame1.shape[0]))
            return h, img_warp, frame1