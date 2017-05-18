import cv2
import time
import numpy as np

from detect import detect_markers


class calibrate:
	@staticmethod
	def detect_from_stereo(img_1, img_2):

		markers_1 = detect_markers(img_1, 200) # world
		markers_2 = detect_markers(img_2, 127) # car

		available_markers_1 = []
		available_markers_2 = []
		marker_id = []
		found_markers = []

		for marker_1 in markers_1:

			# marker_1_id = marker_1.id

			if marker_1.id in found_markers:
				continue

			for marker_2 in markers_2:
				if marker_2.id == marker_1.id:
					found_markers.append(marker_1.id)
					available_markers_1.append([float(marker_1.center[0]), float(marker_1.center[1])])
					available_markers_2.append([float(marker_2.center[0]), float(marker_2.center[1])])
					marker_id.append([float(marker_2.center[0]), float(marker_2.center[1]), int(marker_1.id)])
					# print(marker_1.id)
					break

			# print(markers_1['3789'])
			# print(marker.id)
			# if marker.id in markers_1:
				# print('yes it is')

			# all_markers[marker.id]['x'] = marker.center[0]
			# all_markers[marker.id]['y'] = marker.center[1]

		# return all_markers

		# print(found_markers)

		# print 'world: %s car: %s'%(len(markers_1), len(markers_2))

		return available_markers_1, available_markers_2, len(found_markers), markers_1, markers_2, marker_id
	@staticmethod
	def find_homo(img_1, img_2):
		available_markers_1, available_markers_2, length, m1, m2, marker_id = calibrate.detect_from_stereo(img_1, img_2)
		if length >= 4:
			try:
				h, status = cv2.findHomography(np.array(available_markers_1), np.array(available_markers_2), cv2.RANSAC, 75)
				return h, True, available_markers_1, available_markers_2, m1, m2, marker_id
			except:
				return False, False, available_markers_1, available_markers_2, m1, m2, marker_id
			
		else:
			return False, False, available_markers_1, available_markers_2, m1, m2, marker_id

