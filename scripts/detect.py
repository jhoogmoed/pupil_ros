#!/usr/bin/env python
#This code detects markers in an image
import cv2
import numpy as np

from numpy import array, rot90
#From lib.coding import decode, extract_hamming_code
from marker import MARKER_SIZE, HammingMarker

#Matrix of coordinates that should be black for the marker to be valid
BORDER_3_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4],
    [1, 0], [1, 4],
    [2, 0], [2, 4],
    [3, 0], [3, 4],
    [4, 0], [4, 1], [4, 2], [4, 3], [4, 4],
]

#Array of found markers
FOUND_MARKERS = []


#Checks if a marker is valid and rotates it upright
def validate_3(marker):
    for crd in BORDER_3_COORDINATES:
        if marker[crd[0], crd[1]] != 0.0:
            raise ValueError('Border contians not entirely black parts.')

    rotation = 0
    if marker[3][1] == 0.0:
        rotation = 0
    elif marker[1][1] == 0.0:
        rotation = 1
    elif marker[1][3] == 0.0:
        rotation = 2
    elif marker[3][3] == 0.0:
        rotation = 3
    else:
        print('invalid!')
        raise ValueError('Not valid marker.')

    marker = rot90(marker, k=rotation)

    if marker[3][3] == 0.0 or marker[1][1] == 0.0 or marker[1][3] == 0.0:
         raise ValueError('Not valid marker.')

    return marker

#Calculate the marker id
def get_marker_id(marker):
    marker_id = 31-(marker[2][2]*(2**0) + marker[2][1]*(2**1) + marker[3][2]*(2**2) + marker[2][3]*(2**3) + marker[1][2]*(2**4))
    return marker, marker_id

#Detect the marker id's and coordinates in an image, using a threshold
def detect_markers(img, threshold):
    width, height, _ = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 10, 100)

    #Find contours in image
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]

    # We only keep the long enough contours
    min_contour_length = min(width, height) / 50
    contours = [contour for contour in contours if len(contour) > min_contour_length]
    warped_size = 25
    canonical_marker_coords = array(((0, 0),
                                     (warped_size - 1, 0),
                                     (warped_size - 1, warped_size - 1),
                                     (0, warped_size - 1)),
                                    dtype='float32')
    #Initialize
    markers_list = []
    FOUND_MARKERS = []

    #Loop through found contours
    for contour in contours:

        #Skip if contour is open or invalid length
        approx_curve = cv2.approxPolyDP(contour, len(contour) * 0.01, True)
        if not (len(approx_curve) == 4 and cv2.isContourConvex(approx_curve)):
            continue

        #Warp the contour to a square and convert to grayscale
        sorted_curve = array(cv2.convexHull(approx_curve, clockwise=False),
                             dtype='float32')
        persp_transf = cv2.getPerspectiveTransform(sorted_curve, canonical_marker_coords)
        warped_img = cv2.warpPerspective(img, persp_transf, (warped_size, warped_size))
        warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        
        #Convert to binary matrix using a threshold. The binary matrix is a 5x5 matrix, where the first and last row and column must be 1 (= black)
        _, warped_bin = cv2.threshold(warped_gray, threshold, 255, cv2.THRESH_BINARY)
        marker = warped_bin.reshape(
            [MARKER_SIZE, int(warped_size / MARKER_SIZE), MARKER_SIZE, int(warped_size / MARKER_SIZE)]
        )
        marker = marker.mean(axis=3).mean(axis=1)
        marker[marker < 127] = 0
        marker[marker >= 127] = 1

        try:
            #Check if the marker is valid, and calculate the id of the marker
            marker = validate_3(marker)
            marker, marker_id = get_marker_id(marker)

            #Skip if marker has already been found
            if FOUND_MARKERS.__contains__(marker_id):
                raise ValueError('Marker already found.')
            else:
                FOUND_MARKERS.append(marker_id)

            #Add found marker to list of markers as HammingMarker obj
            markers_list.append(HammingMarker(id=marker_id, contours=approx_curve))

        except ValueError:
            continue

    return markers_list
