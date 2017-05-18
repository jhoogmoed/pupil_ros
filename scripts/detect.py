import cv2

import numpy as np
from numpy import array, rot90

# from lib.coding import decode, extract_hamming_code
from marker import MARKER_SIZE, HammingMarker

BORDER_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [1, 0], [1, 6], [2, 0], [2, 6], [3, 0],
    [3, 6], [4, 0], [4, 6], [5, 0], [5, 6], [6, 0], [6, 1], [6, 2], [6, 3], [6, 4], [6, 5], [6, 6],
]

BORDER_3_COORDINATES = [
    [0, 0], [0, 1], [0, 2], [0, 3], [0, 4],
    [1, 0], [1, 4],
    [2, 0], [2, 4],
    [3, 0], [3, 4],
    [4, 0], [4, 1], [4, 2], [4, 3], [4, 4],
]


ORIENTATION_MARKER_COORDINATES = [[1, 1], [1, 5], [5, 1], [5, 5]]

FOUND_MARKERS = []

def validate_3(marker):
    for crd in BORDER_3_COORDINATES:
        if marker[crd[0], crd[1]] != 0.0:
            raise ValueError('Border contians not entirely black parts.')

    # print(marker[3][1])
    # print(marker[2][2])
    # print(marker)
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

    # if np.allclose(marker, MARKER_2):
    #     return marker
    # else:
    #     raise ValueError('Not valid marker.')

def get_marker_id(marker):

    # print(MARKER_2)
    # print(marker)
    marker_id = 31-(marker[2][2]*(2**0) + marker[2][1]*(2**1) + marker[3][2]*(2**2) + marker[2][3]*(2**3) + marker[1][2]*(2**4))
    # print(marker_id)

    # FOUND_MARKERS = []

    # if FOUND_MARKERS.__contains__(marker_id):
    #     raise ValueError('Marker already found.')

    # FOUND_MARKERS.append(marker_id)

    # print('found %s' % marker_id)

    return marker, marker_id


def detect_markers(img):
    width, height, _ = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray, 10, 100)
    contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2:]

    # We only keep the long enough contours
    min_contour_length = min(width, height) / 50
    contours = [contour for contour in contours if len(contour) > min_contour_length]
    # warped_size = 49
    warped_size = 25
    canonical_marker_coords = array(((0, 0),
                                     (warped_size - 1, 0),
                                     (warped_size - 1, warped_size - 1),
                                     (0, warped_size - 1)),
                                    dtype='float32')
    markers_list = []

    FOUND_MARKERS = []

    for contour in contours:
        approx_curve = cv2.approxPolyDP(contour, len(contour) * 0.01, True)
        if not (len(approx_curve) == 4 and cv2.isContourConvex(approx_curve)):
            continue

        sorted_curve = array(cv2.convexHull(approx_curve, clockwise=False),
                             dtype='float32')
        persp_transf = cv2.getPerspectiveTransform(sorted_curve, canonical_marker_coords)
        warped_img = cv2.warpPerspective(img, persp_transf, (warped_size, warped_size))
        warped_gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        
        _, warped_bin = cv2.threshold(warped_gray, 127, 255, cv2.THRESH_BINARY)
        marker = warped_bin.reshape(
            [MARKER_SIZE, int(warped_size / MARKER_SIZE), MARKER_SIZE, int(warped_size / MARKER_SIZE)]
        )

        marker = marker.mean(axis=3).mean(axis=1)

        marker[marker < 127] = 0
        marker[marker >= 127] = 1

        # print(marker)
        # print('test')

        try:
            cv2.imshow('warped_img', warped_img)
            cv2.imshow('marker_img', marker)
            print(marker)
            marker = validate_3(marker)
            marker, marker_id = get_marker_id(marker)



            if FOUND_MARKERS.__contains__(marker_id):
                raise ValueError('Marker already found.')
            else:
                FOUND_MARKERS.append(marker_id)

            # marker_id = int(np.sum(marker))
            # cv2.imshow('video_%s' % marker_id, warped_img)
            
            # print('marker: %s' % marker_id)
            # print(marker)
            
            # hamming_code = extract_hamming_code(marker)
            # marker_id = int(decode(hamming_code), 2)
            markers_list.append(HammingMarker(id=marker_id, contours=approx_curve))
            # print(marker)
            # print()
            # print('result')
            # cv2.imshow('video_123', warped_bin)
        except ValueError:
            continue

        # print(FOUND_MARKERS)

    return markers_list
