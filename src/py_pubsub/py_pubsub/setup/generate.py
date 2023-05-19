import cv2
import numpy as np
from detection.defines import ARUCO_DICTIONARIES, tag_size, aruco_type, NUMBER_OF_MARKERS, COLOR_BITS, BORDER_MARKER, \
    IDS_OF_MARKERS
from detection.utils import read_markerIds, aruco_save

# Get all markers from dictionary of the type
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICTIONARIES[aruco_type])

# Read all ids
IDS_OF_MARKERS.append(read_markerIds(NUMBER_OF_MARKERS))

for markerID in IDS_OF_MARKERS:
    print("Aruco type '{}' with ID '{}'".format(aruco_type, markerID))

    # Matrix of array initialised by zeroes and has 1 bit for colour (black and white)
    tag = np.zeros((tag_size, tag_size, COLOR_BITS), dtype="uint8")

    # Filling tag by marker's pixels
    cv2.aruco.drawMarker(arucoDict, markerID, tag_size, tag, BORDER_MARKER)

    # Save img
    aruco_save(markerID, aruco_type, tag)
