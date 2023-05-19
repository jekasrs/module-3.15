import cv2

# Each dictionary:
#   Indicates the number of bits and the number of markers contained
#   Is predefined in openCV
#
# E.g. DICT_4X4_1000 = 4x4 bits, 1000 codes
import numpy as np

ARUCO_DICTIONARIES = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

# ids are used in algorithms
IDS_OF_MARKERS = [8, 9]

# tuning parameters of camera after collaboration
matrix_coefficients = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
distortion_coefficients = np.array((-0.43948, 0.18514, 0, 0))

# matrix_coefficients = None
# distortion_coefficients = None

# Type of dictionary
aruco_type = "DICT_4X4_1000"

# The size of aruco marker
# Equals in abbreviation in title of aruco_type
tag_size = 1000

# Defines how many colour will be used for generated marker's image
# Step of 2
# E.g. 2^1 = 2 [black and white]
COLOR_BITS = 1

# Border of marker on the picture
BORDER_MARKER = 1

# How many markers is used
NUMBER_OF_MARKERS = 10

