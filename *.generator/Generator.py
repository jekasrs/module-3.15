import cv2
import numpy as np


class Generator:
    """Creates aruco markers from defined vocabulary and saves as .png in directory '/examples_aruco_markers'
        1. ARUCO_DICTIONARIES is array of dictionaries:
            Indicates the number of bits and the number of markers contained
            Is predefined in openCV
            E.g. DICT_4X4_1000 = 4x4 bits, 1000 codes
        2. COLOR_BITS:
            Defines how many colour will be used for generated marker's image (step of 2)
            E.g. 2^1 = 2 [black and white]
        3. BORDER_MARKER: Border of marker on the picture"""

    ARUCO_DICTIONARIES = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
    }
    COLOUR_BITS = 1
    BORDER_MARKER = 1

    @classmethod
    def validate_aruco_type_num(cls, aruco_type_num):
        if type(aruco_type_num) is not int:
            raise TypeError("aruco_type_num must be int type")

    def __init__(self, aruco_type_num, ids):
        self.validate_aruco_type_num(aruco_type_num)

        if aruco_type_num == 1:
            self.aruco_type = "DICT_4X4_50"
            self.tag_size = 50
        elif aruco_type_num == 2:
            self.aruco_type = "DICT_4X4_100"
            self.tag_size = 100
        elif aruco_type_num == 3:
            self.aruco_type = "DICT_4X4_250"
            self.tag_size = 250
        elif aruco_type_num == 4:
            self.aruco_type = "DICT_4X4_1000"
            self.tag_size = 1000
        else:
            raise ValueError("aruco_type_num=" + str(aruco_type_num) + " is not defined")

        # Get all markers from dictionary of the type
        self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICTIONARIES[self.aruco_type])

        self.ids_of_markers = ids

    def generate_all(self):
        for markerID in self.ids_of_markers:
            # Matrix of array initialised by zeroes and has 1 bit for colour (black and white)
            tag = np.zeros((self.tag_size, self.tag_size, self.COLOUR_BITS), dtype="uint8")

            # Filling tag by marker's pixels
            cv2.aruco.drawMarker(self.arucoDict, markerID, self.tag_size, tag, self.BORDER_MARKER)

            # Save marker as .png to director:
            # Name of each picture is like '{aruco_type}_{id}.png'
            tag_name = "examples_aruco_markers/" + self.aruco_type + "_" + str(markerID) + ".png"
            cv2.imwrite(tag_name, tag)
            print("Aruco type '{}' with ID '{}' is saved".format(self.aruco_type, markerID))
