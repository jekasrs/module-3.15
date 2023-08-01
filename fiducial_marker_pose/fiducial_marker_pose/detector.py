import cv2
import numpy as np


class Detector:
    ARUCO_DICTIONARIES = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000
    }

    def __init__(self, aruco_type, intrinsics, distortion_coefficients):
        self.__aruco_type = aruco_type
        self.__intrinsics = np.float32(intrinsics).reshape((3, 3))
        self.__distortion_coefficients = distortion_coefficients

        self.__aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICTIONARIES[aruco_type])
        self.__aruco_params = cv2.aruco.DetectorParameters()
        self.__detector = cv2.aruco.ArucoDetector(self.__aruco_dict, self.__aruco_params)

        self.__image = None
        self.__ids = None
        self.__corners = None

    def feed(self, image):
        corners, ids, _ = self.__detector.detectMarkers(image)
        self.__image = image
        self.__ids = ids
        self.__corners = corners

    def get_pose(self, marker_size, corners):
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0.0],
                                  [marker_size / 2, marker_size / 2, 0.0],
                                  [marker_size / 2, -marker_size / 2, 0.0],
                                  [-marker_size / 2, -marker_size / 2, 0.0]],
                                 dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(marker_points,
                                           corners,
                                           self.__intrinsics,
                                           self.__distortion_coefficients,
                                           flags=cv2.SOLVEPNP_IPPE_SQUARE)

        return rvec, tvec

    def visualise(self, marker_size):
        image = self.__image
        if len(self.__corners) > 0:
            image = cv2.aruco.drawDetectedMarkers(image, self.__corners, self.__ids)

            for i in range(0, len(self.__ids)):
                rvec, tvec = self.get_pose(marker_size, self.__corners[i])
                cv2.drawFrameAxes(image,
                                  self.__intrinsics, self.__distortion_coefficients,
                                  rvec, tvec, 0.1)

        return image

    def get_corners(self):
        return self.__corners

    def get_ids(self):
        return self.__ids
