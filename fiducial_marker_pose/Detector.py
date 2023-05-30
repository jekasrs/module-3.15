import cv2
import numpy as np


class Detector:
    """Displays markers and/or vectors on the image"""

    ARUCO_DICTIONARIES = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
    }
    matrix_coefficients = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
    distortion_coefficients = np.array((-0.43948, 0.18514, 0, 0))

    @staticmethod
    def aruco_display(corners, ids, image):
        if len(corners) > 0:

            ids = ids.flatten()  # get one dimension array

            # for each recognized marker
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))  # 4 points (x,y)
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # drawing lines(boundary box) to circle the marker
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (255, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (255, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (255, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (255, 255, 0), 2)

                # drawing center of marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                # sign marker with id
                cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return image

    @classmethod
    def aruco_get_pose_of_markers(cls, corners, ids):
        rvec_array = []
        tvec_array = []

        # rvec и tvec в OpenCV являются векторами numpy.
        # rvec содержит значения углов поворота в радианах (ориентация объекта в пространстве)
        # tvec содержит транслацию объекта в метрах (положение)
        if len(corners) > 0:
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02,
                                                                               cls.matrix_coefficients,
                                                                               cls.distortion_coefficients)
                rvec_array.append(rvec)
                tvec_array.append(tvec)
        return rvec_array, tvec_array

    def __init__(self, aruco_type):
        self.__image = None
        self.__ids = None
        self.__corners = None

        self.__is_markers_show = False
        self.__is_vectors_show = False

        self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICTIONARIES[aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def feed(self, image):
        corners, ids, _ = cv2.aruco.detectMarkers(image, self.arucoDict, parameters=self.arucoParams)
        self.__image = image
        self.__corners = corners
        self.__ids = ids

    def visualise(self):
        res_img = self.__image
        if len(self.__corners) > 0:
            if self.__is_markers_show:
                res_img = self.aruco_display(self.__corners, self.__ids, res_img)

            if self.__is_vectors_show:
                rvec_arr, tvec_arr = self.aruco_get_pose_of_markers(self.__corners, self.__ids)
                for r, t in zip(rvec_arr, tvec_arr):
                    cv2.drawFrameAxes(res_img, self.matrix_coefficients, self.distortion_coefficients, r, t, 0.01)
        return res_img

    def setShowMarkers(self, arg):
        self.__is_markers_show = arg

    def setShowVectors(self, arg):
        self.__is_vectors_show = arg


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    aruco_type = "DICT_4X4_1000"
    detector = Detector(aruco_type)

    detector.setShowMarkers(True)
    detector.setShowVectors(True)

    while cap.isOpened():
        _, img = cap.read()
        detector.feed(img)
        img = detector.visualise()
        flipped_img = cv2.flip(img, 1)
        cv2.imshow("Image", flipped_img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    cap.release()
