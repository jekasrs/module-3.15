import cv2
import numpy as np

from defines import ARUCO_DICTIONARIES, aruco_type, IDS_OF_MARKERS, distortion_coefficients, matrix_coefficients

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICTIONARIES[aruco_type])
arucoParams = cv2.aruco.DetectorParameters_create()


# Returns an array of id markers that the user wants to generate
# input:
#   size - number of markers
# return:
#   ids  - array of id markers
def read_markerIds(size):
    ids = []
    for i in range(0, size):
        id = int(input("Enter next id: "))
        ids.append(id)

    return ids


# Save marker as .png to director:
# The location of folder is in the project ../arucoMarkers
# Name of each picture is like '{aruco_type}_{id}.png'
# input:
#   markerID - id of marker
#   aruco_type - type of generated markers from dictionary
#   aruco_pixels - two-dimensional array of each bite of image
# return:
#   void
def aruco_save(markerID, aruco_type, aruco_pixels):
    tag_name = "arucoMarkers/" + aruco_type + "_" + str(markerID) + ".png"
    cv2.imwrite(tag_name, aruco_pixels)


# Display aruco on the screen
# input:
#   corners - array of corner points
#   ids - all id, that were recognized
#   image - captured image
# return:
#   image - the modified image
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


# Detect all possible markers, but recognize only then
# that were generated
# input:
#   image - current processed image
# return:
#   corners - array of corner points
#   ids - all id, that were recognized
def aruco_detect(image):
    corners, ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    indexes = []  # must be deleted elements of indexes in array
    pointer = 0
    if len(corners) > 0:
        list_of_corners = list(corners)
        ids = ids.flatten()  # get one dimension array
        for markerId in ids:
            if not IDS_OF_MARKERS.__contains__(markerId):
                indexes.append(pointer)
                list_of_corners.pop(pointer)
            pointer = pointer + 1
        ids_new = np.delete(ids, indexes)
        corners_new = tuple(list_of_corners)
        return corners_new, ids_new
    return corners, ids


# rvec и tvec в OpenCV являются векторами numpy.
# rvec содержит значения углов поворота в радианах (ориентация объекта в пространстве)
# tvec содержит транслацию объекта в метрах (положение)
def aruco_get_pose_of_marker(corners, ids):
    rvec_array = []
    tvec_array = []
    if len(corners) > 0:
        for i in range(0, len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02,
                                                                           matrix_coefficients,
                                                                           distortion_coefficients)
            rvec_array.append(rvec)
            tvec_array.append(tvec)
    return rvec_array, tvec_array


# Draw vectors (x,y,z) of the marker in the image
def draw_vectors(image, rvec, tvec):
    cv2.drawFrameAxes(image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


# Draw each group of vectors on markers in the image
def draw_all_vectors(image, rvec_arr, tvec_arr):
    for r, v in zip(rvec_arr, tvec_arr):
        draw_vectors(image, r, v)
    return image


