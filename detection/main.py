import cv2

from detection.utils import aruco_display, aruco_detect, aruco_get_pose_of_marker, draw_all_vectors

cap = cv2.VideoCapture(0)

while cap.isOpened():
    _, img = cap.read()

    corners, ids = aruco_detect(img)

    if len(corners) != 0:
        rvec_arr, tvec_arr = aruco_get_pose_of_marker(corners, ids)
        img = aruco_display(corners, ids, img)
        img = draw_all_vectors(img, rvec_arr, tvec_arr)

    flipped_img = cv2.flip(img, 1)

    cv2.imshow("Image", flipped_img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
cap.release()
