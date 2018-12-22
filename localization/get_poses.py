import numpy as np
import cv2
import cv2.aruco as aruco

camera_mtx = np.array([[509.55744588,   0.        , 331.84201483],
                       [  0.        , 512.03515271, 250.87030742],
                       [  0.        ,   0.        ,   1.        ]])


distortion = np.array([[ 0.01855503,  0.13617834,  0.00036737,  0.00710654, -0.36514252]]) * 0
# ignore nonlinear camera effects for now

def find_markers(image):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    return corners

def get_poses(corners):
    distortion_coeffs = distortion * 0
    if corners:
        rs, ts, _ = aruco.estimatePoseSingleMarkers(corners, 5, camera_mtx, distortion_coeffs)
        return rs, ts
    return [], []

    
for i in range(10,385,1):
    gray = cv2.imread('/home/isaac/Downloads/isaac/jet_image_' + str(i) +'.jpg',0)
    if gray is not None:
        print get_poses(find_markers(gray))
