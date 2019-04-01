import numpy as np
import cv2
from cv2 import aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
#img = np.full((2480, 3508), 255)
img = np.full((2530, 3558), 255)
# 1cm = 124px
cm = 124
sz = 3.5
try:
    for t in range(int(2480 / (cm * sz))):
        for l in range(int(3508 / (cm * sz))):
            m = aruco.drawMarker(aruco_dict, 80 + t * int(3508 / (cm * sz)) + l, cm * 3)
            for i in range(len(m)):
                for k in range(len(m[0])):
                    img[int(20 + i + t * cm * sz)][int(20 + k + l * cm * sz)] = m[i][k]
except:
    pass

cv2.imwrite("field.png", img)