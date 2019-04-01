import numpy as np
import cv2
from cv2 import aruco

#img = np.full((2480, 3508), 255)
# 1cm = 124px
cm = 124
sz = 3.5
markers = np.array([[10, 11, 12],
                    [13, 14, 15],
                    [16, 17, 18]])
'''
for t in range(8):
    print('[', end = "")
    for l in range(3):
        print(str(t * 3 + l), end = "")
        if l != 2:
            print(", ", end = "")
    print('],')
'''

for t in range(8):
    y = 15 + 20 * t
    print('[', end = "")
    for l in range(3):
        x = 15 + 20 * l
        print('[', x, ', ', y, ']', end = "", sep = "")
        if l != 2:
            print(", ", end = "")
    print('],')
